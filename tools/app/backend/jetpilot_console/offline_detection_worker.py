from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import signal
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Sequence

from .analysis_worker import Progress, _atomic_json


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _stop_process(process: subprocess.Popen[str] | None, timeout: float = 20.0) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=timeout)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5.0)


def _start(command: list[str], env: dict[str, str]) -> subprocess.Popen[str]:
    return subprocess.Popen(command, env=env, text=True, start_new_session=True)


def _wait_for_detection_publisher(
    env: dict[str, str],
    process: subprocess.Popen[str],
    topic: str,
    timeout_s: float,
) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"Detector launch exited early with status {process.returncode}")
        probe = subprocess.run(
            ["ros2", "topic", "info", topic],
            env=env,
            text=True,
            capture_output=True,
            timeout=10.0,
            check=False,
        )
        if probe.returncode == 0 and re.search(r"Publisher count:\s*[1-9]", probe.stdout):
            return
        time.sleep(1.0)
    raise RuntimeError(f"Timed out waiting for detector publisher on {topic}")


def _validate_sidecar(path: Path, topic: str) -> int:
    from .preflight import parse_rosbag_metadata

    metadata = path / "metadata.yaml"
    if not metadata.is_file():
        raise RuntimeError("Offline detection recorder did not create metadata.yaml")
    parsed = parse_rosbag_metadata(
        metadata.read_text(encoding="utf-8", errors="replace")
    )
    topics = parsed.get("topics") if isinstance(parsed, dict) else {}
    topic_record = topics.get(topic) if isinstance(topics, dict) else None
    count = int(topic_record.get("message_count") or 0) if isinstance(topic_record, dict) else 0
    if count <= 0:
        raise RuntimeError(
            "Offline detector produced no Detection2DArray messages; check image topics and model bindings"
        )
    return count


def run(args: argparse.Namespace) -> dict[str, object]:
    rosbag = args.rosbag.expanduser().resolve()
    model_root = args.model_root.expanduser().resolve()
    output_bag = args.output_bag.expanduser().resolve()
    manifest_path = args.manifest.expanduser().resolve()
    progress = Progress(args.status_file.expanduser().resolve())
    if not (rosbag / "metadata.yaml").is_file():
        raise FileNotFoundError(f"Source rosbag is invalid: {rosbag}")
    model_path = model_root / "model.onnx"
    if not model_path.is_file():
        raise FileNotFoundError(f"YOLOv8 model.onnx was not found: {model_path}")
    if output_bag.exists():
        raise FileExistsError(f"Detection sidecar already exists: {output_bag}")
    output_bag.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.parent.mkdir(parents=True, exist_ok=True)

    env = dict(os.environ)
    launch_process: subprocess.Popen[str] | None = None
    recorder_process: subprocess.Popen[str] | None = None
    player_process: subprocess.Popen[str] | None = None
    interrupted = False

    def handle_signal(_signum: int, _frame: object) -> None:
        nonlocal interrupted
        interrupted = True
        raise InterruptedError("Offline detection was interrupted")

    previous_term = signal.signal(signal.SIGTERM, handle_signal)
    previous_int = signal.signal(signal.SIGINT, handle_signal)
    try:
        progress.update("offline_detection_prepare", 0.45, "物体検出モデルを起動しています。")
        launch_process = _start(
            [
                "ros2", "launch", "jetpilot_object_detection", "yolov8_tensor_rt.launch.py",
                "run_standalone:=true",
                "use_sim_time:=true",
                f"image_topic:={args.image_topic}",
                f"camera_info_topic:={args.camera_info_topic}",
                f"model_root:={model_root}",
                f"source_width:={args.source_width}",
                f"source_height:={args.source_height}",
                f"network_width:={args.network_width}",
                f"network_height:={args.network_height}",
                f"max_inference_fps:={args.max_inference_fps:g}",
                f"confidence_threshold:={args.confidence_threshold:g}",
                f"nms_threshold:={args.nms_threshold:g}",
                "force_engine_update:=false",
            ],
            env,
        )
        _wait_for_detection_publisher(
            env, launch_process, args.detections_topic, args.readiness_timeout_s
        )

        progress.update("offline_detection_record", 0.48, "検出結果sidecarの記録を開始しています。")
        recorder_process = _start(
            [
                "ros2", "bag", "record", "-s", "mcap", "-o", str(output_bag),
                args.detections_topic, args.diagnostics_topic,
            ],
            env,
        )
        time.sleep(2.0)
        if recorder_process.poll() is not None:
            raise RuntimeError(
                f"Detection sidecar recorder exited early with status {recorder_process.returncode}"
            )

        progress.update("offline_detection_replay", 0.50, "rosbag画像を物体検出器へ再生しています。")
        player_process = _start(
            [
                "ros2", "bag", "play", str(rosbag), "--clock", "--rate",
                f"{args.replay_rate:g}", "--topics", args.image_topic, args.camera_info_topic,
            ],
            env,
        )
        return_code = player_process.wait()
        player_process = None
        if return_code != 0:
            raise RuntimeError(f"rosbag replay failed with status {return_code}")
        time.sleep(2.0)
        progress.update("offline_detection_finalize", 0.64, "検出結果sidecarを確定しています。")
        _stop_process(recorder_process)
        recorder_process = None
        _stop_process(launch_process)
        launch_process = None

        detection_count = _validate_sidecar(output_bag, args.detections_topic)
        engine_path = model_root / "model.plan"
        manifest = {
            "schema_version": 1,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "source_rosbag": str(rosbag),
            "source_rosbag_metadata_sha256": _sha256(rosbag / "metadata.yaml"),
            "model_root": str(model_root),
            "model": {
                "onnx": str(model_path),
                "onnx_sha256": _sha256(model_path),
                "engine": str(engine_path) if engine_path.is_file() else "",
                "engine_sha256": _sha256(engine_path) if engine_path.is_file() else "",
            },
            "classes": ["vehicle", "barrier"],
            "input": {
                "source_size": [args.source_width, args.source_height],
                "network_size": [args.network_width, args.network_height],
                "max_inference_fps": args.max_inference_fps,
                "image_topic": args.image_topic,
                "camera_info_topic": args.camera_info_topic,
            },
            "decoder": {
                "implementation": "jetpilot_object_detection/0.1.0",
                "confidence_threshold": args.confidence_threshold,
                "nms_threshold": args.nms_threshold,
                "detections_topic": args.detections_topic,
            },
            "replay_rate": args.replay_rate,
            "detection_message_count": detection_count,
            "sidecar_bag": str(output_bag),
        }
        _atomic_json(manifest_path, manifest)
        return manifest
    finally:
        _stop_process(player_process, 5.0)
        _stop_process(recorder_process)
        _stop_process(launch_process)
        signal.signal(signal.SIGTERM, previous_term)
        signal.signal(signal.SIGINT, previous_int)
        if interrupted:
            progress.update(
                "stopped", 1.0, "オフライン物体検出を停止しました。", status="stopped"
            )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Replay a rosbag through JetPilot YOLOv8")
    parser.add_argument("--rosbag", type=Path, required=True)
    parser.add_argument("--output-bag", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--status-file", type=Path, required=True)
    parser.add_argument("--model-root", type=Path, required=True)
    parser.add_argument("--image-topic", required=True)
    parser.add_argument("--camera-info-topic", required=True)
    parser.add_argument("--source-width", type=int, default=424)
    parser.add_argument("--source-height", type=int, default=240)
    parser.add_argument("--network-width", type=int, default=224)
    parser.add_argument("--network-height", type=int, default=224)
    parser.add_argument("--max-inference-fps", type=float, default=15.0)
    parser.add_argument("--confidence-threshold", type=float, default=0.35)
    parser.add_argument("--nms-threshold", type=float, default=0.45)
    parser.add_argument("--replay-rate", type=float, default=0.5)
    parser.add_argument("--readiness-timeout-s", type=float, default=120.0)
    parser.add_argument("--detections-topic", default="/perception/detections")
    parser.add_argument(
        "--diagnostics-topic", default="/perception/object_detection/diagnostics"
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    for name in (
        "source_width", "source_height", "network_width", "network_height"
    ):
        if getattr(args, name) <= 0:
            raise ValueError(f"{name} must be positive")
    for name in ("max_inference_fps", "replay_rate", "readiness_timeout_s"):
        if getattr(args, name) <= 0:
            raise ValueError(f"{name} must be positive")
    for name in ("confidence_threshold", "nms_threshold"):
        if not 0.0 <= getattr(args, name) <= 1.0:
            raise ValueError(f"{name} must be in [0, 1]")
    try:
        run(args)
    except InterruptedError:
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
