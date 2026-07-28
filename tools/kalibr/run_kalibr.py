#!/usr/bin/env python3
"""Create a ROS 1 bag from an exported dataset and run Kalibr."""

from __future__ import annotations

import argparse
import datetime
import json
import shlex
import subprocess
import sys
from pathlib import Path

import yaml


SUPPORTED_CAMERA_MODELS = {
    "pinhole-radtan",
    "pinhole-equi",
    "pinhole-fov",
    "omni-none",
    "omni-radtan",
    "eucm-none",
    "ds-none",
}


def load_job(input_dir: Path) -> dict:
    job_path = input_dir / "job.yaml"
    if not job_path.is_file():
        raise ValueError(f"missing Kalibr job file: {job_path}")
    with job_path.open("r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError("job.yaml must contain a mapping")
    if value.get("schema_version") != 1:
        raise ValueError("unsupported job.yaml schema_version")
    return value


def safe_input_path(input_dir: Path, relative_value: str) -> Path:
    relative = Path(relative_value)
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError(f"input path must be relative and remain in /input: {relative}")
    result = (input_dir / relative).resolve()
    if input_dir not in result.parents:
        raise ValueError(f"input path escapes /input: {relative}")
    return result


def validate_job(input_dir: Path, job: dict) -> tuple[list[str], list[str], Path]:
    cameras = job.get("cameras")
    if not isinstance(cameras, list) or not cameras:
        raise ValueError("job.yaml cameras must be a non-empty list")

    topics = []
    models = []
    expected_camera_dirs = []
    for index, camera in enumerate(cameras):
        if not isinstance(camera, dict):
            raise ValueError(f"job.yaml cameras[{index}] must be a mapping")
        expected_camera = f"cam{index}"
        if camera.get("camera") != expected_camera:
            raise ValueError(
                f"camera chain must be contiguous; expected {expected_camera}"
            )
        expected_camera_dirs.append(expected_camera)
        camera_dir = input_dir / expected_camera
        images = sorted(camera_dir.glob("*.png")) if camera_dir.is_dir() else []
        if not images:
            raise ValueError(f"{camera_dir} contains no PNG images")
        expected_frames = int(camera.get("frames", 0))
        if expected_frames != len(images):
            raise ValueError(
                f"{expected_camera} contains {len(images)} images, "
                f"but job.yaml records {expected_frames}"
            )
        invalid_names = [path.name for path in images if not path.stem.isdigit()]
        if invalid_names:
            raise ValueError(
                f"{expected_camera} contains non-timestamp PNG names: "
                + ", ".join(invalid_names[:3])
            )
        expected_topic = f"/{expected_camera}/image_raw"
        topic = str(camera.get("topic", expected_topic))
        if topic != expected_topic:
            raise ValueError(
                f"{expected_camera} topic must be {expected_topic}, got {topic}"
            )
        topics.append(topic)
        model = str(camera.get("model", ""))
        if model not in SUPPORTED_CAMERA_MODELS:
            raise ValueError(f"unsupported model for {expected_camera}: {model}")
        models.append(model)

    actual_camera_dirs = sorted(
        path.name
        for path in input_dir.iterdir()
        if path.is_dir() and path.name.startswith("cam")
    )
    if set(actual_camera_dirs) != set(expected_camera_dirs):
        raise ValueError(
            "camera directories do not match job.yaml: "
            f"expected {expected_camera_dirs}, got {actual_camera_dirs}"
        )

    target = safe_input_path(input_dir, str(job.get("target", "target.yaml")))
    if not target.is_file():
        raise ValueError(f"target file does not exist: {target}")
    return topics, models, target


def run_logged(command: list[str], log_path: Path, *, cwd: Path) -> None:
    with log_path.open("a", encoding="utf-8") as log:
        rendered = shlex.join(command)
        log.write(f"$ {rendered}\n")
        log.flush()
        print(f"$ {rendered}", flush=True)
        process = subprocess.Popen(
            command,
            cwd=str(cwd),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        assert process.stdout is not None
        for line in process.stdout:
            sys.stdout.write(line)
            log.write(line)
        return_code = process.wait()
        if return_code:
            raise subprocess.CalledProcessError(return_code, command)


def utc_now() -> str:
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_dir", nargs="?", default="/input")
    parser.add_argument("output_dir", nargs="?", default="/output")
    args = parser.parse_args()

    input_dir = Path(args.input_dir).resolve()
    output_dir = Path(args.output_dir).resolve()
    if not input_dir.is_dir():
        parser.error(f"input directory does not exist: {input_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    if input_dir == output_dir:
        parser.error("input and output directories must be different")
    if any(output_dir.iterdir()):
        parser.error(f"output directory must be empty: {output_dir}")

    started_at = utc_now()
    log_path = output_dir / "kalibr.log"
    metadata_path = output_dir / "run_metadata.json"
    bag_path = output_dir / "kalibr.bag"
    commit_path = Path("/opt/kalibr_commit")
    kalibr_commit = (
        commit_path.read_text(encoding="utf-8").strip()
        if commit_path.is_file()
        else "unknown"
    )
    metadata = {
        "schema_version": 1,
        "status": "running",
        "started_at": started_at,
        "kalibr_commit": kalibr_commit,
        "input": str(input_dir),
        "output": str(output_dir),
    }
    metadata_path.write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    try:
        job = load_job(input_dir)
        topics, models, target = validate_job(input_dir, job)
        approx_sync = float(job.get("approximate_sync_s", 0.02))
        if approx_sync < 0.0:
            raise ValueError("approximate_sync_s must be non-negative")

        bag_command = [
            "kalibr_bagcreater",
            "--folder",
            str(input_dir),
            "--output-bag",
            str(bag_path),
        ]
        calibration_command = [
            "kalibr_calibrate_cameras",
            "--bag",
            str(bag_path),
            "--topics",
            *topics,
            "--models",
            *models,
            "--target",
            str(target),
            "--approx-sync",
            str(approx_sync),
            "--dont-show-report",
        ]
        run_logged(bag_command, log_path, cwd=output_dir)
        run_logged(calibration_command, log_path, cwd=output_dir)

        expected = [
            output_dir / "kalibr-camchain.yaml",
            output_dir / "kalibr-results-cam.txt",
            output_dir / "kalibr-report-cam.pdf",
        ]
        missing = [str(path) for path in expected if not path.is_file()]
        if missing:
            raise RuntimeError(
                "Kalibr completed without expected output files: "
                + ", ".join(missing)
            )

        metadata.update(
            {
                "status": "completed",
                "finished_at": utc_now(),
                "topics": topics,
                "models": models,
                "target": str(target),
                "approximate_sync_s": approx_sync,
                "outputs": [str(path) for path in expected],
            }
        )
    except Exception as exc:
        metadata.update(
            {
                "status": "failed",
                "finished_at": utc_now(),
                "error": str(exc),
            }
        )
        metadata_path.write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        raise

    metadata_path.write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
