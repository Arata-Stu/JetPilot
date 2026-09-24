#!/usr/bin/env python3
"""Serve the LED sync UI and approved JSON files from a Docker workspace."""

from __future__ import annotations

import argparse
import json
import mimetypes
import re
import shutil
import subprocess
import sys
import threading
import time
import uuid
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse


def _inside(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
    except ValueError:
        return False
    return True


def build_handler(html_path: Path, record_root: Path, camchain_path: Path):
    jobs: dict[str, dict] = {}
    jobs_lock = threading.Lock()

    def set_job(job_id: str, **values) -> None:
        with jobs_lock:
            jobs[job_id].update(values)

    def run_overlay_job(
        job_id: str,
        *,
        session: Path,
        event_file: Path,
        time_sync: Path,
        output_dir: Path,
        event_window_ms: float,
        event_dilate_px: int,
        alpha: float,
        log_path: Path,
    ) -> None:
        command = [
            sys.executable,
            "-m",
            "multi_sensor_calibration.cli",
            "scenario-overlay",
            "--bag",
            str(session),
            "--event-file",
            str(event_file),
            "--time-sync",
            str(time_sync),
            "--camchain",
            str(camchain_path),
            "--projection",
            "rotation-only",
            "--event-window-ms",
            str(event_window_ms),
            "--event-dilate-px",
            str(event_dilate_px),
            "--alpha",
            str(alpha),
            "--output-dir",
            str(output_dir),
        ]
        set_job(job_id, status="running", started_at=time.time())
        try:
            log_path.parent.mkdir(parents=True, exist_ok=True)
            with log_path.open("w", encoding="utf-8") as log:
                process = subprocess.run(
                    command,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    text=True,
                    check=False,
                )
            if process.returncode != 0:
                tail = log_path.read_text(encoding="utf-8", errors="replace")[-4000:]
                set_job(
                    job_id,
                    status="failed",
                    finished_at=time.time(),
                    error="動画生成に失敗しました。ログを確認してください。",
                    log_tail=tail,
                )
                return
            videos = {
                name: str(output_dir / filename)
                for name, filename in {
                    "comparison": "rgb_vs_overlay.mp4",
                    "overlay": "overlay_polarity.mp4",
                    "events": "polarity_only.mp4",
                }.items()
                if (output_dir / filename).is_file()
            }
            if "comparison" not in videos:
                set_job(
                    job_id,
                    status="failed",
                    finished_at=time.time(),
                    error="処理は終了しましたが、比較動画が見つかりません。",
                )
                return
            set_job(
                job_id,
                status="completed",
                finished_at=time.time(),
                videos=videos,
            )
        except Exception as exc:  # background worker must report errors to the UI
            set_job(
                job_id,
                status="failed",
                finished_at=time.time(),
                error=f"動画生成を開始できませんでした: {exc}",
            )

    class Handler(BaseHTTPRequestHandler):
        server_version = "LedSyncInspector/1.0"

        def _headers(self, status: HTTPStatus, content_type: str, length: int) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(length))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Referrer-Policy", "no-referrer")
            self.end_headers()

        def _send_bytes(
            self,
            payload: bytes,
            *,
            status: HTTPStatus = HTTPStatus.OK,
            content_type: str = "application/octet-stream",
        ) -> None:
            self._headers(status, content_type, len(payload))
            self.wfile.write(payload)

        def _send_json(self, value, status: HTTPStatus = HTTPStatus.OK) -> None:
            payload = json.dumps(value, ensure_ascii=False).encode("utf-8")
            self._send_bytes(
                payload,
                status=status,
                content_type="application/json; charset=utf-8",
            )

        def _send_file(self, candidate: Path) -> None:
            content_type = mimetypes.guess_type(candidate.name)[0] or "application/octet-stream"
            size = candidate.stat().st_size
            range_header = self.headers.get("Range") if candidate.suffix.lower() == ".mp4" else None
            if range_header:
                match = re.fullmatch(r"bytes=(\d*)-(\d*)", range_header.strip())
                if not match:
                    self.send_response(HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
                    self.send_header("Content-Range", f"bytes */{size}")
                    self.end_headers()
                    return
                start_text, end_text = match.groups()
                if start_text:
                    start = int(start_text)
                    end = int(end_text) if end_text else size - 1
                elif end_text:
                    length = min(int(end_text), size)
                    start, end = size - length, size - 1
                else:
                    start, end = 0, size - 1
                end = min(end, size - 1)
                if start < 0 or start >= size or end < start:
                    self.send_response(HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
                    self.send_header("Content-Range", f"bytes */{size}")
                    self.end_headers()
                    return
                length = end - start + 1
                self.send_response(HTTPStatus.PARTIAL_CONTENT)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(length))
                self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
                self.send_header("Accept-Ranges", "bytes")
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.end_headers()
                with candidate.open("rb") as stream:
                    stream.seek(start)
                    remaining = length
                    while remaining:
                        chunk = stream.read(min(1024 * 1024, remaining))
                        if not chunk:
                            break
                        self.wfile.write(chunk)
                        remaining -= len(chunk)
                return
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(size))
            if candidate.suffix.lower() == ".mp4":
                self.send_header("Accept-Ranges", "bytes")
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Referrer-Policy", "no-referrer")
            self.end_headers()
            with candidate.open("rb") as stream:
                shutil.copyfileobj(stream, self.wfile)

        def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
            request = urlparse(self.path)
            if request.path in {"/", "/index.html"}:
                payload = html_path.read_bytes()
                self._send_bytes(payload, content_type="text/html; charset=utf-8")
                return
            if request.path == "/health":
                self._send_json(
                    {
                        "status": "ok",
                        "record_root": str(record_root),
                        "camchain": str(camchain_path),
                    }
                )
                return
            if request.path == "/api/overlay-job":
                values = parse_qs(request.query).get("id", [])
                if len(values) != 1:
                    self._send_json({"error": "job id is required"}, HTTPStatus.BAD_REQUEST)
                    return
                with jobs_lock:
                    job = dict(jobs.get(values[0], {}))
                if not job:
                    self._send_json({"error": "job not found"}, HTTPStatus.NOT_FOUND)
                    return
                self._send_json(job)
                return
            if request.path == "/api/datasets":
                datasets = []
                for candidate in record_root.rglob("led_sync_data.json"):
                    if not candidate.is_file():
                        continue
                    stat = candidate.stat()
                    datasets.append(
                        {
                            "path": str(candidate),
                            "relative_path": str(candidate.relative_to(record_root)),
                            "session": candidate.parent.name,
                            "modified_ns": stat.st_mtime_ns,
                            "size": stat.st_size,
                        }
                    )
                datasets.sort(key=lambda item: item["modified_ns"], reverse=True)
                self._send_json(
                    {"record_root": str(record_root), "datasets": datasets}
                )
                return
            if request.path == "/api/data":
                values = parse_qs(request.query).get("path", [])
                if len(values) != 1 or not values[0].strip():
                    self._send_json(
                        {"error": "query parameter path is required"},
                        HTTPStatus.BAD_REQUEST,
                    )
                    return
                requested = Path(values[0])
                candidate = (
                    requested if requested.is_absolute() else record_root / requested
                ).resolve()
                if not _inside(candidate, record_root):
                    self._send_json(
                        {"error": f"path must be inside {record_root}"},
                        HTTPStatus.FORBIDDEN,
                    )
                    return
                if candidate.suffix.lower() != ".json":
                    self._send_json(
                        {"error": "only JSON files may be loaded"},
                        HTTPStatus.BAD_REQUEST,
                    )
                    return
                if not candidate.is_file():
                    self._send_json(
                        {"error": f"file not found: {candidate}"},
                        HTTPStatus.NOT_FOUND,
                    )
                    return
                content_type = mimetypes.guess_type(candidate.name)[0] or "application/json"
                size = candidate.stat().st_size
                self._headers(HTTPStatus.OK, f"{content_type}; charset=utf-8", size)
                with candidate.open("rb") as stream:
                    shutil.copyfileobj(stream, self.wfile)
                return
            if request.path == "/api/file":
                values = parse_qs(request.query).get("path", [])
                if len(values) != 1 or not values[0].strip():
                    self._send_json(
                        {"error": "query parameter path is required"},
                        HTTPStatus.BAD_REQUEST,
                    )
                    return
                requested = Path(values[0])
                candidate = (
                    requested if requested.is_absolute() else record_root / requested
                ).resolve()
                if not _inside(candidate, record_root):
                    self._send_json(
                        {"error": f"path must be inside {record_root}"},
                        HTTPStatus.FORBIDDEN,
                    )
                    return
                if candidate.suffix.lower() not in {
                    ".jpg",
                    ".jpeg",
                    ".png",
                    ".webp",
                    ".bin",
                    ".mp4",
                }:
                    self._send_json(
                        {"error": "only preview images, videos, and ROI binary data may be loaded"},
                        HTTPStatus.BAD_REQUEST,
                    )
                    return
                if not candidate.is_file():
                    self._send_json(
                        {"error": f"file not found: {candidate}"},
                        HTTPStatus.NOT_FOUND,
                    )
                    return
                self._send_file(candidate)
                return
            self._send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)

        def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
            request = urlparse(self.path)
            if request.path not in {"/api/save-time-sync", "/api/generate-overlay"}:
                self._send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError:
                length = 0
            if length <= 0 or length > 65536:
                self._send_json(
                    {"error": "invalid request size"}, HTTPStatus.BAD_REQUEST
                )
                return
            try:
                payload = json.loads(self.rfile.read(length))
            except (UnicodeDecodeError, json.JSONDecodeError):
                self._send_json({"error": "invalid JSON"}, HTTPStatus.BAD_REQUEST)
                return
            requested = Path(str(payload.get("dataset_path", "")))
            dataset = (
                requested if requested.is_absolute() else record_root / requested
            ).resolve()
            if (
                not _inside(dataset, record_root)
                or dataset.name != "led_sync_data.json"
                or not dataset.is_file()
            ):
                self._send_json(
                    {"error": "dataset must be an existing led_sync_data.json inside record root"},
                    HTTPStatus.FORBIDDEN,
                )
                return
            content = payload.get("yaml")
            if not isinstance(content, str) or not content.strip():
                self._send_json(
                    {"error": "yaml content is required"}, HTTPStatus.BAD_REQUEST
                )
                return
            output = dataset.parent / "time_sync_led.yaml"
            temporary = output.with_suffix(".yaml.tmp")
            temporary.write_text(content, encoding="utf-8")
            temporary.replace(output)
            if request.path == "/api/save-time-sync":
                self._send_json({"path": str(output)})
                return

            if not camchain_path.is_file():
                self._send_json(
                    {"error": f"校正ファイルが見つかりません: {camchain_path}"},
                    HTTPStatus.PRECONDITION_FAILED,
                )
                return
            try:
                if dataset.parents[1].name != "led_sync" or dataset.parents[2].name != "analysis":
                    raise ValueError("解析データの配置が標準構成ではありません")
                experiment_root = dataset.parents[3]
                session = (experiment_root / dataset.parent.name).resolve()
            except (IndexError, ValueError) as exc:
                self._send_json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
                return
            if not _inside(session, record_root) or not session.is_dir():
                self._send_json(
                    {"error": f"元の記録セッションが見つかりません: {session}"},
                    HTTPStatus.NOT_FOUND,
                )
                return
            raw_files = sorted(session.glob("*.raw"))
            if len(raw_files) != 1:
                self._send_json(
                    {"error": f"RAWファイルを1つに特定できません（{len(raw_files)}件）"},
                    HTTPStatus.BAD_REQUEST,
                )
                return
            if not any(session.glob("*.mcap")):
                self._send_json(
                    {"error": "元のMCAP記録が見つかりません"}, HTTPStatus.BAD_REQUEST
                )
                return
            options = payload.get("options") if isinstance(payload.get("options"), dict) else {}
            try:
                event_window_ms = float(options.get("event_window_ms", 10.0))
                event_dilate_px = int(options.get("event_dilate_px", 1))
                alpha = float(options.get("alpha", 0.85))
            except (TypeError, ValueError):
                self._send_json({"error": "動画設定が不正です"}, HTTPStatus.BAD_REQUEST)
                return
            if not 1.0 <= event_window_ms <= 100.0 or not 1 <= event_dilate_px <= 5 or not 0.1 <= alpha <= 1.0:
                self._send_json({"error": "動画設定が範囲外です"}, HTTPStatus.BAD_REQUEST)
                return
            with jobs_lock:
                running = next(
                    (job for job in jobs.values() if job.get("status") in {"queued", "running"}),
                    None,
                )
            if running:
                self._send_json(
                    {"error": "別の動画を生成中です。完了後にもう一度実行してください。"},
                    HTTPStatus.CONFLICT,
                )
                return
            stamp = time.strftime("%Y%m%d_%H%M%S")
            output_dir = experiment_root / "analysis" / "scenario_overlay" / session.name / f"rotation_only_{stamp}"
            log_path = output_dir.parent / f"{output_dir.name}.log"
            job_id = uuid.uuid4().hex
            jobs[job_id] = {
                "id": job_id,
                "status": "queued",
                "session": session.name,
                "output_dir": str(output_dir),
                "log_path": str(log_path),
                "created_at": time.time(),
            }
            worker = threading.Thread(
                target=run_overlay_job,
                kwargs={
                    "job_id": job_id,
                    "session": session,
                    "event_file": raw_files[0],
                    "time_sync": output,
                    "output_dir": output_dir,
                    "event_window_ms": event_window_ms,
                    "event_dilate_px": event_dilate_px,
                    "alpha": alpha,
                    "log_path": log_path,
                },
                daemon=True,
            )
            worker.start()
            self._send_json(dict(jobs[job_id]), HTTPStatus.ACCEPTED)

        def log_message(self, format: str, *args) -> None:
            print(f"[led-sync-gui] {self.address_string()} {format % args}")

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--record-root", default="/workspaces/record")
    parser.add_argument(
        "--html",
        default=str(Path(__file__).resolve().parent / "dist" / "index.html"),
    )
    parser.add_argument(
        "--camchain",
        default="/workspaces/ros2_ws/src/tool/multi_sensor_calibration/config/calibrations/rc_popout_default/kalibr-camchain.yaml",
        help="Default EVS/RGB Kalibr camchain used for DSEC-style overlays.",
    )
    args = parser.parse_args()

    html_path = Path(args.html).resolve()
    record_root = Path(args.record_root).resolve()
    camchain_path = Path(args.camchain).resolve()
    if not html_path.is_file():
        parser.error(f"HTML file not found: {html_path}")
    if not record_root.is_dir():
        parser.error(f"record root not found: {record_root}")
    if not (1 <= args.port <= 65535):
        parser.error("--port must be between 1 and 65535")

    server = ThreadingHTTPServer(
        (args.host, args.port),
        build_handler(html_path, record_root, camchain_path),
    )
    print(f"LED Sync Inspector: http://localhost:{args.port}")
    print(f"JSON access root: {record_root}")
    print(f"Overlay calibration: {camchain_path}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nLED Sync Inspector stopped")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
