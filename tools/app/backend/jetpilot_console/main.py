from __future__ import annotations

import argparse
import array
import fcntl
import glob
import json
import mimetypes
import os
import select
import shlex
import struct
import subprocess
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, unquote, urlparse

from .config import ConsoleConfig
from .indexes import scan_maps, scan_rosbags
from .map_detail import build_map_detail, resolve_allowed_path
from .map_pipeline import (
    build_vgl_vslam_script,
    generate_preview_script,
    generate_raceline_script,
    prepare_hd_raster_script,
    scan_camera_topic_configs,
)
from .tasks import TaskManager


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JSIOCGAXES = 0x80016A11
JSIOCGBUTTONS = 0x80016A12
JSIOCGNAME = lambda length: 0x80006A13 + (length << 16)


def read_js_device_snapshot(path_text: str = "/dev/input/js0", duration_s: float = 0.03) -> dict[str, Any]:
    path = Path(path_text)
    if not path.exists():
        devices = sorted(glob.glob("/dev/input/js*"))
        if not devices:
            return {"ok": False, "error": "no /dev/input/js* devices found", "path": path_text}
        path = Path(devices[0])

    fd = os.open(str(path), os.O_RDONLY | os.O_NONBLOCK)
    try:
        axes_count = array.array("B", [0])
        buttons_count = array.array("B", [0])
        name_buffer = array.array("B", [0] * 128)
        fcntl.ioctl(fd, JSIOCGAXES, axes_count, True)
        fcntl.ioctl(fd, JSIOCGBUTTONS, buttons_count, True)
        try:
            fcntl.ioctl(fd, JSIOCGNAME(len(name_buffer)), name_buffer, True)
            name = name_buffer.tobytes().split(b"\0", 1)[0].decode(errors="replace")
        except OSError:
            name = path.name

        axes = [0 for _ in range(int(axes_count[0]))]
        buttons = [0 for _ in range(int(buttons_count[0]))]
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            ready, _, _ = select.select([fd], [], [], max(0.0, deadline - time.monotonic()))
            if not ready:
                break
            while True:
                try:
                    data = os.read(fd, 8)
                except BlockingIOError:
                    break
                if len(data) != 8:
                    break
                _, value, event_type, number = struct.unpack("IhBB", data)
                kind = event_type & ~JS_EVENT_INIT
                if kind == JS_EVENT_AXIS and number < len(axes):
                    axes[number] = int(value)
                elif kind == JS_EVENT_BUTTON and number < len(buttons):
                    buttons[number] = int(value)

        return {
            "ok": True,
            "path": str(path),
            "name": name,
            "axes": axes,
            "axes_normalized": [max(-1.0, min(1.0, value / 32767.0)) for value in axes],
            "buttons": buttons,
        }
    except OSError as exc:
        return {"ok": False, "error": str(exc), "path": str(path)}
    finally:
        os.close(fd)


class ConsoleState:
    def __init__(self, config: ConsoleConfig):
        self.config = config
        self.tasks = TaskManager(config.state_dir, config.repo_root)


class ConsoleHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, server_address: tuple[str, int], handler_class: type[BaseHTTPRequestHandler], state: ConsoleState):
        super().__init__(server_address, handler_class)
        self.state = state


class Handler(BaseHTTPRequestHandler):
    server: ConsoleHTTPServer

    def log_message(self, format: str, *args: Any) -> None:
        print(f"[{self.log_date_time_string()}] {format % args}")

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path
        query = parse_qs(parsed.query)

        if path == "/api/health":
            self._json({"ok": True})
            return
        if path == "/api/config":
            self._json(self.server.state.config.as_json())
            return
        if path == "/api/joy/js0":
            device_path = query.get("path", ["/dev/input/js0"])[0] or "/dev/input/js0"
            self._json(read_js_device_snapshot(device_path))
            return
        if path in {"/joy-profile-editor", "/joy-profile-editor.html"}:
            self._joy_profile_editor()
            return
        if path == "/api/tasks":
            self._json({"tasks": self.server.state.tasks.list_tasks()})
            return
        if path.startswith("/api/tasks/"):
            parts = path.strip("/").split("/")
            if len(parts) == 3:
                task = self.server.state.tasks.get_task(parts[2])
                self._json({"task": task.to_json() if task else None}, HTTPStatus.OK if task else HTTPStatus.NOT_FOUND)
                return
            if len(parts) == 4 and parts[3] == "log":
                tail = int(query.get("tail", ["0"])[0] or 0)
                text = self.server.state.tasks.read_log(parts[2], tail if tail > 0 else None)
                self._text(text)
                return
            if len(parts) == 4 and parts[3] == "stream":
                tail = int(query.get("tail", ["0"])[0] or 0)
                self._stream_task(parts[2], tail if tail > 0 else None)
                return
        if path == "/api/rosbags/local":
            self._json({"rosbags": scan_rosbags(self.server.state.config.record_root)})
            return
        if path == "/api/maps/local":
            self._json({"maps": scan_maps(self.server.state.config.map_root)})
            return
        if path == "/api/maps/detail":
            self._map_detail(query)
            return
        if path == "/api/files":
            self._local_file(query)
            return
        if path == "/api/map-builder/camera-topic-configs":
            self._json({"configs": scan_camera_topic_configs(self.server.state.config)})
            return
        if path == "/api/jetson/inspect":
            self._json(self._inspect_jetson(query))
            return

        self._static(path)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path
        body = self._read_json()

        if path == "/api/tasks/run":
            command = body.get("command", "")
            if isinstance(command, str):
                command_list = ["bash", "-lc", command]
            elif isinstance(command, list) and all(isinstance(part, str) for part in command):
                command_list = command
            else:
                self._json({"error": "command must be a string or string list"}, HTTPStatus.BAD_REQUEST)
                return
            task = self.server.state.tasks.start(
                kind=str(body.get("kind", "custom")),
                title=str(body.get("title", "Custom command")),
                command=command_list,
                cwd=str(body.get("cwd") or self.server.state.config.repo_root),
            )
            self._json({"task": task.to_json()}, HTTPStatus.CREATED)
            return

        if path.startswith("/api/tasks/") and path.endswith("/stop"):
            task_id = path.strip("/").split("/")[2]
            ok = self.server.state.tasks.stop(task_id)
            self._json({"ok": ok}, HTTPStatus.OK if ok else HTTPStatus.NOT_FOUND)
            return

        if path == "/api/joy-profile/save":
            self._save_joy_profile_files(body)
            return

        if path == "/api/transfers/jetson-to-local":
            self._start_transfer(body, direction="jetson-to-local")
            return
        if path == "/api/transfers/local-to-jetson":
            self._start_transfer(body, direction="local-to-jetson")
            return
        if path == "/api/maps/build-vgl-vslam":
            self._start_map_build(body)
            return
        if path == "/api/maps/prepare-hd-raster":
            self._start_map_stage(body, "prepare-hd-raster")
            return
        if path == "/api/maps/generate-raceline":
            self._start_map_stage(body, "generate-raceline")
            return
        if path == "/api/maps/generate-preview":
            self._start_map_stage(body, "generate-preview")
            return

        self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0") or 0)
        if length <= 0:
            return {}
        data = self.rfile.read(length)
        try:
            value = json.loads(data.decode("utf-8"))
            return value if isinstance(value, dict) else {}
        except json.JSONDecodeError:
            return {}

    def _json(self, data: dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        payload = json.dumps(data, ensure_ascii=True).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def _text(self, text: str, status: HTTPStatus = HTTPStatus.OK) -> None:
        payload = text.encode("utf-8", errors="replace")
        self.send_response(status)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def _static(self, path: str) -> None:
        config = self.server.state.config
        if path == "/":
            path = "/index.html"
        relative = Path(unquote(path.lstrip("/")))
        if ".." in relative.parts:
            self._json({"error": "invalid path"}, HTTPStatus.BAD_REQUEST)
            return
        file_path = config.frontend_root / relative
        if not file_path.exists() or not file_path.is_file():
            file_path = config.frontend_root / "index.html"
        try:
            payload = file_path.read_bytes()
        except OSError:
            self._json({"error": "static file unavailable"}, HTTPStatus.NOT_FOUND)
            return
        content_type = mimetypes.guess_type(str(file_path))[0] or "application/octet-stream"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def _map_detail(self, query: dict[str, list[str]]) -> None:
        map_dir = query.get("path", [""])[0]
        if not map_dir:
            self._json({"error": "path is required"}, HTTPStatus.BAD_REQUEST)
            return
        try:
            self._json(build_map_detail(self.server.state.config, map_dir))
        except FileNotFoundError as exc:
            self._json({"error": str(exc)}, HTTPStatus.NOT_FOUND)
        except ValueError as exc:
            self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
        except Exception as exc:
            self._json({"error": f"failed to read map detail: {exc}"}, HTTPStatus.INTERNAL_SERVER_ERROR)

    def _local_file(self, query: dict[str, list[str]]) -> None:
        file_value = query.get("path", [""])[0]
        if not file_value:
            self._json({"error": "path is required"}, HTTPStatus.BAD_REQUEST)
            return
        try:
            file_path = resolve_allowed_path(self.server.state.config, file_value)
        except ValueError as exc:
            self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
            return
        if not file_path.exists() or not file_path.is_file():
            self._json({"error": "file not found"}, HTTPStatus.NOT_FOUND)
            return
        try:
            payload = file_path.read_bytes()
        except OSError:
            self._json({"error": "file unavailable"}, HTTPStatus.NOT_FOUND)
            return
        content_type = mimetypes.guess_type(str(file_path))[0] or "application/octet-stream"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def _save_joy_profile_files(self, body: dict[str, Any]) -> None:
        files = body.get("files")
        if not isinstance(files, dict):
            self._json({"error": "files must be an object"}, HTTPStatus.BAD_REQUEST)
            return
        output_root = self.server.state.config.ros2_ws / "joy_profiles"
        saved = []
        for name, content in files.items():
            if not isinstance(name, str) or not isinstance(content, str):
                self._json({"error": "file names and contents must be strings"}, HTTPStatus.BAD_REQUEST)
                return
            path = Path(name).expanduser()
            if not path.is_absolute():
                path = output_root / path
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding="utf-8")
            saved.append(str(path))
        self._json({"ok": True, "saved": saved})

    def _joy_profile_editor(self) -> None:
        config = self.server.state.config
        candidates = [
            config.ros2_ws / "src/tool/jetpilot_teleop_tools/scripts/joy_profile_editor.html",
            config.repo_root / "ros2_ws/src/tool/jetpilot_teleop_tools/scripts/joy_profile_editor.html",
            config.app_root / "frontend/joy_profile_editor.html",
        ]
        for file_path in candidates:
            if file_path.exists() and file_path.is_file():
                payload = file_path.read_bytes()
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
                return
        self._json(
            {
                "error": "joy_profile_editor.html not found",
                "searched": [str(path) for path in candidates],
            },
            HTTPStatus.NOT_FOUND,
        )

    def _stream_task(self, task_id: str, initial_tail: int | None = None) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()

        offset = 0
        idle_after_finish = 0
        sent_initial_tail = initial_tail is None
        while True:
            task = self.server.state.tasks.get_task(task_id)
            if task is None:
                self._sse({"error": "task not found"})
                return
            path = Path(task.log_path)
            chunk = ""
            if path.exists():
                if not sent_initial_tail:
                    chunk = self.server.state.tasks.read_log(task_id, initial_tail)
                    offset = path.stat().st_size
                    sent_initial_tail = True
                else:
                    with path.open("rb") as handle:
                        handle.seek(offset)
                        data = handle.read()
                        offset = handle.tell()
                    chunk = data.decode("utf-8", errors="replace")
            self._sse({"task": task.to_json(), "chunk": chunk})
            if task.status in {"success", "failed", "stopped", "lost"}:
                idle_after_finish += 1
                if idle_after_finish >= 2:
                    return
            time.sleep(1)

    def _sse(self, payload: dict[str, Any]) -> None:
        try:
            self.wfile.write(f"data: {json.dumps(payload, ensure_ascii=True)}\n\n".encode("utf-8"))
            self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            raise SystemExit

    def _inspect_jetson(self, query: dict[str, list[str]]) -> dict[str, Any]:
        config = self.server.state.config
        host = query.get("host", [config.jetson_ips[0] if config.jetson_ips else ""])[0]
        user = query.get("user", [config.jetson_user])[0]
        map_root = query.get("map_root", [config.jetson_map_root])[0]
        record_root = query.get("record_root", [config.jetson_record_root])[0]
        if not host:
            return {"ok": False, "error": "host is required"}

        remote = f"{user}@{host}"
        script = f"""
set -e
echo "[host]"
hostname || true
echo "[disk]"
df -h {shlex.quote(map_root)} {shlex.quote(record_root)} 2>/dev/null || df -h
echo "[latest]"
readlink -f {shlex.quote(map_root)}/latest 2>/dev/null || true
echo "[maps]"
find {shlex.quote(map_root)} -maxdepth 2 -mindepth 1 -type d -printf '%TY-%Tm-%Td %TH:%TM %p\\n' 2>/dev/null | sort | tail -80 || true
echo "[rosbags]"
find {shlex.quote(record_root)} -name metadata.yaml -printf '%TY-%Tm-%Td %TH:%TM %h\\n' 2>/dev/null | sort | tail -80 || true
"""
        command = [
            "ssh",
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout=5",
            remote,
            script,
        ]
        try:
            result = subprocess.run(
                command,
                cwd=self.server.state.config.repo_root,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=12,
                check=False,
            )
            return {
                "ok": result.returncode == 0,
                "host": host,
                "user": user,
                "command": command,
                "exit_code": result.returncode,
                "output": result.stdout,
            }
        except Exception as exc:
            return {"ok": False, "host": host, "user": user, "error": str(exc), "command": command}

    def _start_transfer(self, body: dict[str, Any], direction: str) -> None:
        config = self.server.state.config
        host = str(body.get("host") or (config.jetson_ips[0] if config.jetson_ips else ""))
        user = str(body.get("user") or config.jetson_user)
        if not host:
            self._json({"error": "host is required"}, HTTPStatus.BAD_REQUEST)
            return
        remote = f"{user}@{host}"

        if direction == "jetson-to-local":
            remote_path = str(body.get("remote_path") or "")
            local_path = str(body.get("local_path") or config.record_root)
            if not remote_path:
                self._json({"error": "remote_path is required"}, HTTPStatus.BAD_REQUEST)
                return
            script = (
                f"set -euo pipefail\nmkdir -p {shlex.quote(local_path)}\n"
                f"rsync -avhP --info=progress2 {shlex.quote(remote + ':' + remote_path)} "
                f"{shlex.quote(local_path.rstrip('/') + '/')}\n"
            )
            title = "Transfer Jetson to notebook"
        else:
            local_path = str(body.get("local_path") or "")
            remote_path = str(body.get("remote_path") or config.jetson_map_root)
            if not local_path:
                self._json({"error": "local_path is required"}, HTTPStatus.BAD_REQUEST)
                return
            script = (
                f"set -euo pipefail\nssh {shlex.quote(remote)} "
                f"{shlex.quote('mkdir -p ' + shlex.quote(remote_path))}\n"
                f"rsync -avhP --info=progress2 {shlex.quote(local_path.rstrip('/') + '/')} "
                f"{shlex.quote(remote + ':' + remote_path.rstrip('/') + '/')}\n"
            )
            title = "Transfer notebook to Jetson"

        task = config_task = self.server.state.tasks.start(
            kind=direction,
            title=title,
            command=["bash", "-lc", script],
            cwd=str(config.repo_root),
        )
        self._json({"task": config_task.to_json()}, HTTPStatus.CREATED)

    def _start_map_build(self, body: dict[str, Any]) -> None:
        config = self.server.state.config
        rosbag = str(body.get("rosbag") or "")
        map_dir = str(body.get("map_dir") or "")
        if not rosbag or not map_dir:
            self._json({"error": "rosbag and map_dir are required"}, HTTPStatus.BAD_REQUEST)
            return
        script = build_vgl_vslam_script(
            config=config,
            rosbag=rosbag,
            map_dir=map_dir,
            topic_config=str(body.get("topic_config") or "") or None,
            steps=str(body.get("steps") or "edex compute_poses cuvgl"),
            fs_model_res=str(body.get("fs_model_res") or "low_res"),
            output_model_dir=str(
                body.get("output_model_dir")
                or config.ros2_ws / "isaac_ros_assets/models/visual_global_localization"
            ),
            enable_rviz=bool(body.get("enable_rviz", False)),
        )
        task = self.server.state.tasks.start(
            kind="map-build",
            title="Build VGL/VSLAM map",
            command=["bash", "-lc", script],
            cwd=str(config.repo_root),
        )
        self._json({"task": task.to_json()}, HTTPStatus.CREATED)

    def _start_map_stage(self, body: dict[str, Any], stage: str) -> None:
        config = self.server.state.config
        map_dir = str(body.get("map_dir") or "")
        if not map_dir:
            self._json({"error": "map_dir is required"}, HTTPStatus.BAD_REQUEST)
            return
        if stage == "prepare-hd-raster":
            script = prepare_hd_raster_script(config, map_dir)
            title = "Prepare HD map raster"
        elif stage == "generate-raceline":
            script = generate_raceline_script(config, map_dir)
            title = "Generate raceline"
        elif stage == "generate-preview":
            script = generate_preview_script(config, map_dir)
            title = "Generate line preview"
        else:
            self._json({"error": "unknown stage"}, HTTPStatus.BAD_REQUEST)
            return
        task = self.server.state.tasks.start(
            kind=stage,
            title=title,
            command=["bash", "-lc", script],
            cwd=str(config.repo_root),
        )
        self._json({"task": task.to_json()}, HTTPStatus.CREATED)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the local JetPilot Console.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()

    config = ConsoleConfig.from_env()
    config.state_dir.mkdir(parents=True, exist_ok=True)
    state = ConsoleState(config)
    server = ConsoleHTTPServer((args.host, args.port), Handler, state)
    print(f"JetPilot Console: http://{args.host}:{args.port}")
    print(f"State directory : {config.state_dir}")
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
