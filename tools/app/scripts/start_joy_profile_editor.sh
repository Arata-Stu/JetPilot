#!/usr/bin/env bash
set -euo pipefail

HOST="0.0.0.0"
PORT="8766"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host)
      HOST="$2"
      shift 2
      ;;
    --port)
      PORT="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
APP_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd -- "${APP_ROOT}/../.." && pwd)"

export JETPILOT_REPO_ROOT="${JETPILOT_REPO_ROOT:-$REPO_ROOT}"
export ROS2_WS="${ROS2_WS:-${REPO_ROOT}/ros2_ws}"

exec python3 - "$HOST" "$PORT" <<'PY'
from __future__ import annotations

import json
import os
import array
import fcntl
import glob
import select
import struct
import time
import sys
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse


HOST = sys.argv[1]
PORT = int(sys.argv[2])

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JSIOCGAXES = 0x80016A11
JSIOCGBUTTONS = 0x80016A12
JSIOCGNAME = lambda length: 0x80006A13 + (length << 16)


def candidate_paths() -> list[Path]:
    repo_root = Path(os.environ.get("JETPILOT_REPO_ROOT", ".")).expanduser().resolve()
    ros2_ws = Path(os.environ.get("ROS2_WS", repo_root / "ros2_ws")).expanduser().resolve()
    return [
        ros2_ws / "src/tool/jetpilot_teleop_tools/scripts/joy_profile_editor.html",
        repo_root / "ros2_ws/src/tool/jetpilot_teleop_tools/scripts/joy_profile_editor.html",
        Path("/workspaces/ros2_ws/src/tool/jetpilot_teleop_tools/scripts/joy_profile_editor.html"),
    ]


def editor_path() -> Path | None:
    for path in candidate_paths():
        if path.exists() and path.is_file():
            return path
    return None


def default_output_dir() -> Path:
    ros2_ws = Path(os.environ.get("ROS2_WS", "/workspaces/ros2_ws")).expanduser()
    return ros2_ws / "joy_profiles"


def resolve_output_path(path_text: str) -> Path:
    path = Path(path_text).expanduser()
    if not path.is_absolute():
        path = default_output_dir() / path
    return path


def read_js_device_snapshot(path_text: str = "/dev/input/js0", duration_s: float = 0.03) -> dict[str, object]:
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


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt: str, *args: object) -> None:
        print(f"[{self.log_date_time_string()}] {fmt % args}")

    def do_GET(self) -> None:
        path = unquote(urlparse(self.path).path)
        if path in {"/", "/joy-profile-editor", "/joy-profile-editor.html"}:
            self._editor()
            return
        if path == "/api/health":
            self._json({"ok": True})
            return
        if path == "/api/joy/js0":
            self._json(read_js_device_snapshot())
            return
        self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        path = unquote(urlparse(self.path).path)
        body = self._read_json()
        if path == "/api/joy-profile/save":
            self._save_files(body)
            return
        self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)

    def _read_json(self) -> dict[str, object]:
        length = int(self.headers.get("Content-Length", "0") or 0)
        if length <= 0:
            return {}
        try:
            value = json.loads(self.rfile.read(length).decode("utf-8"))
        except json.JSONDecodeError:
            return {}
        return value if isinstance(value, dict) else {}

    def _save_files(self, body: dict[str, object]) -> None:
        files = body.get("files")
        if not isinstance(files, dict):
            self._json({"error": "files must be an object"}, HTTPStatus.BAD_REQUEST)
            return
        saved = []
        for name, content in files.items():
            if not isinstance(name, str) or not isinstance(content, str):
                self._json({"error": "file names and contents must be strings"}, HTTPStatus.BAD_REQUEST)
                return
            path = resolve_output_path(name)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding="utf-8")
            saved.append(str(path))
        self._json({"ok": True, "saved": saved})

    def _editor(self) -> None:
        path = editor_path()
        if path is None:
            self._json(
                {
                    "error": "joy_profile_editor.html not found",
                    "searched": [str(item) for item in candidate_paths()],
                },
                HTTPStatus.NOT_FOUND,
            )
            return
        payload = path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def _json(self, data: dict[str, object], status: HTTPStatus = HTTPStatus.OK) -> None:
        payload = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)


server = ThreadingHTTPServer((HOST, PORT), Handler)
server.daemon_threads = True
print(f"Joy Profile Editor: http://{HOST}:{PORT}/joy-profile-editor")
server.serve_forever()
PY
