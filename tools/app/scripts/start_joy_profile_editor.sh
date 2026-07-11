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
import sys
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse


HOST = sys.argv[1]
PORT = int(sys.argv[2])


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
        self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)

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
