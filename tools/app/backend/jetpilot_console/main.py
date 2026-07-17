from __future__ import annotations

import argparse
import array
import fcntl
import glob
import json
import mimetypes
import os
import re
import select
import shlex
import socket
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
from .map_detail import build_map_detail, resolve_allowed_path, save_hd_map, save_section_gates
from .map_pipeline import (
    DEFAULT_RACELINE_SAFETY_MARGIN_M,
    DEFAULT_RACELINE_VEHICLE_WIDTH_M,
    build_vgl_vslam_script,
    default_topic_config,
    generate_preview_script,
    generate_raceline_script,
    localization_config_dir,
    prepare_hd_raster_script,
    scan_camera_topic_configs,
)
from .security import (
    RequestRejected,
    decode_json_object,
    is_loopback_bind,
    resolve_under_root,
    save_joy_profile_files,
    validate_remote_absolute_path,
    validate_request_host,
    validate_ssh_target,
    validate_json_request_headers,
)
from .tasks import TaskManager


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JSIOCGAXES = 0x80016A11
JSIOCGBUTTONS = 0x80016A12
JSIOCGNAME = lambda length: 0x80006A13 + (length << 16)
_MAP_BUILD_TOKEN = re.compile(r"^[A-Za-z0-9_][A-Za-z0-9_.-]{0,63}$")


def local_ip_candidates() -> list[str]:
    candidates: set[str] = set()

    try:
        hostname = socket.gethostname()
        for family, _, _, _, sockaddr in socket.getaddrinfo(hostname, None, socket.AF_INET):
            if family == socket.AF_INET:
                address = sockaddr[0]
                if not address.startswith("127."):
                    candidates.add(address)
    except OSError:
        pass

    for target in ("10.42.0.1", "192.168.55.1", "8.8.8.8"):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.connect((target, 9))
            address = sock.getsockname()[0]
            if not address.startswith("127."):
                candidates.add(address)
        except OSError:
            pass
        finally:
            sock.close()

    try:
        output = subprocess.run(
            ["ifconfig"],
            check=False,
            capture_output=True,
            text=True,
            timeout=1.0,
        ).stdout
        for address in re.findall(r"\binet\s+(\d+\.\d+\.\d+\.\d+)\b", output):
            if not address.startswith("127."):
                candidates.add(address)
    except (OSError, subprocess.SubprocessError):
        pass

    return sorted(candidates)


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
    def __init__(
        self,
        config: ConsoleConfig,
        *,
        joy_only: bool = False,
        loopback_only: bool = True,
    ):
        self.config = config
        self.joy_only = joy_only
        self.loopback_only = loopback_only
        self.tasks = None if joy_only else TaskManager(config.state_dir, config.repo_root)


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
        if not self._request_host_allowed():
            return
        parsed = urlparse(self.path)
        path = parsed.path
        query = parse_qs(parsed.query)

        if path == "/api/health":
            self._json({"ok": True, "mode": "joy-only" if self.server.state.joy_only else "console"})
            return
        if path == "/api/joy/js0":
            device_path = query.get("path", ["/dev/input/js0"])[0] or "/dev/input/js0"
            self._json(read_js_device_snapshot(device_path))
            return
        if path in {"/joy-profile-editor", "/joy-profile-editor.html"} or (
            self.server.state.joy_only and path == "/"
        ):
            self._joy_profile_editor()
            return
        if self.server.state.joy_only:
            self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            return
        if path == "/api/config":
            self._json(self.server.state.config.as_json())
            return
        if path == "/api/network/local-ips":
            self._json({"ips": local_ip_candidates()})
            return
        if path == "/api/tasks":
            self._json({"tasks": self.server.state.tasks.list_tasks()})  # type: ignore[union-attr]
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
        if path.startswith("/api/"):
            self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            return
        self._static(path)

    def do_POST(self) -> None:
        if not self._request_host_allowed():
            return
        parsed = urlparse(self.path)
        path = parsed.path
        try:
            body = self._read_json()
        except RequestRejected as exc:
            self._json({"error": exc.message}, HTTPStatus(exc.status))
            return

        if path == "/api/joy-profile/save":
            self._save_joy_profile_files(body)
            return

        if self.server.state.joy_only:
            self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            return

        if path == "/api/tasks/run":
            if (
                not self.server.state.config.enable_custom_commands
                or not self.server.state.loopback_only
            ):
                self._json(
                    {
                        "error": (
                            "custom command execution is disabled; set "
                            "JETPILOT_CONSOLE_ENABLE_CUSTOM_COMMANDS=true only in a trusted local environment"
                        )
                    },
                    HTTPStatus.FORBIDDEN,
                )
                return
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

        if path == "/api/transfers/jetson-to-local":
            self._start_transfer(body, direction="jetson-to-local")
            return
        if path == "/api/transfers/local-to-jetson":
            self._start_transfer(body, direction="local-to-jetson")
            return
        if path == "/api/jetson/inspect":
            self._json(self._inspect_jetson(body))
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
        if path == "/api/maps/save-hd-map":
            self._save_hd_map(body)
            return
        if path == "/api/maps/save-section-gates":
            self._save_section_gates(body)
            return

        self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)

    def _read_json(self) -> dict[str, Any]:
        length = validate_json_request_headers(
            content_type=self.headers.get("Content-Type"),
            content_length=self.headers.get("Content-Length"),
            transfer_encoding=self.headers.get("Transfer-Encoding"),
            host=self.headers.get("Host"),
            origin=self.headers.get("Origin"),
        )
        data = self.rfile.read(length)
        if len(data) != length:
            raise RequestRejected(400, "request body ended before Content-Length bytes were received")
        return decode_json_object(data)

    def _request_host_allowed(self) -> bool:
        try:
            validate_request_host(
                self.headers.get("Host"),
                loopback_only=self.server.state.loopback_only,
            )
            return True
        except RequestRejected as exc:
            self._json({"error": exc.message}, HTTPStatus(exc.status))
            return False

    def _json(self, data: dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        payload = json.dumps(data, ensure_ascii=True).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def end_headers(self) -> None:
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "SAMEORIGIN")
        self.send_header("Content-Security-Policy", "frame-ancestors 'self'")
        self.send_header("Referrer-Policy", "no-referrer")
        super().end_headers()

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
        decoded = unquote(path)
        relative = Path(decoded[1:] if decoded.startswith("/") else decoded)
        if "\0" in decoded or relative.is_absolute() or ".." in relative.parts:
            self._json({"error": "invalid path"}, HTTPStatus.BAD_REQUEST)
            return
        try:
            file_path = resolve_under_root(relative, config.frontend_root, label="static path")
        except ValueError:
            self._json({"error": "invalid path"}, HTTPStatus.BAD_REQUEST)
            return
        if not file_path.exists() or not file_path.is_file():
            try:
                file_path = resolve_under_root("index.html", config.frontend_root, label="static path")
            except ValueError:
                self._json({"error": "static file unavailable"}, HTTPStatus.NOT_FOUND)
                return
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

    def _save_hd_map(self, body: dict[str, Any]) -> None:
        try:
            self._json(save_hd_map(self.server.state.config, body))
        except FileNotFoundError as exc:
            self._json({"error": str(exc)}, HTTPStatus.NOT_FOUND)
        except ValueError as exc:
            self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
        except Exception as exc:
            self._json({"error": f"failed to save HD map: {exc}"}, HTTPStatus.INTERNAL_SERVER_ERROR)

    def _save_section_gates(self, body: dict[str, Any]) -> None:
        try:
            self._json(save_section_gates(self.server.state.config, body))
        except FileNotFoundError as exc:
            self._json({"error": str(exc)}, HTTPStatus.NOT_FOUND)
        except ValueError as exc:
            self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
        except Exception as exc:
            self._json({"error": f"failed to save section gates: {exc}"}, HTTPStatus.INTERNAL_SERVER_ERROR)

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
        output_root = self.server.state.config.ros2_ws / "joy_profiles"
        try:
            saved = save_joy_profile_files(output_root, body.get("files"))
        except (OSError, ValueError) as exc:
            self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
            return
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

    def _inspect_jetson(self, body: dict[str, Any]) -> dict[str, Any]:
        config = self.server.state.config
        host = str(body.get("host") or (config.jetson_ips[0] if config.jetson_ips else ""))
        user = str(body.get("user") or config.jetson_user)
        map_root = str(body.get("map_root") or config.jetson_map_root)
        record_root = str(body.get("record_root") or config.jetson_record_root)
        if not host:
            return {"ok": False, "error": "host is required"}

        try:
            remote = validate_ssh_target(user, host)
            map_root = validate_remote_absolute_path(map_root, label="remote map root")
            record_root = validate_remote_absolute_path(record_root, label="remote rosbag root")
        except ValueError as exc:
            return {"ok": False, "host": host, "user": user, "error": str(exc)}
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
            error = None
            if result.returncode != 0:
                output = result.stdout.strip()
                if "Permission denied" in output or "BatchMode" in output:
                    error = (
                        "SSH password prompts are not supported by Inspect Jetson. "
                        "Set up SSH key authentication or an ssh-agent for this host, "
                        f"then retry: ssh {remote}"
                    )
                elif output:
                    error = output.splitlines()[-1]
                else:
                    error = f"ssh exited with code {result.returncode}"
            return {
                "ok": result.returncode == 0,
                "host": host,
                "user": user,
                "command": command,
                "exit_code": result.returncode,
                "output": result.stdout,
                "error": error,
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
        try:
            remote = validate_ssh_target(user, host)
        except ValueError as exc:
            self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
            return

        if direction == "jetson-to-local":
            remote_path = str(body.get("remote_path") or "")
            local_path = str(body.get("local_path") or config.record_root)
            if not remote_path:
                self._json({"error": "remote_path is required"}, HTTPStatus.BAD_REQUEST)
                return
            try:
                remote_path = validate_remote_absolute_path(
                    remote_path,
                    label="remote rosbag source",
                )
                local_path = str(
                    resolve_under_root(
                        local_path,
                        config.record_root,
                        label="local rosbag destination",
                        require_directory=True,
                    )
                )
            except ValueError as exc:
                self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
                return
            script = (
                f"set -euo pipefail\nmkdir -p {shlex.quote(local_path)}\n"
                f"rsync -avhP --protect-args {shlex.quote(remote + ':' + remote_path)} "
                f"{shlex.quote(local_path.rstrip('/') + '/')}\n"
            )
            title = "Transfer Jetson to notebook"
        else:
            local_path = str(body.get("local_path") or "")
            remote_path = str(body.get("remote_path") or config.jetson_map_root)
            if not local_path:
                self._json({"error": "local_path is required"}, HTTPStatus.BAD_REQUEST)
                return
            try:
                remote_path = validate_remote_absolute_path(
                    remote_path,
                    label="remote map destination",
                )
                local_path = str(
                    resolve_under_root(
                        local_path,
                        config.map_root,
                        label="local map source",
                        require_exists=True,
                        require_directory=True,
                    )
                )
            except ValueError as exc:
                self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
                return
            script = (
                f"set -euo pipefail\nssh {shlex.quote(remote)} "
                f"{shlex.quote('mkdir -p ' + shlex.quote(remote_path))}\n"
                f"rsync -avhP --protect-args {shlex.quote(local_path.rstrip('/') + '/')} "
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
        try:
            steps = str(body.get("steps") or "edex compute_poses cuvgl").split()
            if not steps or len(steps) > 16 or any(
                not _MAP_BUILD_TOKEN.fullmatch(step) for step in steps
            ):
                raise ValueError("mapping steps contain unsupported values")
            fs_model_res = str(body.get("fs_model_res") or "low_res").strip()
            if not _MAP_BUILD_TOKEN.fullmatch(fs_model_res):
                raise ValueError("fs_model_res contains unsupported characters")

            rosbag = str(
                resolve_under_root(
                    rosbag,
                    config.record_root,
                    label="rosbag",
                    require_exists=True,
                    require_directory=True,
                )
            )
            map_dir = str(
                resolve_under_root(
                    map_dir,
                    config.map_root,
                    label="map output directory",
                    require_directory=True,
                )
            )
            topic_config = resolve_under_root(
                str(body.get("topic_config") or default_topic_config(config)),
                localization_config_dir(config),
                label="camera topic config",
                require_exists=True,
            )
            if not topic_config.is_file():
                raise ValueError("camera topic config must be a file")
            model_root = config.ros2_ws / "isaac_ros_assets" / "models"
            output_model_dir = resolve_under_root(
                str(
                    body.get("output_model_dir")
                    or model_root / "visual_global_localization"
                ),
                model_root,
                label="VGL model directory",
                require_exists=True,
                require_directory=True,
            )
        except ValueError as exc:
            self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
            return
        script = build_vgl_vslam_script(
            config=config,
            rosbag=rosbag,
            map_dir=map_dir,
            topic_config=str(topic_config),
            steps=" ".join(steps),
            fs_model_res=fs_model_res,
            output_model_dir=str(output_model_dir),
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
        try:
            map_dir = str(
                resolve_under_root(
                    map_dir,
                    config.map_root,
                    label="map directory",
                    require_exists=True,
                    require_directory=True,
                )
            )
        except ValueError as exc:
            self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
            return
        if stage == "prepare-hd-raster":
            script = prepare_hd_raster_script(config, map_dir)
            title = "Prepare HD map raster"
        elif stage == "generate-raceline":
            vehicle_width_m = body.get(
                "vehicle_width_m",
                DEFAULT_RACELINE_VEHICLE_WIDTH_M,
            )
            safety_margin_m = body.get(
                "safety_margin_m",
                DEFAULT_RACELINE_SAFETY_MARGIN_M,
            )
            if vehicle_width_m is None:
                vehicle_width_m = DEFAULT_RACELINE_VEHICLE_WIDTH_M
            if safety_margin_m is None:
                safety_margin_m = DEFAULT_RACELINE_SAFETY_MARGIN_M
            try:
                script = generate_raceline_script(
                    config,
                    map_dir,
                    vehicle_width_m=vehicle_width_m,
                    safety_margin_m=safety_margin_m,
                )
            except (TypeError, ValueError) as exc:
                self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
                return
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
    parser.add_argument(
        "--allow-remote",
        action="store_true",
        help="allow a non-loopback bind (the Console has no user authentication)",
    )
    parser.add_argument(
        "--joy-only",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    args = parser.parse_args()

    if not is_loopback_bind(args.host) and not args.allow_remote:
        parser.error(
            "non-loopback --host requires --allow-remote; use 127.0.0.1 with Docker host networking"
        )

    config = ConsoleConfig.from_env()
    if (
        config.enable_custom_commands
        and not is_loopback_bind(args.host)
        and not args.joy_only
    ):
        parser.error(
            "custom command execution cannot be combined with a non-loopback bind"
        )
    config.state_dir.mkdir(parents=True, exist_ok=True)
    state = ConsoleState(
        config,
        joy_only=args.joy_only,
        loopback_only=is_loopback_bind(args.host),
    )
    server = ConsoleHTTPServer((args.host, args.port), Handler, state)
    if args.joy_only:
        print(f"Joy Profile Editor: http://{args.host}:{args.port}/joy-profile-editor")
    else:
        print(f"JetPilot Console: http://{args.host}:{args.port}")
        print(f"State directory : {config.state_dir}")
    if args.allow_remote and not is_loopback_bind(args.host):
        print("WARNING: remote access is enabled and this server has no user authentication")
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
