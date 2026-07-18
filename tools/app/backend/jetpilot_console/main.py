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

from .bag_analysis import AnalysisRepository, build_analysis_script, rosbag_detail
from .config import ConsoleConfig
from .fpv_stream import FpvStreamManager, FpvStreamSettings
from .indexes import scan_maps, scan_rosbags
from .map_detail import (
    build_map_detail,
    directory_fingerprint,
    resolve_allowed_path,
    save_hd_map,
    save_section_gates,
)
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
from .preflight import evaluate_preflight
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
from .tasks import TaskManager, TaskResourceConflict


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
        self.analyses = None if joy_only else AnalysisRepository(config.analysis_root)
        self.fpv_stream = None if joy_only else FpvStreamManager()


class ConsoleHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, server_address: tuple[str, int], handler_class: type[BaseHTTPRequestHandler], state: ConsoleState):
        super().__init__(server_address, handler_class)
        self.state = state


class Handler(BaseHTTPRequestHandler):
    server: ConsoleHTTPServer

    def log_message(self, format: str, *args: Any) -> None:
        if getattr(self, "path", "") in {"/api/fpv/status", "/api/fpv/heartbeat"}:
            return
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
        if path == "/api/fpv/status":
            self._json({"fpv": self.server.state.fpv_stream.status()})  # type: ignore[union-attr]
            return
        if path == "/api/fpv/stream":
            self._fpv_stream(query)
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
        if path == "/api/rosbags/detail":
            self._rosbag_detail(query)
            return
        if path == "/api/analyses":
            self._json({"analyses": self.server.state.analyses.list()})
            return
        if path.startswith("/api/analyses/"):
            self._analysis_get(path)
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

        if path == "/api/preflight":
            action = body.get("action")
            if not isinstance(action, str) or not action.strip():
                self._json({"error": "action must be a non-empty string"}, HTTPStatus.BAD_REQUEST)
                return
            try:
                preflight = evaluate_preflight(self.server.state.config, action, body)
            except (TypeError, ValueError) as exc:
                self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
                return
            self._json(preflight)
            return

        if path == "/api/fpv/start":
            try:
                settings = FpvStreamSettings.from_mapping(body)
                status = self.server.state.fpv_stream.start(settings)  # type: ignore[union-attr]
            except (RuntimeError, ValueError) as exc:
                self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
                return
            self._json({"fpv": status}, HTTPStatus.CREATED)
            return

        if path == "/api/fpv/heartbeat":
            session_id = str(body.get("session_id") or "")
            ok = self.server.state.fpv_stream.heartbeat(session_id)  # type: ignore[union-attr]
            self._json({"ok": ok}, HTTPStatus.OK if ok else HTTPStatus.CONFLICT)
            return

        if path == "/api/fpv/stop":
            session_id = str(body.get("session_id") or "")
            if not session_id:
                self._json({"error": "session_id is required"}, HTTPStatus.BAD_REQUEST)
                return
            stopped = self.server.state.fpv_stream.stop(session_id)  # type: ignore[union-attr]
            self._json(
                {"ok": True, "stopped": stopped, "fpv": self.server.state.fpv_stream.status()},  # type: ignore[union-attr]
            )
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
        if path in {"/api/analyses", "/api/analyses/start"}:
            self._start_analysis(body)
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

    def _rosbag_detail(self, query: dict[str, list[str]]) -> None:
        path = query.get("path", [""])[0]
        if not path:
            self._json({"error": "path is required"}, HTTPStatus.BAD_REQUEST)
            return
        try:
            self._json({"rosbag": rosbag_detail(self.server.state.config, path)})
        except FileNotFoundError as exc:
            self._json({"error": str(exc)}, HTTPStatus.NOT_FOUND)
        except (OSError, ValueError) as exc:
            self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)

    def _analysis_get(self, path: str) -> None:
        parts = path.strip("/").split("/")
        if len(parts) < 3 or parts[:2] != ["api", "analyses"]:
            self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            return
        analysis_id = unquote(parts[2])
        try:
            if len(parts) == 3:
                self._json({"analysis": self.server.state.analyses.detail(analysis_id)})
                return
            if len(parts) == 4 and parts[3] == "timeline":
                self._json(self.server.state.analyses.timeline(analysis_id))
                return
            if len(parts) >= 5 and parts[3] == "frames":
                relative_path = unquote("/".join(parts[4:]))
                self._analysis_frame(analysis_id, relative_path)
                return
            self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)
        except FileNotFoundError as exc:
            self._json({"error": str(exc)}, HTTPStatus.NOT_FOUND)
        except (OSError, ValueError) as exc:
            self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)

    def _analysis_frame(self, analysis_id: str, relative_path: str) -> None:
        frame_path, content_type = self.server.state.analyses.frame(
            analysis_id,
            relative_path,
        )
        payload = frame_path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "private, max-age=31536000, immutable")
        self.end_headers()
        self.wfile.write(payload)

    def _fpv_stream(self, query: dict[str, list[str]]) -> None:
        manager = self.server.state.fpv_stream
        if manager is None:
            self._json({"error": "FPV receiver is unavailable"}, HTTPStatus.NOT_FOUND)
            return
        requested_session = str(query.get("session", [""])[0] or "")
        session_id = requested_session or manager.current_session_id()
        if not session_id or not manager.session_is_running(session_id):
            self._json({"error": "FPV receiver is not running"}, HTTPStatus.CONFLICT)
            return

        boundary = b"jetpilot-frame"
        self.send_response(HTTPStatus.OK)
        self.send_header(
            "Content-Type",
            f"multipart/x-mixed-replace; boundary={boundary.decode('ascii')}",
        )
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.end_headers()

        sequence = 0
        try:
            while manager.session_is_running(session_id):
                item = manager.wait_for_frame(session_id, sequence, timeout=2.0)
                if item is None:
                    continue
                sequence, frame = item
                self.wfile.write(b"--" + boundary + b"\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError, TimeoutError):
            return

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
            map_dir = resolve_allowed_path(
                self.server.state.config, str(body.get("map_dir") or "")
            )
            with self.server.state.tasks.guard_resources([f"map-dir:{map_dir}"]):
                result = save_hd_map(self.server.state.config, body)
            self._json(result)
        except TaskResourceConflict as exc:
            self._json(
                {
                    "error": "The Map is in use by an analysis or Map task. Stop it or wait for it to finish.",
                    "active_task": exc.active_task,
                },
                HTTPStatus.CONFLICT,
            )
        except FileNotFoundError as exc:
            self._json({"error": str(exc)}, HTTPStatus.NOT_FOUND)
        except ValueError as exc:
            self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
        except Exception as exc:
            self._json({"error": f"failed to save HD map: {exc}"}, HTTPStatus.INTERNAL_SERVER_ERROR)

    def _save_section_gates(self, body: dict[str, Any]) -> None:
        try:
            map_dir = resolve_allowed_path(
                self.server.state.config, str(body.get("map_dir") or "")
            )
            with self.server.state.tasks.guard_resources([f"map-dir:{map_dir}"]):
                result = save_section_gates(self.server.state.config, body)
            self._json(result)
        except TaskResourceConflict as exc:
            self._json(
                {
                    "error": "The Map is in use by an analysis or Map task. Stop it or wait for it to finish.",
                    "active_task": exc.active_task,
                },
                HTTPStatus.CONFLICT,
            )
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

    def _start_analysis(self, body: dict[str, Any]) -> None:
        config = self.server.state.config
        try:
            preflight = evaluate_preflight(config, "analyze-rosbag", body)
        except (TypeError, ValueError) as exc:
            self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
            return
        if not preflight.get("ready"):
            self._json(
                {
                    "error": "required preflight checks did not pass",
                    "preflight": preflight,
                },
                HTTPStatus.CONFLICT,
            )
            return

        resolved = preflight.get("resolved")
        if not isinstance(resolved, dict):
            self._json({"error": "preflight did not resolve analysis inputs"}, HTTPStatus.CONFLICT)
            return
        try:
            rosbag = Path(str(resolved["rosbag"]))
            image_topic = str(resolved["image_topic"])
            trajectory_mode = str(resolved["trajectory_mode"])
            offline_localization_mode = str(
                resolved.get("offline_localization_mode") or "auto"
            )
            max_fps = float(resolved["max_fps"])
            map_dir = Path(str(resolved["map_dir"])) if resolved.get("map_dir") else None
            expected_map_fingerprint = directory_fingerprint(map_dir) if map_dir else ""
            topic_config = (
                Path(str(resolved["topic_config"])) if resolved.get("topic_config") else None
            )
            model_dir = (
                Path(str(resolved["output_model_dir"]))
                if resolved.get("output_model_dir")
                else None
            )
        except (KeyError, TypeError, ValueError) as exc:
            self._json({"error": f"preflight result is incomplete: {exc}"}, HTTPStatus.CONFLICT)
            return

        label = str(body.get("label") or rosbag.name).strip()[:120] or rosbag.name
        accepted_fields = (
            "rosbag",
            "map_dir",
            "image_topic",
            "control_topic",
            "mode_topic",
            "pose_topic",
            "speed_topic",
            "trajectory_mode",
            "offline_localization_mode",
            "max_fps",
            "topic_config",
        )
        stored_request = {key: body.get(key) for key in accepted_fields if key in body}
        stored_request["label"] = label
        initial_phase = (
            "offline-localization" if trajectory_mode == "offline" else "extracting"
        )

        analysis_id = ""
        try:
            analysis = self.server.state.analyses.create(
                label=label,
                request=stored_request,
                preflight=preflight,
                initial_phase=initial_phase,
            )
            analysis_id = str(analysis["analysis_id"])
            analysis_dir = Path(str(analysis["path"]))
            script = build_analysis_script(
                config,
                analysis_dir=analysis_dir,
                rosbag=rosbag,
                image_topic=image_topic,
                control_topic=str(resolved.get("control_topic") or ""),
                mode_topic=str(resolved.get("mode_topic") or ""),
                pose_topic=str(resolved.get("pose_topic") or ""),
                speed_topic=str(resolved.get("speed_topic") or ""),
                map_dir=map_dir,
                trajectory_mode=trajectory_mode,
                offline_localization_mode=offline_localization_mode,
                max_fps=max_fps,
                expected_map_fingerprint=expected_map_fingerprint,
                topic_config=topic_config,
                model_dir=model_dir,
            )
        except (OSError, TypeError, ValueError) as exc:
            if analysis_id:
                try:
                    self.server.state.analyses.update_status(
                        analysis_id,
                        status="failed",
                        phase="failed",
                        progress=0.0,
                        message=f"Could not prepare analysis: {exc}",
                    )
                except (OSError, ValueError):
                    pass
            self._json({"error": f"could not prepare analysis: {exc}"}, HTTPStatus.BAD_REQUEST)
            return

        artifacts = [
            {"name": "manifest", "path": str(analysis_dir / "manifest.json")},
            {"name": "status", "path": str(analysis_dir / "status.json")},
            {"name": "timeline", "path": str(analysis_dir / "timeline.json")},
            {"name": "frames", "path": str(analysis_dir / "frames")},
        ]
        resource_key = (
            f"analysis-ros-domain:{config.analysis_ros_domain_id}"
            if trajectory_mode == "offline"
            else f"analysis-bag:{rosbag}"
        )
        resource_keys = [resource_key]
        if map_dir is not None:
            resource_keys.append(f"map-dir:{map_dir}")
        try:
            task = self.server.state.tasks.start(
                kind="analyze-rosbag",
                title=f"Analyze rosbag: {label}",
                command=["bash", "-lc", script],
                cwd=str(config.repo_root),
                artifacts=artifacts,
                resource_key=resource_key,
                resource_keys=resource_keys,
            )
        except TaskResourceConflict as exc:
            active = exc.active_task
            if exc.resource_key.startswith("map-dir:"):
                conflict_message = (
                    "The selected map folder is already in use: "
                    f"{active.get('title') or active.get('kind')} "
                    f"({active.get('task_id')}). Stop it or wait for it to finish."
                )
            elif trajectory_mode == "offline":
                conflict_message = (
                    "Another offline localization analysis is using the isolated ROS domain: "
                    f"{active.get('title') or active.get('kind')} "
                    f"({active.get('task_id')}). Stop it or wait for it to finish."
                )
            else:
                conflict_message = (
                    "This rosbag is already being analyzed: "
                    f"{active.get('title') or active.get('kind')} "
                    f"({active.get('task_id')}). Stop it or wait for it to finish."
                )
            try:
                analysis = self.server.state.analyses.update_status(
                    analysis_id,
                    status="failed",
                    phase="blocked",
                    progress=0.0,
                    message=conflict_message,
                )
            except (OSError, ValueError):
                pass
            self._json(
                {
                    "error": conflict_message,
                    "active_task": active,
                    "analysis": analysis,
                    "preflight": preflight,
                },
                HTTPStatus.CONFLICT,
            )
            return
        except Exception as exc:  # noqa: BLE001 - convert task startup failures into API state.
            message = f"Could not start analysis task: {exc}"
            try:
                analysis = self.server.state.analyses.update_status(
                    analysis_id,
                    status="failed",
                    phase="failed",
                    progress=0.0,
                    message=message,
                )
            except (OSError, ValueError):
                pass
            self._json(
                {"error": message, "analysis": analysis, "preflight": preflight},
                HTTPStatus.INTERNAL_SERVER_ERROR,
            )
            return

        analysis = self.server.state.analyses.attach_task(analysis_id, task.to_json())
        self._json(
            {"analysis": analysis, "task": task.to_json(), "preflight": preflight},
            HTTPStatus.CREATED,
        )

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
        try:
            preflight = evaluate_preflight(config, "map-build", body)
        except (TypeError, ValueError) as exc:
            self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
            return
        if not preflight.get("ready"):
            self._json(
                {
                    "error": "required preflight checks did not pass",
                    "preflight": preflight,
                },
                HTTPStatus.CONFLICT,
            )
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
        try:
            task = self.server.state.tasks.start(
                kind="map-build",
                title="Build VGL/VSLAM map",
                command=["bash", "-lc", script],
                cwd=str(config.repo_root),
                resource_key=f"map-dir:{map_dir}",
            )
        except TaskResourceConflict as exc:
            self._map_task_conflict(exc)
            return
        self._json({"task": task.to_json(), "preflight": preflight}, HTTPStatus.CREATED)

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
        try:
            preflight = evaluate_preflight(config, stage, body)
        except (TypeError, ValueError) as exc:
            self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
            return
        if not preflight.get("ready"):
            self._json(
                {
                    "error": "required preflight checks did not pass",
                    "preflight": preflight,
                },
                HTTPStatus.CONFLICT,
            )
            return
        try:
            task = self.server.state.tasks.start(
                kind=stage,
                title=title,
                command=["bash", "-lc", script],
                cwd=str(config.repo_root),
                resource_key=f"map-dir:{map_dir}",
            )
        except TaskResourceConflict as exc:
            self._map_task_conflict(exc)
            return
        self._json({"task": task.to_json(), "preflight": preflight}, HTTPStatus.CREATED)

    def _map_task_conflict(self, conflict: TaskResourceConflict) -> None:
        active = conflict.active_task
        self._json(
            {
                "error": (
                    "Another task is already writing to or using this map folder: "
                    f"{active.get('title') or active.get('kind')} "
                    f"({active.get('task_id')}). Stop it or wait for it to finish."
                ),
                "active_task": active,
            },
            HTTPStatus.CONFLICT,
        )


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
    try:
        server.serve_forever()
    finally:
        if state.fpv_stream is not None:
            state.fpv_stream.stop()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
