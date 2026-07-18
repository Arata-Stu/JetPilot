from __future__ import annotations

import os
import shutil
import signal
import subprocess
import threading
import time
from dataclasses import asdict, dataclass
from typing import Any, Mapping


SUPPORTED_CODECS = frozenset({"h264", "h265", "mjpeg", "raw"})
WEB_MAX_WIDTH = 1280
WEB_MAX_HEIGHT = 720
WEB_MAX_FPS = 30
FPV_LEASE_TIMEOUT_S = 8.0
_MAX_JPEG_BUFFER_BYTES = 32 * 1024 * 1024
_MAX_STDERR_BYTES = 64 * 1024


def _trim_jpeg_buffer(buffer: bytearray, maximum_bytes: int = _MAX_JPEG_BUFFER_BYTES) -> None:
    if len(buffer) <= maximum_bytes:
        return
    latest_start = buffer.rfind(b"\xff\xd8")
    if latest_start > 0:
        del buffer[:latest_start]
    if len(buffer) > maximum_bytes:
        possible_prefix = b"\xff" if buffer.endswith(b"\xff") else b""
        buffer.clear()
        buffer.extend(possible_prefix)


def _bounded_int(
    value: object,
    *,
    field: str,
    default: int,
    minimum: int,
    maximum: int,
) -> int:
    if value is None or value == "":
        return default
    if isinstance(value, bool):
        raise ValueError(f"{field} must be an integer")
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        raise ValueError(f"{field} must be an integer") from None
    if str(parsed) != str(value).strip() and not isinstance(value, int):
        raise ValueError(f"{field} must be an integer")
    if parsed < minimum or parsed > maximum:
        raise ValueError(f"{field} must be between {minimum} and {maximum}")
    return parsed


def scale_to_fit(
    width: int,
    height: int,
    *,
    maximum_width: int = WEB_MAX_WIDTH,
    maximum_height: int = WEB_MAX_HEIGHT,
) -> tuple[int, int]:
    scale = min(1.0, maximum_width / width, maximum_height / height)
    return max(1, round(width * scale)), max(1, round(height * scale))


@dataclass(frozen=True)
class FpvStreamSettings:
    codec: str = "h264"
    width: int = 424
    height: int = 240
    fps: int = 60
    port: int = 5004
    payload: int = 96
    jitter_latency_ms: int = 0
    jpeg_quality: int = 80

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> "FpvStreamSettings":
        codec = str(values.get("codec") or "h264").strip().lower()
        if codec not in SUPPORTED_CODECS:
            supported = ", ".join(sorted(SUPPORTED_CODECS))
            raise ValueError(f"codec must be one of: {supported}")
        payload = _bounded_int(
            values.get("payload"), field="payload", default=96, minimum=0, maximum=127
        )
        return cls(
            codec=codec,
            width=_bounded_int(
                values.get("width"), field="width", default=424, minimum=1, maximum=8192
            ),
            height=_bounded_int(
                values.get("height"), field="height", default=240, minimum=1, maximum=8192
            ),
            fps=_bounded_int(
                values.get("fps"), field="fps", default=60, minimum=1, maximum=240
            ),
            port=_bounded_int(
                values.get("port"), field="port", default=5004, minimum=1, maximum=65535
            ),
            payload=26 if codec == "mjpeg" else payload,
            jitter_latency_ms=_bounded_int(
                values.get("jitter_latency_ms"),
                field="jitter_latency_ms",
                default=0,
                minimum=0,
                maximum=5000,
            ),
            jpeg_quality=_bounded_int(
                values.get("jpeg_quality"),
                field="jpeg_quality",
                default=80,
                minimum=20,
                maximum=95,
            ),
        )

    def browser_dimensions(self) -> tuple[int, int]:
        return scale_to_fit(self.width, self.height)

    def as_json(self) -> dict[str, object]:
        result = asdict(self)
        output_width, output_height = self.browser_dimensions()
        result.update(
            {
                "output_width": output_width,
                "output_height": output_height,
                "output_fps": min(self.fps, WEB_MAX_FPS),
            }
        )
        return result


def build_gstreamer_command(
    settings: FpvStreamSettings,
    *,
    executable: str = "gst-launch-1.0",
) -> list[str]:
    command = [executable, "-q"]
    if settings.codec == "raw":
        caps = (
            "application/x-rtp,media=(string)video,clock-rate=(int)90000,"
            "encoding-name=(string)RAW,sampling=(string)RGB,depth=(string)8,"
            f"width=(string){settings.width},height=(string){settings.height},"
            f"payload=(int){settings.payload}"
        )
        decode_chain = ["rtpvrawdepay"]
    elif settings.codec == "mjpeg":
        caps = (
            "application/x-rtp,media=video,clock-rate=90000,"
            "encoding-name=JPEG,payload=26"
        )
        decode_chain = ["rtpjpegdepay", "!", "jpegdec"]
    elif settings.codec == "h265":
        caps = (
            "application/x-rtp,media=video,clock-rate=90000,"
            f"encoding-name=H265,payload={settings.payload}"
        )
        decode_chain = ["rtph265depay", "!", "h265parse", "!", "decodebin"]
    else:
        caps = (
            "application/x-rtp,media=video,clock-rate=90000,"
            f"encoding-name=H264,payload={settings.payload}"
        )
        decode_chain = ["rtph264depay", "!", "h264parse", "!", "decodebin"]

    output_width, output_height = settings.browser_dimensions()
    output_fps = min(settings.fps, WEB_MAX_FPS)
    command.extend(
        [
            "udpsrc",
            f"port={settings.port}",
            f"caps={caps}",
            "!",
            "rtpjitterbuffer",
            f"latency={settings.jitter_latency_ms}",
            "drop-on-latency=true",
            "!",
            "queue",
            "leaky=downstream",
            "max-size-buffers=8",
            "!",
            *decode_chain,
            "!",
            "videorate",
            "drop-only=true",
            "!",
            f"video/x-raw,framerate={output_fps}/1",
            "!",
            "videoconvert",
            "!",
            "videoscale",
            "!",
            (
                "video/x-raw,format=RGB,pixel-aspect-ratio=1/1,"
                f"width={output_width},height={output_height}"
            ),
            "!",
            "jpegenc",
            f"quality={settings.jpeg_quality}",
            "!",
            "fdsink",
            "fd=1",
            "sync=false",
        ]
    )
    return command


class FpvStreamManager:
    """Own one RTP receiver and expose its newest JPEG frame to HTTP clients."""

    def __init__(self, executable: str = "gst-launch-1.0") -> None:
        self.executable = executable
        self._condition = threading.Condition(threading.RLock())
        self._lifecycle_lock = threading.RLock()
        self._process: subprocess.Popen[bytes] | None = None
        self._settings: FpvStreamSettings | None = None
        self._session_id = ""
        self._generation = 0
        self._running = False
        self._started_at = 0.0
        self._last_frame_at = 0.0
        self._frame_sequence = 0
        self._frame_count = 0
        self._byte_count = 0
        self._latest_frame: bytes | None = None
        self._stderr_tail = bytearray()
        self._last_error = ""
        self._lease_deadline = 0.0

    def available(self) -> bool:
        return shutil.which(self.executable) is not None

    def start(self, settings: FpvStreamSettings) -> dict[str, object]:
        if not self.available():
            raise RuntimeError(f"{self.executable} was not found")

        with self._lifecycle_lock:
            self.stop()
            command = build_gstreamer_command(settings, executable=self.executable)
            try:
                process = subprocess.Popen(
                    command,
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    bufsize=0,
                    start_new_session=True,
                )
            except OSError as exc:
                raise RuntimeError(f"could not start RTP receiver: {exc}") from exc

            now = time.time()
            with self._condition:
                self._generation += 1
                generation = self._generation
                self._process = process
                self._settings = settings
                self._session_id = f"{int(now * 1000)}-{generation}"
                self._running = True
                self._started_at = now
                self._last_frame_at = 0.0
                self._frame_sequence = 0
                self._frame_count = 0
                self._byte_count = 0
                self._latest_frame = None
                self._stderr_tail.clear()
                self._last_error = ""
                self._lease_deadline = time.monotonic() + FPV_LEASE_TIMEOUT_S
                self._condition.notify_all()

            stderr_thread = threading.Thread(
                target=self._read_stderr,
                args=(process, generation),
                daemon=True,
                name="jetpilot-fpv-stderr",
            )
            reader_thread = threading.Thread(
                target=self._read_frames,
                args=(process, generation, stderr_thread),
                daemon=True,
                name="jetpilot-fpv-reader",
            )
            stderr_thread.start()
            reader_thread.start()
            threading.Thread(
                target=self._watch_lease,
                args=(self._session_id, generation),
                daemon=True,
                name="jetpilot-fpv-lease",
            ).start()
            return self.status()

    def stop(self, session_id: str | None = None) -> bool:
        with self._lifecycle_lock:
            with self._condition:
                if session_id is not None and session_id != self._session_id:
                    return False
                process = self._process
                was_running = process is not None and process.poll() is None
                self._generation += 1
                self._process = None
                self._running = False
                self._latest_frame = None
                self._last_error = ""
                self._lease_deadline = 0.0
                self._condition.notify_all()

            if process is not None and process.poll() is None:
                self._terminate_process(process)
            return was_running

    def heartbeat(self, session_id: str) -> bool:
        with self._condition:
            if not self._running or not session_id or session_id != self._session_id:
                return False
            self._lease_deadline = time.monotonic() + FPV_LEASE_TIMEOUT_S
            self._condition.notify_all()
            return True

    def status(self) -> dict[str, object]:
        with self._condition:
            now = time.time()
            settings = self._settings.as_json() if self._settings else None
            return {
                "available": self.available(),
                "running": self._running,
                "session_id": self._session_id,
                "started_at": self._started_at or None,
                "last_frame_age_s": (
                    round(max(0.0, now - self._last_frame_at), 3)
                    if self._last_frame_at
                    else None
                ),
                "frame_count": self._frame_count,
                "jpeg_bytes": self._byte_count,
                "settings": settings,
                "last_error": self._last_error,
                "lease_remaining_s": (
                    round(max(0.0, self._lease_deadline - time.monotonic()), 3)
                    if self._running
                    else 0.0
                ),
            }

    def current_session_id(self) -> str:
        with self._condition:
            return self._session_id if self._running else ""

    def wait_for_frame(
        self,
        session_id: str,
        after_sequence: int,
        *,
        timeout: float = 2.0,
    ) -> tuple[int, bytes] | None:
        deadline = time.monotonic() + timeout
        with self._condition:
            while (
                self._running
                and self._session_id == session_id
                and self._frame_sequence <= after_sequence
            ):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(remaining)
            if (
                self._session_id != session_id
                or self._latest_frame is None
                or self._frame_sequence <= after_sequence
            ):
                return None
            return self._frame_sequence, self._latest_frame

    def session_is_running(self, session_id: str) -> bool:
        with self._condition:
            return self._running and self._session_id == session_id

    def _read_stderr(self, process: subprocess.Popen[bytes], generation: int) -> None:
        stream = process.stderr
        if stream is None:
            return
        while True:
            chunk = stream.read(4096)
            if not chunk:
                return
            with self._condition:
                if generation != self._generation or process is not self._process:
                    continue
                self._stderr_tail.extend(chunk)
                if len(self._stderr_tail) > _MAX_STDERR_BYTES:
                    del self._stderr_tail[: len(self._stderr_tail) - _MAX_STDERR_BYTES]

    def _watch_lease(self, session_id: str, generation: int) -> None:
        while True:
            with self._condition:
                if (
                    generation != self._generation
                    or not self._running
                    or session_id != self._session_id
                ):
                    return
                remaining = self._lease_deadline - time.monotonic()
                if remaining > 0:
                    self._condition.wait(min(remaining, 2.0))
                    continue
            with self._lifecycle_lock:
                with self._condition:
                    if (
                        generation != self._generation
                        or not self._running
                        or session_id != self._session_id
                        or self._lease_deadline > time.monotonic()
                    ):
                        continue
                self.stop(session_id)
                return

    def _read_frames(
        self,
        process: subprocess.Popen[bytes],
        generation: int,
        stderr_thread: threading.Thread,
    ) -> None:
        stream = process.stdout
        buffer = bytearray()
        reader_error = ""
        return_code: int | None = None
        try:
            if stream is not None:
                while True:
                    chunk = stream.read(64 * 1024)
                    if not chunk:
                        break
                    buffer.extend(chunk)
                    self._extract_frames(buffer, process, generation)
                    _trim_jpeg_buffer(buffer)
            return_code = process.wait()
        except Exception as exc:  # noqa: BLE001 - background workers must publish failure state.
            reader_error = f"RTP frame reader failed: {exc}"
            if process.poll() is None:
                self._terminate_process(process)
            return_code = process.poll()
        finally:
            stderr_thread.join(timeout=0.25)
            with self._condition:
                if generation == self._generation and process is self._process:
                    self._process = None
                    self._running = False
                    error_text = self._stderr_tail.decode("utf-8", errors="replace").strip()
                    if reader_error:
                        self._last_error = "\n".join(
                            part for part in (reader_error, error_text) if part
                        )
                    elif return_code != 0:
                        self._last_error = error_text or f"RTP receiver exited with code {return_code}"
                    elif self._frame_count == 0:
                        self._last_error = error_text or "RTP receiver stopped before receiving a frame"
                    else:
                        self._last_error = error_text
                    self._condition.notify_all()

    @staticmethod
    def _terminate_process(process: subprocess.Popen[bytes]) -> None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except (AttributeError, OSError):
            process.terminate()
        try:
            process.wait(timeout=1.5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except (AttributeError, OSError):
                process.kill()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                pass

    def _extract_frames(
        self,
        buffer: bytearray,
        process: subprocess.Popen[bytes],
        generation: int,
    ) -> None:
        while True:
            start = buffer.find(b"\xff\xd8")
            if start < 0:
                if len(buffer) > 1:
                    del buffer[:-1]
                return
            if start:
                del buffer[:start]
            end = buffer.find(b"\xff\xd9", 2)
            if end < 0:
                return
            end += 2
            frame = bytes(buffer[:end])
            del buffer[:end]
            with self._condition:
                if generation != self._generation or process is not self._process:
                    return
                self._latest_frame = frame
                self._frame_sequence += 1
                self._frame_count += 1
                self._byte_count += len(frame)
                self._last_frame_at = time.time()
                self._condition.notify_all()
