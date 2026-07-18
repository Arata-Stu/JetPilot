from __future__ import annotations

import io
import json
import unittest
from email.message import Message
from types import SimpleNamespace

from jetpilot_console.fpv_stream import (
    FpvStreamSettings,
    FpvStreamManager,
    _trim_jpeg_buffer,
    build_gstreamer_command,
    scale_to_fit,
)
from jetpilot_console.main import Handler


class FpvStreamSettingsTests(unittest.TestCase):
    def test_defaults_and_high_resolution_browser_limit(self) -> None:
        settings = FpvStreamSettings.from_mapping(
            {"codec": "h264", "width": 3840, "height": 2160, "fps": 60}
        )
        self.assertEqual(settings.browser_dimensions(), (1280, 720))
        self.assertEqual(settings.as_json()["output_fps"], 30)
        self.assertEqual(scale_to_fit(720, 1280), (405, 720))

    def test_rejects_unsupported_or_out_of_range_values(self) -> None:
        invalid_values = (
            {"codec": "vp9"},
            {"codec": "h264; touch /tmp/unsafe"},
            {"port": 0},
            {"port": 65536},
            {"payload": 128},
            {"width": "2.5"},
            {"fps": True},
            {"transport": "websocket"},
            {"transport": "webrtc", "codec": "h265"},
        )
        for values in invalid_values:
            with self.subTest(values=values), self.assertRaises(ValueError):
                FpvStreamSettings.from_mapping(values)

    def test_command_is_an_argument_list_with_expected_rtp_chain(self) -> None:
        settings = FpvStreamSettings.from_mapping(
            {
                "codec": "h265",
                "width": 1920,
                "height": 1080,
                "fps": 60,
                "port": 5004,
                "payload": 97,
            }
        )
        command = build_gstreamer_command(settings)
        self.assertEqual(command[:2], ["gst-launch-1.0", "-q"])
        self.assertIn("port=5004", command)
        self.assertIn("encoding-name=H265,payload=97", " ".join(command))
        self.assertIn("rtph265depay", command)
        self.assertIn("decodebin", command)
        self.assertIn("video/x-raw,framerate=30/1", command)
        self.assertIn(
            "video/x-raw,format=RGB,pixel-aspect-ratio=1/1,width=1280,height=720",
            command,
        )
        self.assertNotIn("bash", command)
        self.assertNotIn("sh", command)

    def test_raw_caps_use_validated_source_dimensions(self) -> None:
        command = build_gstreamer_command(
            FpvStreamSettings.from_mapping(
                {"codec": "raw", "width": 424, "height": 240, "payload": 96}
            )
        )
        joined = " ".join(command)
        self.assertIn("encoding-name=(string)RAW", joined)
        self.assertIn("width=(string)424", joined)
        self.assertIn("height=(string)240", joined)
        self.assertIn("rtpvrawdepay", command)

    def test_mjpeg_payload_is_fixed_to_26(self) -> None:
        settings = FpvStreamSettings.from_mapping(
            {"codec": "mjpeg", "payload": 96}
        )
        self.assertEqual(settings.payload, 26)
        self.assertIn("encoding-name=JPEG,payload=26", " ".join(build_gstreamer_command(settings)))


class _RecordingFpvManager:
    def __init__(self) -> None:
        self.settings: FpvStreamSettings | None = None
        self.running = False

    def start(self, settings: FpvStreamSettings) -> dict[str, object]:
        self.settings = settings
        self.running = True
        return self.status()

    def stop(self, session_id: str | None = None) -> bool:
        if session_id is not None and session_id != "test-session":
            return False
        was_running = self.running
        self.running = False
        return was_running

    def heartbeat(self, session_id: str) -> bool:
        return self.running and session_id == "test-session"

    def status(self) -> dict[str, object]:
        return {
            "available": True,
            "running": self.running,
            "session_id": "test-session" if self.running else "",
            "settings": self.settings.as_json() if self.settings else None,
        }

    def create_offer(self, session_id: str) -> dict[str, object]:
        if not self.running or session_id != "test-session":
            raise RuntimeError("stale WebRTC session")
        return {"session_id": session_id, "type": "offer", "sdp": "v=0\r\nm=video 9 UDP/TLS/RTP/SAVPF 96\r\n"}

    def set_answer(
        self,
        session_id: str,
        description_type: str,
        sdp: str,
    ) -> dict[str, object]:
        if not self.running or session_id != "test-session":
            raise RuntimeError("stale WebRTC session")
        if description_type != "answer" or "m=video" not in sdp:
            raise ValueError("invalid WebRTC answer")
        return self.status()

    def current_session_id(self) -> str:
        return ""

    def session_is_running(self, _: str) -> bool:
        return False


class FpvHttpEndpointTests(unittest.TestCase):
    def request(
        self, method: str, path: str, body: dict[str, object] | None = None
    ) -> tuple[int, dict[str, object]]:
        encoded = json.dumps(body or {}).encode("utf-8") if method == "POST" else b""
        headers = Message()
        headers["Host"] = "127.0.0.1:8765"
        if method == "POST":
            headers["Content-Type"] = "application/json"
            headers["Content-Length"] = str(len(encoded))

        handler = Handler.__new__(Handler)
        handler.path = path
        handler.command = method
        handler.request_version = "HTTP/1.1"
        handler.requestline = f"{method} {path} HTTP/1.1"
        handler.client_address = ("127.0.0.1", 12345)
        handler.close_connection = True
        handler.headers = headers
        handler.rfile = io.BytesIO(encoded)
        handler.wfile = io.BytesIO()
        handler.server = SimpleNamespace(
            state=SimpleNamespace(
                config=SimpleNamespace(),
                joy_only=False,
                loopback_only=True,
                fpv_stream=self.manager,
            )
        )
        if method == "POST":
            handler.do_POST()
        else:
            handler.do_GET()
        raw_headers, raw_body = handler.wfile.getvalue().split(b"\r\n\r\n", 1)
        status = int(raw_headers.splitlines()[0].split()[1])
        return status, json.loads(raw_body.decode("utf-8"))

    def setUp(self) -> None:
        self.manager = _RecordingFpvManager()

    def test_controlled_start_status_heartbeat_and_stop(self) -> None:
        status, payload = self.request(
            "POST",
            "/api/fpv/start",
            {
                "codec": "h264",
                "width": 1920,
                "height": 1080,
                "fps": 60,
                "port": 5004,
                "payload": 96,
            },
        )
        self.assertEqual(status, 201)
        self.assertTrue(payload["fpv"]["running"])
        self.assertEqual(self.manager.settings.browser_dimensions(), (1280, 720))

        status, payload = self.request("GET", "/api/fpv/status")
        self.assertEqual(status, 200)
        self.assertTrue(payload["fpv"]["running"])

        status, payload = self.request(
            "POST", "/api/fpv/heartbeat", {"session_id": "test-session"}
        )
        self.assertEqual(status, 200)
        self.assertTrue(payload["ok"])

        status, payload = self.request(
            "POST", "/api/fpv/stop", {"session_id": "test-session"}
        )
        self.assertEqual(status, 200)
        self.assertTrue(payload["stopped"])
        self.assertFalse(payload["fpv"]["running"])

    def test_stale_stop_cannot_stop_a_new_session(self) -> None:
        self.manager.running = True
        status, payload = self.request(
            "POST", "/api/fpv/stop", {"session_id": "older-session"}
        )
        self.assertEqual(status, 200)
        self.assertFalse(payload["stopped"])
        self.assertTrue(payload["fpv"]["running"])

        status, payload = self.request("POST", "/api/fpv/stop", {})
        self.assertEqual(status, 400)
        self.assertIn("session_id", payload["error"])

    def test_invalid_start_is_rejected_without_starting_a_process(self) -> None:
        status, payload = self.request(
            "POST", "/api/fpv/start", {"codec": "h264; unsafe", "port": 5004}
        )
        self.assertEqual(status, 400)
        self.assertIn("codec", payload["error"])
        self.assertIsNone(self.manager.settings)

    def test_stream_requires_an_active_session(self) -> None:
        status, payload = self.request("GET", "/api/fpv/stream")
        self.assertEqual(status, 409)
        self.assertIn("not running", payload["error"])

    def test_webrtc_offer_and_answer_are_scoped_to_the_active_session(self) -> None:
        self.manager.running = True
        status, payload = self.request(
            "POST", "/api/fpv/webrtc/offer", {"session_id": "test-session"}
        )
        self.assertEqual(status, 200)
        self.assertEqual(payload["offer"]["type"], "offer")

        status, payload = self.request(
            "POST",
            "/api/fpv/webrtc/answer",
            {
                "session_id": "test-session",
                "type": "answer",
                "sdp": "v=0\r\nm=video 9 UDP/TLS/RTP/SAVPF 96\r\n",
            },
        )
        self.assertEqual(status, 200)
        self.assertTrue(payload["ok"])

        status, payload = self.request(
            "POST", "/api/fpv/webrtc/offer", {"session_id": "stale-session"}
        )
        self.assertEqual(status, 409)
        self.assertIn("stale", payload["error"])

    def test_webrtc_answer_requires_string_sdp(self) -> None:
        self.manager.running = True
        status, payload = self.request(
            "POST",
            "/api/fpv/webrtc/answer",
            {"session_id": "test-session", "type": "answer", "sdp": 42},
        )
        self.assertEqual(status, 400)
        self.assertIn("string", payload["error"])


class FpvStreamManagerRegressionTests(unittest.TestCase):
    def test_oversized_incomplete_jpeg_is_bounded(self) -> None:
        buffer = bytearray(b"\xff\xd8" + (b"x" * 64))
        _trim_jpeg_buffer(buffer, maximum_bytes=16)
        self.assertLessEqual(len(buffer), 1)

        buffer = bytearray((b"old" * 10) + b"\xff\xd8new-frame")
        _trim_jpeg_buffer(buffer, maximum_bytes=16)
        self.assertEqual(buffer, bytearray(b"\xff\xd8new-frame"))

    def test_reader_exception_transitions_receiver_to_failed(self) -> None:
        class RaisingStream:
            def read(self, _: int) -> bytes:
                raise OSError("pipe failed")

        class FailedProcess:
            def __init__(self) -> None:
                self.stdout = RaisingStream()
                self.stderr = None
                self.return_code: int | None = None

            def poll(self) -> int | None:
                return self.return_code

            def terminate(self) -> None:
                self.return_code = -15

            def kill(self) -> None:
                self.return_code = -9

            def wait(self, timeout: float | None = None) -> int:
                del timeout
                if self.return_code is None:
                    self.return_code = 0
                return self.return_code

        manager = FpvStreamManager()
        process = FailedProcess()
        with manager._condition:
            manager._generation = 1
            manager._process = process  # type: ignore[assignment]
            manager._running = True
            manager._session_id = "reader-test"
        manager._read_frames(  # type: ignore[arg-type]
            process,
            1,
            SimpleNamespace(join=lambda timeout: None),
        )
        status = manager.status()
        self.assertFalse(status["running"])
        self.assertIn("pipe failed", status["last_error"])


if __name__ == "__main__":
    unittest.main()
