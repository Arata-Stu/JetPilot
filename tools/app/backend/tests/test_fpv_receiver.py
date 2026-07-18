from __future__ import annotations

import unittest

from jetpilot_console.fpv_receiver import FpvReceiverManager
from jetpilot_console.fpv_stream import FpvStreamSettings


class _FakeTransport:
    def __init__(self, *, webrtc: bool = False) -> None:
        self.webrtc = webrtc
        self.running = False
        self.session_id = ""
        self.settings: FpvStreamSettings | None = None
        self.stop_calls = 0

    def start(self, settings: FpvStreamSettings) -> dict[str, object]:
        self.running = True
        self.session_id = "webrtc-session" if self.webrtc else "mjpeg-session"
        self.settings = settings
        return self.status()

    def stop(self, session_id: str | None = None) -> bool:
        if session_id is not None and session_id != self.session_id:
            return False
        was_running = self.running
        self.running = False
        self.stop_calls += 1
        return was_running

    def heartbeat(self, session_id: str) -> bool:
        return self.running and session_id == self.session_id

    def status(self) -> dict[str, object]:
        return {
            "available": True,
            "running": self.running,
            "session_id": self.session_id,
            "settings": self.settings.as_json() if self.settings else None,
            "last_error": "",
        }

    def current_session_id(self) -> str:
        return self.session_id if self.running else ""

    def session_is_running(self, session_id: str) -> bool:
        return self.running and session_id == self.session_id

    def wait_for_frame(
        self,
        session_id: str,
        after_sequence: int,
        *,
        timeout: float,
    ) -> tuple[int, bytes] | None:
        del timeout
        if not self.session_is_running(session_id):
            return None
        return after_sequence + 1, b"jpeg"

    def availability(self) -> dict[str, object]:
        return {"available": True, "error": "", "passthrough": True}

    def create_offer(self, session_id: str) -> dict[str, object]:
        if not self.session_is_running(session_id):
            raise RuntimeError("stale session")
        return {"type": "offer", "sdp": "v=0\r\nm=video 9 UDP/TLS/RTP/SAVPF 96\r\n"}

    def set_answer(
        self,
        session_id: str,
        description_type: str,
        sdp: str,
    ) -> dict[str, object]:
        if not self.session_is_running(session_id):
            raise RuntimeError("stale session")
        if description_type != "answer" or "m=video" not in sdp:
            raise ValueError("invalid answer")
        return self.status()

    def shutdown(self) -> None:
        self.stop()


class FpvReceiverManagerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.mjpeg = _FakeTransport()
        self.webrtc = _FakeTransport(webrtc=True)
        self.manager = FpvReceiverManager(
            mjpeg_manager=self.mjpeg,  # type: ignore[arg-type]
            webrtc_manager=self.webrtc,  # type: ignore[arg-type]
        )

    def test_selects_one_transport_and_switching_stops_the_previous_one(self) -> None:
        status = self.manager.start(
            FpvStreamSettings.from_mapping({"codec": "h264", "transport": "webrtc"})
        )
        self.assertEqual(status["transport"], "webrtc")
        self.assertTrue(self.webrtc.running)
        self.assertFalse(self.mjpeg.running)

        status = self.manager.start(
            FpvStreamSettings.from_mapping({"codec": "h265", "transport": "mjpeg"})
        )
        self.assertEqual(status["transport"], "mjpeg")
        self.assertFalse(self.webrtc.running)
        self.assertTrue(self.mjpeg.running)

    def test_stale_stop_cannot_clear_the_active_transport(self) -> None:
        self.manager.start(
            FpvStreamSettings.from_mapping({"codec": "h264", "transport": "webrtc"})
        )
        self.assertFalse(self.manager.stop("older-session"))
        self.assertTrue(self.manager.status()["running"])
        self.assertEqual(self.manager.status()["transport"], "webrtc")

    def test_stop_status_describes_the_transport_that_just_stopped(self) -> None:
        self.manager.start(
            FpvStreamSettings.from_mapping({"codec": "h264", "transport": "webrtc"})
        )
        self.assertTrue(self.manager.stop("webrtc-session"))
        status = self.manager.status()
        self.assertFalse(status["running"])
        self.assertIsNone(status["transport"])
        self.assertEqual(status["session_id"], "webrtc-session")
        self.assertEqual(status["settings"]["transport"], "webrtc")

    def test_mjpeg_frames_are_not_exposed_for_a_webrtc_session(self) -> None:
        self.manager.start(
            FpvStreamSettings.from_mapping({"codec": "h264", "transport": "webrtc"})
        )
        self.assertFalse(self.manager.stream_is_mjpeg("webrtc-session"))
        self.assertIsNone(self.manager.wait_for_frame("webrtc-session", 0))

        self.manager.start(
            FpvStreamSettings.from_mapping({"codec": "h264", "transport": "mjpeg"})
        )
        self.assertTrue(self.manager.stream_is_mjpeg("mjpeg-session"))
        self.assertEqual(
            self.manager.wait_for_frame("mjpeg-session", 0),
            (1, b"jpeg"),
        )

    def test_offer_and_answer_are_only_available_for_webrtc(self) -> None:
        self.manager.start(
            FpvStreamSettings.from_mapping({"codec": "h264", "transport": "webrtc"})
        )
        offer = self.manager.create_offer("webrtc-session")
        self.assertEqual(offer["type"], "offer")
        status = self.manager.set_answer(
            "webrtc-session",
            "answer",
            "v=0\r\nm=video 9 UDP/TLS/RTP/SAVPF 96\r\n",
        )
        self.assertTrue(status["running"])

        self.manager.start(
            FpvStreamSettings.from_mapping({"codec": "h264", "transport": "mjpeg"})
        )
        with self.assertRaises(RuntimeError):
            self.manager.create_offer("mjpeg-session")


if __name__ == "__main__":
    unittest.main()
