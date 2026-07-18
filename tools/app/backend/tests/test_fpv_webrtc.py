from __future__ import annotations

import time
import unittest
from unittest import mock

from jetpilot_console import fpv_webrtc
from jetpilot_console.fpv_stream import FpvStreamSettings
from jetpilot_console.fpv_webrtc import (
    _SDP_MAX_BYTES,
    _validate_answer_sdp,
    build_webrtc_pipeline_description,
    WebRtcStreamManager,
)


def webrtc_settings(**updates: object) -> FpvStreamSettings:
    settings = FpvStreamSettings.from_mapping(
        {
            "transport": "webrtc",
            "codec": "h264",
            "port": 5004,
            "payload": 97,
            "jitter_latency_ms": 10,
        }
    )
    # Support the transition while transport is being added to the shared settings.
    if not hasattr(settings, "transport"):
        object.__setattr__(settings, "transport", "webrtc")
    for name, value in updates.items():
        object.__setattr__(settings, name, value)
    return settings


class WebRtcPipelineTests(unittest.TestCase):
    def test_pipeline_repacketizes_h264_without_transcoding(self) -> None:
        pipeline = build_webrtc_pipeline_description(webrtc_settings())
        self.assertIn("webrtcbin name=webrtc", pipeline)
        self.assertIn("udpsrc name=rtp_source port=5004", pipeline)
        self.assertIn("payload=(int)97", pipeline)
        self.assertIn("latency=10", pipeline)
        self.assertIn("rtph264depay", pipeline)
        self.assertIn("h264parse", pipeline)
        self.assertIn("rtph264pay pt=96", pipeline)
        self.assertIn("packetization-mode=(string)1", pipeline)
        self.assertIn("aggregate-mode=zero-latency", pipeline)
        for forbidden in ("decode", "jpeg", "x264", "nvh264", "avenc", "queue"):
            self.assertNotIn(forbidden, pipeline.lower())

    def test_pipeline_rejects_non_webrtc_or_non_h264_settings(self) -> None:
        with self.assertRaisesRegex(ValueError, "transport"):
            build_webrtc_pipeline_description(webrtc_settings(transport="mjpeg"))
        with self.assertRaisesRegex(ValueError, "h264"):
            build_webrtc_pipeline_description(webrtc_settings(codec="h265"))
        with self.assertRaisesRegex(ValueError, "port"):
            build_webrtc_pipeline_description(webrtc_settings(port=0))
        with self.assertRaisesRegex(ValueError, "payload"):
            build_webrtc_pipeline_description(webrtc_settings(payload="96"))


class WebRtcSdpValidationTests(unittest.TestCase):
    VALID_VIDEO_SDP = (
        "v=0\r\n"
        "o=- 1 1 IN IP4 127.0.0.1\r\n"
        "s=-\r\n"
        "t=0 0\r\n"
        "m=video 9 UDP/TLS/RTP/SAVPF 96\r\n"
        "a=rtpmap:96 H264/90000\r\n"
    )

    def test_accepts_answer_with_video(self) -> None:
        self.assertEqual(
            _validate_answer_sdp("answer", self.VALID_VIDEO_SDP),
            self.VALID_VIDEO_SDP,
        )

    def test_rejects_bad_type_missing_video_and_other_media(self) -> None:
        with self.assertRaisesRegex(ValueError, "type"):
            _validate_answer_sdp("offer", self.VALID_VIDEO_SDP)
        with self.assertRaisesRegex(ValueError, "video"):
            _validate_answer_sdp("answer", "v=0\r\nm=audio 9 RTP/AVP 0\r\n")
        with self.assertRaisesRegex(ValueError, "video media only"):
            _validate_answer_sdp(
                "answer", self.VALID_VIDEO_SDP + "m=audio 9 RTP/AVP 0\r\n"
            )

    def test_rejects_malformed_or_oversized_sdp(self) -> None:
        for sdp in ("", "m=video 9 RTP/AVP 96\r\n", "v=0\x00\r\nm=video 9 RTP/AVP 96"):
            with self.subTest(sdp=sdp), self.assertRaises(ValueError):
                _validate_answer_sdp("answer", sdp)
        oversized = "v=0\r\nm=video 9 RTP/AVP 96\r\na=x:" + ("x" * _SDP_MAX_BYTES)
        with self.assertRaisesRegex(ValueError, "exceed"):
            _validate_answer_sdp("answer", oversized)


class WebRtcManagerStateTests(unittest.TestCase):
    def test_gstreamer_older_than_1_20_is_reported_unavailable(self) -> None:
        class OldGst:
            @staticmethod
            def version() -> tuple[int, int, int, int]:
                return (1, 18, 6, 0)

        with mock.patch.multiple(
            fpv_webrtc,
            Gst=OldGst,
            GstSdp=object(),
            GstWebRTC=object(),
            GLib=object(),
        ):
            availability = WebRtcStreamManager().availability()

        self.assertFalse(availability["available"])
        self.assertIn("1.20.0 or newer", availability["reason"])
        self.assertEqual(availability["gstreamer_version"], "1.18.6")

    def test_non_loopback_manager_is_explicitly_unavailable(self) -> None:
        manager = WebRtcStreamManager(allow=False)
        availability = manager.availability()
        self.assertFalse(availability["available"])
        self.assertIn("non-loopback", availability["reason"])
        with self.assertRaises(RuntimeError):
            manager.start(webrtc_settings())

    def test_status_has_legacy_and_webrtc_fields(self) -> None:
        manager = WebRtcStreamManager(allow=False)
        status = manager.status()
        for key in (
            "available",
            "running",
            "session_id",
            "started_at",
            "last_frame_age_s",
            "frame_count",
            "jpeg_bytes",
            "settings",
            "last_error",
            "lease_remaining_s",
        ):
            self.assertIn(key, status)
        self.assertEqual(status["transport"], "webrtc")
        self.assertFalse(status["webrtc"]["transcoding"])
        self.assertTrue(status["webrtc"]["passthrough"])
        self.assertEqual(status["rtp_packet_count"], 0)
        self.assertEqual(status["udp_packet_count"], 0)
        self.assertEqual(status["udp_bytes"], 0)

    def test_secure_session_matching_prevents_stale_control(self) -> None:
        manager = WebRtcStreamManager(allow=False)
        with manager._condition:
            manager._session_id = "secure-current-session"
            manager._running = True
            manager._lease_deadline = time.monotonic() + 8.0
        self.assertTrue(manager.session_is_running("secure-current-session"))
        self.assertFalse(manager.session_is_running("stale-session"))
        self.assertTrue(manager.heartbeat("secure-current-session"))
        self.assertFalse(manager.heartbeat("stale-session"))
        self.assertFalse(manager.stop("stale-session"))
        self.assertTrue(manager.status()["running"])
        self.assertTrue(manager.stop("secure-current-session"))
        self.assertFalse(manager.status()["running"])

    def test_expired_lease_stops_only_its_generation(self) -> None:
        manager = WebRtcStreamManager(allow=False)
        with manager._condition:
            manager._generation = 4
            manager._session_id = "expiring-session"
            manager._running = True
            manager._lease_deadline = time.monotonic() - 1.0
        manager._watch_lease("expiring-session", 4)
        self.assertFalse(manager.status()["running"])

        with manager._condition:
            manager._generation = 6
            manager._session_id = "newer-session"
            manager._running = True
            manager._lease_deadline = time.monotonic() + 8.0
        manager._watch_lease("older-session", 5)
        self.assertTrue(manager.status()["running"])
        manager.stop("newer-session")


if __name__ == "__main__":
    unittest.main()
