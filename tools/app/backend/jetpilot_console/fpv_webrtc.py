from __future__ import annotations

import hashlib
import secrets
import threading
import time
from typing import Any

from .fpv_stream import FPV_LEASE_TIMEOUT_S, FpvStreamSettings


_SDP_MAX_BYTES = 64 * 1024
_WEBRTC_OPERATION_TIMEOUT_S = 8.0
_WEBRTC_OUTPUT_PAYLOAD = 96
_MIN_GSTREAMER_VERSION = (1, 20, 0)
_REQUIRED_ELEMENTS = (
    "webrtcbin",
    "nicesrc",
    "nicesink",
    "udpsrc",
    "rtpjitterbuffer",
    "rtph264depay",
    "h264parse",
    "rtph264pay",
)


try:  # Optional on the macOS development host and in non-WebRTC deployments.
    import gi

    gi.require_version("Gst", "1.0")
    gi.require_version("GstSdp", "1.0")
    gi.require_version("GstWebRTC", "1.0")
    from gi.repository import GLib, Gst, GstSdp, GstWebRTC

    Gst.init(None)
    _GST_IMPORT_ERROR = ""
except Exception as exc:  # noqa: BLE001 - availability is reported through the API.
    GLib = None  # type: ignore[assignment]
    Gst = None  # type: ignore[assignment]
    GstSdp = None  # type: ignore[assignment]
    GstWebRTC = None  # type: ignore[assignment]
    _GST_IMPORT_ERROR = str(exc)


def _validated_int(
    settings: FpvStreamSettings,
    name: str,
    *,
    minimum: int,
    maximum: int,
) -> int:
    value = getattr(settings, name, None)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be a validated integer")
    if value < minimum or value > maximum:
        raise ValueError(f"{name} must be between {minimum} and {maximum}")
    return value


def _validate_webrtc_settings(settings: FpvStreamSettings) -> None:
    if not isinstance(settings, FpvStreamSettings):
        raise ValueError("settings must be validated FpvStreamSettings")
    if str(getattr(settings, "transport", "")).strip().lower() != "webrtc":
        raise ValueError("transport must be webrtc")
    if str(settings.codec).strip().lower() != "h264":
        raise ValueError("WebRTC currently supports only the h264 codec")
    _validated_int(settings, "port", minimum=1, maximum=65535)
    _validated_int(settings, "payload", minimum=0, maximum=127)
    _validated_int(settings, "jitter_latency_ms", minimum=0, maximum=5000)


def build_webrtc_pipeline_description(settings: FpvStreamSettings) -> str:
    """Build an H.264-only RTP-to-WebRTC pipeline without video transcoding.

    The depay/parse/pay section normalizes the incoming RTP payload type for the
    WebRTC session. H.264 access units remain compressed throughout the pipeline.
    """

    _validate_webrtc_settings(settings)
    port = _validated_int(settings, "port", minimum=1, maximum=65535)
    payload = _validated_int(settings, "payload", minimum=0, maximum=127)
    latency = _validated_int(
        settings, "jitter_latency_ms", minimum=0, maximum=5000
    )
    input_caps = (
        "application/x-rtp,media=(string)video,clock-rate=(int)90000,"
        f"encoding-name=(string)H264,payload=(int){payload}"
    )
    output_caps = (
        "application/x-rtp,media=(string)video,clock-rate=(int)90000,"
        "encoding-name=(string)H264,packetization-mode=(string)1,"
        f"payload=(int){_WEBRTC_OUTPUT_PAYLOAD}"
    )
    return " ".join(
        (
            "webrtcbin name=webrtc bundle-policy=max-bundle",
            "udpsrc name=rtp_source",
            f"port={port}",
            f'caps="{input_caps}"',
            "! rtpjitterbuffer",
            f"latency={latency}",
            "drop-on-latency=true",
            "! rtph264depay",
            "! h264parse",
            "config-interval=-1",
            "! rtph264pay",
            f"pt={_WEBRTC_OUTPUT_PAYLOAD}",
            "config-interval=-1",
            "aggregate-mode=zero-latency",
            f"! {output_caps}",
            "! webrtc.",
        )
    )


def _enum_name(value: object) -> str:
    nick = getattr(value, "value_nick", None)
    if nick:
        return str(nick).replace("_", "-").lower()
    text = str(value)
    if "." in text:
        text = text.rsplit(".", 1)[-1]
    return text.replace("_", "-").lower()


def _validate_answer_sdp(description_type: object, sdp: object) -> str:
    if not isinstance(description_type, str) or description_type.strip().lower() != "answer":
        raise ValueError("type must be answer")
    if not isinstance(sdp, str):
        raise ValueError("sdp must be a string")
    if not sdp or "\x00" in sdp:
        raise ValueError("sdp must be a non-empty text description")
    try:
        encoded = sdp.encode("utf-8")
    except UnicodeEncodeError as exc:
        raise ValueError("sdp must be valid UTF-8") from exc
    if len(encoded) > _SDP_MAX_BYTES:
        raise ValueError(f"sdp must not exceed {_SDP_MAX_BYTES} bytes")

    lines = [line.strip() for line in sdp.replace("\r\n", "\n").split("\n") if line.strip()]
    if not lines or lines[0] != "v=0":
        raise ValueError("sdp must start with v=0")
    media: list[str] = []
    for line in lines:
        if not line.startswith("m="):
            continue
        fields = line[2:].split()
        if not fields:
            raise ValueError("sdp contains an invalid media section")
        media.append(fields[0].lower())
    if "video" not in media:
        raise ValueError("sdp must contain a video media section")
    if any(kind != "video" for kind in media):
        raise ValueError("sdp must contain video media only")
    return sdp


class WebRtcStreamManager:
    """Own a single H.264 RTP-to-WebRTC session.

    PyGObject/GStreamer are optional. Every mutating GStreamer operation is
    serialized by ``_lifecycle_lock`` while callbacks only touch protected state.
    """

    def __init__(self, allow: bool = True) -> None:
        self._allow = bool(allow)
        self._condition = threading.Condition(threading.RLock())
        self._lifecycle_lock = threading.RLock()
        self._pipeline: Any | None = None
        self._webrtc: Any | None = None
        self._source: Any | None = None
        self._source_probe_id = 0
        self._bus: Any | None = None
        self._bus_handler_id = 0
        self._settings: FpvStreamSettings | None = None
        self._session_id = ""
        self._generation = 0
        self._running = False
        self._started_at = 0.0
        self._last_packet_at = 0.0
        self._packet_count = 0
        self._byte_count = 0
        self._last_error = ""
        self._lease_deadline = 0.0
        self._offer_created = False
        self._offer_sdp = ""
        self._answer_set = False
        self._answer_digest = ""
        self._ice_gathering_state = "new"
        self._connection_state = "new"
        self._signaling_state = "stable"
        self._main_loop: Any | None = None
        self._main_loop_thread: threading.Thread | None = None

    def availability(self) -> dict[str, object]:
        missing: list[str] = []
        if not self._allow:
            reason = "WebRTC is disabled for a non-loopback Console binding"
            return {
                "available": False,
                "reason": reason,
                "error": reason,
                "missing": ["loopback-only access"],
            }
        if Gst is None or GstSdp is None or GstWebRTC is None or GLib is None:
            reason = "PyGObject GStreamer WebRTC bindings are unavailable"
            if _GST_IMPORT_ERROR:
                reason = f"{reason}: {_GST_IMPORT_ERROR}"
            return {
                "available": False,
                "reason": reason,
                "error": reason,
                "missing": ["Gst", "GstSdp", "GstWebRTC"],
            }
        try:
            gst_version = tuple(int(part) for part in Gst.version()[:3])
        except Exception:  # noqa: BLE001 - an incomplete binding is unavailable.
            gst_version = (0, 0, 0)
        if gst_version < _MIN_GSTREAMER_VERSION:
            detected = ".".join(str(part) for part in gst_version)
            required = ".".join(str(part) for part in _MIN_GSTREAMER_VERSION)
            reason = (
                f"GStreamer {required} or newer is required for non-trickle "
                f"WebRTC (detected {detected})"
            )
            return {
                "available": False,
                "reason": reason,
                "error": reason,
                "missing": [f"GStreamer >= {required}"],
                "gstreamer_version": detected,
            }
        for name in _REQUIRED_ELEMENTS:
            if Gst.ElementFactory.find(name) is None:
                missing.append(name)
        reason = (
            ""
            if not missing
            else "Missing GStreamer elements: " + ", ".join(missing)
        )
        return {
            "available": not missing,
            "reason": reason,
            "error": reason,
            "missing": missing,
            "passthrough": True,
            "gstreamer_version": ".".join(str(part) for part in gst_version),
        }

    def available(self) -> bool:
        return bool(self.availability()["available"])

    def start(self, settings: FpvStreamSettings) -> dict[str, object]:
        _validate_webrtc_settings(settings)
        availability = self.availability()
        if not availability["available"]:
            raise RuntimeError(str(availability["reason"]))

        with self._lifecycle_lock:
            self._stop_locked(clear_error=True)
            self._ensure_main_loop_locked()
            description = build_webrtc_pipeline_description(settings)
            try:
                pipeline = Gst.parse_launch(description)
            except Exception as exc:  # noqa: BLE001 - normalize GStreamer errors for HTTP.
                raise RuntimeError(f"could not create WebRTC pipeline: {exc}") from exc

            webrtc = pipeline.get_by_name("webrtc")
            source = pipeline.get_by_name("rtp_source")
            if webrtc is None or source is None:
                pipeline.set_state(Gst.State.NULL)
                raise RuntimeError("WebRTC pipeline is missing required named elements")

            now = time.time()
            session_id = secrets.token_urlsafe(32)
            with self._condition:
                self._generation += 1
                generation = self._generation
                self._pipeline = pipeline
                self._webrtc = webrtc
                self._source = source
                self._settings = settings
                self._session_id = session_id
                self._running = True
                self._started_at = now
                self._last_packet_at = 0.0
                self._packet_count = 0
                self._byte_count = 0
                self._last_error = ""
                self._lease_deadline = time.monotonic() + FPV_LEASE_TIMEOUT_S
                self._offer_created = False
                self._offer_sdp = ""
                self._answer_set = False
                self._answer_digest = ""
                self._ice_gathering_state = "new"
                self._connection_state = "new"
                self._signaling_state = "stable"

            try:
                source_pad = source.get_static_pad("src")
                if source_pad is None:
                    raise RuntimeError("WebRTC RTP source pad is unavailable")
                self._source_probe_id = source_pad.add_probe(
                    Gst.PadProbeType.BUFFER | Gst.PadProbeType.BUFFER_LIST,
                    self._on_rtp_packet,
                    generation,
                )

                bus = pipeline.get_bus()
                if bus is not None:
                    bus.add_signal_watch()
                    self._bus_handler_id = bus.connect(
                        "message", self._on_bus_message, generation, pipeline
                    )
                    self._bus = bus
                webrtc.connect(
                    "notify::ice-gathering-state", self._on_webrtc_state, generation
                )
                webrtc.connect(
                    "notify::connection-state", self._on_webrtc_state, generation
                )
                webrtc.connect(
                    "notify::signaling-state", self._on_webrtc_state, generation
                )
                self._set_send_only(webrtc)

                state_result = pipeline.set_state(Gst.State.PLAYING)
                if state_result == Gst.StateChangeReturn.FAILURE:
                    raise RuntimeError("WebRTC pipeline failed to enter PLAYING state")
            except Exception:
                self._stop_locked(clear_error=True)
                raise

            threading.Thread(
                target=self._watch_lease,
                args=(session_id, generation),
                daemon=True,
                name="jetpilot-fpv-webrtc-lease",
            ).start()
            return self.status()

    def status(self) -> dict[str, object]:
        availability = self.availability()
        with self._condition:
            now = time.time()
            packet_age = (
                round(max(0.0, now - self._last_packet_at), 3)
                if self._last_packet_at
                else None
            )
            settings = self._settings.as_json() if self._settings else None
            phase = (
                "error"
                if self._last_error
                else "connected"
                if self._connection_state == "connected"
                else "connecting"
                if self._answer_set
                else "waiting-answer"
                if self._offer_created
                else "waiting-offer"
            )
            return {
                "available": bool(availability["available"]),
                "running": self._running,
                "session_id": self._session_id,
                "started_at": self._started_at or None,
                "last_frame_age_s": packet_age,
                "frame_count": self._packet_count,
                "jpeg_bytes": 0,
                "settings": settings,
                "last_error": self._last_error,
                "lease_remaining_s": (
                    round(max(0.0, self._lease_deadline - time.monotonic()), 3)
                    if self._running
                    else 0.0
                ),
                "transport": "webrtc",
                "packet_count": self._packet_count,
                "rtp_packet_count": self._packet_count,
                "rtp_bytes": self._byte_count,
                "last_packet_age_s": packet_age,
                "udp_packet_count": self._packet_count,
                "udp_bytes": self._byte_count,
                "last_udp_packet_age_s": packet_age,
                "webrtc": {
                    "available": bool(availability["available"]),
                    "error": str(availability.get("error") or ""),
                    "phase": phase,
                    "offer_created": self._offer_created,
                    "answer_set": self._answer_set,
                    "ice_gathering_state": self._ice_gathering_state,
                    "connection_state": self._connection_state,
                    "signaling_state": self._signaling_state,
                    "non_trickle": True,
                    "video_codec": "h264",
                    "transcoding": False,
                    "passthrough": True,
                },
                "availability": availability,
            }

    def current_session_id(self) -> str:
        with self._condition:
            return self._session_id if self._running else ""

    def session_is_running(self, session_id: str) -> bool:
        with self._condition:
            return bool(
                session_id
                and self._running
                and secrets.compare_digest(session_id, self._session_id)
            )

    def heartbeat(self, session_id: str) -> bool:
        with self._condition:
            if not (
                session_id
                and self._running
                and secrets.compare_digest(session_id, self._session_id)
            ):
                return False
            self._lease_deadline = time.monotonic() + FPV_LEASE_TIMEOUT_S
            self._condition.notify_all()
            return True

    def create_offer(self, session_id: str) -> dict[str, str]:
        with self._lifecycle_lock:
            webrtc, generation = self._require_session_locked(session_id)
            with self._condition:
                if self._answer_set:
                    raise RuntimeError("WebRTC answer is already set")
                if self._offer_created and self._offer_sdp:
                    return {
                        "type": "offer",
                        "sdp": self._offer_sdp,
                        "session_id": self._session_id,
                    }

            completed = threading.Event()
            result: dict[str, object] = {}

            def on_offer_created(promise: Any, *_: object) -> None:
                try:
                    promise_result = promise.wait()
                    if promise_result != Gst.PromiseResult.REPLIED:
                        raise RuntimeError(
                            f"webrtcbin offer promise ended as {_enum_name(promise_result)}"
                        )
                    reply = promise.get_reply()
                    if reply is not None and reply.has_field("error"):
                        raise RuntimeError(str(reply.get_value("error")))
                    offer = reply.get_value("offer") if reply is not None else None
                    if offer is None:
                        raise RuntimeError("webrtcbin returned no SDP offer")
                    local_promise = Gst.Promise.new()
                    webrtc.emit("set-local-description", offer, local_promise)
                    result["offer"] = offer
                except Exception as exc:  # noqa: BLE001 - crosses a GStreamer callback.
                    result["error"] = exc
                finally:
                    completed.set()

            promise = Gst.Promise.new_with_change_func(on_offer_created, None, None)
            webrtc.emit("create-offer", None, promise)
            if not completed.wait(_WEBRTC_OPERATION_TIMEOUT_S):
                raise RuntimeError("timed out while creating the WebRTC offer")
            if "error" in result:
                raise RuntimeError(f"could not create WebRTC offer: {result['error']}")
            self._require_session_locked(session_id, generation=generation)
            self._wait_for_ice_gathering_locked(session_id, generation)

            local_description = webrtc.get_property("local-description")
            if local_description is None:
                local_description = result.get("offer")
            sdp = self._description_text(local_description)
            try:
                _validate_answer_sdp("answer", sdp)
            except ValueError as exc:
                raise RuntimeError("webrtcbin returned an invalid SDP offer") from exc
            with self._condition:
                self._require_session_state(session_id, generation)
                self._offer_created = True
                self._offer_sdp = sdp
            return {"type": "offer", "sdp": sdp, "session_id": session_id}

    def set_answer(
        self,
        session_id: str,
        description_type: object,
        sdp: object,
    ) -> dict[str, object]:
        validated_sdp = _validate_answer_sdp(description_type, sdp)
        digest = hashlib.sha256(validated_sdp.encode("utf-8")).hexdigest()
        with self._lifecycle_lock:
            webrtc, generation = self._require_session_locked(session_id)
            with self._condition:
                if not self._offer_created:
                    raise RuntimeError("create the WebRTC offer before setting an answer")
                if self._answer_set:
                    if secrets.compare_digest(digest, self._answer_digest):
                        return self.status()
                    raise RuntimeError("a different WebRTC answer is already set")

            parse_result, message = GstSdp.SDPMessage.new()
            if parse_result != GstSdp.SDPResult.OK or message is None:
                raise RuntimeError("could not allocate an SDP message")
            parse_result = GstSdp.sdp_message_parse_buffer(
                validated_sdp.encode("utf-8"), message
            )
            if parse_result != GstSdp.SDPResult.OK:
                raise ValueError("sdp could not be parsed")
            answer = GstWebRTC.WebRTCSessionDescription.new(
                GstWebRTC.WebRTCSDPType.ANSWER, message
            )

            completed = threading.Event()
            answer_result: dict[str, object] = {}

            def on_answer_set(promise: Any, *unused: object) -> None:
                del unused
                try:
                    promise_result = promise.wait()
                    if promise_result != Gst.PromiseResult.REPLIED:
                        raise RuntimeError(
                            f"webrtcbin answer promise ended as {_enum_name(promise_result)}"
                        )
                    reply = promise.get_reply()
                    if reply is not None and reply.has_field("error"):
                        raise RuntimeError(str(reply.get_value("error")))
                except Exception as exc:  # noqa: BLE001 - crosses a GStreamer callback.
                    answer_result["error"] = exc
                finally:
                    completed.set()

            promise = Gst.Promise.new_with_change_func(on_answer_set, None, None)
            webrtc.emit("set-remote-description", answer, promise)
            if not completed.wait(_WEBRTC_OPERATION_TIMEOUT_S):
                raise RuntimeError("timed out while setting the WebRTC answer")
            if "error" in answer_result:
                raise RuntimeError(f"could not set WebRTC answer: {answer_result['error']}")
            self._require_session_locked(session_id, generation=generation)
            with self._condition:
                self._answer_set = True
                self._answer_digest = digest
            return self.status()

    def stop(self, session_id: str | None = None) -> bool:
        with self._lifecycle_lock:
            return self._stop_locked(session_id=session_id, clear_error=True)

    def shutdown(self) -> None:
        with self._lifecycle_lock:
            self._stop_locked(clear_error=True)
            loop = self._main_loop
            thread = self._main_loop_thread
            self._main_loop = None
            self._main_loop_thread = None
            if loop is not None:
                loop.quit()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)

    def _stop_locked(
        self,
        session_id: str | None = None,
        *,
        clear_error: bool,
    ) -> bool:
        with self._condition:
            if session_id is not None and not (
                self._session_id
                and secrets.compare_digest(session_id, self._session_id)
            ):
                return False
            pipeline = self._pipeline
            source = self._source
            source_probe_id = self._source_probe_id
            bus = self._bus
            bus_handler_id = self._bus_handler_id
            was_running = self._running or pipeline is not None
            self._generation += 1
            self._pipeline = None
            self._webrtc = None
            self._source = None
            self._source_probe_id = 0
            self._bus = None
            self._bus_handler_id = 0
            self._running = False
            self._lease_deadline = 0.0
            self._offer_created = False
            self._offer_sdp = ""
            self._answer_set = False
            self._answer_digest = ""
            self._ice_gathering_state = "closed"
            self._connection_state = "closed"
            self._signaling_state = "closed"
            if clear_error:
                self._last_error = ""
            self._condition.notify_all()

        if source is not None and source_probe_id:
            source_pad = source.get_static_pad("src")
            if source_pad is not None:
                try:
                    source_pad.remove_probe(source_probe_id)
                except Exception:  # noqa: BLE001 - element may already be disposed.
                    pass
        if bus is not None:
            if bus_handler_id:
                try:
                    bus.disconnect(bus_handler_id)
                except Exception:  # noqa: BLE001 - bus may already be disposed.
                    pass
            try:
                bus.remove_signal_watch()
            except Exception:  # noqa: BLE001 - signal watch may not have been installed.
                pass
        if pipeline is not None:
            pipeline.set_state(Gst.State.NULL)
        return was_running

    def _ensure_main_loop_locked(self) -> None:
        if self._main_loop_thread is not None and self._main_loop_thread.is_alive():
            return
        self._main_loop = GLib.MainLoop.new(None, False)
        self._main_loop_thread = threading.Thread(
            target=self._main_loop.run,
            daemon=True,
            name="jetpilot-fpv-webrtc-glib",
        )
        self._main_loop_thread.start()

    def _require_session_locked(
        self, session_id: str, *, generation: int | None = None
    ) -> tuple[Any, int]:
        with self._condition:
            active_generation = self._generation if generation is None else generation
            self._require_session_state(session_id, active_generation)
            return self._webrtc, active_generation

    def _require_session_state(self, session_id: str, generation: int) -> None:
        if not session_id:
            raise ValueError("session_id is required")
        if not (
            self._running
            and generation == self._generation
            and secrets.compare_digest(session_id, self._session_id)
            and self._webrtc is not None
        ):
            raise RuntimeError("WebRTC session is not running or is stale")

    def _wait_for_ice_gathering_locked(self, session_id: str, generation: int) -> None:
        deadline = time.monotonic() + _WEBRTC_OPERATION_TIMEOUT_S
        with self._condition:
            while True:
                self._require_session_state(session_id, generation)
                state = self._ice_gathering_state
                if state == "complete":
                    return
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError("timed out while gathering WebRTC ICE candidates")
                self._condition.wait(min(remaining, 0.25))

    @staticmethod
    def _description_text(description: Any) -> str:
        sdp_message = getattr(description, "sdp", None)
        if sdp_message is None:
            raise RuntimeError("WebRTC description does not contain SDP")
        return str(sdp_message.as_text())

    @staticmethod
    def _set_send_only(webrtc: Any) -> None:
        try:
            transceivers = webrtc.emit("get-transceivers") or []
            for transceiver in transceivers:
                transceiver.set_property(
                    "direction",
                    GstWebRTC.WebRTCRTPTransceiverDirection.SENDONLY,
                )
        except Exception:  # noqa: BLE001 - older webrtcbin versions infer send-only from pads.
            pass

    def _on_rtp_packet(self, _pad: Any, info: Any, generation: int) -> Any:
        byte_count = 0
        packet_count = 0
        buffer = info.get_buffer()
        if buffer is not None:
            packet_count = 1
            byte_count = int(buffer.get_size())
        else:
            buffer_list = info.get_buffer_list()
            if buffer_list is not None:
                packet_count = int(buffer_list.length())
                for index in range(packet_count):
                    item = buffer_list.get(index)
                    if item is not None:
                        byte_count += int(item.get_size())
        if packet_count:
            with self._condition:
                if generation == self._generation and self._running:
                    self._packet_count += packet_count
                    self._byte_count += byte_count
                    self._last_packet_at = time.time()
                    self._condition.notify_all()
        return Gst.PadProbeReturn.OK

    def _on_webrtc_state(self, webrtc: Any, _property: Any, generation: int) -> None:
        with self._condition:
            if generation != self._generation or webrtc is not self._webrtc:
                return
            self._ice_gathering_state = _enum_name(
                webrtc.get_property("ice-gathering-state")
            )
            self._connection_state = _enum_name(webrtc.get_property("connection-state"))
            self._signaling_state = _enum_name(webrtc.get_property("signaling-state"))
            self._condition.notify_all()

    def _on_bus_message(
        self, _bus: Any, message: Any, generation: int, pipeline: Any
    ) -> None:
        error_text = ""
        if message.type == Gst.MessageType.ERROR:
            error, debug = message.parse_error()
            error_text = str(error)
            if debug:
                error_text = f"{error_text}\n{debug}"
        elif message.type == Gst.MessageType.EOS:
            error_text = "WebRTC pipeline reached end of stream"
        if not error_text:
            return
        with self._condition:
            if generation != self._generation or pipeline is not self._pipeline:
                return
            self._last_error = error_text
            self._running = False
            self._lease_deadline = 0.0
            self._condition.notify_all()
        threading.Thread(
            target=self._fail_pipeline,
            args=(generation, pipeline),
            daemon=True,
            name="jetpilot-fpv-webrtc-error",
        ).start()

    def _fail_pipeline(self, generation: int, pipeline: Any) -> None:
        with self._lifecycle_lock:
            with self._condition:
                if generation != self._generation or pipeline is not self._pipeline:
                    return
            self._stop_locked(clear_error=False)

    def _watch_lease(self, session_id: str, generation: int) -> None:
        while True:
            with self._condition:
                if (
                    generation != self._generation
                    or not self._running
                    or not secrets.compare_digest(session_id, self._session_id)
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
                        or not secrets.compare_digest(session_id, self._session_id)
                    ):
                        return
                    if self._lease_deadline > time.monotonic():
                        continue
                self._stop_locked(session_id=session_id, clear_error=True)
                return
