from __future__ import annotations

import threading
from typing import Any

from .fpv_stream import FpvStreamManager, FpvStreamSettings
from .fpv_webrtc import WebRtcStreamManager


class FpvReceiverManager:
    """Select exactly one browser transport while preserving the FPV API contract."""

    def __init__(
        self,
        *,
        allow_webrtc: bool = True,
        mjpeg_manager: FpvStreamManager | None = None,
        webrtc_manager: WebRtcStreamManager | None = None,
    ) -> None:
        self._lock = threading.RLock()
        self._mjpeg = mjpeg_manager or FpvStreamManager()
        self._webrtc = webrtc_manager or WebRtcStreamManager(allow=allow_webrtc)
        self._active_transport = ""
        self._last_status: dict[str, object] | None = None

    def start(self, settings: FpvStreamSettings) -> dict[str, object]:
        with self._lock:
            self._stop_active()
            manager = self._webrtc if settings.transport == "webrtc" else self._mjpeg
            status = manager.start(settings)
            self._active_transport = settings.transport
            self._last_status = status
            return self._merge_status(status)

    def stop(self, session_id: str | None = None) -> bool:
        with self._lock:
            manager = self._active_manager()
            if manager is None:
                return False
            current_session = self._status_session_id(manager.status())
            stopped = manager.stop(session_id)
            if session_id is None or session_id == current_session:
                self._last_status = manager.status()
                self._active_transport = ""
            return stopped

    def heartbeat(self, session_id: str) -> bool:
        with self._lock:
            manager = self._active_manager()
            return bool(manager and manager.heartbeat(session_id))

    def status(self) -> dict[str, object]:
        with self._lock:
            manager = self._active_manager()
            if manager is not None:
                status = manager.status()
            elif self._last_status is not None:
                status = dict(self._last_status)
            else:
                status = self._mjpeg.status()
            return self._merge_status(status)

    def create_offer(self, session_id: str) -> dict[str, object]:
        with self._lock:
            if self._active_transport != "webrtc":
                raise RuntimeError("the active FPV receiver is not using WebRTC")
            return self._webrtc.create_offer(session_id)

    def set_answer(
        self,
        session_id: str,
        description_type: str,
        sdp: str,
    ) -> dict[str, object]:
        with self._lock:
            if self._active_transport != "webrtc":
                raise RuntimeError("the active FPV receiver is not using WebRTC")
            result = self._webrtc.set_answer(session_id, description_type, sdp)
            if isinstance(result, dict):
                self._last_status = result
                return self._merge_status(result)
            return self.status()

    def current_session_id(self) -> str:
        with self._lock:
            manager = self._active_manager()
            return manager.current_session_id() if manager else ""

    def session_is_running(self, session_id: str) -> bool:
        with self._lock:
            manager = self._active_manager()
            return bool(manager and manager.session_is_running(session_id))

    def wait_for_frame(
        self,
        session_id: str,
        after_sequence: int,
        *,
        timeout: float = 2.0,
    ) -> tuple[int, bytes] | None:
        with self._lock:
            if self._active_transport != "mjpeg":
                return None
            manager = self._mjpeg
        return manager.wait_for_frame(session_id, after_sequence, timeout=timeout)

    def stream_is_mjpeg(self, session_id: str) -> bool:
        with self._lock:
            return (
                self._active_transport == "mjpeg"
                and self._mjpeg.session_is_running(session_id)
            )

    def shutdown(self) -> None:
        with self._lock:
            self._stop_active()
            self._webrtc.shutdown()

    def _active_manager(self) -> Any | None:
        if self._active_transport == "webrtc":
            return self._webrtc
        if self._active_transport == "mjpeg":
            return self._mjpeg
        return None

    def _stop_active(self) -> None:
        manager = self._active_manager()
        if manager is not None:
            manager.stop()
            self._last_status = manager.status()
        self._active_transport = ""

    def _merge_status(self, status: dict[str, object]) -> dict[str, object]:
        merged = dict(status)
        merged["transport"] = self._active_transport or None
        availability = self._webrtc.availability()
        details = merged.get("webrtc")
        if isinstance(details, dict):
            details = {**availability, **details}
        else:
            details = availability
        merged["webrtc"] = details
        return merged

    @staticmethod
    def _status_session_id(status: dict[str, object]) -> str:
        return str(status.get("session_id") or "")
