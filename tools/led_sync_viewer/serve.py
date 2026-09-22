#!/usr/bin/env python3
"""Serve the LED sync UI and approved JSON files from a Docker workspace."""

from __future__ import annotations

import argparse
import json
import mimetypes
import shutil
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse


def _inside(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
    except ValueError:
        return False
    return True


def build_handler(html_path: Path, record_root: Path):
    class Handler(BaseHTTPRequestHandler):
        server_version = "LedSyncInspector/1.0"

        def _headers(self, status: HTTPStatus, content_type: str, length: int) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(length))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Referrer-Policy", "no-referrer")
            self.end_headers()

        def _send_bytes(
            self,
            payload: bytes,
            *,
            status: HTTPStatus = HTTPStatus.OK,
            content_type: str = "application/octet-stream",
        ) -> None:
            self._headers(status, content_type, len(payload))
            self.wfile.write(payload)

        def _send_json(self, value, status: HTTPStatus = HTTPStatus.OK) -> None:
            payload = json.dumps(value, ensure_ascii=False).encode("utf-8")
            self._send_bytes(
                payload,
                status=status,
                content_type="application/json; charset=utf-8",
            )

        def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
            request = urlparse(self.path)
            if request.path in {"/", "/index.html"}:
                payload = html_path.read_bytes()
                self._send_bytes(payload, content_type="text/html; charset=utf-8")
                return
            if request.path == "/health":
                self._send_json({"status": "ok", "record_root": str(record_root)})
                return
            if request.path == "/api/data":
                values = parse_qs(request.query).get("path", [])
                if len(values) != 1 or not values[0].strip():
                    self._send_json(
                        {"error": "query parameter path is required"},
                        HTTPStatus.BAD_REQUEST,
                    )
                    return
                requested = Path(values[0])
                candidate = (
                    requested if requested.is_absolute() else record_root / requested
                ).resolve()
                if not _inside(candidate, record_root):
                    self._send_json(
                        {"error": f"path must be inside {record_root}"},
                        HTTPStatus.FORBIDDEN,
                    )
                    return
                if candidate.suffix.lower() != ".json":
                    self._send_json(
                        {"error": "only JSON files may be loaded"},
                        HTTPStatus.BAD_REQUEST,
                    )
                    return
                if not candidate.is_file():
                    self._send_json(
                        {"error": f"file not found: {candidate}"},
                        HTTPStatus.NOT_FOUND,
                    )
                    return
                content_type = mimetypes.guess_type(candidate.name)[0] or "application/json"
                size = candidate.stat().st_size
                self._headers(HTTPStatus.OK, f"{content_type}; charset=utf-8", size)
                with candidate.open("rb") as stream:
                    shutil.copyfileobj(stream, self.wfile)
                return
            if request.path == "/api/file":
                values = parse_qs(request.query).get("path", [])
                if len(values) != 1 or not values[0].strip():
                    self._send_json(
                        {"error": "query parameter path is required"},
                        HTTPStatus.BAD_REQUEST,
                    )
                    return
                requested = Path(values[0])
                candidate = (
                    requested if requested.is_absolute() else record_root / requested
                ).resolve()
                if not _inside(candidate, record_root):
                    self._send_json(
                        {"error": f"path must be inside {record_root}"},
                        HTTPStatus.FORBIDDEN,
                    )
                    return
                if candidate.suffix.lower() not in {".jpg", ".jpeg", ".png", ".webp"}:
                    self._send_json(
                        {"error": "only preview image files may be loaded"},
                        HTTPStatus.BAD_REQUEST,
                    )
                    return
                if not candidate.is_file():
                    self._send_json(
                        {"error": f"file not found: {candidate}"},
                        HTTPStatus.NOT_FOUND,
                    )
                    return
                content_type = mimetypes.guess_type(candidate.name)[0] or "application/octet-stream"
                size = candidate.stat().st_size
                self._headers(HTTPStatus.OK, content_type, size)
                with candidate.open("rb") as stream:
                    shutil.copyfileobj(stream, self.wfile)
                return
            self._send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)

        def log_message(self, format: str, *args) -> None:
            print(f"[led-sync-gui] {self.address_string()} {format % args}")

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--record-root", default="/workspaces/record")
    parser.add_argument(
        "--html",
        default=str(Path(__file__).resolve().parent / "dist" / "index.html"),
    )
    args = parser.parse_args()

    html_path = Path(args.html).resolve()
    record_root = Path(args.record_root).resolve()
    if not html_path.is_file():
        parser.error(f"HTML file not found: {html_path}")
    if not record_root.is_dir():
        parser.error(f"record root not found: {record_root}")
    if not (1 <= args.port <= 65535):
        parser.error("--port must be between 1 and 65535")

    server = ThreadingHTTPServer(
        (args.host, args.port),
        build_handler(html_path, record_root),
    )
    print(f"LED Sync Inspector: http://localhost:{args.port}")
    print(f"JSON access root: {record_root}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nLED Sync Inspector stopped")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
