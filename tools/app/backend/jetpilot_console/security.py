from __future__ import annotations

import ipaddress
import json
import os
import re
import tempfile
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any
from urllib.parse import urlsplit


MAX_JSON_BODY_BYTES = 1024 * 1024
JOY_PROFILE_FILENAMES = frozenset(
    {
        "joy_profile.yaml",
        "teleop_cmd.param.yaml",
        "joy_button_mapping.param.yaml",
        "serial_reader_node.param.yaml",
        "pca9685_rc_driver_node.param.yaml",
    }
)
_SSH_USER_PATTERN = re.compile(r"[A-Za-z0-9_][A-Za-z0-9_.-]{0,63}")
_HOSTNAME_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9.-]{0,252}")


@dataclass(frozen=True)
class RequestRejected(ValueError):
    status: int
    message: str

    def __str__(self) -> str:
        return self.message


def env_flag(value: str | None, *, default: bool = False) -> bool:
    if value is None:
        return default
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off", ""}:
        return False
    return default


def is_loopback_bind(host: str) -> bool:
    normalized = host.strip().lower()
    if normalized == "localhost":
        return True
    if normalized.startswith("[") and normalized.endswith("]"):
        normalized = normalized[1:-1]
    try:
        return ipaddress.ip_address(normalized).is_loopback
    except ValueError:
        return False


def validate_request_host(host_header: str | None, *, loopback_only: bool) -> None:
    if not host_header:
        raise RequestRejected(400, "Host header is required")
    try:
        parsed = urlsplit(f"//{host_header}")
        hostname = parsed.hostname
        _ = parsed.port
    except ValueError:
        raise RequestRejected(400, "invalid Host header") from None
    if (
        not hostname
        or parsed.username is not None
        or parsed.password is not None
        or parsed.path
        or parsed.query
        or parsed.fragment
    ):
        raise RequestRejected(400, "invalid Host header")
    if not loopback_only:
        return

    if hostname.lower() == "localhost":
        return
    try:
        if ipaddress.ip_address(hostname).is_loopback:
            return
    except ValueError:
        pass
    raise RequestRejected(403, "Host must be localhost or a loopback IP address")


def resolve_under_root(
    value: str | Path,
    root: Path,
    *,
    label: str = "path",
    require_exists: bool = False,
    require_directory: bool = False,
) -> Path:
    root_path = Path(root).expanduser().resolve(strict=False)
    candidate = Path(value).expanduser()
    if not candidate.is_absolute():
        candidate = root_path / candidate
    resolved = candidate.resolve(strict=False)
    try:
        resolved.relative_to(root_path)
    except ValueError:
        raise ValueError(f"{label} must be under {root_path}") from None

    if require_exists and not resolved.exists():
        raise ValueError(f"{label} does not exist: {resolved}")
    if require_directory and resolved.exists() and not resolved.is_dir():
        raise ValueError(f"{label} must be a directory: {resolved}")
    return resolved


def validate_ssh_target(user: str, host: str) -> str:
    normalized_user = user.strip()
    normalized_host = host.strip()
    if not _SSH_USER_PATTERN.fullmatch(normalized_user):
        raise ValueError("SSH user contains unsupported characters")
    if normalized_host.startswith("[") and normalized_host.endswith("]"):
        normalized_host = normalized_host[1:-1]

    try:
        address = ipaddress.ip_address(normalized_host)
        rendered_host = f"[{address.compressed}]" if address.version == 6 else address.compressed
    except ValueError:
        if not _HOSTNAME_PATTERN.fullmatch(normalized_host):
            raise ValueError("SSH host is not a valid IP address or hostname") from None
        labels = normalized_host.split(".")
        if any(not label or label.startswith("-") or label.endswith("-") for label in labels):
            raise ValueError("SSH host is not a valid IP address or hostname")
        rendered_host = normalized_host
    return f"{normalized_user}@{rendered_host}"


def validate_remote_absolute_path(value: str, *, label: str) -> str:
    if not value or "\0" in value or "\n" in value or "\r" in value:
        raise ValueError(f"{label} must be a non-empty absolute path")
    path = PurePosixPath(value)
    if not path.is_absolute() or ".." in path.parts:
        raise ValueError(f"{label} must be an absolute path without '..'")
    return value


def same_origin_allowed(host_header: str | None, origin_header: str | None) -> bool:
    """Allow requests without Origin (CLI clients) or an exact HTTP(S) origin."""

    if not origin_header:
        return True
    if not host_header:
        return False

    try:
        parsed = urlsplit(origin_header)
        if parsed.scheme not in {"http", "https"} or not parsed.hostname:
            return False
        if (
            parsed.username is not None
            or parsed.password is not None
            or parsed.path
            or parsed.query
            or parsed.fragment
        ):
            return False

        host = urlsplit(f"//{host_header}")
        if not host.hostname:
            return False

        origin_port = parsed.port or (443 if parsed.scheme == "https" else 80)
        host_port = host.port or (443 if parsed.scheme == "https" else 80)
        return parsed.hostname.lower() == host.hostname.lower() and origin_port == host_port
    except ValueError:
        return False


def validate_json_request_headers(
    *,
    content_type: str | None,
    content_length: str | None,
    transfer_encoding: str | None,
    host: str | None,
    origin: str | None,
    maximum_bytes: int = MAX_JSON_BODY_BYTES,
) -> int:
    if transfer_encoding:
        raise RequestRejected(400, "Transfer-Encoding is not supported")
    if not same_origin_allowed(host, origin):
        raise RequestRejected(403, "cross-origin POST requests are not allowed")

    media_type = (content_type or "").partition(";")[0].strip().lower()
    if media_type != "application/json":
        raise RequestRejected(415, "Content-Type must be application/json")
    if content_length is None:
        raise RequestRejected(411, "Content-Length is required")
    try:
        length = int(content_length, 10)
    except (TypeError, ValueError):
        raise RequestRejected(400, "invalid Content-Length") from None
    if length < 0:
        raise RequestRejected(400, "invalid Content-Length")
    if length > maximum_bytes:
        raise RequestRejected(413, f"JSON request body exceeds {maximum_bytes} bytes")
    return length


def decode_json_object(data: bytes) -> dict[str, Any]:
    try:
        value = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        raise RequestRejected(400, "request body must be valid UTF-8 JSON") from None
    if not isinstance(value, dict):
        raise RequestRejected(400, "JSON request body must be an object")
    return value


def _validated_joy_files(files: object) -> dict[str, str]:
    if not isinstance(files, dict) or not files:
        raise ValueError("files must be a non-empty object")

    validated: dict[str, str] = {}
    for name, content in files.items():
        if not isinstance(name, str) or not isinstance(content, str):
            raise ValueError("file names and contents must be strings")
        if name not in JOY_PROFILE_FILENAMES:
            allowed = ", ".join(sorted(JOY_PROFILE_FILENAMES))
            raise ValueError(f"unsupported Joy profile file: {name!r}; allowed: {allowed}")
        validated[name] = content
    return validated


def save_joy_profile_files(output_root: Path, files: object) -> list[str]:
    """Atomically save the fixed Joy YAML set without following output symlinks."""

    validated = _validated_joy_files(files)
    output_root = Path(output_root)
    if output_root.is_symlink():
        raise ValueError("Joy profile output directory must not be a symlink")
    output_root.mkdir(parents=True, exist_ok=True)
    if output_root.is_symlink() or not output_root.is_dir():
        raise ValueError("Joy profile output path must be a directory")
    resolved_root = output_root.resolve(strict=True)

    targets: dict[str, Path] = {}
    for name in validated:
        target = resolved_root / name
        if target.is_symlink():
            raise ValueError(f"refusing to replace symlink: {name}")
        if target.exists() and not target.is_file():
            raise ValueError(f"Joy profile output is not a regular file: {name}")
        if target.resolve(strict=False).parent != resolved_root:
            raise ValueError(f"Joy profile output escapes the configured directory: {name}")
        targets[name] = target

    saved: list[str] = []
    temporary_paths: list[Path] = []
    try:
        for name, content in validated.items():
            file_descriptor, temporary_name = tempfile.mkstemp(
                dir=resolved_root,
                prefix=f".{name}.",
                suffix=".tmp",
            )
            temporary_path = Path(temporary_name)
            temporary_paths.append(temporary_path)
            try:
                os.fchmod(file_descriptor, 0o644)
                with os.fdopen(file_descriptor, "w", encoding="utf-8", newline="") as handle:
                    handle.write(content)
                    handle.flush()
                    os.fsync(handle.fileno())
            except Exception:
                try:
                    os.close(file_descriptor)
                except OSError:
                    pass
                raise
            os.replace(temporary_path, targets[name])
            temporary_paths.remove(temporary_path)
            saved.append(str(targets[name]))
    finally:
        for temporary_path in temporary_paths:
            try:
                temporary_path.unlink()
            except FileNotFoundError:
                pass

    return saved
