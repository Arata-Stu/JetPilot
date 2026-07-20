from __future__ import annotations

import fcntl
import json
import math
import mimetypes
import os
import re
import shlex
import uuid
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterator, Mapping

from .config import ConsoleConfig
from .preflight import parse_rosbag_metadata
from .security import resolve_under_root


ANALYSIS_FORMAT_VERSION = 1
MAX_TIMELINE_BYTES = 64 * 1024 * 1024
ANALYSIS_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")
FRAME_EXTENSIONS = frozenset({".jpg", ".jpeg", ".png", ".webp"})


def _q(value: str | Path) -> str:
    return shlex.quote(str(value))


def _now() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


@contextmanager
def _json_lock(path: Path) -> Iterator[None]:
    path.parent.mkdir(parents=True, exist_ok=True)
    lock_path = path.with_name(f".{path.name}.lock")
    with lock_path.open("a+b") as lock_file:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)


def _write_json_unlocked(path: Path, value: Mapping[str, Any]) -> None:
    temporary = path.with_name(
        f".{path.name}.{os.getpid()}.{uuid.uuid4().hex}.tmp"
    )
    try:
        temporary.write_text(
            json.dumps(dict(value), ensure_ascii=True, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def _atomic_json(path: Path, value: Mapping[str, Any]) -> None:
    with _json_lock(path):
        _write_json_unlocked(path, value)


def _update_json_object(
    path: Path,
    update: Callable[[dict[str, Any]], Mapping[str, Any]],
) -> dict[str, Any]:
    """Read, merge, and atomically replace one JSON object under one lock."""

    with _json_lock(path):
        current: dict[str, Any] = {}
        if path.is_file():
            try:
                loaded = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                raise ValueError(str(exc)) from exc
            if not isinstance(loaded, dict):
                raise ValueError("JSON root must be an object")
            current = loaded
        updated = dict(update(dict(current)))
        _write_json_unlocked(path, updated)
        return updated


def _optional_integer(text: str, pattern: str) -> int | None:
    match = re.search(pattern, text, flags=re.MULTILINE)
    return int(match.group(1)) if match else None


def rosbag_detail(config: ConsoleConfig, value: str | Path) -> dict[str, Any]:
    """Return metadata-only bag details without opening rosbag storage.

    The directory and every referenced storage file remain constrained to
    ``RECORD_ROOT``. Nanosecond epoch values are strings because JavaScript
    cannot represent them exactly.
    """

    bag_dir = resolve_under_root(
        value,
        config.record_root,
        label="rosbag",
        require_exists=True,
        require_directory=True,
    )
    metadata_path = bag_dir / "metadata.yaml"
    if not metadata_path.is_file():
        raise FileNotFoundError(f"rosbag metadata not found: {metadata_path}")
    text = metadata_path.read_text(encoding="utf-8", errors="replace")
    if "rosbag2_bagfile_information:" not in text:
        raise ValueError("metadata.yaml does not contain rosbag2 bag information")

    metadata = parse_rosbag_metadata(text)
    topics = metadata.get("topics")
    if not isinstance(topics, dict):
        topics = {}

    relative_paths = metadata.get("relative_file_paths")
    if not isinstance(relative_paths, list):
        relative_paths = []
    storage_paths: list[Path] = []
    for raw_path in relative_paths:
        candidate = resolve_under_root(
            bag_dir / str(raw_path),
            bag_dir,
            label="rosbag storage file",
        )
        storage_paths.append(candidate)
    if not storage_paths:
        storage_paths = sorted([*bag_dir.glob("*.mcap"), *bag_dir.glob("*.db3")])

    duration_ns = _optional_integer(
        text,
        r"^\s*duration:\s*$\n\s*nanoseconds:\s*(\d+)\s*$",
    )
    starting_time_ns = _optional_integer(
        text,
        r"^\s*starting_time:\s*$\n\s*nanoseconds_since_epoch:\s*(\d+)\s*$",
    )
    # The bag-level field is conventionally indented two spaces. Requiring the
    # exact indentation avoids accidentally returning a per-topic count.
    message_count = _optional_integer(text, r"^  message_count:\s*(\d+)\s*$")

    topic_rows = []
    for name, raw_topic in sorted(topics.items()):
        topic = raw_topic if isinstance(raw_topic, dict) else {}
        topic_rows.append(
            {
                "name": str(name),
                "type": str(topic.get("type") or ""),
                "serialization_format": str(topic.get("serialization_format") or ""),
                "message_count": topic.get("message_count"),
            }
        )

    storage_files = []
    total_size = 0
    for path in storage_paths:
        exists = path.is_file()
        size = path.stat().st_size if exists else 0
        total_size += size
        storage_files.append(
            {
                "name": path.name,
                "path": str(path),
                "exists": exists,
                "size_bytes": size,
            }
        )

    return {
        "name": bag_dir.name,
        "path": str(bag_dir),
        "metadata_path": str(metadata_path),
        "storage_identifier": str(metadata.get("storage_identifier") or ""),
        "storage_files": storage_files,
        "size_bytes": total_size,
        "duration_ns": str(duration_ns) if duration_ns is not None else None,
        "duration_seconds": duration_ns / 1.0e9 if duration_ns is not None else None,
        "starting_time_ns": str(starting_time_ns) if starting_time_ns is not None else None,
        "message_count": message_count,
        "topic_count": len(topic_rows),
        "topics": topic_rows,
    }


def _safe_slug(value: str, fallback: str = "analysis") -> str:
    slug = re.sub(r"[^A-Za-z0-9_.-]+", "-", value.strip()).strip(".-_")
    return (slug or fallback)[:48]


class AnalysisRepository:
    def __init__(self, root: Path):
        root_path = Path(root).expanduser()
        if root_path.is_symlink():
            raise ValueError("analysis root must not be a symlink")
        root_path.mkdir(parents=True, exist_ok=True)
        self.root = root_path.resolve(strict=True)

    def _directory(self, analysis_id: str, *, require_exists: bool = True) -> Path:
        if not ANALYSIS_ID_PATTERN.fullmatch(analysis_id):
            raise ValueError("invalid analysis id")
        path = resolve_under_root(
            analysis_id,
            self.root,
            label="analysis",
            require_exists=require_exists,
            require_directory=True,
        )
        if require_exists and not path.is_dir():
            raise FileNotFoundError(f"analysis not found: {analysis_id}")
        return path

    @staticmethod
    def _read_optional_json(path: Path) -> tuple[dict[str, Any], str]:
        if not path.is_file():
            return {}, ""
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            return {}, str(exc)
        if not isinstance(value, dict):
            return {}, "JSON root must be an object"
        return value, ""

    def create(
        self,
        *,
        label: str,
        request: Mapping[str, Any],
        preflight: Mapping[str, Any],
        initial_phase: str,
    ) -> dict[str, Any]:
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        analysis_id = f"{stamp}-{_safe_slug(label)}-{uuid.uuid4().hex[:8]}"
        directory = self._directory(analysis_id, require_exists=False)
        directory.mkdir(mode=0o755, parents=False, exist_ok=False)

        resolved = preflight.get("resolved")
        manifest = {
            "format_version": ANALYSIS_FORMAT_VERSION,
            "analysis_id": analysis_id,
            "created_at": _now(),
            "label": label,
            "request": dict(request),
            "resolved": dict(resolved) if isinstance(resolved, Mapping) else {},
            "timeline": "timeline.json",
            "frames_root": "frames",
        }
        status = {
            "analysis_id": analysis_id,
            "status": "queued",
            "phase": initial_phase,
            "progress": 0.0,
            "message": "Analysis task is queued.",
            "updated_at": _now(),
        }
        _atomic_json(directory / "manifest.json", manifest)
        _atomic_json(directory / "status.json", status)
        return self.detail(analysis_id)

    def list(self) -> list[dict[str, Any]]:
        analyses = []
        for path in self.root.iterdir():
            if not path.is_dir() or path.is_symlink() or not ANALYSIS_ID_PATTERN.fullmatch(path.name):
                continue
            try:
                analyses.append(self.detail(path.name))
            except (FileNotFoundError, ValueError):
                continue
        return sorted(
            analyses,
            key=lambda item: str(item.get("manifest", {}).get("created_at") or item["analysis_id"]),
            reverse=True,
        )

    def attach_task(self, analysis_id: str, task: Mapping[str, Any]) -> dict[str, Any]:
        directory = self._directory(analysis_id)
        task_id = str(task.get("task_id") or "")
        try:
            _update_json_object(
                directory / "manifest.json",
                lambda current: {**current, "task_id": task_id},
            )
        except ValueError as exc:
            raise ValueError(f"analysis manifest is invalid: {exc}") from exc
        try:
            _update_json_object(
                directory / "status.json",
                lambda current: {
                    **current,
                    "task_id": task_id,
                    # `create()` already writes queued. Do not copy an older
                    # TaskManager state over worker running/completed/failed.
                    "status": str(current.get("status") or task.get("status") or "queued"),
                    "updated_at": str(current.get("updated_at") or _now()),
                },
            )
        except ValueError as exc:
            raise ValueError(f"analysis status is invalid: {exc}") from exc
        return self.detail(analysis_id)

    def update_status(
        self,
        analysis_id: str,
        *,
        status: str,
        phase: str,
        progress: float,
        message: str,
    ) -> dict[str, Any]:
        """Merge a terminal wrapper/API state without discarding worker fields."""

        if not math.isfinite(progress):
            raise ValueError("analysis progress must be finite")
        directory = self._directory(analysis_id)
        try:
            _update_json_object(
                directory / "status.json",
                lambda current: {
                    **current,
                    "analysis_id": analysis_id,
                    "status": str(status),
                    "stage": str(phase),
                    "phase": str(phase),
                    "progress": max(0.0, min(1.0, float(progress))),
                    "message": str(message),
                    "updated_at": _now(),
                },
            )
        except ValueError as exc:
            raise ValueError(f"analysis status is invalid: {exc}") from exc
        return self.detail(analysis_id)

    def detail(self, analysis_id: str) -> dict[str, Any]:
        directory = self._directory(analysis_id)
        manifest, manifest_error = self._read_optional_json(directory / "manifest.json")
        status, status_error = self._read_optional_json(directory / "status.json")
        timeline_path = directory / "timeline.json"
        frames_path = directory / "frames"
        return {
            "analysis_id": analysis_id,
            "path": str(directory),
            "manifest": manifest,
            "status": status,
            "manifest_error": manifest_error,
            "status_error": status_error,
            "timeline_available": timeline_path.is_file(),
            "timeline_size_bytes": timeline_path.stat().st_size if timeline_path.is_file() else 0,
            "frames_available": frames_path.is_dir(),
        }

    def timeline(self, analysis_id: str) -> dict[str, Any]:
        directory = self._directory(analysis_id)
        path = resolve_under_root(
            "timeline.json",
            directory,
            label="analysis timeline",
            require_exists=True,
        )
        if not path.is_file():
            raise FileNotFoundError(f"timeline not found for analysis: {analysis_id}")
        size = path.stat().st_size
        if size > MAX_TIMELINE_BYTES:
            raise ValueError(
                f"timeline is too large for the JSON endpoint ({size} bytes); maximum is {MAX_TIMELINE_BYTES}"
            )
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            raise ValueError(f"analysis timeline is invalid JSON: {exc}") from exc
        if not isinstance(value, dict):
            raise ValueError("analysis timeline root must be an object")
        return value

    def frame(self, analysis_id: str, relative_path: str) -> tuple[Path, str]:
        directory = self._directory(analysis_id)
        frames_root = directory / "frames"
        if not frames_root.is_dir():
            raise FileNotFoundError(f"frames not found for analysis: {analysis_id}")
        decoded = str(relative_path).lstrip("/")
        if decoded.startswith("frames/"):
            decoded = decoded[len("frames/") :]
        candidate = resolve_under_root(
            decoded,
            frames_root,
            label="analysis frame",
            require_exists=True,
        )
        if not candidate.is_file() or candidate.suffix.lower() not in FRAME_EXTENSIONS:
            raise ValueError("analysis frame must be a JPEG, PNG, or WebP file")
        return candidate, mimetypes.guess_type(str(candidate))[0] or "application/octet-stream"


def _source_ros_setup(config: ConsoleConfig) -> str:
    setup = config.ros2_ws / "install" / "setup.bash"
    return (
        f'test -f {_q(setup)} || {{ echo "ROS workspace setup is missing: {_q(setup)}"; exit 1; }}; '
        f'set +u; source {_q(setup)}; set -u'
    )


def build_analysis_script(
    config: ConsoleConfig,
    *,
    analysis_dir: Path,
    rosbag: Path,
    image_topic: str,
    control_topic: str,
    mode_topic: str,
    pose_topic: str,
    speed_topic: str,
    map_dir: Path | None,
    trajectory_mode: str,
    max_fps: float,
    offline_localization_mode: str = "auto",
    expected_map_fingerprint: str = "",
    topic_config: Path | None = None,
    model_dir: Path | None = None,
    enable_warmup_step: bool = True,
) -> str:
    """Build the Linux/Docker task script used by ``POST /api/analyses``.

    ``analysis_worker`` is intentionally a separate CLI so direct rosbag
    extraction can be tested without the HTTP server. Its stable contract is:

    ``--rosbag --analysis-dir --image-topic --status-file`` plus optional
    ``--control-topic --mode-topic --pose-topic --speed-topic --map-dir
    --trajectory-snapshot`` and ``--max-fps``.
    """

    if not math.isfinite(max_fps) or max_fps <= 0.0 or max_fps > 240.0:
        raise ValueError("max_fps must be a finite value greater than 0 and at most 240")
    if trajectory_mode not in {"recorded", "offline", "none"}:
        raise ValueError("resolved trajectory_mode must be recorded, offline, or none")
    if offline_localization_mode not in {"auto", "vgl", "vslam", "vslam_from_scratch"}:
        raise ValueError("offline_localization_mode must be auto, vgl, vslam, or vslam_from_scratch")

    status_file = analysis_dir / "status.json"
    snapshot = analysis_dir / "localization" / "vslam_snapshot.json"
    localization_method_file = analysis_dir / "localization" / "method.txt"
    status_writer = " ".join(
        [
            _q(config.python_bin),
            "-m",
            "jetpilot_console.analysis_worker",
            "--analysis-dir",
            _q(analysis_dir),
            "--status-file",
            _q(status_file),
        ]
    )
    lines = [
        "set -euo pipefail",
        f"mkdir -p {_q(analysis_dir)} {_q(snapshot.parent)}",
        'offline_launch_pid=""',
        "analysis_exit() {",
        "  exit_code=$?",
        "  set +e",
        '  if [ -n "$offline_launch_pid" ] && kill -0 "$offline_launch_pid" 2>/dev/null; then',
        '    kill -TERM "$offline_launch_pid" 2>/dev/null',
        '    wait "$offline_launch_pid" 2>/dev/null',
        "  fi",
        '  if [ "$exit_code" -ne 0 ]; then',
        '    if [ "$exit_code" -eq 130 ] || [ "$exit_code" -eq 143 ]; then',
        (
            f"      {status_writer} --set-status stopped --stage stopped --progress 1.0 "
            f"--message {_q('Analysis task was stopped by the user.')}"
        ),
        "    else",
        (
            f"      {status_writer} --set-status failed --stage failed --progress 1.0 "
            f"--message {_q('Analysis task failed before completion. See the task log for details.')}"
        ),
        "    fi",
        "  fi",
        '  exit "$exit_code"',
        "}",
        "trap analysis_exit EXIT",
        _source_ros_setup(config),
    ]

    if trajectory_mode == "offline":
        if topic_config is None:
            raise ValueError("offline localization requires a camera topic config path")
        if offline_localization_mode != "vslam_from_scratch" and map_dir is None:
            raise ValueError("offline localization requires a map path (unless using vslam_from_scratch)")
        if offline_localization_mode in {"auto", "vgl"} and model_dir is None:
            raise ValueError("VGL localization requires a VGL model path")
        lines.extend(
            [
                f"export ROS_DOMAIN_ID={int(config.analysis_ros_domain_id)}",
                'echo "[stage] offline localization"',
                (
                    f"{status_writer} --set-status running --stage offline_localization "
                    f"--progress 0.05 --message {_q('Preparing offline localization.')}"
                ),
                f"rm -f {_q(snapshot)} {_q(localization_method_file)}",
                "offline_stop_launch() {",
                '  offline_stop_signal="$1"',
                "  offline_stop_status=0",
                '  if [ -n "$offline_launch_pid" ]; then',
                '    if kill -0 "$offline_launch_pid" 2>/dev/null; then',
                '      kill -s "$offline_stop_signal" "$offline_launch_pid" 2>/dev/null || true',
                "    fi",
                "    offline_wait_count=0",
                '    while kill -0 "$offline_launch_pid" 2>/dev/null && [ "$offline_wait_count" -lt 20 ]; do',
                "      sleep 1",
                "      offline_wait_count=$((offline_wait_count + 1))",
                "    done",
                '    if kill -0 "$offline_launch_pid" 2>/dev/null; then',
                '      echo "offline launch did not exit cleanly within 20 seconds; sending TERM/KILL"',
                '      kill -s TERM "$offline_launch_pid" 2>/dev/null || true',
                "      sleep 3",
                '      if kill -0 "$offline_launch_pid" 2>/dev/null; then',
                '        kill -s KILL "$offline_launch_pid" 2>/dev/null || true',
                "      fi",
                "    fi",
                '    wait "$offline_launch_pid" 2>/dev/null || true',
                "    offline_stop_status=0",
                "  fi",
                '  offline_launch_pid=""',
                '  return "$offline_stop_status"',
                "}",
                "wait_for_offline_graph_quiescence() {",
                "  offline_quiet_attempt=0",
                "  offline_quiet_streak=0",
                '  while [ "$offline_quiet_attempt" -lt 30 ]; do',
                '    offline_nodes="$(ros2 node list 2>/dev/null || true)"',
                '    offline_resume_type="$(ros2 service type /rosbag2_player/resume 2>/dev/null || true)"',
                '    if [[ "$offline_nodes" != *visual_slam_node* ]] && [[ "$offline_nodes" != *visual_global_localization_node* ]] && [[ "$offline_nodes" != *localization_manager* ]] && [[ "$offline_nodes" != *vslam_reference_snapshot_recorder* ]] && [[ "$offline_resume_type" != *rosbag2_interfaces/srv/Resume* ]]; then',
                "      offline_quiet_streak=$((offline_quiet_streak + 1))",
                "    else",
                "      offline_quiet_streak=0",
                "    fi",
                '    if [ "$offline_quiet_streak" -ge 3 ]; then',
                "      return 0",
                "    fi",
                "    offline_quiet_attempt=$((offline_quiet_attempt + 1))",
                "    sleep 1",
                "  done",
                '  echo "previous offline localization graph did not disappear within 30 seconds; refusing to overlap attempts"',
                "  return 29",
                "}",
                "run_offline_localization_attempt() {",
                '  offline_method="$1"',
                '  offline_replay_progress="$2"',
                '  offline_drain_progress="$3"',
                f"  target_map_dir={_q(map_dir or '')}",
                '  if [ "$offline_method" = "vgl" ]; then',
                "    offline_enable_vgl=true",
                "    offline_require_vgl_node=1",
                "    offline_publish_identity_hint=0",
                '    offline_method_label="VGL + VSLAM"',
                '    offline_map_dir="$target_map_dir"',
                "    offline_require_localized=true",
                '  elif [ "$offline_method" = "vslam_from_scratch" ]; then',
                "    offline_enable_vgl=false",
                "    offline_require_vgl_node=0",
                "    offline_publish_identity_hint=0",
                '    offline_method_label="VSLAM from scratch (no map)"',
                '    offline_map_dir=""',
                "    offline_require_localized=false",
                "  else",
                "    offline_enable_vgl=false",
                "    offline_require_vgl_node=0",
                "    offline_publish_identity_hint=1",
                '    offline_method_label="VSLAM identity hint"',
                '    offline_map_dir="$target_map_dir"',
                "    offline_require_localized=true",
                "  fi",
                f"  rm -f {_q(snapshot)} || return 20",
                '  echo "[offline] starting $offline_method_label"',
                f"  {status_writer} --set-status running --stage offline_localization --progress \"$offline_replay_progress\" --message \"Starting $offline_method_label.\" || return 21",
                f"  ros2 launch {_q(config.launch_package)} bringup.launch.py \\",
                "    use_sim_time:=true \\",
                "    enable_rosbag_replay:=true \\",
                "    replay_additional_args:='--clock --start-paused' \\",
                "    rosbag_start_delay_s:=0.0 \\",
                "    rosbag_shutdown_on_exit:=true \\",
                "    enable_operation:=false \\",
                "    enable_control:=false \\",
                "    enable_vehicle:=false \\",
                "    publish_vehicle_description:=false \\",
                "    enable_sensor_kit:=false \\",
                "    enable_localization:=true \\",
                "    enable_vslam:=true \\",
                "    vslam_enable_slam:=true \\",
                "    vslam_enable_visualization:=false \\",
                '    enable_vgl:="$offline_enable_vgl" \\',
                "    enable_localization_manager:=true \\",
                f"    vgl_topic_config_file:={_q(topic_config)} \\",
                *(
                    [f"    vgl_model_dir:={_q(model_dir)} \\"]
                    if model_dir else []
                ),
                '    ${offline_map_dir:+map_dir:="$offline_map_dir"} \\',
                "    enable_tool:=true \\",
                "    enable_bag_manager:=false \\",
                "    enable_joy:=false \\",
                "    enable_teleop:=false \\",
                "    enable_rc_serial:=false \\",
                "    enable_vslam_snapshot:=true \\",
                f"    vslam_snapshot_output:={_q(snapshot)} \\",
                "    vslam_snapshot_path_topic:=/visual_slam/tracking/slam_path \\",
                "    vslam_snapshot_odom_topic:=/visual_slam/tracking/odometry \\",
                "    vslam_snapshot_localization_state_topic:=/localization/pose_hint_state \\",
                "    vslam_snapshot_tf_topic:=/tf \\",
                "    vslam_snapshot_map_frame:=map \\",
                '    vslam_snapshot_require_localized_map:="$offline_require_localized" \\',
                "    vslam_snapshot_write_interval_s:=2.0 \\",
                "    enable_rviz:=false \\",
                f"    rosbag:={_q(rosbag)} &",
                "  offline_launch_pid=$!",
                # ── readiness polling (same as before) ──────────────────────
                "  offline_attempt=0",
                "  offline_ready=0",
                '  while [ "$offline_attempt" -lt 180 ]; do',
                '    if ! kill -0 "$offline_launch_pid" 2>/dev/null; then',
                '      echo "offline $offline_method launch exited before replay became ready"',
                "      offline_stop_launch TERM || true",
                "      return 22",
                "    fi",
                '    offline_nodes="$(ros2 node list 2>/dev/null || true)"',
                "    offline_vgl_ready=1",
                '    if [ "$offline_require_vgl_node" -eq 1 ] && [[ "$offline_nodes" != *visual_global_localization_node* ]]; then',
                "      offline_vgl_ready=0",
                "    fi",
                '    if [[ "$offline_nodes" == *visual_slam_node* ]] && [ "$offline_vgl_ready" -eq 1 ] && [[ "$offline_nodes" == *localization_manager* ]] && [[ "$offline_nodes" == *vslam_reference_snapshot_recorder* ]]; then',
                "      offline_ready=1",
                "      break",
                "    fi",
                "    offline_attempt=$((offline_attempt + 1))",
                "    sleep 1",
                "  done",
                '  if [ "$offline_ready" -ne 1 ]; then',
                '    echo "offline $offline_method localization readiness timed out after 180 seconds"',
                "    offline_stop_launch TERM || true",
                "    return 23",
                "  fi",
                # ── identity hint (vslam_identity path only) ─────────────────
                '  if [ "$offline_publish_identity_hint" -eq 1 ]; then',
                '    echo "[offline] VGL is disabled; publishing identity map pose to /initialpose"',
                '    if ! ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}}}}"; then',
                "      offline_stop_launch TERM || true",
                "      return 24",
                "    fi",
                "  fi",
                f"  if ! {status_writer} --set-status running --stage offline_replay --progress \"$offline_replay_progress\" --message \"$offline_method_label is ready; starting paused rosbag replay.\"; then",
                "    offline_stop_launch TERM || true",
                "    return 25",
                "  fi",
                # ── warmup ──────────────────────────────────────────────────
                f"  offline_enable_warmup=\"${{ENABLE_ROSBAG_WARMUP_STEP:-{'true' if enable_warmup_step else 'false'}}}\"",
                "  if [ \"$offline_enable_warmup\" = \"true\" ] || [ \"$offline_enable_warmup\" = \"1\" ]; then",
                "    echo \"[offline] starting 2-stage wait (warmup step)\"",
                "    sleep 5",
                "    echo \"[offline] advancing rosbag by ~1 image frame (0.15s) for lazy initialization\"",
                "    if ! ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume '{}'; then",
                "      offline_stop_launch TERM || true",
                "      return 26",
                "    fi",
                "    sleep 0.15",
                "    ros2 service call /rosbag2_player/pause rosbag2_interfaces/srv/Pause '{}' || true",
                "    echo \"[offline] waiting 5s for VGL/VSLAM initialization to settle during pause\"",
                "    sleep 5",
                "    echo \"[offline] resuming rosbag playback after warmup\"",
                "    if ! ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume '{}'; then",
                "      offline_stop_launch TERM || true",
                "      return 26",
                "    fi",
                "  else",
                "    sleep 5",
                "    if ! ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume '{}'; then",
                "      offline_stop_launch TERM || true",
                "      return 26",
                "    fi",
                "  fi",
                # ── wait for launch to exit naturally (bag end → auto-shutdown) ─
                '  echo "[offline] rosbag replay running; waiting for bag-end auto-shutdown of ros2 launch"',
                f"  {status_writer} --set-status running --stage offline_replay --progress \"$offline_drain_progress\" --message \"Rosbag replay in progress; waiting for bag-end auto-shutdown.\" || true",
                "  offline_completion_wait=0",
                '  while kill -0 "$offline_launch_pid" 2>/dev/null && [ "$offline_completion_wait" -lt 600 ]; do',
                "    sleep 1",
                "    offline_completion_wait=$((offline_completion_wait + 1))",
                "  done",
                '  if kill -0 "$offline_launch_pid" 2>/dev/null; then',
                '    echo "offline $offline_method launch did not exit within 600 s of bag-end; forcing shutdown"',
                '    kill -s TERM "$offline_launch_pid" 2>/dev/null || true',
                "    sleep 5",
                '    if kill -0 "$offline_launch_pid" 2>/dev/null; then',
                '      kill -s KILL "$offline_launch_pid" 2>/dev/null || true',
                "    fi",
                "  fi",
                '  wait "$offline_launch_pid" 2>/dev/null || true',
                '  offline_launch_pid=""',
                f'  if [ ! -s {_q(snapshot)} ]; then echo "offline $offline_method localization produced no VSLAM snapshot"; return 28; fi',
                "  return 0",
                "}",
            ]
        )
        if offline_localization_mode == "auto":
            lines.extend(
                [
                    "set +e",
                    "run_offline_localization_attempt vgl 0.15 0.40",
                    "offline_primary_status=$?",
                    "set -e",
                    'if [ "$offline_primary_status" -eq 0 ]; then',
                    f"  printf '%s\\n' vgl > {_q(localization_method_file)}",
                    "else",
                    '  echo "[warning] VGL offline localization failed with status $offline_primary_status; restarting the bag with VGL disabled and an identity VSLAM pose hint."',
                    "  wait_for_offline_graph_quiescence",
                    (
                        f"  {status_writer} --set-status running --stage offline_fallback "
                        f"--progress 0.405 --message {_q('VGL failed; retrying from the bag start with the saved VSLAM map and an identity pose hint.')}"
                    ),
                    f"  printf '%s\\n' vslam_identity_fallback > {_q(localization_method_file)}",
                    "  run_offline_localization_attempt vslam 0.41 0.415",
                    "fi",
                ]
            )
        elif offline_localization_mode == "vgl":
            lines.extend(
                [
                    "run_offline_localization_attempt vgl 0.15 0.40",
                    f"printf '%s\\n' vgl > {_q(localization_method_file)}",
                ]
            )
        elif offline_localization_mode == "vslam_from_scratch":
            lines.extend(
                [
                    "run_offline_localization_attempt vslam_from_scratch 0.15 0.40",
                    f"printf '%s\\n' vslam_from_scratch > {_q(localization_method_file)}",
                ]
            )
        else:
            lines.extend(
                [
                    f"printf '%s\\n' vslam_identity > {_q(localization_method_file)}",
                    "run_offline_localization_attempt vslam 0.15 0.40",
                ]
            )

    worker = [
        _q(config.python_bin),
        "-m",
        "jetpilot_console.analysis_worker",
        "--rosbag",
        _q(rosbag),
        "--analysis-dir",
        _q(analysis_dir),
        "--image-topic",
        _q(image_topic),
        "--max-fps",
        f"{max_fps:.9g}",
        "--status-file",
        _q(status_file),
    ]
    for option, value in (
        ("--control-topic", control_topic),
        ("--mode-topic", mode_topic),
        ("--pose-topic", pose_topic if trajectory_mode == "recorded" else ""),
        ("--speed-topic", speed_topic),
        ("--map-dir", str(map_dir) if map_dir is not None else ""),
        ("--trajectory-snapshot", str(snapshot) if trajectory_mode == "offline" else ""),
        ("--expected-map-fingerprint", expected_map_fingerprint if map_dir is not None else ""),
    ):
        if value:
            worker.extend([option, _q(value)])
    lines.extend(['echo "[stage] extract and synchronize rosbag topics"', " ".join(worker)])
    return "\n".join(lines) + "\n"
