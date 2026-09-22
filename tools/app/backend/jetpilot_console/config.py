from __future__ import annotations

import os
import sys
from dataclasses import dataclass
from pathlib import Path

from .security import env_flag
from .host_config import host_environment


def _app_root() -> Path:
    env_root = os.environ.get("JETPILOT_CONSOLE_APP_ROOT")
    if env_root:
        return Path(env_root).expanduser().resolve()

    current = Path(__file__).resolve()
    for parent in current.parents:
        if parent.name in {"app", "jetpilot_console_app"} and (parent / "frontend").exists():
            return parent
    return current.parents[2]


def _workspace_root(app_root: Path) -> Path:
    env_root = os.environ.get("JETPILOT_WORKSPACE_ROOT") or os.environ.get("JETPILOT_REPO_ROOT")
    if env_root:
        return Path(env_root).expanduser().resolve()

    if app_root.name == "jetpilot_console_app" and app_root.parent.name == "python_ws":
        return app_root.parent.parent
    if app_root.name == "app" and app_root.parent.name == "tools":
        return app_root.parent.parent
    return app_root.parent


def _bounded_int_env(name: str, *, default: int, minimum: int, maximum: int) -> int:
    raw_value = os.environ.get(name)
    if raw_value is None or not raw_value.strip():
        return default
    try:
        value = int(raw_value)
    except ValueError as exc:
        raise ValueError(f"{name} must be an integer") from exc
    if value < minimum or value > maximum:
        raise ValueError(f"{name} must be between {minimum} and {maximum}")
    return value


@dataclass(frozen=True)
class ConsoleConfig:
    repo_root: Path
    app_root: Path
    frontend_root: Path
    state_dir: Path
    task_dir: Path
    record_root: Path
    map_root: Path
    analysis_root: Path
    ros2_ws: Path
    python_ws: Path
    python_bin: str
    launch_package: str
    analysis_ros_domain_id: int
    jetson_user: str
    jetson_ips: list[str]
    jetson_map_root: str
    jetson_record_root: str
    enable_custom_commands: bool
    jetson_workspace_root: str = ""

    @classmethod
    def from_env(cls) -> "ConsoleConfig":
        app_root = _app_root()
        repo_root = _workspace_root(app_root)
        environment = host_environment(repo_root)
        state_dir = Path(
            environment.get("JETPILOT_CONSOLE_STATE_DIR", str(app_root / ".state"))
        ).expanduser().resolve(strict=False)
        ros2_ws = Path(environment["ROS2_WS"]).expanduser().resolve(strict=False)
        python_ws = Path(environment["PYTHON_WS"]).expanduser().resolve(strict=False)
        record_root = Path(environment["RECORD_ROOT"]).expanduser().resolve(strict=False)
        map_root = Path(environment["MAP_ROOT"]).expanduser().resolve(strict=False)
        analysis_root = Path(
            environment.get(
                "ANALYSIS_ROOT",
                environment.get(
                    "JETPILOT_ANALYSIS_ROOT",
                    str(record_root / ".jetpilot_analysis"),
                ),
            )
        ).expanduser().resolve(strict=False)
        jetson_ips = environment["JETSON_REMOTE_IPS"].split()

        return cls(
            repo_root=repo_root,
            app_root=app_root,
            frontend_root=app_root / "frontend",
            state_dir=state_dir,
            task_dir=state_dir / "tasks",
            record_root=record_root,
            map_root=map_root,
            analysis_root=analysis_root,
            ros2_ws=ros2_ws,
            python_ws=python_ws,
            python_bin=environment.get(
                "PYTHON_BIN",
                "/opt/env/bin/python"
                if Path("/opt/env/bin/python").is_file()
                else sys.executable,
            ),
            launch_package=environment.get("JETPILOT_LAUNCH_PACKAGE", "jetpilot_system_launch"),
            analysis_ros_domain_id=_bounded_int_env(
                "JETPILOT_ANALYSIS_ROS_DOMAIN_ID",
                default=92,
                minimum=0,
                maximum=232,
            ),
            jetson_workspace_root=environment["JETSON_WORKSPACE_ROOT"],
            jetson_user=environment["JETSON_REMOTE_USER"],
            jetson_ips=jetson_ips,
            jetson_map_root=environment["JETSON_MAP_ROOT"],
            jetson_record_root=environment["JETSON_RECORD_ROOT"],
            enable_custom_commands=env_flag(
                environment.get("JETPILOT_CONSOLE_ENABLE_CUSTOM_COMMANDS"),
                default=False,
            ),
        )

    def as_json(self) -> dict[str, object]:
        return {
            "repo_root": str(self.repo_root),
            "app_root": str(self.app_root),
            "state_dir": str(self.state_dir),
            "record_root": str(self.record_root),
            "map_root": str(self.map_root),
            "analysis_root": str(self.analysis_root),
            "ros2_ws": str(self.ros2_ws),
            "python_ws": str(self.python_ws),
            "python_bin": self.python_bin,
            "launch_package": self.launch_package,
            "analysis_ros_domain_id": self.analysis_ros_domain_id,
            "jetson_user": self.jetson_user,
            "jetson_workspace_root": self.jetson_workspace_root,
            "jetson_ips": self.jetson_ips,
            "jetson_map_root": self.jetson_map_root,
            "jetson_record_root": self.jetson_record_root,
            "custom_commands_enabled": self.enable_custom_commands,
        }
