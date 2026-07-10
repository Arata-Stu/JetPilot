from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[4]


@dataclass(frozen=True)
class ConsoleConfig:
    repo_root: Path
    app_root: Path
    frontend_root: Path
    state_dir: Path
    task_dir: Path
    record_root: Path
    map_root: Path
    ros2_ws: Path
    python_ws: Path
    python_bin: str
    launch_package: str
    jetson_user: str
    jetson_ips: list[str]
    jetson_map_root: str
    jetson_record_root: str

    @classmethod
    def from_env(cls) -> "ConsoleConfig":
        repo_root = _repo_root()
        app_root = repo_root / "tools" / "app"
        state_dir = Path(
            os.environ.get("JETPILOT_CONSOLE_STATE_DIR", str(app_root / ".state"))
        ).expanduser()
        ros2_ws = Path(os.environ.get("ROS2_WS", "/workspaces/ros2_ws")).expanduser()
        python_ws = Path(
            os.environ.get("PYTHON_WS", str(ros2_ws.parent / "python_ws"))
        ).expanduser()
        jetson_ips = os.environ.get(
            "JETSON_REMOTE_IPS", "10.42.0.1 192.168.55.1 192.168.11.190"
        ).split()

        return cls(
            repo_root=repo_root,
            app_root=app_root,
            frontend_root=app_root / "frontend",
            state_dir=state_dir,
            task_dir=state_dir / "tasks",
            record_root=Path(os.environ.get("RECORD_ROOT", "/workspaces/record")).expanduser(),
            map_root=Path(os.environ.get("MAP_ROOT", "/workspaces/map")).expanduser(),
            ros2_ws=ros2_ws,
            python_ws=python_ws,
            python_bin=os.environ.get("PYTHON_BIN", "/opt/env/bin/python"),
            launch_package=os.environ.get("JETPILOT_LAUNCH_PACKAGE", "jetpilot_system_launch"),
            jetson_user=os.environ.get("JETSON_REMOTE_USER", "tamiya"),
            jetson_ips=jetson_ips,
            jetson_map_root=os.environ.get(
                "JETSON_MAP_ROOT", "/home/tamiya/workspaces/JetPilot/map"
            ),
            jetson_record_root=os.environ.get(
                "JETSON_RECORD_ROOT", "/home/tamiya/workspaces/JetPilot/record"
            ),
        )

    def as_json(self) -> dict[str, object]:
        return {
            "repo_root": str(self.repo_root),
            "app_root": str(self.app_root),
            "state_dir": str(self.state_dir),
            "record_root": str(self.record_root),
            "map_root": str(self.map_root),
            "ros2_ws": str(self.ros2_ws),
            "python_ws": str(self.python_ws),
            "python_bin": self.python_bin,
            "launch_package": self.launch_package,
            "jetson_user": self.jetson_user,
            "jetson_ips": self.jetson_ips,
            "jetson_map_root": self.jetson_map_root,
            "jetson_record_root": self.jetson_record_root,
        }

