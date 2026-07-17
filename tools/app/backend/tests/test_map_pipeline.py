from __future__ import annotations

import math
import unittest
from pathlib import Path
from types import SimpleNamespace

from jetpilot_console.map_pipeline import build_vgl_vslam_script, generate_raceline_script


class BuildVglVslamScriptTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = SimpleNamespace(
            ros2_ws=Path("/workspaces/ros2_ws"),
            launch_package="jetpilot_system_launch",
        )

    def test_offline_eval_uses_controlled_replay_shutdown(self) -> None:
        script = build_vgl_vslam_script(
            self.config,
            "/workspaces/record/run_001",
            "/workspaces/map/course_a",
            "/workspaces/ros2_ws/src/launch/jetpilot_system_launch/config/localization/vgl_camera_topics.yaml",
            "edex compute_poses cuvgl",
            "low_res",
            "/workspaces/ros2_ws/isaac_ros_assets/models/visual_global_localization",
            enable_rviz=True,
        )

        self.assertIn("replay_additional_args:='--clock --start-paused'", script)
        self.assertIn("rosbag_shutdown_on_exit:=false", script)
        self.assertIn("ros2 service call /rosbag2_player/resume", script)
        self.assertIn("offline_stop_launch INT", script)
        self.assertIn("trap 'offline_stop_launch TERM || true' EXIT", script)
        self.assertIn("enable_vgl:=false", script)
        self.assertIn("enable_rviz:=false", script)
        self.assertNotIn("enable_rviz:=true", script)
        self.assertNotIn("visual_global_localization_node", script)


class GenerateRacelineScriptTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = SimpleNamespace(
            python_bin="/opt/env/bin/python",
            python_ws=Path("/workspaces/python_ws"),
        )

    def test_passes_default_vehicle_clearance_explicitly(self) -> None:
        script = generate_raceline_script(self.config, "/workspaces/map/course_a")

        self.assertIn("--vehicle-width-m 0.25", script)
        self.assertIn("--safety-margin-m 0.05", script)

    def test_passes_custom_vehicle_clearance(self) -> None:
        script = generate_raceline_script(
            self.config,
            "/workspaces/map/course_a",
            vehicle_width_m=0.187,
            safety_margin_m=0.02,
        )

        self.assertIn("--vehicle-width-m 0.187", script)
        self.assertIn("--safety-margin-m 0.02", script)

    def test_rejects_invalid_vehicle_clearance(self) -> None:
        for value in (-0.01, math.inf, math.nan, True):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, "finite value"):
                    generate_raceline_script(
                        self.config,
                        "/workspaces/map/course_a",
                        vehicle_width_m=value,
                    )


if __name__ == "__main__":
    unittest.main()
