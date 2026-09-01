from __future__ import annotations

import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from jetpilot_console.map_pipeline import (
    build_vgl_vslam_script,
    generate_raceline_script,
    scan_camera_topic_configs,
)


class ScanCameraTopicConfigsTest(unittest.TestCase):
    def test_exposes_topics_used_to_match_a_rosbag(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            ros2_ws = Path(temporary_dir)
            config_dir = (
                ros2_ws
                / "src"
                / "launch"
                / "jetpilot_system_launch"
                / "config"
                / "localization"
            )
            config_dir.mkdir(parents=True)
            config_path = config_dir / "vgl_camera_topics_secondary.yaml"
            config_path.write_text(
                """
stereo_cameras:
  - name: secondary_stereo
    left: /secondary/left/image_rect
    left_camera_info: /secondary/left/camera_info
    right: /secondary/right/image_rect
    right_camera_info: /secondary/right/camera_info
""".strip(),
                encoding="utf-8",
            )

            configs = scan_camera_topic_configs(SimpleNamespace(ros2_ws=ros2_ws))

            self.assertEqual(len(configs), 1)
            self.assertEqual(
                configs[0]["required_topics"],
                [
                    "/secondary/left/camera_info",
                    "/secondary/left/image_rect",
                    "/secondary/right/camera_info",
                    "/secondary/right/image_rect",
                ],
            )


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
        self.assertIn("trap 'offline_stop_launch TERM 5 || kill -KILL", script)
        self.assertIn("offline_stop_launch INT 20", script)
        self.assertIn("offline eval did not stop after SIGINT", script)
        self.assertIn("offline_topic_publishers", script)
        self.assertIn("VSLAM publishers did not become available", script)
        self.assertIn('rm -f "$snapshot"', script)
        self.assertIn("offline eval will load cuVSLAM map", script)
        self.assertIn("offline eval use_sim_time: true", script)
        self.assertIn("offline eval VSLAM visualization: true", script)
        self.assertIn("produced no VSLAM snapshot messages after replay started", script)
        self.assertIn("refusing to drain an empty run", script)
        self.assertIn("vslam_localize_on_startup:=true", script)
        self.assertNotIn("vslam_save_map_folder_path", script)
        self.assertNotIn('"$offline_launch_status" -ne 143', script)
        self.assertIn("enable_vgl:=false", script)
        self.assertIn("enable_rviz:=true", script)
        self.assertIn("rviz_config_file:=/workspaces/ros2_ws/install/jetpilot_system_launch/share/jetpilot_system_launch/rviz/vslam_debug.rviz", script)
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
        self.assertIn("--max-speed 3", script)
        self.assertIn("--min-speed 0.8", script)
        self.assertIn("--lateral-accel-limit 2.5", script)
        self.assertIn("--accel-limit 1.5", script)
        self.assertIn("--decel-limit 2.5", script)

    def test_passes_custom_vehicle_clearance(self) -> None:
        script = generate_raceline_script(
            self.config,
            "/workspaces/map/course_a",
            vehicle_width_m=0.187,
            safety_margin_m=0.02,
        )

        self.assertIn("--vehicle-width-m 0.187", script)
        self.assertIn("--safety-margin-m 0.02", script)

    def test_passes_custom_speed_profile(self) -> None:
        script = generate_raceline_script(
            self.config,
            "/workspaces/map/course_a",
            max_speed_mps=4.2,
            min_speed_mps=1.1,
            lateral_accel_limit_mps2=3.4,
            accel_limit_mps2=2.2,
            decel_limit_mps2=3.1,
        )

        self.assertIn("--max-speed 4.2", script)
        self.assertIn("--min-speed 1.1", script)
        self.assertIn("--lateral-accel-limit 3.4", script)
        self.assertIn("--accel-limit 2.2", script)
        self.assertIn("--decel-limit 3.1", script)

    def test_passes_selected_lap_direction(self) -> None:
        forward = generate_raceline_script(self.config, "/workspaces/map/course_a")
        reverse = generate_raceline_script(
            self.config,
            "/workspaces/map/course_a",
            direction="reverse",
        )

        self.assertIn("--direction forward", forward)
        self.assertIn("--direction reverse", reverse)

    def test_rejects_invalid_lap_direction(self) -> None:
        for value in ("both", "sideways", "", True):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, "forward or reverse"):
                    generate_raceline_script(
                        self.config,
                        "/workspaces/map/course_a",
                        direction=value,
                    )

    def test_rejects_invalid_vehicle_clearance(self) -> None:
        for value in (-0.01, math.inf, math.nan, True):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, "finite value"):
                    generate_raceline_script(
                        self.config,
                        "/workspaces/map/course_a",
                        vehicle_width_m=value,
                    )

    def test_rejects_invalid_speed_profile(self) -> None:
        cases = (
            {"max_speed_mps": 0.0},
            {"min_speed_mps": -0.1},
            {"min_speed_mps": 4.0, "max_speed_mps": 3.0},
            {"lateral_accel_limit_mps2": 0.0},
            {"accel_limit_mps2": math.inf},
            {"decel_limit_mps2": True},
        )
        for kwargs in cases:
            with self.subTest(kwargs=kwargs):
                with self.assertRaisesRegex(ValueError, "finite value|less than or equal"):
                    generate_raceline_script(self.config, "/workspaces/map/course_a", **kwargs)


if __name__ == "__main__":
    unittest.main()
