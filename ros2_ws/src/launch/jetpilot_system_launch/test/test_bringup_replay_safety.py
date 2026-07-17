from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest

from launch import LaunchContext
from launch.substitutions import LaunchConfiguration


LAUNCH_PATH = Path(__file__).resolve().parents[1] / "launch" / "bringup.launch.py"
SPEC = importlib.util.spec_from_file_location("jetpilot_bringup_launch", LAUNCH_PATH)
assert SPEC is not None and SPEC.loader is not None
BRINGUP = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BRINGUP)


class ReplaySafetyTest(unittest.TestCase):
    def test_safe_replay_covers_driver_inputs(self) -> None:
        expected_driver_inputs = {
            "/control_cmd",
            "/ackermann_cmd",
            "/commands/motor/duty_cycle",
            "/commands/motor/current",
            "/commands/motor/brake",
            "/commands/motor/speed",
            "/commands/motor/position",
            "/commands/servo/position",
            "/steer_offset_inc",
            "/steer_offset_dec",
            "/speed_offset_inc",
            "/speed_offset_dec",
        }

        self.assertTrue(
            expected_driver_inputs.issubset(set(BRINGUP._REPLAY_ISOLATED_TOPICS)))

    def test_safe_replay_remaps_every_isolated_and_configured_topic(self) -> None:
        arguments = BRINGUP._compose_replay_arguments("--clock", False)

        self.assertTrue(arguments.startswith("--clock --remap "))
        for topic in BRINGUP._REPLAY_ISOLATED_TOPICS:
            with self.subTest(topic=topic):
                self.assertIn(f"{topic}:=/replay{topic}", arguments)

        configured = BRINGUP._compose_replay_arguments(
            "", False, ("/custom/vehicle_input",))
        self.assertIn(
            "/custom/vehicle_input:=/replay/custom/vehicle_input", configured)

    def test_safe_replay_rejects_user_remap(self) -> None:
        for option in (
            "--remap /vehicle/control_cmd:=/unsafe",
            "-m /joy:=/joy",
            "-m=/joy:=/joy",
            "-m/joy:=/joy",
        ):
            with self.subTest(option=option):
                with self.assertRaisesRegex(RuntimeError, "cannot contain"):
                    BRINGUP._compose_replay_arguments(option, False)

    def test_explicit_unsafe_override_preserves_arguments(self) -> None:
        arguments = "--clock --remap /vehicle/control_cmd:=/vehicle/control_cmd"

        self.assertEqual(BRINGUP._compose_replay_arguments(arguments, True), arguments)

    def test_normalized_boolean_matches_replay_override_parser(self) -> None:
        for value in ("1", "true", "yes", "on", "TRUE"):
            with self.subTest(value=value):
                context = LaunchContext()
                context.launch_configurations["flag"] = value
                substitution = BRINGUP._LaunchBoolean(LaunchConfiguration("flag"))
                self.assertEqual(substitution.perform(context), "true")

    def test_vehicle_guard_combinations(self) -> None:
        defaults = {
            "enable_rosbag_replay": "true",
            "rosbag": "/bags/run",
            "enable_vehicle": "true",
            "allow_unsafe_replay_with_vehicle": "false",
            "allow_unsafe_replay_control_topics": "false",
            "replay_additional_args": "",
        }

        context = LaunchContext()
        context.launch_configurations.update(defaults)
        with self.assertRaisesRegex(RuntimeError, "vehicle interface"):
            BRINGUP._validate_replay_vehicle_safety(context)

        context.launch_configurations["enable_vehicle"] = "false"
        self.assertEqual(BRINGUP._validate_replay_vehicle_safety(context), [])

        context.launch_configurations["enable_vehicle"] = "true"
        context.launch_configurations["allow_unsafe_replay_with_vehicle"] = "1"
        self.assertEqual(BRINGUP._validate_replay_vehicle_safety(context), [])


if __name__ == "__main__":
    unittest.main()
