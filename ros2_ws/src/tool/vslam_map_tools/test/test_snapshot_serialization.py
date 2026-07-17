from __future__ import annotations

import json
import unittest
from types import SimpleNamespace

from vslam_map_tools.snapshot_serialization import (
    invert_transform,
    legacy_full_vslam_path,
    odometry_to_sample,
    transform_odometry_sample,
)


def vector(x: float, y: float, z: float) -> SimpleNamespace:
    return SimpleNamespace(x=x, y=y, z=z)


def odometry(*, sec: int, nanosec: int) -> SimpleNamespace:
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
            frame_id="map",
        ),
        child_frame_id="base_link",
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=vector(1.25, -2.5, 0.125),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.5, w=0.866),
            )
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=vector(3.0, 0.1, 0.0),
                angular=vector(0.0, 0.0, -0.2),
            )
        ),
    )


class SnapshotSerializationTest(unittest.TestCase):
    def test_odometry_sample_retains_time_frames_pose_and_twist(self) -> None:
        sample = odometry_to_sample(odometry(sec=1_721_185_200, nanosec=123_456_789))

        self.assertEqual(sample["timestamp_ns"], "1721185200123456789")
        self.assertIsInstance(sample["timestamp_ns"], str)
        self.assertEqual(sample["frame_id"], "map")
        self.assertEqual(sample["child_frame_id"], "base_link")
        self.assertEqual(sample["pose"]["position"]["x"], 1.25)
        self.assertEqual(sample["pose"]["orientation"]["w"], 0.866)
        self.assertEqual(sample["twist"]["linear"]["x"], 3.0)
        self.assertEqual(sample["twist"]["angular"]["z"], -0.2)
        self.assertIn('"timestamp_ns": "1721185200123456789"', json.dumps(sample))

    def test_zero_header_stamp_is_retained(self) -> None:
        sample = odometry_to_sample(
            odometry(sec=0, nanosec=0), received_timestamp_ns=42
        )
        self.assertEqual(sample["timestamp_ns"], "0")
        self.assertEqual(sample["received_timestamp_ns"], "42")

    def test_legacy_full_path_keeps_pose_only_schema(self) -> None:
        samples = [
            odometry_to_sample(odometry(sec=1, nanosec=0)),
            odometry_to_sample(odometry(sec=2, nanosec=0)),
        ]

        path = legacy_full_vslam_path(samples)

        self.assertEqual(path["frame_id"], "map")
        self.assertEqual(len(path["poses"]), 2)
        self.assertEqual(path["poses"][0], samples[0]["pose"])
        self.assertNotIn("timestamp_ns", path["poses"][0])
        self.assertIsNone(legacy_full_vslam_path([]))

    def test_odometry_pose_is_transformed_into_map_frame(self) -> None:
        sample = odometry_to_sample(odometry(sec=1, nanosec=0))
        sample["frame_id"] = "odom"
        sample["pose"]["position"] = {"x": 1.0, "y": 0.0, "z": 0.0}
        sample["pose"]["orientation"] = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        half_sqrt = 2.0 ** -0.5
        transform = {
            "translation": {"x": 10.0, "y": 20.0, "z": 0.0},
            "rotation": {"x": 0.0, "y": 0.0, "z": half_sqrt, "w": half_sqrt},
        }

        transformed = transform_odometry_sample(sample, transform, parent_frame="map")

        self.assertAlmostEqual(transformed["pose"]["position"]["x"], 10.0)
        self.assertAlmostEqual(transformed["pose"]["position"]["y"], 21.0)
        self.assertAlmostEqual(transformed["pose"]["orientation"]["z"], half_sqrt)
        self.assertAlmostEqual(transformed["pose"]["orientation"]["w"], half_sqrt)
        self.assertEqual(transformed["frame_id"], "map")
        self.assertEqual(transformed["source_frame_id"], "odom")

    def test_inverse_transform_round_trip(self) -> None:
        sample = odometry_to_sample(odometry(sec=1, nanosec=0))
        transform = {
            "translation": {"x": 3.0, "y": -4.0, "z": 0.5},
            "rotation": {"x": 0.0, "y": 0.0, "z": 0.3826834324, "w": 0.9238795325},
        }

        moved = transform_odometry_sample(sample, transform, parent_frame="map")
        restored = transform_odometry_sample(moved, invert_transform(transform), parent_frame="odom")

        for axis in ("x", "y", "z"):
            self.assertAlmostEqual(
                restored["pose"]["position"][axis], sample["pose"]["position"][axis]
            )


if __name__ == "__main__":
    unittest.main()
