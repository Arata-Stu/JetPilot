from __future__ import annotations

import unittest

import numpy as np

from e2e_learning.data.event_tensor import EventTensorAccumulator, EventTensorConfig


class EventTensorAccumulatorTests(unittest.TestCase):
    def test_snapshot_at_emits_every_boundary_crossed_by_one_packet(self) -> None:
        accumulator = EventTensorAccumulator(EventTensorConfig(
            bins=2,
            window_ms=4.0,
            stride_ms=2.0,
            width=2,
            height=1,
            polarity_layout="polarity_major",
            temporal_interpolation="none",
        ))
        accumulator.add(
            np.arange(0, 7000, 1000, dtype=np.int64),
            np.zeros(7, dtype=np.int32),
            np.zeros(7, dtype=np.int32),
            np.ones(7, dtype=np.bool_),
            source_width=2,
            source_height=1,
        )

        first = accumulator.snapshot_at(4_000_000)
        second = accumulator.snapshot_at(6_000_000)

        self.assertIsNotNone(first)
        self.assertIsNotNone(second)
        assert first is not None and second is not None
        self.assertEqual(first[1]["events"], 4)
        self.assertEqual(second[1]["events"], 4)
        self.assertEqual(first[1]["window_end_sensor_ns"], 4_000_000)
        self.assertEqual(second[1]["window_end_sensor_ns"], 6_000_000)


if __name__ == "__main__":
    unittest.main()
