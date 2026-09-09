import unittest
from types import SimpleNamespace

from e2e_learning.data.timestamps import alignment_timestamp_ns


def message(sec, nanosec=0):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)))


class AlignmentClockTests(unittest.TestCase):
    def test_event_device_clock_and_control_wall_clock_align_on_bag_time(self):
        image = message(12)
        control = message(1_788_942_930)
        image_recorded = 1_788_942_930_030_000_000
        control_recorded = 1_788_942_930_050_000_000
        self.assertLess(
            abs(alignment_timestamp_ns(image, image_recorded) - alignment_timestamp_ns(control, control_recorded)),
            100_000_000,
        )
        self.assertGreater(
            abs(alignment_timestamp_ns(image, image_recorded, "header") - alignment_timestamp_ns(control, control_recorded, "header")),
            100_000_000,
        )

    def test_header_mode_preserves_synchronized_capture_stamp(self):
        self.assertEqual(alignment_timestamp_ns(message(10, 123), 99, "header"), 10_000_000_123)

    def test_invalid_or_missing_header_falls_back_to_bag_clock(self):
        for msg in (message(0), SimpleNamespace()):
            self.assertEqual(alignment_timestamp_ns(msg, 1234, "header"), 1234)

    def test_invalid_clock_is_rejected(self):
        with self.assertRaises(ValueError):
            alignment_timestamp_ns(message(10), 1234, "automatic")


if __name__ == "__main__":
    unittest.main()
