import importlib.util
import struct
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts/export_rgb_event_paper_frames.py"
SPEC = importlib.util.spec_from_file_location("paper_frames", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class PaperFrameSyncTest(unittest.TestCase):
    def test_maps_bag_elapsed_time_into_sensor_domain(self):
        self.assertEqual(
            MODULE.map_bag_to_sensor_us(
                10_250_000_000, 10_000_000_000, 7_500_000, 3.5
            ),
            7_753_500,
        )

    def test_reads_evbin_header(self):
        header = struct.pack(
            "<8sIIIIQQ24s",
            b"EVSBENCH",
            1,
            64,
            640,
            480,
            1,
            1000,
            b"\0" * 24,
        )
        event = struct.pack("<qHHB3s", 1234, 12, 34, 1, b"\0" * 3)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "test.evbin"
            path.write_bytes(header + event)
            parsed = MODULE.read_evbin_header(path)
        self.assertEqual(parsed.width, 640)
        self.assertEqual(parsed.height, 480)
        self.assertEqual(parsed.event_count, 1)

    def test_rejects_truncated_evbin(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "test.evbin"
            path.write_bytes(b"EVSBENCH")
            with self.assertRaises(ValueError):
                MODULE.read_evbin_header(path)

    def test_prefers_recorded_start_request(self):
        requests = [
            MODULE.RawRequestRecord(100, 101, 1, "start"),
            MODULE.RawRequestRecord(5_000_000_100, 5_000_000_101, 2, "stop"),
        ]
        anchor, method = MODULE._choose_raw_start_anchor(requests, 2000, 5_002_000, None)
        self.assertEqual(anchor, 100)
        self.assertEqual(method, "recorded_start_request")

    def test_infers_start_from_stop_when_start_was_not_recorded(self):
        requests = [MODULE.RawRequestRecord(8_000_000_000, 8_000_000_001, 2, "stop")]
        anchor, method = MODULE._choose_raw_start_anchor(
            requests, 2_000_000, 7_000_000, None
        )
        self.assertEqual(anchor, 3_000_000_000)
        self.assertEqual(method, "inferred_from_stop_minus_raw_duration")


if __name__ == "__main__":
    unittest.main()
