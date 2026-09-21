import importlib.util
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "fit_clock_map.py"
SPEC = importlib.util.spec_from_file_location("fit_clock_map", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
import sys

sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class ClockMapTest(unittest.TestCase):
    def test_fits_known_affine_map(self):
        scale = 1.000025
        offset = -0.625
        edges = []
        for marker, values in (("start", [1.0, 1.3, 1.8, 2.4]), ("end", [61.0, 61.3, 61.8, 62.4])):
            for index, rgb_mid in enumerate(values):
                edges.append(
                    MODULE.Edge(
                        marker=marker,
                        edge=index,
                        state="on" if index % 2 == 0 else "off",
                        rgb_before_s=rgb_mid - 0.01,
                        rgb_after_s=rgb_mid + 0.01,
                        evs_time_s=scale * rgb_mid + offset,
                    )
                )
        result = MODULE.fit_affine(edges)
        self.assertAlmostEqual(result["scale"], scale, places=12)
        self.assertAlmostEqual(result["offset_s"], offset, places=12)
        self.assertAlmostEqual(result["rmse_ms"], 0.0, places=9)
        self.assertEqual(result["warnings"], [])

    def test_loads_csv_and_rejects_reversed_rgb_interval(self):
        payload = (
            "marker,edge,state,rgb_before_s,rgb_after_s,evs_time_s\n"
            "start,0,on,2.0,1.0,1.5\n"
            "end,0,on,3.0,4.0,3.5\n"
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "edges.csv"
            path.write_text(payload, encoding="utf-8")
            with self.assertRaises(ValueError):
                MODULE.load_edges(path)


if __name__ == "__main__":
    unittest.main()
