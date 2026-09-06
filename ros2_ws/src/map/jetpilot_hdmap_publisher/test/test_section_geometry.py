"""Run with standard-library unittest; ROS imports are deliberately unnecessary."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

PACKAGE = Path(__file__).resolve().parents[1] / 'jetpilot_hdmap_publisher'


def load_functions(path, names, namespace):
    tree = ast.parse(path.read_text())
    definitions = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in names]
    for node in tree.body:
        if isinstance(node, ast.ClassDef):
            definitions.extend(child for child in node.body if isinstance(child, ast.FunctionDef) and child.name in names)
    # Defer annotations so the actual production geometry/resolve functions can be
    # executed without importing ROS message types or creating ROS nodes.
    definitions.insert(0, ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0))
    exec(compile(ast.fix_missing_locations(ast.Module(body=definitions, type_ignores=[])), str(path), 'exec'), namespace)


class SectionGeometryTest(unittest.TestCase):
    def setUp(self):
        self.ns = {'math': math}
        exec((PACKAGE / 'section_geometry.py').read_text(), self.ns)
        load_functions(PACKAGE / 'hd_map_publisher_node.py',
                       {'distance_2d', 'cumulative_s', 'interpolate_at_s', 'section_points'}, self.ns)
        load_functions(PACKAGE / 'hd_map_section_localizer_node.py',
                       {'project_point_to_lane_s', 'section_contains_s', 'resolve_section'}, self.ns)
        self.lane = SimpleNamespace(lane_id='lane', closed_loop=True,
                                   centerline=[(0,0,0), (10,0,0), (10,10,0), (0,10,0)])
        self.section = SimpleNamespace(section_id='only', lane_id='lane', start_gate_id='gate',
                                       end_gate_id='gate', start_s_m=5, end_s_m=45)

    def test_single_closed_section_covers_entire_lap_including_seam(self):
        for end in (5, 45):  # Legacy same-station and explicit unwrapped full-lap YAML.
            self.section.end_s_m = end
            for station in (0, 4.99, 5, 5.01, 39.99, 40, 85):
                self.assertTrue(self.ns['section_contains_s'](self.section, self.lane, station))

    def test_resolver_returns_single_section_but_keeps_distance_and_missing_section_guards(self):
        node = SimpleNamespace(hd_map=SimpleNamespace(lanes=[self.lane], sections=[self.section]), max_lane_distance_m=1)
        for point in [(0,0,0), (5,0,0), (10,5,0), (5,10,0), (0,5,0)]:
            self.assertEqual(self.ns['resolve_section'](node, point), 'only')
        self.assertEqual(self.ns['resolve_section'](node, (100,100,0)), 'unknown')
        node.hd_map.sections = []
        self.assertEqual(self.ns['resolve_section'](node, (5,0,0)), 'unknown')

    def test_open_lane_last_point_is_in_final_section(self):
        contains = self.ns['contains_station']
        self.assertTrue(contains(0, 10, 10, 10, False))
        self.assertFalse(contains(0, 5, 5, 10, False))
        self.assertTrue(contains(5, 10, 5, 10, False))
        self.assertFalse(contains(0, 5, 10, 10, False))

    def test_multiple_sections_keep_half_open_and_wrap_semantics(self):
        contains = self.ns['contains_station']
        self.assertTrue(contains(5, 25, 5, 40, True))
        self.assertFalse(contains(5, 25, 25, 40, True))
        self.assertTrue(contains(25, 5, 0, 40, True))
        self.assertFalse(contains(25, 5, 5, 40, True))
        self.assertFalse(contains(5, 5, 10, 40, True))
        self.assertFalse(contains(0, 40, float('nan'), 40, True))

    def test_full_lap_marker_traces_all_corners(self):
        for end in (5,45):
            points = self.ns['section_points'](self.lane.centerline, True, 5, end)
            self.assertEqual(points[0], points[-1])
            for corner in self.lane.centerline:
                self.assertIn(corner, points)
