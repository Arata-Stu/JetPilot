"""HD environment parsing and visualization without loading ROS packages."""
import json
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from test_section_geometry import PACKAGE, load_functions


class EnvironmentMarkersTest(unittest.TestCase):
    def test_loads_separate_bounds_and_draws_obstacle_height(self):
        def marker():
            return NS(header=NS(), pose=NS(orientation=NS()), scale=NS(), color=NS())
        marker.LINE_STRIP, marker.LINE_LIST, marker.ADD = 4, 5, 0
        ns = {'math':math, 'yaml':NS(safe_load=json.loads), 'Lane':lambda **kw:NS(**kw), 'HdMap':lambda **kw:NS(**kw),
              'Marker':marker, 'MarkerArray':lambda:NS(markers=[]), 'Point':lambda:NS()}
        load_functions(PACKAGE/'hd_map_publisher_node.py', {'read_physical_points','read_points','load_hd_map','build_lane_markers','color_for_field','to_geometry_point'}, ns)
        lane={'id':'lane','closed_loop':False,'left_bound':[[0,1,0],[2,1,0]],'right_bound':[[0,-1,0],[2,-1,0]],
              'centerline':[[0,0,0],[2,0,0]], 'drivable_left_bound':[[0,2,0],[2,2,0]], 'drivable_right_bound':[[0,-2,0],[2,-2,0]]}
        obstacle={'id':'box','polygon':[[.9,0,0],[1.1,0,0],[1,.2,0]],'height_m':.4}
        with tempfile.TemporaryDirectory() as td:
            path=Path(td)/'map.yaml';path.write_text(json.dumps({'lanes':[lane],'obstacles':[obstacle]}))
            hd=ns['load_hd_map'](path,'')
        publisher=NS(hd_map=hd,marker_line_width_m=.02,primary_marker_scale=2,marker_z_offset_m=0)
        markers=ns['build_lane_markers'](publisher,123).markers
        self.assertEqual(len(markers),6)
        physical=next(m for m in markers if m.ns.endswith('/drivable_left_bound'))
        self.assertEqual(physical.points[0].y,2)
        box=next(m for m in markers if m.ns=='hd_map/obstacles/box')
        self.assertEqual(box.type,marker.LINE_LIST)
        self.assertEqual({p.z for p in box.points},{0,.4})
        self.assertEqual(len(box.points),18)

if __name__ == '__main__': unittest.main()
