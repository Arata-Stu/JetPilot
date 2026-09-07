"""Exercise the production Path builder without ROS dependencies."""
import math
from types import SimpleNamespace as NS
import unittest

from test_section_geometry import PACKAGE, load_functions


class PrimaryPathTest(unittest.TestCase):
    def build(self, points, closed):
        def pose():
            return NS(header=None, pose=NS(position=NS(x=0., y=0., z=0.),
                                          orientation=NS(z=0., w=1.)))
        ns = {'math': math, 'PathMsg': lambda: NS(header=NS(), poses=[]), 'PoseStamped': pose}
        load_functions(PACKAGE / 'hd_map_publisher_node.py', {'yaw_at', 'build_primary_path'}, ns)
        lane = NS(centerline=points, closed_loop=closed)
        publisher = NS(hd_map=NS(primary_lane=lambda: lane, frame_id='map'), path_z_offset_m=0.)
        return ns['build_primary_path'](publisher, 123)

    def test_sparse_closed_lane_explicitly_returns_to_start(self):
        points = [(0., 0., 0.), (4., 0., 0.), (4., 4., 0.), (0., 4., 0.)]
        path = self.build(points, True)
        self.assertEqual(len(path.poses), 5)
        self.assertEqual(path.poses[-1], path.poses[0])
        self.assertEqual(len(points), 4)  # Do not change the authored HD map.

    def test_open_lane_retains_its_endpoint(self):
        path = self.build([(0., 0., 0.), (4., 0., 0.), (4., 4., 0.)], False)
        self.assertEqual(len(path.poses), 3)
        self.assertNotEqual(path.poses[-1], path.poses[0])

    def test_already_closed_lane_does_not_gain_another_duplicate(self):
        path = self.build([(0., 0., 0.), (4., 0., 0.), (4., 4., 0.), (0., 0., 0.)], True)
        self.assertEqual(len(path.poses), 4)


if __name__ == '__main__':
    unittest.main()
