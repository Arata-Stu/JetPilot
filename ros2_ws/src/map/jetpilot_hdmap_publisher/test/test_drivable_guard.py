"""Runtime safety geometry tests requiring only the Python standard library."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from jetpilot_hdmap_publisher.drivable_guard import Environment, Settings, motion_issue, route_issue


def corridor(width=2,obstacles=()):
    return Environment([([(-10,width),(10,width)],[(-10,-width),(10,-width)],False)],obstacles)


class GuardTest(unittest.TestCase):
    def setUp(self):
        self.cfg=Settings(front_m=.1,rear_m=.1,width_m=.1,margin_m=.02,braking_mps2=1.)

    def test_generation_corridor_is_not_an_input(self):
        self.assertEqual(motion_issue(corridor(),(0,1,0),(1,0,0),self.cfg),'')

    def test_speed_and_direction_determine_imminent_departure(self):
        env=corridor()
        self.assertEqual(motion_issue(env,(0,1.5,0),(.2,0,0),self.cfg),'')
        self.assertIn('bounds',motion_issue(env,(0,1.5,math.pi/2),(1,0,0),self.cfg))
        self.assertEqual(motion_issue(env,(0,1.5,-math.pi/2),(1,0,0),self.cfg),'')

    def test_body_outside_while_base_inside(self):
        self.assertIn('bounds',motion_issue(corridor(),(0,1.99,0),(0,0,0),self.cfg))

    def test_reverse_and_lateral_velocity(self):
        env=corridor(obstacles=[('box',[(-.7,-.1),(-.69,-.1),(-.69,.1),(-.7,.1)],0)])
        self.assertIn('box',motion_issue(env,(0,0,0),(-1,0,0),self.cfg))
        self.assertEqual(motion_issue(env,(0,0,0),(1,0,0),self.cfg),'')
        self.assertIn('bounds',motion_issue(corridor(),(0,1.5,0),(0,1,0),self.cfg))

    def test_thin_obstacle_between_samples(self):
        env=corridor(obstacles=[('thin',[(.321,-.01),(.322,-.01),(.322,.01),(.321,.01)],0)])
        cfg=Settings(front_m=.001,rear_m=.001,width_m=.001,margin_m=0,step_m=1)
        self.assertIn('thin',motion_issue(env,(0,0,0),(1,0,0),cfg))

    def test_obstacle_margin_and_enclosed_obstacle(self):
        env=corridor(obstacles=[('box',[(.1,.1),(.2,.1),(.2,.2),(.1,.2)],.3)])
        self.assertIn('box',motion_issue(env,(0,0,0),(0,0,0),self.cfg))

    def test_closed_lane_has_inner_forbidden_island_and_no_seam(self):
        env=Environment([([(-3,-3),(3,-3),(3,3),(-3,3)],
                          [(-1,-1),(1,-1),(1,1),(-1,1)],True)],[])
        self.assertEqual(motion_issue(env,(-2,0,math.pi/2),(.5,0,0),self.cfg),'')
        self.assertIn('bounds',motion_issue(env,(0,0,0),(0,0,0),self.cfg))
        self.assertIn('bounds',motion_issue(env,(-2,0,0),(2,0,0),self.cfg))

    def test_turning_motion_hits_wall(self):
        self.assertIn('bounds',motion_issue(corridor(1),(0,.5,0),(1,0,2),self.cfg))

    def test_unsafe_route_does_not_imply_current_motion_emergency(self):
        env=corridor(obstacles=[('box',[(.4,-.1),(.5,-.1),(.5,.1),(.4,.1)],0)])
        self.assertEqual(motion_issue(env,(0,0,0),(0,0,0),self.cfg),'')
        self.assertIn('box',route_issue(env,(0,0),[(0,0),(2,0)],1,self.cfg))

    def test_route_looks_across_closed_loop_seam(self):
        env=corridor(obstacles=[('box',[(.2,-.05),(.3,-.05),(.3,.05),(.2,.05)],0)])
        path=[(0,0),(2,0),(2,1),(0,1)]
        self.assertIn('box',route_issue(env,(0,.1),path,1,self.cfg,closed=True))

    def test_safe_other_lane_and_conservative_seam(self):
        env=Environment([([(0,1),(5,1)],[(0,-1),(5,-1)],False),
                         ([(4,1),(10,1)],[(4,-1),(10,-1)],False)],[])
        self.assertEqual(route_issue(env,(4.5,0),[(4,0),(9,0)],1,self.cfg),'')

    def test_invalid_geometry_fails_closed(self):
        for lanes,obstacles in [([],[]),([([(0,0),(1,1)],[(1,0),(0,1)],False)],[]),
             ([([(0,0),(1,0),(1,1)],[(2,2),(3,2),(3,3)],True)],[])]:
            with self.assertRaises(ValueError): Environment(lanes,obstacles)
        with self.assertRaises(ValueError): corridor(obstacles=[('x',[(0,0),(1,0),(0,1)],float('nan'))])
        with self.assertRaises(ValueError): motion_issue(corridor(),(0,0,0),(float('nan'),0,0),self.cfg)
        with self.assertRaises(ValueError): Settings(braking_mps2=0)


if __name__ == '__main__': unittest.main()
