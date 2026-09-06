import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from jetpilot_object_detection.opponent_geometry import camera_ray, project_contact, Trails


def calibration():
    return {'k':[500.,0,320,0,500,240,0,0,1], 'p':[500.,0,320,0,0,500,240,0,0,0,1,0],
            'r':[1.,0,0,0,1,0,0,0,1], 'd':[0.]*5, 'distortion_model':'plumb_bob',
            'roi':[0,0,0,0,False], 'binning':(1,1), 'image_size':(640,480)}


class ProjectionTests(unittest.TestCase):
    def test_contact_projects_into_map_with_ground_height(self):
        point=project_contact((320,300,50,80),calibration(),((2,3,1.1),(-.5,.5,-.5,.5)),
                              ((0,0,.1),(0,0,0,1)))
        for actual,expected in zip(point,(7,3,.1)): self.assertAlmostEqual(actual,expected)

    def test_horizon_clipped_and_far_boxes_are_rejected(self):
        for box in ((320,200,50,40),(320,460,50,40),(320,230,50,40),(5,300,50,40)):
            with self.assertRaises(ValueError):
                project_contact(box,calibration(),((0,0,1),(-.5,.5,-.5,.5)),((0,0,0),(0,0,0,1)))

    def test_raw_inverse_distortion_and_rectification(self):
        model=calibration();model['d']=[.2,0,0,0,0]
        # Normalized (0.3,0.2) distorted with radial coefficient .2.
        scale=1+.2*(.3**2+.2**2)
        ray=camera_ray(320+500*.3*scale,240+500*.2*scale,model)
        self.assertAlmostEqual(ray[0]/ray[2],.3);self.assertAlmostEqual(ray[1]/ray[2],.2)
        model=calibration();model['p'][3]=-100
        ray=camera_ray(320,340,model,'rectified')
        self.assertAlmostEqual(ray[0],0);self.assertAlmostEqual(ray[1]/ray[2],.2)
        model['r']=[0]*9
        with self.assertRaises(ValueError): camera_ray(320,340,model,'rectified')

    def test_binning_and_unsupported_calibration(self):
        model=calibration();model['binning']=(2,2)
        ray=camera_ray(160,170,model)
        self.assertAlmostEqual(ray[1]/ray[2],.2)
        model['distortion_model']='unknown'
        with self.assertRaises(ValueError): camera_ray(160,170,model)

    def test_history_is_id_isolated_and_breaks_on_gaps_or_jumps(self):
        trails=Trails();trails.expire(0)
        trails.observe('a',0,(0,0,0),True);trails.observe('b',0,(1,0,0),True)
        trails.observe('a',.1,(.1,0,0),True)
        self.assertEqual(len(trails.tracks['a']['points']),2)
        self.assertEqual(len(trails.tracks['b']['points']),1)
        trails.observe('a',1,(.2,0,0),True)
        self.assertEqual(len(trails.tracks['a']['points']),1)
        trails.observe('a',1.1,(20,0,0),True)
        self.assertEqual(len(trails.tracks['a']['points']),1)
        trails.observe('a',1.2,(20.1,0,0),False)
        self.assertEqual(len(trails.tracks['a']['points']),1)

    def test_history_is_bounded_expires_and_clears_on_clock_reset(self):
        trails=Trails(max_tracks=2,max_points=3,retain_seconds=2)
        trails.expire(0)
        for i in range(5): trails.observe('a',i*.1,(0,0,0),True)
        self.assertEqual(len(trails.tracks['a']['points']),3)
        trails.observe('b',1,(0,0,0),True);trails.observe('c',1.1,(0,0,0),True)
        self.assertEqual(set(trails.tracks),{'b','c'})
        trails.expire(4);self.assertFalse(trails.tracks)
        trails.observe('d',4,(0,0,0),True);trails.expire(1)
        self.assertFalse(trails.tracks)


if __name__=='__main__': unittest.main()

class WiringTests(unittest.TestCase):
    def test_projection_launch_and_foxglove_share_topics(self):
        import ast
        import re
        root=next(path for path in Path(__file__).resolve().parents if (path/'scripts/bringup.sh').is_file())
        source=(root/'ros2_ws/src/launch/jetpilot_system_launch/launch/bringup.launch.py').read_text()
        tree=ast.parse(source)
        patterns=next(ast.literal_eval(ast.literal_eval(node.value)) for node in tree.body
                      if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='_DEFAULT_FOXGLOVE_TOPIC_WHITELIST' for t in node.targets))
        for topic in ('/perception/opponents/markers','/perception/opponents/status','/perception/opponents/track_0/path'):
            self.assertTrue(any(re.fullmatch(pattern,topic) for pattern in patterns))
        self.assertIn("'enable_opponent_projection', False",source)
        self.assertIn("'camera_info_topic': args.object_detection_camera_info_topic",source)
        self.assertIn("'source_width': args.object_detection_source_width",source)
