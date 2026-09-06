"""Exercise node processing/rendering with stdlib message doubles, not a ROS runtime."""
import ast
from collections import deque
import copy
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as N
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from jetpilot_object_detection.opponent_geometry import project_contact, Trails, rotate


def point(**values): return N(x=values.get('x',0.),y=values.get('y',0.),z=values.get('z',0.))
def pose(): return N(position=point(),orientation=N(x=0.,y=0.,z=0.,w=0.))
def stamped(): return N(header=N(frame_id='',stamp=None),pose=pose())
def path(): return N(header=N(frame_id='',stamp=None),poses=[])
class Marker:
    DELETEALL=3;ADD=0;LINE_STRIP=4;TEXT_VIEW_FACING=9;SPHERE=2
    def __init__(self):
        self.header=N(frame_id='',stamp=None);self.pose=pose();self.color=N();self.scale=N()
class Time:
    def __init__(self,nanoseconds=0,seconds=0): self.nanoseconds=nanoseconds+round(seconds*1e9)
    def to_msg(self): return N(sec=self.nanoseconds//10**9,nanosec=self.nanoseconds%10**9)
class Pub:
    def publish(self,message): self.message=copy.deepcopy(message)
class MissingTF(Exception): pass


def node():
    source=Path(__file__).resolve().parents[1]/'jetpilot_object_detection/opponent_projection_node.py'
    tree=ast.parse(source.read_text())
    tree.body=[item for item in tree.body if isinstance(item,(ast.FunctionDef,ast.ClassDef))]
    scope={'Node':object,'Time':Time,'Duration':Time,'Marker':Marker,'MarkerArray':lambda:N(markers=[]),
           'Path':path,'PoseStamped':stamped,'Point':point,'String':lambda **kw:N(**kw),
           'TransformException':MissingTF,'project_contact':project_contact,'Trails':Trails,'rotate':rotate,
           'json':json,'math':math}
    exec(compile(tree,str(source),'exec'),scope)
    value=scope['OpponentProjectionNode'].__new__(scope['OpponentProjectionNode'])
    value.p={'source_width':640,'source_height':480,'map_frame':'map','odom_frame':'odom',
             'image_geometry':'raw','max_range_m':8.,'gap_seconds':.5}
    value.trails=Trails();value.last_stamp=None;value.last_frame=None;value.map_alignment=None
    value.markers=Pub();value.status=Pub();value.paths=[Pub() for _ in range(8)]
    value.infos=deque([N(header=N(frame_id='optical',stamp=Time(seconds=1).to_msg()),
        width=640,height=480,roi=N(x_offset=0,y_offset=0,width=0,height=0,do_rectify=False),
        binning_x=0,binning_y=0,k=[500.,0,320,0,500,240,0,0,1],d=[0.]*5,
        r=[1.,0,0,0,1,0,0,0,1],p=[500.,0,320,0,0,500,240,0,0,0,1,0],distortion_model='plumb_bob')])
    def lookup(child,stamp):
        if child=='optical': return ((0,0,1),(-.5,.5,-.5,.5))
        if child=='tt02_ground_estimate': return ((0,0,0),(0,0,0,1))
        raise MissingTF(child)
    value.lookup=lookup
    return value


def detection(t,identity='session:1'):
    return N(header=N(frame_id='optical',stamp=Time(seconds=t).to_msg()),detections=[N(id=identity,
        bbox=N(center=N(position=N(x=320.,y=300.)),size_x=50.,size_y=80.),
        results=[N(hypothesis=N(class_id='vehicle',score=.9))])])


class ViewTests(unittest.TestCase):
    def test_tf_fallback_and_path_identity_and_lost_marker(self):
        n=node();n.process(detection(1));n.process(detection(1.1))
        n.publish_views(Time(seconds=1.2),1.2)
        path=n.paths[0].message
        self.assertEqual(path.header.frame_id,'map');self.assertEqual(len(path.poses),2)
        self.assertAlmostEqual(path.poses[0].pose.position.x,5.)
        state=json.loads(n.status.message.data)
        self.assertEqual(state['slots']['0']['track_id'],'session:1')
        self.assertTrue(state['slots']['0']['estimated_ground'])
        self.assertTrue(any(m.type==Marker.SPHERE for m in n.markers.message.markers if m.action==Marker.ADD))
        n.publish_views(Time(seconds=2),2)
        self.assertFalse(any(m.type==Marker.SPHERE for m in n.markers.message.markers if m.action==Marker.ADD))
        n.process(detection(2.1));n.publish_views(Time(seconds=2.2),2.2)
        self.assertEqual(len(n.paths[0].message.poses),1)
        n.trails.expire(20);n.publish_views(Time(seconds=20),20)
        self.assertFalse(n.paths[0].message.poses)

    def test_missing_calibration_and_unconfirmed_ids_do_not_create_position(self):
        n=node();n.process(detection(1,''));self.assertFalse(n.trails.tracks)
        n=node();n.infos.clear()
        with self.assertRaisesRegex(ValueError,'CameraInfo'): n.process(detection(1))
        self.assertFalse(n.trails.tracks)

    def test_map_alignment_jump_clears_history(self):
        n=node();original=n.lookup
        offset=[0.]
        def lookup(child,stamp):
            if child=='odom': return ((offset[0],0,0),(0,0,0,1))
            return original(child,stamp)
        n.lookup=lookup;n.process(detection(1));n.process(detection(1.1))
        offset[0]=1.;n.process(detection(1.2))
        self.assertEqual(len(n.trails.tracks['session:1']['points']),1)
