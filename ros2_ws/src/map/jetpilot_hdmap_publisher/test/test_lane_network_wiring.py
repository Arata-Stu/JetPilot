"""Execute planner/localizer production callbacks without importing ROS."""
import ast
import json
import math
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS, MethodType
from test_lane_network import graph
from jetpilot_hdmap_publisher.lane_network import Tracker, stations

PACKAGE=Path(__file__).resolve().parents[1]/'jetpilot_hdmap_publisher'


def methods(path,names,namespace):
    tree=ast.parse(path.read_text())
    defs=[n for cls in tree.body if isinstance(cls,ast.ClassDef) for n in cls.body
          if isinstance(n,ast.FunctionDef) and n.name in names]
    exec(compile(ast.Module(body=defs,type_ignores=[]),str(path),'exec'),namespace)
    return namespace


class Message:
    def __init__(self,**kwargs):
        self.header=NS(stamp=None,frame_id='')
        self.pose=NS(position=NS(x=0.,y=0.),orientation=NS(z=0.,w=1.))
        self.poses=[];self.points=[]
        self.__dict__.update(kwargs)


class ClockTime:
    def to_msg(self):return 0
    def __sub__(self,other):return NS(nanoseconds=0)
    @staticmethod
    def from_msg(stamp):return 0


class WiringTest(unittest.TestCase):
    def planner(self):
        ns=dict(math=math,json=json,PathMessage=Message,Trajectory=Message,PoseStamped=Message,
                TrajectoryPoint=Message,Float32=Message,Bool=Message,String=Message,
                Time=ClockTime,stations=stations)
        methods(PACKAGE/'lane_network_planner_node.py',{'tick','on_choices'},ns)
        node=NS(map_error='',choice_error='',tracker=Tracker(graph(),'entry'),frame='map',
                fingerprint='abc',mode='centerline',speed=.5,deceleration=1.,lateral_accel=1.,
                endpoint_margin=.4,base_frame='base_link',get_clock=lambda:NS(now=ClockTime))
        node.tf=NS(lookup_transform=lambda *args:NS(header=NS(stamp=0),transform=NS(translation=NS(x=1.,y=0.))))
        for name in ('path','profile','speed','ready','lane','selected','status'):
            values=[];setattr(node,name+'_values',values);setattr(node,name+'_pub',NS(publish=values.append))
        for name in ('tick','on_choices'):setattr(node,name,MethodType(ns[name],node))
        return node

    def test_publish_current_lane_and_typed_speed_then_revoke_stale_map(self):
        node=self.planner();node.on_choices(Message(data='{"entry":"a"}'));node.tick()
        self.assertTrue(node.ready_values[-1].data)
        self.assertEqual(node.lane_values[-1].data,'entry')
        self.assertTrue(node.profile_values[-1].points)
        self.assertEqual(node.profile_values[-1].points[-1].longitudinal_velocity_mps,0.)
        node.map_error='map revision changed';node.tick()
        self.assertFalse(node.ready_values[-1].data)
        self.assertEqual(node.path_values[-1].poses,[])
        self.assertEqual(node.profile_values[-1].points,[])
        self.assertEqual(node.speed_values[-1].data,0.)
        self.assertEqual(node.lane_values[-1].data,'')
        self.assertEqual(node.tracker.current_lane_id,'entry')

    def test_invalid_branch_command_cannot_change_lane(self):
        node=self.planner();node.on_choices(Message(data='{"entry":"exit"}'));node.tick()
        self.assertFalse(node.ready_values[-1].data)
        self.assertEqual(node.tracker.current_lane_id,'entry')

    def test_localizer_uses_current_lane_even_when_other_lane_is_closer(self):
        ns=dict(time=time,Optional=object,Point3=tuple,Lane=object,
                project_point_to_lane_s=lambda pose,lane:(1.,0. if lane.lane_id=='wrong' else .2),
                section_contains_s=lambda *args:True)
        # Strip annotations so this standard-library harness need not emulate typing/ROS.
        tree=ast.parse((PACKAGE/'hd_map_section_localizer_node.py').read_text())
        fn=next(n for c in tree.body if isinstance(c,ast.ClassDef) for n in c.body if isinstance(n,ast.FunctionDef) and n.name=='resolve_section')
        fn.returns=None
        for a in fn.args.args:a.annotation=None
        for n in ast.walk(fn):
            if isinstance(n,ast.AnnAssign):n.annotation=ast.Name(id='object',ctx=ast.Load())
        exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),'localizer','exec'),ns)
        lanes=[NS(lane_id='right',successor_ids=['wrong'],centerline=[0,1]),NS(lane_id='wrong',successor_ids=[],centerline=[0,1])]
        sections=[NS(lane_id=l.lane_id,section_id=l.lane_id+'_section') for l in lanes]
        node=NS(hd_map=NS(lanes=lanes,sections=sections),current_lane_id='right',current_lane_received=time.monotonic(),max_lane_distance_m=1.)
        self.assertEqual(ns['resolve_section'](node,(0,0,0)),'right_section')
        node.current_lane_received-=1.
        self.assertEqual(ns['resolve_section'](node,(0,0,0)),'unknown')

    def test_launch_has_one_planning_authority(self):
        import xml.etree.ElementTree as ET
        path=PACKAGE.parents[2]/'planning/jetpilot_planning/launch/jetpilot_planning.launch.xml'
        nodes=ET.parse(path).getroot().findall('node')
        graph_node=next(n for n in nodes if n.get('name')=='lane_network_planner')
        selector=next(n for n in nodes if n.get('name')=='route_lane_selector')
        self.assertEqual(graph_node.get('if'),selector.get('unless'))
