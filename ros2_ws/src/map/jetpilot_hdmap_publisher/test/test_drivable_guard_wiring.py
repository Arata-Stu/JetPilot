"""Exercise production callbacks with ROS-free message and clock doubles."""
import math
import time
import unittest
from types import SimpleNamespace as NS, MethodType
from test_section_geometry import PACKAGE, load_functions
from test_drivable_guard import corridor
from jetpilot_hdmap_publisher.drivable_guard import Environment, Settings, motion_issue, route_issue


def msg(**kw):
    return NS(header=NS(frame_id='map',stamp=True),**kw)


def xy(x,y): return NS(x=x,y=y,z=0.)


class GuardWiringTest(unittest.TestCase):
    def guard(self):
        ns=dict(math=math,time=time,Environment=Environment,motion_issue=motion_issue,
                route_issue=route_issue,Time=lambda:0,PlanningSafetyStatus=msg)
        load_functions(PACKAGE/'drivable_guard_node.py',{'evaluate','fresh','on_map','tick'},ns)
        guard=NS(cfg=Settings(),environment=corridor(),environment_key=None,map_frame='map',
                 map_error='',map_received=time.monotonic(),input_timeout=.5,map_timeout=3.,
                 base_frame='base_link',odom_received=time.monotonic(),path_received=time.monotonic(),
                 speed_received=time.monotonic(),profile=None,profile_received=None,
                 stamp_fresh=lambda s:bool(s))
        transform=msg(transform=NS(translation=xy(0,0),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
        guard.tf=NS(lookup_transform=lambda *args:transform)
        guard.odom=msg(child_frame_id='base_link',twist=NS(twist=NS(linear=xy(0,0),angular=NS(z=0.))))
        guard.path=msg(poses=[NS(pose=NS(position=xy(x,0))) for x in (0,2)])
        guard.speed=NS(data=1.)
        for name in ('evaluate','fresh','on_map','tick'):
            setattr(guard,name,MethodType(ns[name],guard))
        return guard

    def test_safe_ready_and_input_expiry_is_not_emergency(self):
        guard=self.guard()
        self.assertEqual(guard.evaluate()[:2],(True,False))
        for name in ('map','odom','path','speed'):
            received=getattr(guard,name+'_received')
            setattr(guard,name+'_received',time.monotonic()-10)
            self.assertEqual(guard.evaluate()[:2],(False,False),name)
            setattr(guard,name+'_received',received)

    def test_emergency_checked_even_if_planner_stopped_publishing(self):
        guard=self.guard()
        guard.environment=corridor(obstacles=[('box',[(.5,-.1),(.6,-.1),(.6,.1),(.5,.1)],0.)])
        self.assertEqual(guard.evaluate()[:2],(False,False))  # bad route, stationary
        guard.path_received=None
        guard.odom.twist.twist.linear.x=1.
        self.assertEqual(guard.evaluate()[:2],(False,True))

    def test_typed_profile_is_authoritative_and_checks_its_speed(self):
        guard=self.guard()
        guard.environment=corridor(obstacles=[('box',[(2,-.1),(2.1,-.1),(2.1,.1),(2,.1)],0.)])
        self.assertEqual(guard.evaluate()[:2],(True,False))
        guard.profile=msg(closed=False,points=[NS(pose=NS(position=xy(x,0)),longitudinal_velocity_mps=3.) for x in (0,4)])
        guard.profile_received=time.monotonic()
        self.assertEqual(guard.evaluate()[:2],(False,False))
        self.assertIn('unsafe',guard.evaluate()[2])
        guard.profile_received-=10
        self.assertIn('stale',guard.evaluate()[2])

    def test_invalid_reload_revokes_previously_valid_environment(self):
        guard=self.guard()
        guard.on_map(msg(valid=False,reason='reload failed'))
        self.assertEqual(guard.evaluate(),(False,False,'reload failed'))
        lanes=[NS(left_bound=[xy(-10,2),xy(10,2)],right_bound=[xy(-10,-2),xy(10,-2)],closed=False)]
        guard.on_map(msg(valid=True,lanes=lanes,obstacles=[]))
        self.assertEqual(guard.evaluate()[:2],(True,False))
        guard.on_map(msg(valid=True,lanes=lanes,obstacles=[NS(id='bad',polygon=[xy(0,0)],margin_m=0.)]))
        self.assertEqual(guard.evaluate()[:2],(False,False))

    def test_wrong_velocity_frame_and_bad_timestamp_block(self):
        guard=self.guard()
        guard.odom.child_frame_id='camera'
        self.assertEqual(guard.evaluate()[:2],(False,False))
        guard.odom.child_frame_id='base_link'
        guard.odom.header.stamp=False
        self.assertEqual(guard.evaluate()[:2],(False,False))

    def test_tf_failure_publishes_atomic_blocked_status(self):
        guard=self.guard()
        def fail(*args): raise RuntimeError('no TF')
        guard.tf.lookup_transform=fail
        outputs=[]
        guard.pub=NS(publish=outputs.append)
        guard.get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:0))
        guard.tick()
        self.assertEqual((outputs[0].ready,outputs[0].emergency),(False,False))
        self.assertIn('no TF',outputs[0].reason)

    def test_publisher_uses_physical_bounds_and_obstacle_margin(self):
        ns=dict(DrivableArea=lambda:msg(lanes=[],obstacles=[]),DrivableLane=lambda:NS(),
                StaticObstacle=lambda:NS(),Point=lambda:NS())
        load_functions(PACKAGE/'hd_map_publisher_node.py',{'publish_environment','to_geometry_point'},ns)
        outputs=[]
        lane=NS(lane_id='main',closed_loop=False,left_bound=[(0,1,0),(2,1,0)],right_bound=[],
                drivable_left_bound=[(0,2,0),(2,2,0)],drivable_right_bound=[(0,-2,0),(2,-2,0)])
        publisher=NS(environment_valid=True,hd_map=NS(frame_id='map',lanes=[lane],obstacles=[
            dict(id='box',polygon=[(0,0,0),(1,0,0),(0,1,0)],margin_m=.2)]),
            get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:0)),environment_pub=NS(publish=outputs.append))
        ns['publish_environment'](publisher)
        self.assertEqual(outputs[-1].lanes[0].left_bound[0].y,2)
        self.assertEqual(outputs[-1].obstacles[0].margin_m,.2)
        publisher.environment_valid=False
        ns['publish_environment'](publisher)
        self.assertFalse(outputs[-1].valid)
        self.assertEqual(outputs[-1].lanes,[])

    def test_failed_file_reload_revokes_permission_and_restore_recovers(self):
        from pathlib import Path
        signature = [None]
        ns = dict(Path=Path, hd_map_file_signature=lambda _:signature[0])
        load_functions(PACKAGE/'hd_map_publisher_node.py', {'try_load_hd_map'}, ns)
        old_map = object()
        publisher=NS(hd_map_yaml_path='/tmp/safety-test-map.yaml',hd_map=old_map,
                     loaded_map_signature=123,rejected_map_signature=None,last_load_issue_key=None,
                     environment_valid=True,log_load_issue=lambda *a,**kw:None)
        self.assertFalse(ns['try_load_hd_map'](publisher))
        self.assertFalse(publisher.environment_valid)
        self.assertIs(publisher.hd_map,old_map)
        signature[0]=123
        self.assertFalse(ns['try_load_hd_map'](publisher))
        self.assertTrue(publisher.environment_valid)

    def test_tuning_environment_uses_applied_snapshot(self):
        from pathlib import Path
        ns=dict(DrivableArea=lambda:msg(lanes=[],obstacles=[]),DrivableLane=lambda:NS(),
                StaticObstacle=lambda:NS(),Point=lambda:NS())
        # The adapter is intentionally not imported: its live ROS runtime is Jetson-only.
        root=Path(__file__).resolve().parents[5]
        import ast
        tree=ast.parse((root/'tools/app/runtime/tuning_bridge.py').read_text())
        function=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='build_environment')
        exec(compile(ast.Module(body=[function],type_ignores=[]),'<tuning adapter>','exec'),ns)
        snapshot={'frame_id':'map','hd_map':{'lanes':[{'id':'lane','closed_loop':False,
            'left_bound':[[0,1],[2,1]],'right_bound':[[0,-1],[2,-1]],
            'drivable_left_bound':[[0,2],[2,2]],'drivable_right_bound':[[0,-2],[2,-2]]}],
            'obstacles':[{'id':'box','polygon':[[0,0],[1,0],[0,1]],'margin_m':.3}]}}
        output=ns['build_environment'](None,snapshot)
        self.assertTrue(output.valid)
        self.assertEqual(output.lanes[0].left_bound[0].y,2.)
        self.assertEqual(output.obstacles[0].margin_m,.3)


if __name__ == '__main__': unittest.main()
