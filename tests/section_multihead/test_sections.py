"""Standard-library-only tests: no ROS, torch, numpy or OpenCV required."""
import importlib.util
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'python_ws/jetpilot_e2e_training/src'))
from e2e_learning.data.sections import CausalLabels, SectionMap, map_digest, split_sequences, validate_heads
from e2e_learning.data.bag_sections import TransformTimeline, recorded_labels
from types import SimpleNamespace as NS

spec = importlib.util.spec_from_file_location('guard', ROOT/'ros2_ws/src/perception/jetpilot_e2e_inference/jetpilot_e2e_inference/section_guard.py')
guard_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(guard_module)


def document():
    return {'frame_id':'map', 'primary_lane_id':'lane',
            'lanes':[{'id':'lane', 'closed_loop':False, 'centerline':[[0,0],[10,0]]}],
            'sections':[{'id':'straight','lane_id':'lane','start_s_m':0,'end_s_m':5},
                        {'id':'curve','lane_id':'lane','start_s_m':5,'end_s_m':10}]}


class SectionsTests(unittest.TestCase):
    def test_boundary_belongs_to_next_section(self):
        geometry = SectionMap(document())
        self.assertEqual(geometry.resolve(4.99,0), 'straight')
        self.assertEqual(geometry.resolve(5,0), 'curve')
        self.assertEqual(geometry.resolve(10,0), 'curve')
        self.assertEqual(geometry.resolve(5,2), 'unknown')
        self.assertEqual(geometry.resolve(float('nan'),0), 'unknown')

    def test_closed_loop_wrap_and_full_lap(self):
        d = document()
        d['lanes'][0].update(closed_loop=True, centerline=[[0,0],[5,0],[5,5],[0,5]])
        d['sections'] = [{'id':'wrap','lane_id':'lane','start_s_m':17,'end_s_m':2}]
        self.assertEqual(SectionMap(d).resolve(0,1),'wrap')
        self.assertEqual(SectionMap(d).resolve(1,0),'wrap')
        self.assertEqual(SectionMap(d).resolve(3,0),'unknown')
        d['sections'][0].update(start_s_m=0,end_s_m=0,start_gate_id='a',end_gate_id='a')
        self.assertEqual(SectionMap(d).resolve(5,2),'wrap')

    def test_lane_network_requires_lane(self):
        d = document(); d['lanes'][0]['successor_ids']=['other']
        geometry = SectionMap(d)
        self.assertEqual(geometry.resolve(2,0),'unknown')
        self.assertEqual(geometry.resolve(2,0,'lane'),'straight')

    def test_labels_never_use_future_or_stale_section(self):
        labels = CausalLabels([(200,'curve'),(100,'straight'),(250,'unknown')], 0.0000001)
        self.assertEqual(labels.at(99),'unknown')
        self.assertEqual(labels.at(199),'straight')
        self.assertEqual(labels.at(200),'curve')
        self.assertEqual(labels.at(251),'unknown')
        self.assertEqual(CausalLabels([(100,'straight')], 0.0000001).at(201),'unknown')

    def test_hash_changes_on_section_geometry(self):
        d = document(); before = map_digest(d)
        self.assertEqual(before,map_digest(dict(reversed(list(d.items())))))
        d['sections'][0]['end_s_m']=4
        self.assertNotEqual(before,map_digest(d))

    def test_routing_contract(self):
        heads = [{'name':'straight','sections':['straight'],'throttle':.3},
                 {'name':'curve','sections':['curve'],'throttle':.2},
                 {'name':'generic','generic':True,'sections':['straight','curve'],'throttle':.2}]
        validate_heads(heads,['straight','curve'])
        heads[1]['sections']=['straight']
        with self.assertRaises(ValueError): validate_heads(heads,['straight','curve'])
        heads[1]['sections']=['curve']; heads[1]['throttle']=float('nan')
        with self.assertRaises(ValueError): validate_heads(heads,['straight','curve'])

    def test_split_holds_out_entire_bags(self):
        rows = [{'sequence_id':s,'stamp':str(i*1_000_000_000)} for s in ['a','b','c'] for i in range(10)]
        train, valid, kind = split_sequences(rows)
        self.assertEqual(kind,'bag')
        self.assertFalse({rows[i]['sequence_id'] for i in train} & {rows[i]['sequence_id'] for i in valid})

    def test_temporal_split_has_gap(self):
        rows = [{'sequence_id':'a','stamp':str(i*500_000_000)} for i in range(20)]
        train, valid, kind = split_sequences(rows)
        self.assertGreater(int(rows[valid[0]]['stamp'])-int(rows[train[-1]]['stamp']),1_000_000_000)
        with self.assertRaises(ValueError): split_sequences(rows[:1])

    def test_tf_composition_rotation_inverse_and_stale(self):
        timeline = TransformTimeline(100)
        def tf(parent,child,x,y,q=(0,0,0,1)):
            return NS(header=NS(frame_id=parent),child_frame_id=child,
                      transform=NS(translation=NS(x=x,y=y,z=0),rotation=NS(x=q[0],y=q[1],z=q[2],w=q[3])))
        timeline.add(tf('map','odom',10,0,(0,0,math.sqrt(.5),math.sqrt(.5))),100)
        timeline.add(tf('odom','base_link',2,0),100)
        timeline.finish()
        result = timeline.position(150,'map','base_link')
        self.assertAlmostEqual(result[0],10); self.assertAlmostEqual(result[1],2)
        self.assertIsNone(timeline.position(201,'map','base_link'))
        self.assertIsNone(timeline.position(99,'map','base_link'))
        inverse = timeline.position(150,'base_link','map')
        self.assertAlmostEqual(inverse[0],-2); self.assertAlmostEqual(inverse[1],10)

    def bag_labels(self, messages, request=None):
        class Reader:
            def __init__(self, paths):
                self.connections = [NS(topic=t, msgtype='fake') for t in dict.fromkeys(m[0] for m in messages)]
            def __enter__(self): return self
            def __exit__(self, *args): pass
            def deserialize(self, raw, kind): return raw
            def messages(self, connections):
                for topic, stamp, msg in messages:
                    yield next(c for c in self.connections if c.topic == topic), stamp, msg
        with patch.dict(sys.modules, {'rosbags':NS(), 'rosbags.highlevel':NS(AnyReader=Reader)}):
            return recorded_labels('/bag',[{'stamp':150},{'stamp':250},{'stamp':350}],
                {'map_document':document(), 'max_label_age_sec':.0000001, **(request or {})})

    def test_recorded_sections_are_causal_and_unknown_is_retained(self):
        result = self.bag_labels([('/localization/current_section',100,NS(data='straight')),
                                 ('/localization/current_section',200,NS(data='unknown'))])
        self.assertEqual(result,{150:'straight',250:'unknown',350:'unknown'})

    def test_stamped_section_map_mismatch_rejected(self):
        msg=NS(header=NS(stamp=NS(sec=0,nanosec=100)),section_id='straight',map_sha256='wrong',valid=True)
        with self.assertRaisesRegex(ValueError,'地図'):
            self.bag_labels([('/localization/section_state',100,msg)])
        result=self.bag_labels([('/localization/section_state',100,msg)],{'label_source':'pose'})
        self.assertTrue(all(value == 'unknown' for value in result.values()))

    def test_stamped_section_preferred_and_health_loss_excluded(self):
        msg=NS(header=NS(stamp=NS(sec=0,nanosec=100)),section_id='curve',map_sha256=map_digest(document()),valid=True)
        result=self.bag_labels([('/localization/current_section',100,NS(data='straight')),
            ('/localization/section_state',100,msg),
            ('/localization/pose_hint_state',100,NS(data='{"state":"localized"}')),
            ('/localization/pose_hint_state',200,NS(data='{"state":"unlocalized"}'))])
        self.assertEqual(result[150],'curve')
        self.assertEqual(result[250],'unknown')

    def test_odom_frame_pose_is_never_used_as_map_pose(self):
        msg=NS(header=NS(frame_id='odom',stamp=NS(sec=0,nanosec=100)),child_frame_id='base_link',
               pose=NS(pose=NS(position=NS(x=2,y=0,z=0))))
        result=self.bag_labels([('/visual_slam/tracking/odometry',100,msg)],{'label_source':'pose'})
        self.assertTrue(all(label == 'unknown' for label in result.values()))

    def test_guard_fallback_and_stable_recovery(self):
        guard = guard_module.SectionGuard(stable_sec=1.,switch_sec=.1,max_speed=3.,jump_margin=.2)
        self.assertEqual(guard.update(0,'straight',True,(0,0,0),0),'unknown')
        self.assertEqual(guard.update(1.1,'straight',True,(.1,0,0),1.1),'straight')
        self.assertEqual(guard.update(1.2,'curve',True,(20,0,0),1.2),'unknown')
        self.assertEqual(guard.update(1.3,'curve',True,(20,0,0),1.3),'unknown')
        self.assertEqual(guard.update(2.4,'curve',True,(20,0,0),2.4),'curve')
        self.assertEqual(guard.update(2.5,'curve',False,(20,0,0),2.5),'unknown')

    def test_guard_section_flicker_does_not_switch(self):
        guard = guard_module.SectionGuard(stable_sec=0.,switch_sec=.15)
        guard.update(0,'straight',True,(0,0,0),0)
        self.assertEqual(guard.update(.2,'straight',True,(0,0,0),.2),'straight')
        self.assertEqual(guard.update(.21,'curve',True,(0,0,0),.21),'straight')
        self.assertEqual(guard.update(.25,'straight',True,(0,0,0),.25),'straight')
        self.assertEqual(guard.update(.3,'unknown',True,(0,0,0),.3),'unknown')


if __name__ == '__main__': unittest.main()
