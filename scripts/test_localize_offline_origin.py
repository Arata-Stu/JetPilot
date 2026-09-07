"""Standard-library tests; ROS interfaces are replaced with fakes."""
import copy
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import localize_offline_origin as target


class OriginTest(unittest.TestCase):
    def run_fake(self, succeeds=True):
        events, callbacks, pending = [], {}, []
        class Pose:
            def __init__(self):
                self.header = NS(frame_id='', stamp=None)
                self.pose = NS(pose=NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=0.)))
        def publish(msg):
            self.assertIn('path', events)
            self.assertEqual(msg.header.frame_id, 'map')
            self.assertEqual(msg.pose.pose.orientation.w, 1.)
            self.assertEqual(msg.pose.pose.position.x, 0.)
            events.append('hint')
            if succeeds:
                pending.append(lambda: callbacks['/localization/pose_hint_state'](NS(data='{"state":"localized"}')))
        def subscribe(_, topic, cb, qos):
            callbacks[topic] = cb
            if topic.endswith('pose_hint_state'):
                cb(NS(data='{"state":"waiting_for_manual"}'))
        def client(_, topic):
            def call(req):
                events.append(topic)
                if topic.endswith('/resume'):
                    def path():
                        events.append('path')
                        callbacks['/visual_slam/tracking/slam_path'](NS(poses=[1], header=NS(stamp=123)))
                    pending.append(path)
                else:
                    self.assertIn('hint', events)
                    self.assertEqual(req.rate, 1.)
                return NS(done=lambda: True, result=lambda: NS(success=True))
            return NS(service_is_ready=lambda: True, call_async=call)
        node = NS(create_subscription=subscribe, create_publisher=lambda *a: NS(publish=publish, get_subscription_count=lambda: 1),
                  create_client=client, destroy_node=lambda: events.append('destroy'))
        def spin(*a, **kw):
            if pending:
                pending.pop(0)()
        modules = {'rclpy': NS(init=lambda:None, create_node=lambda _:node, ok=lambda:True,
                              spin_once=spin, shutdown=lambda:events.append('shutdown')),
                   'rclpy.qos': NS(QoSProfile=lambda **kw:kw, ReliabilityPolicy=NS(BEST_EFFORT=1), DurabilityPolicy=NS(TRANSIENT_LOCAL=1)),
                   'geometry_msgs.msg': NS(PoseWithCovarianceStamped=Pose), 'nav_msgs.msg': NS(Path=object),
                   'std_msgs.msg': NS(String=object), 'rosbag2_interfaces.srv': NS(Resume=NS(Request=NS), SetRate=NS(Request=NS))}
        with patch.dict(sys.modules, modules), patch.object(target.time, 'monotonic', side_effect=range(1000)):
            if succeeds:
                target.localize(20, 1.)
                self.assertEqual(events.count('hint'), 1)
                self.assertIn('/rosbag2_player/set_rate', events)
            else:
                with self.assertRaisesRegex(RuntimeError, 'saved-map localization: timeout'):
                    target.localize(20, 1.)
                self.assertNotIn('/rosbag2_player/set_rate', events)
        self.assertEqual(events[-2:], ['destroy', 'shutdown'])

    def test_resume_track_hint_confirm_rate(self):
        self.run_fake()

    def test_failure_does_not_restore_rate(self):
        self.run_fake(False)

    def test_snapshot_gate(self):
        valid = {'localization': {'required': True, 'confirmed': True},
                 'odometry_samples': [{'frame_id': 'map'}, {'frame_id': 'map'}],
                 'landmarks': {'width': 1, 'height': 1, 'data': 'AAAA'}}
        variants = [valid]
        for key, value in [('localization', {'required': False, 'confirmed': False}),
                           ('odometry_samples', [{'frame_id': 'odom'}]),
                           ('landmarks', {'width': 0, 'height': 1, 'data': ''})]:
            bad = copy.deepcopy(valid)
            bad[key] = value
            variants.append(bad)
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'snapshot.json'
            for i, data in enumerate(variants):
                p.write_text(json.dumps(data))
                if i:
                    with self.assertRaises(ValueError): target.validate_snapshot(p)
                else: target.validate_snapshot(p)


if __name__ == '__main__':
    unittest.main()
