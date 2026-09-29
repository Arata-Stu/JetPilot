"""Bounded motion and abort behavior using a simulated ROS graph, no actuators."""
import contextlib
import importlib.util
import io
import sys
import signal
import types
import unittest
from pathlib import Path
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location(
    'drive', Path(__file__).resolve().parents[1] / 'scripts/experiments/rc_popout_timed_drive.py')
DRIVE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(DRIVE)
NS = types.SimpleNamespace


class Message:
    AUTO, STOP = 1, 3

    def __init__(self):
        self.header = NS(stamp=None)


class FakeROS:
    def __init__(self, failure=None):
        self.now = 0.0
        self.mode = 4 if failure == 'propo' else Message.STOP
        self.callbacks = {}
        self.commands = []
        self.requests = []
        self.failure = failure
        self.started = False
        self.stopped = False

    def create_publisher(self, cls, topic, qos):
        def publish(msg):
            if topic == '/auto/control_cmd':
                self.commands.append((self.now, msg.throttle, msg.brake, msg.reverse))
                self.started |= msg.throttle > 0
            else:
                self.mode = msg.mode
                self.requests.append(msg.mode)
        return NS(publish=publish, get_subscription_count=lambda: 1)

    def create_subscription(self, cls, topic, callback, qos):
        self.callbacks[topic] = callback

    def count_publishers(self, topic):
        return 2 if (self.failure == 'competitor' or
                     self.started and self.failure == 'late_competitor') else 1

    def count_subscribers(self, topic):
        return 1

    def get_clock(self):
        return NS(now=lambda: NS(to_msg=lambda: None))

    def destroy_node(self):
        self.stopped = True

    def sleep(self, seconds):
        self.now += seconds

    def spin(self, node, timeout_sec):
        self.now += timeout_sec
        if self.started and self.failure == 'override':
            self.mode = Message.STOP
        if self.started and self.failure == 'signal':
            signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
        if self.started and self.failure == 'stall':
            self.now += 0.3
        self.callbacks['/operation_mode/state'](NS(mode=self.mode))
        self.callbacks['/bag/status'](NS(
            recording=not (self.started and self.failure == 'recording'),
            current_uri=('/workspaces/record/2026-09-30/test_move_02'
                         if self.failure in ('fixed_name', 'byte_level') else
                         '/record/other' if self.started and self.failure == 'uri_changed' else '/record/trial'),
            last_event='start ignored: already recording' if self.failure == 'already_recording' else 'recording started'))
        if self.failure == 'no_diagnostics' or self.started and self.failure == 'stale':
            return
        values = dict(selector='HOST', status_fresh='true',
                      host_arm_state='ARMED' if self.mode == Message.AUTO else 'DISARMED')
        self.callbacks['/diagnostics'](NS(status=[NS(
            name='jetpilot_bridge_interface',
            level=b'\x00' if self.failure == 'byte_level' else b'\x02' if self.failure == 'byte_error' else 0,
            values=[NS(key=k, value=v) for k, v in values.items()])]))

    def modules(self):
        return {
            'rclpy': NS(init=lambda **kw: None, create_node=lambda name: self,
                        spin_once=self.spin, shutdown=lambda: None),
            'rclpy.qos': NS(QoSProfile=lambda **kw: None,
                            ReliabilityPolicy=NS(BEST_EFFORT=1),
                            DurabilityPolicy=NS(TRANSIENT_LOCAL=1)),
            'rclpy.signals': NS(SignalHandlerOptions=NS(NO=0)),
            'diagnostic_msgs.msg': NS(DiagnosticArray=Message),
            'jetpilot_msgs.msg': NS(ControlCommand=Message, OperationModeRequest=Message,
                                    OperationModeState=Message, BagStatus=Message),
        }


class TimedDriveTests(unittest.TestCase):
    def args(self):
        return DRIVE.arguments(['--throttle', '0.1', '--duration', '2', '--brake', '0.2',
                                '--brake-duration', '1', '--label', 'trial'])

    def execute(self, ros):
        with patch.dict(sys.modules, ros.modules()), \
                patch.object(DRIVE.time, 'monotonic', lambda: ros.now), \
                patch.object(DRIVE.time, 'sleep', ros.sleep), contextlib.redirect_stdout(io.StringIO()):
            DRIVE.run(self.args())

    def test_deadline_brake_and_stop(self):
        ros = FakeROS()
        self.execute(ros)
        drive = [c for c in ros.commands if c[1] > 0]
        brake = [c for c in ros.commands if c[2] > 0]
        self.assertTrue(drive and brake)
        self.assertLessEqual(drive[-1][0] - drive[0][0], 2.0)
        self.assertAlmostEqual(brake[0][0] - drive[0][0], 2.0, delta=0.04)
        self.assertEqual(ros.requests.count(Message.AUTO), 1)
        self.assertEqual(ros.requests[-1], Message.STOP)
        self.assertEqual(ros.commands[-1][1:], (0, 0, 0))
        self.assertTrue(all(not (c[1] and c[2]) and c[3] == 0 for c in ros.commands))

    def test_aborts_do_not_resume_or_rearm(self):
        for failure in ('override', 'recording', 'stall', 'stale', 'late_competitor', 'signal', 'uri_changed'):
            with self.subTest(failure=failure):
                ros = FakeROS(failure)
                with self.assertRaises(RuntimeError):
                    self.execute(ros)
                self.assertEqual(ros.requests.count(Message.AUTO), 1)
                self.assertEqual(ros.requests[-1], Message.STOP)
                self.assertEqual(ros.commands[-1][1:], (0, 0, 0))

    def test_competing_publisher_prevents_arming(self):
        ros = FakeROS('competitor')
        with self.assertRaises(RuntimeError):
            self.execute(ros)
        self.assertFalse(ros.requests)
        self.assertFalse(ros.commands)

    def test_preflight_reports_failed_condition_without_commands(self):
        for failure, expected in (
                ('propo', '[NG] /operation_mode/state: PROPO'),
                ('already_recording', '[NG] Bag STARTの重複拒否なし'),
                ('byte_error', '[NG] JPBB診断level: 2'),
                ('no_diagnostics', '[NG] JPBB診断の受信: 未受信'),
                ('competitor', '[NG] AUTO publisher数: 2')):
            with self.subTest(failure=failure):
                ros = FakeROS(failure)
                with self.assertRaises(RuntimeError) as result:
                    self.execute(ros)
                self.assertIn(expected, str(result.exception))
                self.assertFalse(ros.commands)
                self.assertFalse(ros.requests)

    def test_fixed_recording_name_and_ros_byte_level(self):
        for case in ('fixed_name', 'byte_level'):
            with self.subTest(case=case):
                ros = FakeROS(case)
                self.execute(ros)
                self.assertTrue(ros.started)
                self.assertEqual(ros.requests[-1], Message.STOP)

    def test_diagnostic_level_normalization(self):
        for raw in (0, b'\x00', bytearray([0])):
            self.assertEqual(DRIVE.diagnostic_level(raw), 0)
        for raw in (b'\x01', b'\x02', b'\x03', b'', b'00', '0', None):
            self.assertNotEqual(DRIVE.diagnostic_level(raw), 0)

    def test_invalid_inputs(self):
        for key, value in [('throttle', 'nan'), ('duration', '0'), ('duration', '61'),
                           ('brake', '-0.1'), ('brake-duration', 'inf'), ('steering', '2')]:
            with self.subTest(key=key, value=value), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    DRIVE.arguments(['--throttle', '0.1', '--duration', '2', '--brake', '0.2',
                                     '--' + key, value])


if __name__ == '__main__':
    unittest.main()
