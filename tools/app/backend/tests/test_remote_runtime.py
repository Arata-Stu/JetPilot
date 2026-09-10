import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from jetpilot_console import remote_runtime as transport
from jetpilot_console import remote_runtime_agent as agent


class RuntimeTests(unittest.TestCase):
    def settings(self, **kw):
        body = {'host': 'jetson.local', 'user': 'pilot', 'container': 'isaac', 'container_user': 'admin'}
        body.update(kw)
        return transport.settings(body)

    def test_arguments_and_metadata_auto_mode(self):
        s = self.settings(preset='e2e', model='/workspaces/models/a b')
        args = transport.bringup_args(s)
        self.assertIn('/workspaces/scripts/bringup.sh', args)
        self.assertIn('/workspaces/models/a b', args)
        self.assertNotIn('e2e_fixed_throttle_mode:=true', args)
        args = transport.bringup_args(self.settings(fixed=True))
        self.assertIn('teleop_fixed_throttle_mode:=true', args)
        evs = transport.bringup_args(self.settings(
            sensor='event-camera', evs_window_ms=50, evs_stride_ms=10))
        self.assertIn('sensor_kit_silky_evcam_event_image_window_ms:=50.0', evs)
        self.assertIn('sensor_kit_silky_evcam_event_image_stride_ms:=10.0', evs)
        self.assertIn('sensor_kit_silky_evcam_event_image_fps:=100.0', evs)

    def test_jetson_connection_defaults(self):
        settings = transport.settings({'host': '192.168.11.190'})
        self.assertEqual(settings['target'], 'tamiya@192.168.11.190')
        self.assertEqual(settings['container'], 'isaac_ros_dev_container')
        self.assertEqual(settings['container_user'], 'admin')
        self.assertEqual(settings['host_workspace'], '/home/tamiya/workspaces/JetPilot')

    def test_rejects_invalid_input_and_required_paths(self):
        for kw in ({'host': '-bad'}, {'session': '; ls'}, {'container': '../x'}, {'throttle': 'nan'}, {'rgb_fps': 29}, {'bringup': 'relative'}, {'evs_window_ms': 0}, {'evs_window_ms': 10, 'evs_stride_ms': 20}):
            with self.assertRaises(ValueError):
                self.settings(**kw)
        for preset in ('e2e', 'offline-vslam', 'competition'):
            with self.assertRaises(ValueError):
                transport.bringup_args(self.settings(preset=preset))

    def test_offline_has_no_vehicle_or_live_sensor(self):
        args = transport.bringup_args(self.settings(preset='offline-vslam', bag='/bags/test'))
        self.assertNotIn('--vehicle', args)
        self.assertNotIn('--sensor-kit', args)
        self.assertIn('--bag', args)

    def test_dynamic_parameters_are_allowlisted(self):
        body = {'host':'jetson.local','user':'pilot','container':'isaac','container_user':'admin',
                'node':'/e2e_control_decoder','parameter':'steering_offset','value':-0.2}
        with patch.object(transport.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0, '{"value":-0.2}', '')) as call:
            self.assertEqual(transport.request(body,'param-set')['value'],-0.2)
            payload = json.loads(call.call_args.kwargs['input'])
            self.assertIn('ros_script',payload)
        for changes in ({'node':'/other'}, {'parameter':'fixed_throttle_mode'}, {'value':2}):
            with self.assertRaises(ValueError):
                transport.request(dict(body,**changes),'param-set')

    def test_controller_algorithm_uses_string_and_tuning_requires_map(self):
        body = {'host':'jetson.local','user':'pilot','container':'isaac','container_user':'admin',
                'node':'/path_tracking_controller_node','parameter':'algorithm','value':'map_pursuit'}
        with patch.object(transport.subprocess, 'run', return_value=subprocess.CompletedProcess([],0,'{"value":"map_pursuit"}','')) as call:
            result = transport.request(body,'param-set')
            self.assertEqual(result['value'],'map_pursuit')
            self.assertEqual(json.loads(call.call_args.kwargs['input'])['value'],'map_pursuit')
        with self.assertRaises(ValueError):
            transport.request(dict(body,value='unknown'),'param-set')
        with self.assertRaises(ValueError):
            transport.bringup_args(self.settings(preset='tuning'))
        args = transport.bringup_args(self.settings(preset='tuning',map='/maps/test'))
        self.assertIn('tuning',args)
        self.assertIn('/maps/test',args)

    def fake_remote(self, action, alive=True, screen=False, exists=False, owned=True, dead=False):
        calls = []
        def fake(args, check=True):
            calls.append(args)
            code, out = 0, ''
            if args[:2] == ['docker', 'inspect']:
                out = 'true\n' if alive else 'false\n'
            elif args[:2] == ['screen', '-ls']:
                out = '  123.jetpilot-web (Detached)\n' if screen else ''
            elif 'tmux' in args:
                command = args[args.index('tmux') + 1]
                if command == 'has-session': code = 0 if exists else 1
                if command == 'show-options': out = '1\n' if owned else ''
                if command == 'list-panes':
                    code = 0 if exists else 1
                    out = f'%1 {int(dead)} 0\n' if exists and 'pane_dead_status' in args[-1] else f'%1 {int(dead)}\n' if exists else ''
                if command == 'capture-pane': out = 'example log'
            return subprocess.CompletedProcess(args, code, out, '')
        with tempfile.TemporaryDirectory() as tmp, patch.object(agent.Path, 'home', return_value=Path(tmp)), patch.object(agent, 'run', side_effect=fake):
            s = self.settings(host_workspace=tmp)
            s['action'] = action
            s['command'] = transport.bringup_args(s)
            result = agent.execute(s)
        return result, calls

    def test_prepare_activates_inside_screen_even_when_docker_running(self):
        _, calls = self.fake_remote('prepare', alive=True)
        launch = next(c for c in calls if c[0] == 'screen' and '-dmS' in c)
        self.assertIn('isaac-ros activate', launch[-1])
        self.assertIn('bash', launch)
        _, calls = self.fake_remote('prepare', alive=True, screen=True)
        self.assertFalse(any('-dmS' in c for c in calls))

    def test_status_autodetects_the_only_jetpilot_container(self):
        calls = []
        def fake(args, check=True):
            calls.append(args)
            if args[:3] == ['docker', 'inspect', '-f']:
                if args[-1] == 'actual_isaac_container':
                    return subprocess.CompletedProcess(args, 0, 'true\n', '')
                return subprocess.CompletedProcess(args, 1, '', 'Error: No such object')
            if args[:3] == ['docker', 'ps', '--format']:
                return subprocess.CompletedProcess(args, 0, 'actual_isaac_container\n', '')
            if args[:3] == ['docker', 'exec', 'actual_isaac_container'] and 'test' in args:
                return subprocess.CompletedProcess(args, 0, '', '')
            if args[:2] == ['screen', '-ls']:
                return subprocess.CompletedProcess(args, 0, '', '')
            if 'tmux' in args:
                return subprocess.CompletedProcess(args, 1, '', 'no server running')
            return subprocess.CompletedProcess(args, 0, '', '')
        with tempfile.TemporaryDirectory() as tmp, patch.object(agent.Path, 'home', return_value=Path(tmp)), patch.object(agent, 'run', side_effect=fake):
            settings = self.settings(container='old_saved_name', host_workspace=tmp)
            settings['action'] = 'status'
            result = agent.execute(settings)
        self.assertTrue(result['container_running'])
        self.assertEqual(result['container'], 'actual_isaac_container')
        self.assertIn('自動検出', result['message'])

    def test_missing_container_is_a_normal_disconnected_state(self):
        def fake(args, check=True):
            if args[:3] == ['docker', 'inspect', '-f']:
                return subprocess.CompletedProcess(args, 1, '', 'Error: No such container: isaac_ros_dev_container')
            if args[:3] == ['docker', 'ps', '--format']:
                return subprocess.CompletedProcess(args, 0, '', '')
            if args[:2] == ['screen', '-ls']:
                return subprocess.CompletedProcess(args, 0, '', '')
            return subprocess.CompletedProcess(args, 0, '', '')
        with tempfile.TemporaryDirectory() as tmp, patch.object(agent.Path, 'home', return_value=Path(tmp)), patch.object(agent, 'run', side_effect=fake):
            settings = self.settings(host_workspace=tmp)
            settings['action'] = 'status'
            result = agent.execute(settings)
        self.assertFalse(result['container_running'])
        self.assertFalse(result['screen_running'])

    def test_start_does_not_duplicate_or_take_manual_session(self):
        for owned in (True, False):
            with self.assertRaises(RuntimeError):
                self.fake_remote('start', exists=True, owned=owned)

    def test_stop_only_interrupts_managed_pane(self):
        _, calls = self.fake_remote('stop', exists=True)
        self.assertTrue(any('send-keys' in c and c[-1] == 'C-c' for c in calls))
        self.assertFalse(any('kill-session' in c or 'kill' in c or 'stop' in c for c in calls))

    def test_start_validates_before_create_and_retains_exit_log(self):
        _, calls = self.fake_remote('start')
        preview = next(i for i,c in enumerate(calls) if '--dry-run' in c[-1])
        create = next(i for i,c in enumerate(calls) if 'new-session' in c)
        self.assertLess(preview, create)
        self.assertTrue(any('remain-on-exit' in c for c in calls))
        result, _ = self.fake_remote('status', exists=True, dead=True)
        self.assertEqual(result['state'], 'exited')
        self.assertEqual(result['log'], 'example log')

    def test_ssh_uses_json_payload_and_keeps_timeout_uncertain(self):
        s = {'host':'jetson.local','user':'pilot','container':'isaac','container_user':'admin'}
        with patch.object(transport.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0, '{"state":"running"}', '')) as call:
            self.assertEqual(transport.request(s, 'status')['state'], 'running')
            self.assertIn('BatchMode=yes', call.call_args.args[0])
            self.assertEqual(json.loads(call.call_args.kwargs['input'])['container'], 'isaac')
        with patch.object(transport.subprocess, 'run', side_effect=subprocess.TimeoutExpired('ssh', 25)):
            with self.assertRaisesRegex(ValueError, '未確認'):
                transport.request(s, 'prepare')


if __name__ == '__main__':
    unittest.main()
