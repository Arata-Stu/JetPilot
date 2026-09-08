"""Standard-library-only launcher tests; no ROS or camera is started."""
import errno
import ast
import hashlib
import json
import os
from pathlib import Path
import pty
import select
import shlex
import subprocess
import sys
import tempfile
import time
import unittest
import xml.etree.ElementTree as ET


ROOT = next(p for p in Path(__file__).resolve().parents if (p / 'scripts/bringup.sh').is_file())


class VslamTuiTest(unittest.TestCase):
    def launch(self, mode='vo', overrides=(), preset='localization-only', init='pose-hint', line='current', closed=True, rgb='30', infra='60', multicam='1'):
        with tempfile.TemporaryDirectory() as directory:
            custom = Path(directory) / 'test_custom_line.csv'
            custom.write_text('0;0;0;0;0;1;0\n1;1;0;0;0;1;0\n')
            custom.with_suffix('.meta.json').write_text(json.dumps({
                'format': 'jetpilot_custom_line_v1', 'id': 'test', 'name': 'Test',
                'closed_loop': closed, 'revision': 1, 'source_hash': 'a' * 64,
                'trajectory_csv': custom.name,
                'trajectory_sha256': hashlib.sha256(custom.read_bytes()).hexdigest(),
            }))
            (Path(directory) / 'test_raceline.csv').write_text(custom.read_text())
            fake_fzf = Path(directory) / 'fzf'
            fake_fzf.write_text(
                f'#!{sys.executable}\n'
                'import os, sys\n'
                'options = sys.stdin.read().splitlines()\n'
                'prompt = next(a for a in sys.argv if a.startswith("--prompt="))\n'
                'if "JetPilot bringup preset" in prompt:\n'
                '    print(next(o for o in options if o.split()[0] == os.environ["TEST_PRESET"]))\n'
                'elif "RealSense rgb Hz" in prompt or "RealSense infra Hz" in prompt:\n'
                '    key = "TEST_RGB" if "rgb Hz" in prompt else "TEST_INFRA"\n'
                '    print(key + "_MENU_SHOWN", file=sys.stderr)\n'
                '    if os.environ[key] == "cancel": sys.exit(130)\n'
                '    print(next(o for o in options if o.split()[0] == os.environ[key]))\n'
                'elif "VSLAM 処理モード" in prompt:\n'
                '    print("MULTICAM_MENU_SHOWN", file=sys.stderr)\n'
                '    if os.environ["TEST_MULTICAM"] == "cancel": sys.exit(130)\n'
                '    print(next(o for o in options if o.split()[0] == os.environ["TEST_MULTICAM"]))\n'
                'elif "VSLAM 追跡モード" in prompt:\n'
                '    print("VSLAM_MENU_SHOWN", file=sys.stderr)\n'
                '    if os.environ["TEST_MODE"] == "cancel": sys.exit(130)\n'
                '    print(next(o for o in options if o.split()[0] == os.environ["TEST_MODE"]))\n'
                'elif "初期位置の決め方" in prompt or "走行ライン" in prompt:\n'
                '    key = "TEST_INIT" if "初期位置" in prompt else "TEST_LINE"\n'
                '    print(key + "_MENU_SHOWN", file=sys.stderr)\n'
                '    if os.environ[key] == "cancel": sys.exit(130)\n'
                '    print(next(o for o in options if o.split()[0] == os.environ[key]))\n'
                'else: print(options[0])\n'
            )
            fake_fzf.chmod(0o755)
            env = dict(os.environ, PATH=f'{directory}:{os.environ["PATH"]}',
                       TEST_MODE=mode, TEST_PRESET=preset, TEST_INIT=init, TEST_LINE=line,
                       TEST_RGB=rgb, TEST_INFRA=infra, TEST_MULTICAM=multicam)
            master, slave = pty.openpty()
            process = subprocess.Popen(
                ['bash', str(ROOT / 'scripts/bringup.sh'), '--dry-run', '--no-bag-manager',
                 '--map', directory, *overrides],
                stdin=slave, stdout=slave, stderr=slave, env=env,
            )
            os.close(slave)
            output = bytearray()
            deadline = time.monotonic() + 15
            try:
                while time.monotonic() < deadline:
                    if not select.select([master], [], [], 0.1)[0]:
                        continue
                    try:
                        chunk = os.read(master, 65536)
                    except OSError as error:
                        if error.errno == errno.EIO:
                            break
                        raise
                    if not chunk:
                        break
                    output.extend(chunk)
                else:
                    self.fail('TUI timed out')
                return process.wait(timeout=2), output.decode()
            finally:
                if process.poll() is None:
                    process.kill()
                process.wait()
                os.close(master)

    def test_multicam_modes_reach_launch(self):
        for mode in ('0', '1', '2'):
            with self.subTest(mode=mode):
                code, output = self.launch(multicam=mode)
                self.assertEqual(code, 0, output)
                self.assertIn('MULTICAM_MENU_SHOWN', output)
                self.assertIn(f'vslam_multicam_mode:={mode}', output)

    def test_explicit_multicam_skips_menu(self):
        code, output = self.launch(multicam='cancel',
                                  overrides=('--set', 'vslam_multicam_mode:=2'))
        self.assertEqual(code, 0, output)
        self.assertNotIn('MULTICAM_MENU_SHOWN', output)
        self.assertIn('vslam_multicam_mode:=2', output)

    def test_invalid_multicam_rejected(self):
        code, output = self.launch(overrides=('--set', 'vslam_multicam_mode:=3'))
        self.assertNotEqual(code, 0, output)
        self.assertIn('vslam_multicam_mode must be', output)

    def test_multicam_cancel_stops_launch(self):
        code, output = self.launch(multicam='cancel')
        self.assertNotEqual(code, 0, output)
        self.assertNotIn('Command:', output)

    def test_modes_reach_launch_with_ground_constraints(self):
        for mode in ('vo', 'vio'):
            with self.subTest(mode=mode):
                code, output = self.launch(mode)
                self.assertEqual(code, 0, output)
                self.assertIn('VSLAM_MENU_SHOWN', output)
                command = shlex.split(output.split('Command:', 1)[1].split('Dry-run:', 1)[0])
                self.assertIn(f'vslam_mode:={mode}', command)
                for target in ('odometry', 'slam'):
                    self.assertIn(f'vslam_enable_ground_constraint_in_{target}:=true', command)

    def test_camera_rates_reach_launch(self):
        for rgb, infra in (('30', '90'), ('60', '30'), ('90', '60')):
            with self.subTest(rgb=rgb, infra=infra):
                code, output = self.launch(preset='sensor', rgb=rgb, infra=infra)
                self.assertEqual(code, 0, output)
                self.assertIn('TEST_RGB_MENU_SHOWN', output)
                self.assertIn('TEST_INFRA_MENU_SHOWN', output)
                self.assertIn(f'sensor_kit_rgb_fps:={rgb}', output)
                self.assertIn(f'sensor_kit_infra_fps:={infra}', output)

    def test_camera_rate_override_skips_its_menu(self):
        code, output = self.launch(preset='sensor', rgb='cancel',
                                  overrides=('--set', 'sensor_kit_rgb_fps:=60'))
        self.assertEqual(code, 0, output)
        self.assertNotIn('TEST_RGB_MENU_SHOWN', output)
        self.assertIn('TEST_INFRA_MENU_SHOWN', output)
        self.assertIn('sensor_kit_rgb_fps:=60', output)

    def test_camera_rates_skip_replay_and_cancel_cleanly(self):
        code, output = self.launch(preset='offline-vslam-map', rgb='cancel', infra='cancel',
                                  overrides=('--bag', '/tmp/test-bag'))
        self.assertEqual(code, 0, output)
        self.assertNotIn('TEST_RGB_MENU_SHOWN', output)
        code, output = self.launch(preset='sensor', rgb='cancel')
        self.assertNotEqual(code, 0)
        self.assertNotIn('Command:', output)

    def test_offline_map_uses_origin_without_initialization_menu(self):
        code, output = self.launch(preset='offline-vslam-map', init='cancel',
                                   overrides=('--bag', '/tmp/test-bag'))
        self.assertEqual(code, 0, output)
        self.assertNotIn('TEST_INIT_MENU_SHOWN', output)
        self.assertIn('vslam_localize_on_startup:=true', output)
        self.assertIn('enable_vgl:=false', output)
        self.assertIn('enable_localization_manager:=false', output)

    def test_explicit_mode_skips_menu(self):
        for args in (('--vslam-mode', 'vio'), ('--set', 'vslam_mode:=vio')):
            with self.subTest(args=args):
                code, output = self.launch(overrides=args)
                self.assertEqual(code, 0, output)
                self.assertNotIn('VSLAM_MENU_SHOWN', output)
                self.assertIn('vslam_mode:=vio', output)

    def test_ground_constraints_cannot_be_disabled(self):
        args = tuple(f'vslam_enable_ground_constraint_in_{target}:=false'
                     for target in ('odometry', 'slam'))
        code, output = self.launch(overrides=args)
        self.assertNotEqual(code, 0, output)
        self.assertIn('requires ground constraints', output)

    def test_disabled_localization_skips_menu(self):
        code, output = self.launch(preset='sensor', overrides=('--sensor-kit', 'realsense'))
        self.assertEqual(code, 0, output)
        self.assertNotIn('VSLAM_MENU_SHOWN', output)

    def test_cancel_does_not_launch(self):
        code, output = self.launch(mode='cancel')
        self.assertEqual(code, 130, output)
        self.assertNotIn('Command:', output)

    def test_localization_modes(self):
        for init in ('pose-hint', 'foxglove', 'map-origin'):
            with self.subTest(init=init):
                code, output = self.launch(init=init)
                self.assertEqual(code, 0, output)
                self.assertIn('TEST_INIT_MENU_SHOWN', output)
                self.assertIn('enable_vgl:=' + ('true' if init == 'pose-hint' else 'false'), output)
                self.assertIn('vslam_localize_on_startup:=' + ('true' if init == 'map-origin' else 'false'), output)
                if init == 'foxglove':
                    self.assertIn('enable_foxglove:=true', output)

    def test_explicit_init_skips_menu_and_map_view_can_choose_vgl(self):
        code, output = self.launch(init='cancel', overrides=('--localization-init', 'foxglove'))
        self.assertEqual(code, 0, output)
        self.assertNotIn('TEST_INIT_MENU_SHOWN', output)
        self.assertIn('enable_vgl:=false', output)
        code, output = self.launch(preset='map-view', overrides=('--sensor-kit', 'realsense'))
        self.assertEqual(code, 0, output)
        self.assertIn('enable_vgl:=true', output)

    def test_line_selection_and_competition_routing(self):
        for preset in ('runtime', 'competition'):
            for line in ('centerline', 'raceline', 'custom'):
                with self.subTest(preset=preset, line=line):
                    code, output = self.launch(preset=preset, line=line,
                                              overrides=('--vehicle', 'vesc', '--sensor-kit', 'realsense'))
                    self.assertEqual(code, 0, output)
                    self.assertIn('enable_hd_map_publisher:=true', output)
                    self.assertIn('enable_planning:=' + ('false' if preset == 'competition' else 'true'), output)
                    if line != 'centerline':
                        publisher = 'enable_raceline_publisher' if line == 'raceline' else 'enable_custom_trajectory_publisher'
                        self.assertIn(publisher + ':=true', output)
                        if preset == 'competition':
                            self.assertIn(f'competition_primary_trajectory_topic:=/planning/{line}_trajectory', output)
                        else:
                            self.assertIn(f'route_lane_selector.{line}.param.yaml', output)

    def test_custom_line_retains_open_metadata(self):
        code, output = self.launch(line='custom', closed=False)
        self.assertEqual(code, 0, output)
        self.assertIn('custom_closed:=false', output)

    def test_explicit_line_skips_menu(self):
        code, output = self.launch(line='cancel', overrides=('--raceline', '/tmp/explicit.csv'))
        self.assertEqual(code, 0, output)
        self.assertNotIn('TEST_LINE_MENU_SHOWN', output)
        self.assertIn('raceline_csv:=explicit.csv', output)

    def test_cancel_init_or_line_does_not_launch(self):
        for args in ({'init': 'cancel'}, {'line': 'cancel'}):
            code, output = self.launch(**args)
            self.assertEqual(code, 130, output)
            self.assertNotIn('Command:', output)

    def test_competition_launch_forwards_line_configuration_to_planning(self):
        planning = ROOT / 'ros2_ws/src/planning'
        wrapper = ET.parse(planning / 'jetpilot_planning_manager/launch/competition_planning.launch.xml').getroot()
        child = ET.parse(planning / 'jetpilot_planning/launch/jetpilot_planning.launch.xml').getroot()
        include = next(e for e in wrapper.findall('include') if 'jetpilot_planning)/' in e.attrib['file'])
        forwarded = {e.attrib['name']: e.attrib['value'] for e in include.findall('arg')}
        child_args = {e.attrib['name'] for e in child.findall('arg')}
        for name in ('primary_trajectory_topic', 'enable_raceline_publisher', 'raceline_csv',
                     'raceline_closed', 'enable_custom_trajectory_publisher', 'custom_csv',
                     'custom_closed', 'custom_source_hash'):
            self.assertEqual(forwarded[name], f'$(var {name})')
            self.assertIn(name, child_args)
        selector = next(n for n in child.findall('node') if n.attrib['name'] == 'route_lane_selector')
        params = {p.attrib.get('name'): p.attrib.get('value') for p in selector.findall('param')}
        self.assertEqual(params['primary_trajectory_topic'], '$(var primary_trajectory_topic)')
        tree = ast.parse((ROOT / 'ros2_ws/src/launch/jetpilot_system_launch/launch/bringup.launch.py').read_text())
        call = next(n for n in ast.walk(tree) if isinstance(n, ast.Call) and
                    any(isinstance(a, ast.Constant) and a.value == 'launch/competition_planning.launch.xml' for a in n.args))
        values = next(k.value for k in call.keywords if k.arg == 'launch_arguments')
        args = {key.value: value.attr for key, value in zip(values.keys, values.values) if isinstance(value, ast.Attribute)}
        self.assertEqual(args['primary_trajectory_topic'], 'competition_primary_trajectory_topic')
        for name in ('raceline_csv', 'custom_csv', 'custom_closed', 'enable_raceline_publisher', 'enable_custom_trajectory_publisher'):
            self.assertEqual(args[name], name)


if __name__ == '__main__':
    unittest.main()
