"""Recording path and launcher checks using only the Python standard library."""
import ast
from datetime import datetime
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
NODE = ROOT / 'ros2_ws/src/tool/jetpilot_bag_tools/jetpilot_bag_tools/bag_manager_node.py'


class RecordingDestinationTests(unittest.TestCase):
    def run_launcher(self, *args):
        env = dict(os.environ, RECORD_ROOT='/tmp/record destination')
        env.pop('BRINGUP_RECORD_NAME', None)
        return subprocess.run(
            ['bash', str(ROOT / 'scripts/bringup.sh'), 'record', '--dry-run', *args],
            env=env, text=True, capture_output=True,
        )

    def test_launch_passes_name_and_root(self):
        for name in ('course A', 'コースA_低速', '123', 'true'):
            with self.subTest(name=name):
                result = self.run_launcher('--record-name', name)
                self.assertEqual(result.returncode, 0, result.stderr)
                command = shlex.split(result.stdout.split('Command:\n', 1)[1].split('\n\n')[0])
                self.assertIn('bag_manager_recording_name:=' + name, command)
                self.assertIn('bag_manager_output_dir:=/tmp/record destination', command)
                self.assertIn('/<YYYY-MM-DD>/' + name, result.stdout)

    def test_invalid_names_rejected(self):
        for name in ('', '..', '.', '../other', 'a/b', 'a\\b', ' a', 'a ', 'a\nb'):
            with self.subTest(name=name):
                self.assertNotEqual(self.run_launcher('--record-name', name).returncode, 0)

    def test_disabled_and_legacy_modes(self):
        result = self.run_launcher('--no-bag-manager', '--record-name', 'unused')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn('bag_manager_recording_name:=', result.stdout)
        self.assertNotIn('bag_manager_recording_name:=', self.run_launcher().stdout)

    def test_paths_repeated_recordings_and_midnight(self):
        # Execute the actual pure method without loading ROS or other dependencies.
        tree = ast.parse(NODE.read_text())
        method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'next_recording_path')
        class Clock:
            current = datetime(2026, 9, 11, 23, 59)

            @classmethod
            def now(cls):
                return cls.current

        namespace = {'Path': Path, 'datetime': Clock}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(NODE), 'exec'), namespace)
        choose = namespace['next_recording_path']
        with tempfile.TemporaryDirectory() as folder:
            node = SimpleNamespace(output_dir=Path(folder), recording_name='コースA_低速')
            expected = Path(folder) / '2026-09-11' / node.recording_name
            self.assertEqual(choose(node, 'joy-label'), expected)
            self.assertFalse(expected.exists())
            expected.mkdir(parents=True)
            self.assertEqual(choose(node, ''), expected.with_name(node.recording_name + '_01'))
            expected.with_name(node.recording_name + '_01').symlink_to('/missing-bag-target')
            self.assertEqual(choose(node, ''), expected.with_name(node.recording_name + '_02'))
            Clock.current = datetime(2026, 9, 12, 0, 1)
            self.assertEqual(choose(node, ''), Path(folder) / '2026-09-12' / node.recording_name)
            node.recording_name = ''
            self.assertEqual(choose(node, 'joy label'), Path(folder) / '20260912_000100_joy_label')


if __name__ == '__main__':
    unittest.main()
