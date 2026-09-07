"""Standard-library-only launcher tests; no ROS or camera is started."""
import errno
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


ROOT = next(p for p in Path(__file__).resolve().parents if (p / 'scripts/bringup.sh').is_file())


class VslamTuiTest(unittest.TestCase):
    def launch(self, mode='vo', overrides=(), preset='localization-only'):
        with tempfile.TemporaryDirectory() as directory:
            fake_fzf = Path(directory) / 'fzf'
            fake_fzf.write_text(
                f'#!{sys.executable}\n'
                'import os, sys\n'
                'options = sys.stdin.read().splitlines()\n'
                'prompt = next(a for a in sys.argv if a.startswith("--prompt="))\n'
                'if "JetPilot bringup preset" in prompt:\n'
                '    print(next(o for o in options if o.split()[0] == os.environ["TEST_PRESET"]))\n'
                'elif "VSLAM 追跡モード" in prompt:\n'
                '    print("VSLAM_MENU_SHOWN", file=sys.stderr)\n'
                '    if os.environ["TEST_MODE"] == "cancel": sys.exit(130)\n'
                '    print(next(o for o in options if o.split()[0] == os.environ["TEST_MODE"]))\n'
                'else: print(options[0])\n'
            )
            fake_fzf.chmod(0o755)
            env = dict(os.environ, PATH=f'{directory}:{os.environ["PATH"]}',
                       TEST_MODE=mode, TEST_PRESET=preset)
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

    def test_explicit_mode_skips_menu(self):
        for args in (('--vslam-mode', 'vio'), ('--set', 'vslam_mode:=vio')):
            with self.subTest(args=args):
                code, output = self.launch(overrides=args)
                self.assertEqual(code, 0, output)
                self.assertNotIn('VSLAM_MENU_SHOWN', output)
                self.assertIn('vslam_mode:=vio', output)

    def test_ground_constraints_can_be_disabled(self):
        args = tuple(f'vslam_enable_ground_constraint_in_{target}:=false'
                     for target in ('odometry', 'slam'))
        code, output = self.launch(overrides=args)
        self.assertEqual(code, 0, output)
        for arg in args:
            self.assertIn(arg, output)
        self.assertIn('odometry=false / SLAM=false', output)

    def test_disabled_localization_skips_menu(self):
        code, output = self.launch(preset='sensor', overrides=('--sensor-kit', 'realsense'))
        self.assertEqual(code, 0, output)
        self.assertNotIn('VSLAM_MENU_SHOWN', output)

    def test_cancel_does_not_launch(self):
        code, output = self.launch(mode='cancel')
        self.assertEqual(code, 130, output)
        self.assertNotIn('Command:', output)


if __name__ == '__main__':
    unittest.main()
