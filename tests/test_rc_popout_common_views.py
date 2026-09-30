import contextlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('common_views', ROOT / 'scripts/experiments/generate_rc_popout_common_views.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class CommonViewsTests(unittest.TestCase):
    def test_six_scenes_manual_priority_resume_and_sync_change(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            chain = root / 'chain.yaml'
            chain.write_text('calibration')
            for i in range(6):
                scene = root / f'scene{i}'
                scene.mkdir()
                (scene / 'original_name.mcap').write_text('mcap')
                (scene / 'camera.raw').write_text('raw')
                folder = root / 'analysis/led_sync' / scene.name
                folder.mkdir(parents=True)
                (folder / 'led_sync_data.json').write_text(json.dumps({'meta': {'rgb_timestamp_source': 'header'}}))
                (folder / 'time_sync_led_auto.yaml').write_text('auto')
                if i != 5:
                    (folder / 'time_sync_led.yaml').write_text('manual')
                (folder / 'sequence_annotations.json').write_text('original annotations')
            argv = ['generate', '--record-root', str(root), '--camchain', str(chain)]
            calls = []

            def render(command, **kwargs):
                calls.append(command)
                output = Path(command[command.index('--output-dir') + 1])
                output.mkdir()
                for name in ('rgb_vs_overlay.mp4', 'summary.json', 'summary.yaml', 'frames.csv'):
                    (output / name).write_text('output')
                return types.SimpleNamespace(returncode=0)

            with patch.object(MODULE.sys, 'argv', argv + ['--dry-run']), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(MODULE.main(), 0)
            self.assertFalse((root / 'analysis/scenario_overlay').exists())
            with patch.object(MODULE.sys, 'argv', argv), patch.object(MODULE.subprocess, 'run', render), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(MODULE.main(), 0)
                self.assertEqual(len(calls), 6)
                for i, call in enumerate(calls):
                    self.assertEqual(Path(call[call.index('--time-sync') + 1]).name,
                                     'time_sync_led_auto.yaml' if i == 5 else 'time_sync_led.yaml')
                    self.assertEqual(call[call.index('--view-frame') + 1], 'evs')
                    self.assertEqual(call[call.index('--rgb-timestamp-source') + 1], 'header')
                self.assertEqual(MODULE.main(), 0)
                self.assertEqual(len(calls), 6)
                (root / 'analysis/led_sync/scene0/time_sync_led.yaml').write_text('updated manual')
                self.assertEqual(MODULE.main(), 0)
                self.assertEqual(len(calls), 7)
            for i in range(6):
                self.assertEqual((root / f'analysis/led_sync/scene{i}/sequence_annotations.json').read_text(), 'original annotations')


if __name__ == '__main__':
    unittest.main()
