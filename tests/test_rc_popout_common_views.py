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


def pinned_fixture(root):
    root = root.resolve()
    chain = root/'chain.yaml'; chain.write_text('calibration')
    scene = root/'scene'; scene.mkdir()
    (scene/'data.mcap').write_text('mcap')
    (scene/'data.raw').write_text('raw')
    led = root/'analysis/led_sync/scene'; led.mkdir(parents=True)
    sync = led/'time_sync_led_auto.yaml'; sync.write_text('analysis auto sync')
    (led/'time_sync_led.yaml').write_text('new manual sync')
    (led/'led_sync_data.json').write_text(json.dumps({'meta': {'rgb_timestamp_source': 'header'}}))
    (led/'sequence_annotations.json').write_text('annotations must stay unchanged')
    args = ['--record-root', str(root), '--camchain', str(chain), '--sessions', 'scene',
            '--time-sync', str(sync), '--time-sync-sha256', MODULE.digest(sync),
            '--rgb-timestamp-source', 'bag', '--reference-origin-s', '1000.0']
    return args, sync


def fake_pinned_render(command, **kwargs):
    output = Path(command[command.index('--output-dir')+1]); output.mkdir()
    for name in ('rgb_vs_overlay.mp4', 'summary.yaml', 'frames.csv'):
        (output/name).write_text('synthetic output')
    sync = Path(command[command.index('--time-sync')+1])
    summary = dict(time_sync_sha256=MODULE.digest(sync),
                   rgb_timestamp_source=command[command.index('--rgb-timestamp-source')+1],
                   reference_origin_s=1000.)
    (output/'summary.json').write_text(json.dumps(summary))
    return types.SimpleNamespace(returncode=0)


class CommonViewsTests(unittest.TestCase):
    def test_analysis_sync_and_timestamp_override_manual_priority_and_led_metadata(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); args, sync = pinned_fixture(root)
            originals = {p:p.read_bytes() for p in root.rglob('*') if p.is_file()}
            with patch.object(MODULE.subprocess, 'run', side_effect=fake_pinned_render) as render, \
                 contextlib.redirect_stdout(io.StringIO()) as log:
                self.assertEqual(MODULE.main(args), 0)
                self.assertEqual(MODULE.main(args), 0)
            self.assertEqual(render.call_count, 1)
            command = render.call_args.args[0]
            self.assertEqual(command[command.index('--time-sync')+1], str(sync))
            self.assertEqual(command[command.index('--rgb-timestamp-source')+1], 'bag')
            self.assertIn('pinned to analysis', log.getvalue())
            self.assertEqual(originals, {p:p.read_bytes() for p in originals})

    def test_pin_needs_no_led_export_and_rejects_missing_or_changed_sync_without_fallback(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); args, sync = pinned_fixture(root)
            (sync.parent/'led_sync_data.json').unlink()
            with patch.object(MODULE.subprocess, 'run') as render, \
                 contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(MODULE.main(args+['--dry-run']), 0)
                sync.write_text('modified analysis sync')
                self.assertEqual(MODULE.main(args+['--dry-run']), 1)
                sync.unlink()
                self.assertEqual(MODULE.main(args+['--dry-run']), 1)
            render.assert_not_called()
            self.assertFalse((root/'analysis/scenario_overlay').exists())

    def test_generated_and_cached_metadata_must_match_pinned_inputs(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); args, _ = pinned_fixture(root)
            def bad_render(command, **kwargs):
                result = fake_pinned_render(command, **kwargs)
                path = Path(command[command.index('--output-dir')+1])/'summary.json'
                summary = json.loads(path.read_text()); summary['reference_origin_s'] = 1001.
                path.write_text(json.dumps(summary))
                return result
            with patch.object(MODULE.subprocess, 'run', side_effect=bad_render), \
                 contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()) as log:
                self.assertEqual(MODULE.main(args), 1)
            self.assertIn('reference_origin_s mismatch', log.getvalue())
            self.assertEqual(list(root.glob('analysis/scenario_overlay/*/*/batch_settings.json')), [])
            with patch.object(MODULE.subprocess, 'run', side_effect=fake_pinned_render), \
                 contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(MODULE.main(args), 0)
            marker = next(root.glob('analysis/scenario_overlay/*/*/batch_settings.json'))
            # Move the successful retry to the canonical cache path to exercise reuse validation.
            failed = next(p for p in marker.parent.parent.iterdir() if p.is_dir() and p != marker.parent)
            import shutil
            shutil.rmtree(failed)
            marker.parent.rename(failed)
            path = failed/'summary.json'
            good = path.read_text()
            for key, bad in (('time_sync_sha256', 'wrong'), ('rgb_timestamp_source', 'header'),
                             ('reference_origin_s', 1001.)):
                summary = json.loads(good); summary[key] = bad; path.write_text(json.dumps(summary))
                with patch.object(MODULE.subprocess, 'run') as render, \
                     contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()) as log:
                    self.assertEqual(MODULE.main(args), 1)
                self.assertIn(key+' mismatch', log.getvalue()); render.assert_not_called()

    def test_one_sync_cannot_be_applied_to_multiple_sessions(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); args, _ = pinned_fixture(root)
            (root/'second').mkdir(); (root/'second/data.mcap').write_text('mcap')
            args = args[:args.index('--sessions')]+args[args.index('--sessions')+2:]
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                MODULE.main(args+['--dry-run'])

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
            with patch.object(MODULE.sys, 'argv', argv + ['--dry-run', '--sessions', 'scene1', 'scene4']), contextlib.redirect_stdout(io.StringIO()) as output:
                self.assertEqual(MODULE.main(), 0)
                self.assertIn('sessions=2 failed=0', output.getvalue())
                self.assertNotIn('[1/2] scene0', output.getvalue())
            with patch.object(MODULE.sys, 'argv', argv + ['--dry-run', '--sessions', 'missing']), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    MODULE.main()
                self.assertEqual(error.exception.code, 2)
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
