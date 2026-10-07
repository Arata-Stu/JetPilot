import contextlib
import copy
import csv
import io
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
import evaluate_rc_popout_grid_static as static
import rc_popout_grid_static_videos as review
import rc_popout_candidate_videos as video
import test_rc_popout_grid_static as static_fixture
from test_rc_popout_candidate_videos import fixture as preview_fixture


class StaticVideoTests(unittest.TestCase):
    def setUp(self):
        self.fixture = static_fixture.StaticTransferTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.root, self.frozen = self.fixture.root, self.fixture.frozen
        self.output = self.root/'static'
        # Include temporal-only annotation support in the review contract too.
        ann = Path(static.read_json(self.fixture.config)['sessions'][0]['annotation'])
        value = static.read_json(ann); value['spatial'] = None; video.write_json(ann, value)
        with patch.object(static, 'analyze', side_effect=self.extract), contextlib.redirect_stdout(io.StringIO()):
            static.evaluate(self.fixture.config, self.frozen, self.output)

    def extract(self, config, entry, args, *, tile_sink):
        result, rgb, _ = self.fixture.fake_extract(config, entry, args, tile_sink=tile_sink)
        data = tile_sink.finish()['evs']
        if entry['condition'] != 'none':
            data['counts'][data['time_s'] >= 1., :2] += 300
        evs = list(zip(data['interval'].tolist(), data['time_s'].tolist(), data['counts'].sum(axis=1).tolist()))
        return result, rgb, evs

    def test_complete_population_and_recording_times_without_inference(self):
        before = {p:video.digest(p) for p in self.root.rglob('*') if p.is_file()}
        with patch.object(static, 'score_maps', side_effect=AssertionError('no inference')), \
             patch.object(static, 'analyze', side_effect=AssertionError('no extraction')):
            manifest = review.load_review(self.output, self.frozen)
        self.assertEqual(len(manifest['summaries']), 36)
        self.assertEqual(len(manifest['scenes']), 24)
        self.assertEqual(sum(r['candidates'] == 0 for r in manifest['summaries']), 12)
        for scene in manifest['scenes']:
            self.assertIsNone(scene['drive_start_s'])
            self.assertNotIn('candidate_from_drive_s', scene)
            self.assertEqual(scene['record_root'], str((self.root/'recordings').resolve()))
            self.assertEqual(scene['candidate_recording_s'], scene['source_candidate']['start_relative_time_s'])
        self.assertEqual(before, {p:video.digest(p) for p in before})

    def test_changed_sources_csv_alarm_and_tiles_are_rejected(self):
        scene = self.output/'popout-0928-static_01'
        p = self.output/'candidates.csv'; original = p.read_bytes()
        p.write_text('\n'.join(p.read_text().splitlines()[:-1])+'\n')
        with self.assertRaisesRegex(ValueError, 'CSV differs'):
            review.load_review(self.output, self.frozen)
        p.write_bytes(original)
        p = scene/'evs_background_maps.npz'; original = p.read_bytes()
        with np.load(p, allow_pickle=False) as saved:
            maps = {k:saved[k] for k in saved.files}
        maps['alarm'][np.flatnonzero(maps['alarm'])[0]] = False
        np.savez_compressed(p, **maps)
        with self.assertRaisesRegex(ValueError, 'alarms/timeline'):
            review.load_review(self.output, self.frozen)
        p.write_bytes(original)
        p = scene/'rgb_tiles.npz'; original = p.read_bytes(); p.write_bytes(original+b'changed')
        with self.assertRaisesRegex(ValueError, 'tile arrays changed'):
            review.load_review(self.output, self.frozen)
        p.write_bytes(original)
        p = Path(static.read_json(self.fixture.config)['sessions'][0]['annotation'])
        p.write_text(p.read_text()+' ')
        with self.assertRaisesRegex(ValueError, 'changed since inference'):
            review.load_review(self.output, self.frozen)

    def test_candidate_coordinates_are_checked_against_saved_winner(self):
        p = self.output/'popout-0928-static_01/rgb_candidates.json'
        candidates = static.read_json(p)
        candidates[0].update(pair_tile_a=1, pair_tile_b=2)
        video.write_json(p, candidates)
        with self.assertRaisesRegex(ValueError, 'differs from saved alarm'):
            review.load_review(self.output, self.frozen)

    def test_dry_run_never_prepares_or_writes_and_commands_group_record_roots(self):
        destination = self.root/'review'
        manifest = review.load_review(self.output, self.frozen)
        plans = review.plan_previews(manifest, .3, .3)
        commands = review.preview_commands(manifest, plans)
        self.assertEqual(len(commands), 1)
        command = commands[0]
        sessions = command[command.index('--sessions')+1:]
        self.assertEqual(len(sessions), 12)
        self.assertEqual(command[command.index('--record-root')+1], str((self.root/'recordings').resolve()))
        with patch.object(review.subprocess, 'run') as run, contextlib.redirect_stdout(io.StringIO()):
            code = review.main(['--static-dir', str(self.output), '--frozen-dir', str(self.frozen),
                                '--output', str(destination), '--prepare-previews', '--dry-run'])
        self.assertEqual(code, 1)
        run.assert_not_called(); self.assertFalse(destination.exists())

    def test_failed_previews_are_reported_alongside_zero_candidate_records(self):
        try:
            import cv2
        except ImportError:
            self.skipTest('OpenCV unavailable')
        destination = self.root/'review'
        with patch.object(review.shutil, 'which', return_value='/unused/ffmpeg'), \
             contextlib.redirect_stdout(io.StringIO()):
            code = review.main(['--static-dir', str(self.output), '--frozen-dir', str(self.frozen),
                                '--output', str(destination)])
        self.assertEqual(code, 1)
        self.assertEqual(len(static.read_json(destination/'review_errors.json')), 24)
        self.assertEqual(len(static.read_json(destination/'summary.json')), 24)
        self.assertIn('popout-0928-static_07', (destination/'index.html').read_text())


class StaticRenderTests(unittest.TestCase):
    def test_real_video_without_drive_time_and_explicit_analysis_sync(self):
        try:
            import cv2
        except ImportError:
            self.skipTest('OpenCV unavailable')
        ffmpeg = shutil.which('ffmpeg')
        if not ffmpeg:
            self.skipTest('ffmpeg unavailable')
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            _, manifest = preview_fixture(root, make_video=True)
            scene = copy.deepcopy(manifest['scenes'][0])
            ann = root/'analysis/led_sync'/scene['session']/'sequence_annotations.json'
            saved_sync = ann.parent/'saved_analysis_sync.yaml'
            (ann.parent/'time_sync_led.yaml').rename(saved_sync)
            (ann.parent/'time_sync_led.yaml').write_text('different current manual sync')
            scene.update(drive_start_s=None, annotation_path=str(ann), time_sync_path=str(saved_sync))
            for method in ('rgb', 'evs'):
                scene['method'] = method
                plan = video.find_preview(root, scene, manifest['spatial'], .3, .3)
                self.assertEqual(plan['sync_path'], saved_sync)
                texts = []
                put_text = cv2.putText
                def capture_text(image, text, *args, **kwargs):
                    texts.append(text)
                    return put_text(image, text, *args, **kwargs)
                with patch.object(cv2, 'putText', side_effect=capture_text):
                    report = video.render(scene, manifest, plan, root/method, 4., ffmpeg)
                self.assertEqual(report['frames'], 12)
                self.assertTrue(any('Static recording' in t for t in texts))
                self.assertFalse(any('From drive=' in t for t in texts))
                with (root/method/'frames.csv').open() as stream:
                    rows = list(csv.DictReader(stream))
                self.assertGreater(float(rows[0]['recording_relative_s']), 9.)
                self.assertEqual(float(rows[0]['output_video_time_s']), 0.)
                tile = scene['tiles'][0]
                image = cv2.imread(str(root/method/'candidate_after.png'))
                for shift in (0, 640):
                    self.assertEqual(image[tile['y'], tile['x']+shift].tolist(), [0,150,255])
                self.assertTrue((root/method/'onset_after.png').is_file())


if __name__ == '__main__':
    unittest.main()
