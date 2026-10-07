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

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
import rc_popout_grid_evaluation_videos as review
import evaluate_rc_popout_grid_background as evaluation
import rc_popout_candidate_videos as video
import test_rc_popout_grid_evaluation as evaluation_fixture
from test_rc_popout_candidate_videos import fixture as preview_fixture


class EvaluationVideoTests(unittest.TestCase):
    def setUp(self):
        # Reuse the complete synthetic source/annotation/cache fixture, not real holdout data.
        self.fixture = evaluation_fixture.EvaluationTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.fixture.create_evaluation_cache()
        self.root = self.fixture.root
        self.frozen = self.fixture.frozen
        self.out = self.root/'evaluation'
        for name, sensor, early in [('t_0.2-none_01', 'rgb', False), ('test_02', 'evs', True)]:
            folder = self.fixture.cache/name
            path = folder/f'{sensor}_tiles.npz'
            with np.load(path, allow_pickle=False) as saved:
                data = {k: saved[k] for k in saved.files}
            mask = (data['time_s'] >= .2) & (data['time_s'] < .3) if early else data['time_s'] >= .8
            data['counts'][mask, :2] += 400
            np.savez_compressed(path, **data)
            result = evaluation.read_json(folder/'result.json')
            result['tile_arrays_sha256'][sensor] = video.digest(path)
            video.write_json(folder/'result.json', result)
        with contextlib.redirect_stdout(io.StringIO()):
            evaluation.evaluate(self.fixture.cache, self.frozen, self.out)

    def test_every_candidate_retained_including_early_negative_and_zero_records(self):
        manifest = review.load_review(self.out, self.frozen)
        self.assertEqual(len(manifest['summaries']), 20)
        self.assertEqual(len(manifest['scenes']), sum(r['candidates'] for r in manifest['summaries']))
        self.assertEqual(len({s['output_relative'] for s in manifest['scenes']}), len(manifest['scenes']))
        negative = [s for s in manifest['scenes'] if s['session'] == 't_0.2-none_01']
        self.assertTrue(negative)
        self.assertTrue(all(s['rgb_onset_recording_s'] is None for s in negative))
        self.assertTrue(all(s['time_class'] == 'negative_recording' for s in negative))
        second = [s for s in manifest['scenes'] if s['session'] == 'test_02' and s['method'] == 'evs']
        self.assertEqual(len(second), 2)
        self.assertEqual([s['time_class'] for s in second], ['before_guard', 'onset_to_250ms'])
        self.assertTrue(all(s['record_root_relative'] == 'dynamic' for s in second))
        self.assertTrue(any(r['session'] == 't_0.2-none_02' and r['candidates'] == 0 for r in manifest['summaries']))

    def test_changed_csv_or_saved_alarm_rejected(self):
        p = self.out/'candidates.csv'; original = p.read_bytes()
        lines = p.read_text().splitlines(keepends=True)
        p.write_text(''.join(lines[:-1]))
        with self.assertRaisesRegex(ValueError, 'CSV differs'):
            review.load_review(self.out, self.frozen)
        p.write_bytes(original)
        p = self.out/'test_02/evs_background_maps.npz'
        with np.load(p, allow_pickle=False) as saved:
            data = {k: saved[k] for k in saved.files}
        data['alarm'][np.flatnonzero(data['alarm'])[0]] = False
        np.savez_compressed(p, **data)
        with self.assertRaisesRegex(ValueError, 'alarms differ'):
            review.load_review(self.out, self.frozen)

    def test_changed_current_annotation_rejected(self):
        p = self.fixture.cache/'test_02/annotation.json'
        p.write_text(p.read_text()+' ')
        with self.assertRaisesRegex(ValueError, 'annotation changed'):
            review.load_review(self.out, self.frozen)

    def test_failed_previews_are_listed_not_dropped_or_replaced(self):
        output = self.root/'review'
        manifest = review.load_review(self.out, self.frozen)
        with patch.object(video, 'find_preview', side_effect=ValueError('missing matching preview')), \
             patch.object(review.shutil, 'which', return_value='/unused/ffmpeg'), \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(review.main(['--evaluation-dir', str(self.out), '--frozen-dir', str(self.frozen),
                                          '--record-base', str(self.root), '--output', str(output)]), 1)
        errors = evaluation.read_json(output/'review_errors.json')
        self.assertEqual(len(errors), len(manifest['scenes']))
        self.assertEqual(len(evaluation.read_json(output/'summary.json')), len(manifest['scenes']))
        self.assertIn('t_0.2-none_02', (output/'index.html').read_text())
        self.assertEqual(evaluation.read_json(output/'review_manifest.json')['scenes'], manifest['scenes'])


class OptionalOnsetRenderTests(unittest.TestCase):
    def test_negative_rgb_and_early_evs_clips_do_not_need_onset_in_span(self):
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
            for method, onset in [('rgb', None), ('evs', 11.7)]:
                scene = copy.deepcopy(manifest['scenes'][0])
                scene.update(method=method, candidate_id=2, rgb_onset_recording_s=onset)
                plan = video.find_preview(root, scene, manifest['spatial'], .3, .3)
                dest = root/method
                result = video.render(scene, manifest, plan, dest, 4., ffmpeg, require_onset=False)
                self.assertEqual(result['frames'], 12)
                self.assertEqual(result['method'], method)
                self.assertEqual(result['candidate_id'], 2)
                self.assertFalse(result['require_onset'])
                self.assertEqual(result['detector_window_ms'], None if method == 'rgb' else 2)
                self.assertTrue((dest/'candidate_minus_100ms.png').exists())
                self.assertTrue((dest/'candidate_plus_100ms.png').exists())
                with (dest/'frames.csv').open() as stream:
                    rows = list(csv.DictReader(stream))
                if onset is None:
                    self.assertTrue(all(r['onset_delta_ms'] == '' for r in rows))
                else:
                    self.assertTrue(all(float(r['onset_delta_ms']) < 0 for r in rows))
                tile = scene['tiles'][0]
                image = cv2.imread(str(dest/'candidate_after.png'))
                for shift in (0, 640):
                    self.assertEqual(image[tile['y'], tile['x']+shift].tolist(), [0,150,255])


if __name__ == '__main__':
    unittest.main()
