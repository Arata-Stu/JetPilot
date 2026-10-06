import contextlib
import copy
import io
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
import evaluate_rc_popout_grid_background as m
import analyze_rc_popout_grid_background as development
import rc_popout_grid_background as core
import rc_popout_tile_activity as exporter
from rc_popout_change_detection import check_scene
from test_rc_popout_grid_background import background_fixture


def make_scenes(role):
    split = m.read_json(m.DEFAULT_SPLIT)
    scenes = {}
    negatives = {s for g in split['groups'] if g['condition'] == 'none' for s in g[role]}
    for name in m.select_sessions(split, role):
        arrays = {}
        for sensor in ('rgb', 'evs'):
            data, meta, phases = background_fixture(sensor)
            if name not in negatives:
                data['counts'][data['time_s'] >= 1., :2] += 300
            arrays[sensor] = data
        scenes[name] = dict(meta=meta, data=arrays, alignment=dict(
            session=name, drive_start_s=.5, phases=phases, reference_origin_s=1000., timestamp_source='bag',
            rgb_first_visible_from_drive_s=None if name in negatives else .5),
            source_hashes=dict(annotation_sha256='a'*64, time_sync_sha256='b'*64))
    return scenes


class EvaluationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.dev = self.root/'development'
        self.frozen = self.root/'frozen'
        self.cache = self.root/'tiles'
        self.scenes = make_scenes('development')
        chain = self.root/'chain.yaml'
        chain.write_text('fixture calibration')
        meta = next(iter(self.scenes.values()))['meta']
        self.definition = dict(tile_px=32, roi=meta['roi'], step_ms=1., window_bins=2,
            rgb_topic='/rgb', rgb_pixel_delta=15,
            spatial=dict(view_frame='evs', output_size=meta['output_size'], camchain=str(chain),
                         camchain_sha256=m.digest(chain)))
        with patch.object(development, 'load_input', return_value=(self.scenes, self.definition, {})), contextlib.redirect_stdout(io.StringIO()):
            development.main(['--bundle', 'unused', '--threshold-margin', '2', '--output', str(self.dev)])
            m.freeze(self.dev, self.frozen)

    def create_evaluation_cache(self):
        scenes = make_scenes('evaluation')
        self.cache.mkdir()
        motion = self.root/'motion'; motion.mkdir()
        source_dir = self.root/'scores'; source_dir.mkdir()
        definition = self.definition
        source = dict(common=dict(roi=definition['roi'], spatial=definition['spatial'],
                                 sessions=[dict(session=s) for s in scenes]),
                      parameters={k: definition[k] for k in ('rgb_topic', 'rgb_pixel_delta', 'step_ms', 'window_bins')})
        source_path = source_dir/'run_config.json'; m.write_json(source_path, source)
        run = dict(subset='evaluation', sessions=list(scenes), split=m.read_json(m.DEFAULT_SPLIT),
                   frozen_manifest_sha256=m.digest(self.frozen/'freeze.json'),
                   exporter_sha256=m.digest(ROOT/'tools/rc_popout_tile_activity.py'),
                   extraction_code_sha256=m.digest(ROOT/'tools/rc_popout_detection.py'),
                   source_config_path=str(source_path), source_config_sha256=m.digest(source_path),
                   source_config=source, tile_px=32, motion_dir=str(motion))
        m.write_json(self.cache/'run_config.json', run)
        m.write_json(self.cache/'summary.json', [dict(session=s, status='complete') for s in scenes])
        for name, scene in scenes.items():
            folder = self.cache/name; folder.mkdir()
            scored = source_dir/name; scored.mkdir()
            aligned = motion/name; aligned.mkdir()
            sync = folder/'time_sync.yaml'; sync.write_text('fixture sync')
            annotation = folder/'annotation.json'; m.write_json(annotation, dict(time_sync=str(sync)))
            result = dict(session=name, annotation=str(annotation), annotation_sha256=m.digest(annotation),
                          time_sync_sha256=m.digest(sync), camchain_sha256=definition['spatial']['camchain_sha256'],
                          valid_pixels=sum(t['valid_pixels'] for t in scene['meta']['tiles']))
            m.write_json(scored/'result.json', result)
            alignment = dict(scene['alignment'], input_result_sha256=m.digest(scored/'result.json'),
                             annotation_sha256=m.digest(annotation))
            m.write_json(aligned/'alignment.json', alignment)
            _, hashes = check_scene(aligned, source_dir)
            for sensor, data in scene['data'].items():
                np.savez_compressed(folder/f'{sensor}_tiles.npz', **data)
            m.write_json(folder/'tiles.json', scene['meta'])
            m.write_json(folder/'result.json', dict(source_result=result, alignment=alignment, input_hashes=hashes,
                tile_arrays_sha256={s: m.digest(folder/f'{s}_tiles.npz') for s in ('rgb', 'evs')}))
        return scenes

    def test_freeze_copies_selected_values_and_detects_tampering(self):
        for name in ('detector_parameters.json', 'background_models.json'):
            self.assertEqual((self.dev/name).read_bytes(), (self.frozen/name).read_bytes())
        m.load_frozen(self.frozen)
        for name in ('detector_parameters.json', 'background_models.json', 'split.json'):
            original = (self.frozen/name).read_bytes()
            (self.frozen/name).write_bytes(original+b' ')
            with self.assertRaisesRegex(ValueError, 'frozen file changed'):
                m.load_frozen(self.frozen)
            (self.frozen/name).write_bytes(original)
        with patch.object(m, 'current_code', return_value={}):
            with self.assertRaisesRegex(ValueError, 'runtime code mismatch'):
                m.load_frozen(self.frozen)
        with self.assertRaisesRegex(ValueError, 'already exists'):
            m.freeze(self.dev, self.frozen)

    def test_all_ten_inference_only_no_refitting_and_all_candidates_saved(self):
        self.create_evaluation_cache()
        out = self.root/'evaluation'
        with patch.object(core, 'fit_background', side_effect=AssertionError('no fitting')), \
             patch.object(core, 'calibrate_threshold', side_effect=AssertionError('no calibration')), \
             patch.object(development, 'fit_background', side_effect=AssertionError('no fitting')), \
             patch.object(development, 'calibrate_threshold', side_effect=AssertionError('no calibration')), \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(m.main(['run', '--tile-dir', str(self.cache), '--frozen-dir', str(self.frozen),
                                     '--output', str(out)]), 0)
        summaries = m.read_json(out/'summary.json')
        self.assertEqual(len(summaries), 20)
        self.assertEqual(set(r['session'] for r in summaries), set(m.select_sessions(m.read_json(m.DEFAULT_SPLIT), 'evaluation')))
        self.assertTrue(all(r['candidates'] == (0 if r['session'].startswith('t_') else 1) for r in summaries))
        for name in ('detector_parameters.json', 'background_models.json', 'freeze.json'):
            self.assertEqual((out/name).read_bytes(), (self.frozen/name).read_bytes())
        self.assertEqual(m.read_json(out/'run_config.json')['status'], 'complete')
        self.assertFalse((out/'threshold_sensitivity.csv').exists())
        self.assertFalse((out/'t_0.2-none').exists())

    def test_cached_input_changes_fail_before_output_creation(self):
        self.create_evaluation_cache()
        for file, transform, message in [
            (self.cache/'run_config.json', lambda r: r.update(subset='development'), 'split/package'),
            (self.cache/'summary.json', lambda r: r.pop(), 'incomplete'),
            (self.cache/'test_02/annotation.json', lambda r: r.update(extra=True), 'annotation changed'),
            (self.cache/'test_02/tiles.json', lambda r: r['roi'].update(y=1), 'metadata differs'),
        ]:
            before = file.read_bytes()
            value = m.read_json(file); transform(value); m.write_json(file, value)
            out = self.root/'should_not_exist'
            with self.assertRaisesRegex(ValueError, message):
                m.evaluate(self.cache, self.frozen, out)
            self.assertFalse(out.exists())
            file.write_bytes(before)
        tile = self.cache/'test_02/evs_tiles.npz'
        tile.write_bytes(tile.read_bytes()+b'changed')
        with self.assertRaisesRegex(ValueError, 'tiles changed'):
            m.load_evaluation_tiles(self.cache, self.frozen)

    def test_onset_changes_reporting_not_scores_or_candidate_times(self):
        scenes = make_scenes('evaluation')
        parameters, models, _ = m.load_frozen(self.frozen)
        scene = {'test_02': scenes['test_02']}
        for label, onset in [('original', .5), ('different', -.2)]:
            out = self.root/label; out.mkdir()
            shutil.copyfile(self.frozen/'detector_parameters.json', out/'detector_parameters.json')
            scene['test_02']['alignment']['rgb_first_visible_from_drive_s'] = onset
            with contextlib.redirect_stdout(io.StringIO()):
                m.run_inference(scene, parameters, models, out)
        for sensor in ('rgb', 'evs'):
            a = self.root/'original/test_02'; b = self.root/'different/test_02'
            self.assertEqual((a/f'{sensor}_background_maps.npz').read_bytes(), (b/f'{sensor}_background_maps.npz').read_bytes())
            ca, cb = (m.read_json(p/f'{sensor}_candidates.json') for p in (a,b))
            self.assertEqual([r['start_relative_time_s'] for r in ca], [r['start_relative_time_s'] for r in cb])

    def test_extraction_requires_freeze_and_rejects_overrides_before_raw(self):
        self.create_evaluation_cache()
        common = ['--motion-dir', str(self.root/'motion'), '--score-config', str(self.root/'scores/run_config.json'),
                  '--subset', 'evaluation', '--output', str(self.root/'export')]
        with patch.object(exporter, 'analyze') as analyze, contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                exporter.main(common)
            with self.assertRaises(SystemExit):
                exporter.main(common+['--frozen-dir', str(self.frozen), '--tile-px', '64'])
            analyze.assert_not_called()
        self.assertFalse((self.root/'export').exists())
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            m.main(['run', '--tile-dir', str(self.cache), '--frozen-dir', str(self.frozen),
                    '--output', str(self.root/'out'), '--threshold-margin', '3'])

    def test_evaluation_export_to_inference_and_no_exploratory_panels(self):
        scenes = self.create_evaluation_cache()
        extracted = self.root/'extracted'
        seen = []
        def analyze(config, entry, args, *, tile_sink):
            name = entry['session']; seen.append(name)
            self.assertIn(name, scenes)
            tile_sink.initialize(np.ones((64, 192), bool), [(0., 2.)], .001, 2)
            for a, b in [(.1, .12), (.12, .14)]:
                tile_sink.rgb(0, a, b, np.zeros((64, 192), bool))
            result = m.read_json(self.cache/name/'result.json')['source_result']
            result['rgb_first_visible_s'] = None if name.startswith('t_') else 1.
            return result, [(0, .12, 0.), (0, .14, 0.)], [(0, i*.001, 0) for i in range(2, 2000)]
        with patch.object(exporter, 'analyze', side_effect=analyze), \
             patch.object(exporter, 'make_reviews', side_effect=AssertionError('no tuning panels')), \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(exporter.main([
                '--motion-dir', str(self.root/'motion'), '--score-config', str(self.root/'scores/run_config.json'),
                '--subset', 'evaluation', '--frozen-dir', str(self.frozen), '--output', str(extracted)]), 0)
            m.evaluate(extracted, self.frozen, self.root/'from_export')
        self.assertEqual(seen, m.select_sessions(m.read_json(m.DEFAULT_SPLIT), 'evaluation'))
        self.assertFalse((extracted/'tile_diagnostics.csv').exists())
        self.assertEqual(m.read_json(extracted/'review_errors.json'), [])
        self.assertTrue(all(row['candidates'] == 0 for row in m.read_json(self.root/'from_export/summary.json')))


if __name__ == '__main__':
    unittest.main()
