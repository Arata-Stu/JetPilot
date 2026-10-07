import contextlib
import copy
import io
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
import evaluate_rc_popout_grid_static as m
import evaluate_rc_popout_grid_background as moving
import analyze_rc_popout_grid_background as development
import rc_popout_grid_background as core
from test_rc_popout_grid_evaluation import make_scenes
from test_rc_popout_grid_background import background_fixture


class StaticTransferTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.frozen = self.root/'frozen'
        scenes = make_scenes('development')
        chain = self.root/'chain.yaml'; chain.write_text('fixture calibration')
        meta = next(iter(scenes.values()))['meta']
        self.definition = dict(tile_px=32, roi=meta['roi'], step_ms=1., window_bins=2,
            rgb_topic='/rgb', rgb_pixel_delta=15,
            spatial=dict(view_frame='evs', output_size=meta['output_size'], camchain=str(chain),
                camchain_sha256=m.digest(chain), projection='rotation-only', depth_m=None,
                rgb_to_view_homography=np.eye(3).tolist()))
        with patch.object(development, 'load_input', return_value=(scenes, self.definition, {})), \
             contextlib.redirect_stdout(io.StringIO()):
            development.main(['--bundle', 'unused', '--threshold-margin', '2', '--output', str(self.root/'dev')])
            moving.freeze(self.root/'dev', self.frozen)
        self.config = self.root/'common_detection_roi.json'
        config = dict(roi=dict(x=0, y=32, width=192, height=32),
            spatial=dict(self.definition['spatial'], projection='fixed-depth', depth_m=2.2), sessions=[])
        for record in m.population():
            name = record['session']
            bag = self.root/'recordings'/name; bag.mkdir(parents=True)
            for file in ('metadata.yaml', 'fixture.raw', 'fixture.raw.metadata.yaml', 'fixture_0.mcap'):
                (bag/file).write_text('nonempty synthetic recording placeholder')
            folder = self.root/'recordings/analysis/led_sync'/name; folder.mkdir(parents=True)
            sync = folder/'time_sync_led.yaml'; sync.write_text('synthetic sync placeholder')
            ann = dict(session=name, reference_origin_s=1000., spatial=config['spatial'],
                time_sync=str(sync), time_sync_sha256=m.digest(sync),
                intervals=[dict(label='evaluation', start_s=0., end_s=2.)],
                onset={} if record['condition'] == 'none' else dict(first_visible=dict(rgb_time_s=1001.)))
            path = folder/'sequence_annotations.json'; m.write_json(path, ann)
            config['sessions'].append(dict(session=name, annotation=str(path)))
        m.write_json(self.config, config)

    def fake_extract(self, config, entry, args, *, tile_sink):
        # Decode boundary is synthetic; actual TileSink finishing and the frozen
        # scoring/episode code execute normally in the integration test.
        self.assertEqual(config['roi'], self.definition['roi'])
        self.assertEqual(config['spatial'], self.definition['spatial'])
        self.assertEqual(args.rgb_pixel_delta, 15)
        self.assertEqual((args.step_ms, args.window_bins), (1., 2))
        self.assertFalse(args.spatial)
        tile_sink.initialize(np.ones((64, 192), bool), [(0., 2.)], .001, 2)
        positive = entry['condition'] != 'none'
        # Local signal begins at a fixed acquisition timestamp, independent of annotation.
        for index in range(1, 100):
            changed = np.zeros((64, 192), bool)
            if positive and index*.02 >= 1.:
                changed[:16, :64] = True
            tile_sink.rgb(0, (index-1)*.02, index*.02, changed)
        if positive:
            bins = np.repeat(np.arange(1000, 1400), 2)
            tile_sink.events(0, bins, np.tile([0, 32], 400), np.zeros(len(bins), int))
        arrays = tile_sink.finish()
        # Analyze normally leaves finishing to its caller. Keep this fixture's
        # finished arrays so it can exercise the same adapter lifecycle.
        tile_sink.finish = lambda: arrays
        ann = m.read_json(entry['annotation'])
        onset = None if not ann['onset'] else ann['onset']['first_visible']['rgb_time_s']-ann['reference_origin_s']
        result = dict(session=entry['session'], annotation_sha256=m.digest(entry['annotation']),
            time_sync_sha256=m.digest(ann['time_sync']), camchain_sha256=self.definition['spatial']['camchain_sha256'],
            roi=config['roi'], valid_pixels=64*192, intervals=[(0., 2.)],
            evaluation_seconds=2., rgb_first_visible_s=onset)
        rows = {}
        for sensor, data in arrays.items():
            total = data['counts'].sum(axis=1)
            if sensor == 'rgb': total = total/(64*192)
            rows[sensor] = list(zip(data['interval'].tolist(), data['time_s'].tolist(), total.tolist()))
            result[sensor] = dict(threshold=m.BASELINE_THRESHOLDS[sensor], episodes=0,
                first_trigger_s=None, first_trigger_minus_rgb_onset_ms=None)
        return result, rows['rgb'], rows['evs']

    def test_preflight_only_preserves_sources_and_frozen_geometry(self):
        before = {p:p.read_bytes() for p in self.root.rglob('*') if p.is_file()}
        with patch.object(m, 'analyze') as extract, contextlib.redirect_stdout(io.StringIO()):
            out = self.root/'unused'
            self.assertEqual(m.evaluate(self.config, self.frozen, out, preflight_only=True), 0)
            extract.assert_not_called()
        self.assertFalse(out.exists())
        plan, _, _, _ = m.preflight(self.config, self.frozen)
        self.assertEqual(len(plan['scenes']), 18)
        self.assertEqual(sum(s['condition'] == 'none' for s in plan['scenes']), 6)
        self.assertTrue(all(s['annotation_geometry_changed'] for s in plan['scenes']))
        self.assertNotEqual(plan['original_roi'], plan['effective_config']['roi'])
        self.assertEqual(before, {p:p.read_bytes() for p in self.root.rglob('*') if p.is_file()})

    def test_all_static_records_run_without_fitting_or_drive_timestamp(self):
        original = {p:p.read_bytes() for p in self.root.rglob('*') if p.is_file()}
        out = self.root/'static'
        with patch.object(m, 'analyze', side_effect=self.fake_extract), \
             patch.object(core, 'fit_background', side_effect=AssertionError('no fitting')), \
             patch.object(core, 'calibrate_threshold', side_effect=AssertionError('no calibration')), \
             patch.object(development, 'fit_background', side_effect=AssertionError('no fitting')), \
             patch.object(development, 'calibrate_threshold', side_effect=AssertionError('no calibration')), \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(m.evaluate(self.config, self.frozen, out), 0)
        summaries = m.read_json(out/'summary.json')
        self.assertEqual(len(summaries), 36)
        self.assertTrue(all(r['status'] == 'complete' for r in summaries))
        self.assertTrue(all(not any('from_drive' in k for k in r) for r in summaries))
        self.assertEqual(m.read_json(out/'errors.json'), [])
        for name in ('detector_parameters.json', 'background_models.json', 'freeze.json'):
            self.assertEqual((out/name).read_bytes(), (self.frozen/name).read_bytes())
        for file, value in original.items(): self.assertEqual(file.read_bytes(), value)
        for row in summaries:
            self.assertEqual(row['rgb_first_visible_s'], None if row['condition'] == 'none' else 1.)
            self.assertGreater(row['ready_observed_seconds'], 1.9)
        with self.assertRaisesRegex(ValueError, 'already exists'):
            m.evaluate(self.config, self.frozen, out)

    def test_temporal_only_annotations_keep_unknown_geometry_and_run(self):
        config = m.read_json(self.config)
        selected = [config['sessions'][i] for i in (0, 6)]  # Positive and negative.
        originals = {}
        for index, entry in enumerate(selected):
            path = Path(entry['annotation'])
            value = m.read_json(path)
            if index == 0:
                value['spatial'] = None  # Valid temporal-only UI representation.
            else:
                del value['spatial']
            m.write_json(path, value)
            originals[path] = path.read_bytes()
        plan, _, _, _ = m.preflight(self.config, self.frozen)
        for scene in (plan['scenes'][0], plan['scenes'][6]):
            self.assertIsNone(scene['annotation_spatial'])
            self.assertIsNone(scene['annotation_geometry_changed'])
            self.assertEqual(scene['annotation_spatial_status'], 'not_recorded')
        self.assertEqual(plan['effective_config']['spatial'], self.definition['spatial'])
        out = self.root/'temporal_only'
        log = io.StringIO()
        with patch.object(m, 'analyze', side_effect=self.fake_extract), contextlib.redirect_stdout(log):
            self.assertEqual(m.evaluate(self.config, self.frozen, out), 0)
        self.assertIn('annotation spatial is missing/null', log.getvalue())
        self.assertEqual(m.read_json(out/'errors.json'), [])
        for path, content in originals.items(): self.assertEqual(path.read_bytes(), content)
        summaries = m.read_json(out/'summary.json')
        self.assertEqual(len(summaries), 36)
        self.assertTrue(all(r['status'] == 'complete' for r in summaries))
        self.assertEqual(summaries[0]['rgb_first_visible_s'], 1.)
        self.assertIsNone(summaries[12]['rgb_first_visible_s'])

    def test_null_required_metadata_reports_file_and_field_before_decode(self):
        config = m.read_json(self.config)
        path = Path(config['sessions'][0]['annotation'])
        for file, transform, field in (
            (self.config, lambda v: None, 'expected JSON object'),
            (self.config, lambda v: dict(v, spatial=None), 'spatial'),
            (self.config, lambda v: dict(v, spatial=dict(v['spatial'], camchain=None)), 'spatial.camchain'),
            (self.config, lambda v: dict(v, sessions=None), 'sessions'),
            (self.config, lambda v: dict(v, sessions=[None]), 'sessions[0]'),
            (path, lambda v: None, 'expected JSON object'),
            (path, lambda v: dict(v, intervals=None), 'intervals'),
            (path, lambda v: dict(v, intervals=[None]), 'intervals[0]'),
            (path, lambda v: dict(v, spatial=[]), 'spatial'),
            (path, lambda v: dict(v, spatial=dict(v['spatial'], camchain_sha256=None)), 'spatial.camchain_sha256'),
            (path, lambda v: dict(v, time_sync=None), 'time_sync'),
        ):
            with self.subTest(file=file, field=field):
                before = file.read_bytes()
                m.write_json(file, transform(m.read_json(file)))
                out = self.root/'invalid_null'
                with patch.object(m, 'analyze') as extract, self.assertRaises(ValueError) as error:
                    m.evaluate(self.config, self.frozen, out)
                self.assertIn(str(file), str(error.exception))
                self.assertIn(field, str(error.exception))
                extract.assert_not_called(); self.assertFalse(out.exists())
                file.write_bytes(before)

    def test_debug_preserves_exception_chain(self):
        m.write_json(self.config, None)
        args = ['--config', str(self.config), '--frozen-dir', str(self.frozen),
                '--output', str(self.root/'debug'), '--preflight']
        log = io.StringIO()
        with contextlib.redirect_stderr(log), self.assertRaises(SystemExit):
            m.main(args)
        self.assertIn(str(self.config), log.getvalue())
        self.assertIn('got null', log.getvalue())
        with self.assertRaisesRegex(ValueError, 'got null'):
            m.main(args+['--debug'])

    def test_bad_metadata_fails_before_output_or_decode(self):
        config = m.read_json(self.config)
        annotation = Path(config['sessions'][0]['annotation'])
        sync = Path(m.read_json(annotation)['time_sync'])
        for path, mutate, message in (
            (self.config, lambda v:v['sessions'].pop(), 'missing static'),
            (self.config, lambda v:v['spatial'].update(camchain_sha256='wrong'), 'calibration differs'),
            (annotation, lambda v:v.update(onset={}), 'onset must match'),
            (annotation, lambda v:v.update(onset=None), 'onset must be an object'),
            (annotation, lambda v:v['onset']['first_visible'].update(rgb_time_s=1003.), 'onset must match'),
            (annotation, lambda v:v.update(rgb_timestamp_source='header'), 'bag timestamps'),
            (annotation, lambda v:v.update(time_sync_sha256='wrong'), 'time sync changed'),
        ):
            before = path.read_bytes()
            value = m.read_json(path); mutate(value); m.write_json(path, value)
            out = self.root/'invalid'
            with patch.object(m, 'analyze') as extract, self.assertRaisesRegex(ValueError, message):
                m.evaluate(self.config, self.frozen, out)
            self.assertFalse(out.exists()); extract.assert_not_called()
            path.write_bytes(before)
        sync.write_text('changed')
        with self.assertRaisesRegex(ValueError, 'time sync changed'):
            m.preflight(self.config, self.frozen)

    def test_changes_after_preflight_are_rejected(self):
        plan, _, _, _ = m.preflight(self.config, self.frozen)
        scene = plan['scenes'][0]
        raw = Path(scene['source_identity']['recording_files'][0]['path'])
        raw.write_bytes(raw.read_bytes()+b'change')
        with self.assertRaisesRegex(ValueError, 'changed since preflight'):
            m.check_unchanged(plan, scene, self.frozen)

    def test_extraction_failure_is_not_zero_candidates_and_other_scenes_continue(self):
        def broken(config, entry, args, *, tile_sink):
            result, rgb, evs = self.fake_extract(config, entry, args, tile_sink=tile_sink)
            if entry['session'] == m.population()[0]['session']:
                rgb[0] = (rgb[0][0], rgb[0][1], .9)  # Break count conservation.
            return result, rgb, evs
        out = self.root/'partially_failed'
        with patch.object(m, 'analyze', side_effect=broken), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(m.evaluate(self.config, self.frozen, out), 1)
        summaries = m.read_json(out/'summary.json')
        self.assertEqual(len(summaries), 36)
        bad = [r for r in summaries if r['status'] == 'failed']
        self.assertEqual(len(bad), 2)
        self.assertTrue(all('candidates' not in r for r in bad))
        self.assertIn('conserve whole-ROI', m.read_json(out/'errors.json')[0]['error'])
        self.assertEqual(m.read_json(out/'run_config.json')['status'], 'failed')

    def test_onset_is_reporting_only_and_alarm_duration_uses_left_state(self):
        parameters, models, _ = m.load_frozen(self.frozen)
        data, meta, _ = background_fixture('evs')
        data['counts'][data['time_s'] >= 1., :2] += 300
        result = m.score_maps(data, meta, 'evs', parameters['settings'], models['evs'], 32)
        before = copy.deepcopy(result)
        scene = dict(m.population()[0], intervals=[(0., 2.)], rgb_first_visible_s=1.)
        a, ca, _, _, active = m.summarize(scene, 'evs', result, parameters['calibration']['evs'], parameters['settings'])
        b, cb, _, _, _ = m.summarize(dict(scene, rgb_first_visible_s=1.7), 'evs', result,
                                   parameters['calibration']['evs'], parameters['settings'])
        self.assertTrue(ca)
        self.assertEqual([r['start_relative_time_s'] for r in ca], [r['start_relative_time_s'] for r in cb])
        self.assertNotEqual(a['first_candidate_minus_onset_ms'], b['first_candidate_minus_onset_ms'])
        for key in before: np.testing.assert_array_equal(before[key], result[key])
        self.assertAlmostEqual(a['active_seconds'], np.diff(result['time_s'])[active[:-1]].sum())


if __name__ == '__main__':
    unittest.main()
