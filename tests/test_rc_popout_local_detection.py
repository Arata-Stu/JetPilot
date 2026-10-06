import contextlib
import csv
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
import rc_popout_local_detection as m


def fixture(sensor='evs', duration=1.8, step=None):
    step = step or (.02 if sensor == 'rgb' else .001)
    size = 32
    tiles = [dict(tile_id=i, x=i % 6*size, y=i//6*size, width=size, height=size, valid_pixels=size*size) for i in range(12)]
    meta = dict(tiles=tiles, output_size=[192, 64], roi=dict(x=0, y=0, width=192, height=64), step_s=.001, event_window_s=.002)
    times = np.arange(2 if sensor == 'evs' else 1, round(duration/step))*step
    data = dict(time_s=times, support_start_s=times-(.002 if sensor == 'evs' else step),
                interval=np.zeros(len(times), dtype=int), counts=np.zeros((len(times), 12), dtype=int),
                tile_id=np.arange(12), valid_pixels=np.full(12, size*size))
    return data, meta


class LocalTests(unittest.TestCase):
    def test_constant_and_uniform_launch_do_not_alarm(self):
        for sensor in ['rgb', 'evs']:
            with self.subTest(sensor=sensor):
                data, meta = fixture(sensor)
                for launch in [False, True]:
                    if launch:
                        data['counts'][data['time_s'] >= .8, :] = 400
                    rows, episodes, state = m.detect(data, meta, sensor, m.DEFAULTS, 32)
                    self.assertEqual(episodes, [])
                    self.assertTrue(any(r['ready'] for r in rows))
                    np.testing.assert_array_equal(state['cusum_s'], 0)

    def test_local_pair_triggers_and_reports_current_time(self):
        for sensor in ['rgb', 'evs']:
            data, meta = fixture(sensor)
            data['counts'][data['time_s'] >= .8, :2] = 400
            rows, episodes, state = m.detect(data, meta, sensor, m.DEFAULTS, 32)
            self.assertTrue(episodes)
            e = episodes[0]
            self.assertGreaterEqual(e['start_time_s'], .8)
            self.assertLess(e['start_time_s'], .9)
            self.assertEqual(e['pair_tile_ids_at_start'], [0, 1])
            trigger = next(r for r in rows if r['alarm'])
            self.assertEqual(trigger['relative_time_s'], e['start_time_s'])
            self.assertGreaterEqual(trigger['pair_score_s'], m.DEFAULTS['threshold_s'])

    def test_isolated_or_disconnected_tiles_do_not_agree(self):
        data, meta = fixture()
        for columns in [[0], [0, 5]]:
            data['counts'][:] = 0
            data['counts'][np.ix_(data['time_s'] >= .8, columns)] = 400
            _, episodes, state = m.detect(data, meta, 'evs', m.DEFAULTS, 32)
            self.assertEqual(episodes, [])
            self.assertGreater(state['cusum_s'].max(), m.DEFAULTS['threshold_s'])

    def test_two_pulses_release_and_rearm(self):
        data, meta = fixture()
        for a, b in [(.6, .66), (1.2, 1.26)]:
            data['counts'][(data['time_s'] >= a) & (data['time_s'] < b), :2] = 400
        _, episodes, _ = m.detect(data, meta, 'evs', m.DEFAULTS, 32)
        self.assertEqual(len(episodes), 2)
        self.assertLess(episodes[0]['end_time_s'], 1.2)
        self.assertGreaterEqual(episodes[1]['start_time_s'], 1.2)

    def test_future_and_onset_annotations_do_not_change_prior_decisions(self):
        data, meta = fixture()
        data['counts'][data['time_s'] >= .8, :2] = 200
        original, episodes, states = m.detect(data, meta, 'evs', m.DEFAULTS, 32)
        prefix = data['time_s'] < 1.
        short = {k: v[prefix] if k not in ['tile_id', 'valid_pixels'] else v for k, v in data.items()}
        short_rows, _, short_states = m.detect(short, meta, 'evs', m.DEFAULTS, 32)
        self.assertEqual(short_rows, original[:len(short_rows)])
        np.testing.assert_array_equal(short_states['cusum_s'], states['cusum_s'][prefix])
        data['counts'][~prefix] = 99999
        changed, _, _ = m.detect(data, meta, 'evs', m.DEFAULTS, 32)
        self.assertEqual(changed[:len(short_rows)], short_rows)
        for zero, onset in [(0., None), (20., -100.)]:
            report, e = m.attach_reporting(original, episodes, dict(drive_start_s=zero,
                rgb_first_visible_from_drive_s=onset, phases=[dict(phase='drive', start_s=0., end_s=.5)]))
            self.assertEqual([r['alarm'] for r in report], [r['alarm'] for r in original])
            self.assertEqual([r['start_time_s'] for r in e], [r['start_time_s'] for r in episodes])

    def test_reference_excludes_current_support_even_for_overlapping_windows(self):
        data, meta = fixture()
        rows, _, _ = m.detect(data, meta, 'evs', m.DEFAULTS, 32)
        ready = [r for r in rows if r['ready']]
        self.assertTrue(ready)
        for r in ready:
            self.assertLessEqual(r['background_end_s'], r['support_start_s']+1e-9)
            self.assertLess(r['background_end_s'], r['relative_time_s'])

    def test_gap_and_interval_reset_warmup(self):
        for mode in ['gap', 'interval']:
            data, meta = fixture()
            data['counts'][data['time_s'] >= .8, :2] = 400
            cut = int(np.searchsorted(data['time_s'], .85))
            if mode == 'gap':
                data['time_s'][cut:] += .1; data['support_start_s'][cut:] += .1
            else:
                data['interval'][cut:] = 1
            rows, episodes, states = m.detect(data, meta, 'evs', m.DEFAULTS, 32)
            self.assertTrue(rows[cut]['state_reset'])
            self.assertFalse(rows[cut]['ready'])
            np.testing.assert_array_equal(states['cusum_s'][cut], 0)
            self.assertEqual(episodes[0]['end_reason'], 'interval_or_gap')
            self.assertAlmostEqual(episodes[0]['end_time_s'], data['time_s'][cut-1])

    def test_rgb_long_frame_pair_does_not_seed_reference(self):
        data, meta = fixture('rgb')
        cut = int(np.searchsorted(data['time_s'], .8))
        data['time_s'][cut:] += .2
        data['support_start_s'][cut+1:] += .2
        rows, _, _ = m.detect(data, meta, 'rgb', m.DEFAULTS, 32)
        self.assertTrue(rows[cut]['state_reset'])
        self.assertFalse(rows[cut]['ready'])
        self.assertEqual(rows[cut+1]['background_samples'], 0)

    def test_quantization_floor_handles_partial_tiles(self):
        data, meta = fixture()
        data['valid_pixels'][:] = 4
        for tile in meta['tiles']:
            tile['valid_pixels'] = 4
        data['counts'][data['time_s'] >= .8, :2] = 1
        rows, episodes, _ = m.detect(data, meta, 'evs', m.DEFAULTS, 32)
        self.assertEqual(episodes, [])
        self.assertLessEqual(max(r['peak_residual_z'] or 0 for r in rows), 1.)

    def test_grid_never_wraps_rows_or_joins_diagonals(self):
        _, meta = fixture()
        pairs, _ = m.geometry(meta, 32)
        self.assertNotIn((5, 6), map(tuple, pairs))
        self.assertNotIn((0, 7), map(tuple, pairs))
        self.assertIn((0, 6), map(tuple, pairs))

    def test_invalid_arrays_and_settings_rejected(self):
        for key, value in [('release_ratio', 1.), ('history_s', .1), ('min_samples', 2.5), ('z_clip', float('nan'))]:
            with self.assertRaises(ValueError):
                m.validate_settings(dict(m.DEFAULTS, **{key: value}))
        for damage in ['counts', 'times', 'ids', 'window']:
            data, meta = fixture()
            if damage == 'counts': data['counts'][0, 0] = -1
            if damage == 'times': data['time_s'][1] = data['time_s'][0]
            if damage == 'ids': data['tile_id'][0] = 999
            if damage == 'window': data['support_start_s'][0] -= .01
            with self.subTest(damage=damage), self.assertRaises(ValueError):
                m.detect(data, meta, 'evs', m.DEFAULTS, 32)

    def prepare_cache(self, root):
        tile_dir, motion, source = (root/name for name in ['tiles', 'motion', 'source'])
        for p in [tile_dir, motion, source]: p.mkdir()
        split = dict(groups=[dict(condition='none', development=['none'], evaluation=['held_out_none']),
                             dict(condition='popout', development=['pos'], evaluation=['held_out_pos'])])
        split_path = root/'split.json'; m.write_json(split_path, split)
        _, meta = fixture()
        config = dict(common=dict(roi=meta['roi'], spatial=dict(output_size=meta['output_size'])),
                      parameters=dict(step_ms=1., window_bins=2, rgb_pixel_delta=15, rgb_topic='rgb'))
        m.write_json(source/'run_config.json', config)
        m.write_json(tile_dir/'run_config.json', dict(subset='development', split=split, sessions=['none', 'pos'],
            tile_px=32, source_config=config, source_config_path=str(source/'run_config.json'),
            source_config_sha256=m.digest(source/'run_config.json'), motion_dir=str(motion)))
        m.write_json(tile_dir/'summary.json', [dict(session=s, status='complete') for s in ['none', 'pos']])
        for scene in ['none', 'pos']:
            for parent in [tile_dir, motion, source]: (parent/scene).mkdir()
            sync = root/f'{scene}_sync.yaml'; sync.write_text('sync')
            ann = root/f'{scene}_annotation.json'; m.write_json(ann, dict(time_sync=str(sync)))
            result = dict(session=scene, annotation=str(ann), annotation_sha256=m.digest(ann), time_sync_sha256=m.digest(sync), valid_pixels=12288)
            m.write_json(source/scene/'result.json', result)
            alignment = dict(session=scene, drive_start_s=.4, rgb_first_visible_from_drive_s=None if scene=='none' else .4,
                input_result_sha256=m.digest(source/scene/'result.json'), annotation_sha256=m.digest(ann),
                phases=[dict(phase='drive', start_s=.4, end_s=1.6, end_from_drive_s=1.2)])
            m.write_json(motion/scene/'alignment.json', alignment)
            _, hashes = m.check_scene(motion/scene, source)
            m.write_json(tile_dir/scene/'result.json', dict(source_result=result, alignment=alignment, input_hashes=hashes))
            m.write_json(tile_dir/scene/'tiles.json', meta)
            for sensor in ['rgb', 'evs']:
                data, _ = fixture(sensor, duration=1.2)
                if scene == 'pos': data['counts'][data['time_s'] >= .8, :2] = 400
                np.savez_compressed(tile_dir/scene/f'{sensor}_tiles.npz', **data)
        return ['--tile-dir', str(tile_dir), '--split', str(split_path), '--output', str(root/'out')]

    def test_cli_cache_only_outputs_and_frozen_reuse(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            root = Path(tmp); argv = self.prepare_cache(root)
            self.assertEqual(m.main(argv), 0)
            with (root/'out/summary.csv').open() as stream: rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 4)
            self.assertTrue(all(r['status'] == 'complete' for r in rows))
            self.assertTrue(all(int(r['candidates']) == 0 for r in rows if r['session'] == 'none'))
            self.assertTrue(all(int(r['candidates']) > 0 for r in rows if r['session'] == 'pos'))
            self.assertFalse((root/'out/held_out_pos').exists())
            self.assertTrue((root/'out/pos/evs_local_scores.svg').exists())
            saved = root/'out/detector_parameters.json'
            rerun = argv[:-1]+[str(root/'repeat'), '--parameters', str(saved)]
            self.assertEqual(m.main(rerun), 0)
            self.assertEqual((root/'out/candidates.csv').read_bytes(), (root/'repeat/candidates.csv').read_bytes())
            for extra in [[], ['--threshold-s', '.1']]:
                with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
                    m.main(rerun+extra)
            frozen = json.loads(saved.read_text()); frozen['code_sha256'] = {}
            m.write_json(root/'bad_parameters.json', frozen)
            with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
                m.main(argv[:-1]+[str(root/'bad'), '--parameters', str(root/'bad_parameters.json')])
            # Sync edits cause a scene failure rather than use stale cached activity.
            (root/'pos_sync.yaml').write_text('changed')
            self.assertEqual(m.main(argv[:-1]+[str(root/'stale')]), 1)
            report = json.loads((root/'stale/summary.json').read_text())
            self.assertTrue(any(r['session'] == 'pos' and r['status'] == 'failed' and 'sync changed' in r['error'] for r in report))


if __name__ == '__main__':
    unittest.main()
