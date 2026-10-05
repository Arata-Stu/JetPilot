import contextlib
import csv
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
import rc_popout_tile_activity as m


class TileTests(unittest.TestCase):
    def test_partial_tiles_mask_and_rgb_density(self):
        mask = np.zeros((5, 7), bool)
        mask[1:4, 1:6] = True
        sink = m.TileSink(4)
        sink.initialize(mask, [(0., 1.)], .1, 2)
        np.testing.assert_array_equal(sink.ids, [0, 1])
        np.testing.assert_array_equal(sink.pixels, [9, 6])
        changed = np.ones_like(mask)
        sink.rgb(0, .1, .2, changed)
        data = sink.finish()['rgb']
        np.testing.assert_array_equal(data['counts'], [[9, 6]])
        np.testing.assert_array_equal(data['counts']/data['valid_pixels'], [[1, 1]])
        self.assertEqual(data['support_start_s'][0], .1)
        self.assertEqual(sink.tiles[1]['width'], 3)

    def test_repeated_events_out_of_order_and_no_interval_carry(self):
        sink = m.TileSink(2)
        sink.initialize(np.ones((2, 4), bool), [(0., .5), (1., 1.5)], .1, 2)
        sink.events(0, np.array([2, 0, 1, 1]), np.array([3, 0, 0, 0]), np.array([0, 0, 0, 0]))
        data = sink.finish()['evs']
        np.testing.assert_array_equal(data['counts'][:3], [[3, 0], [2, 1], [0, 1]])
        np.testing.assert_array_equal(data['counts'][3:], 0)
        np.testing.assert_allclose(data['time_s'], [.2, .3, .4, 1.2, 1.3, 1.4])
        np.testing.assert_array_equal(data['interval'], [0, 0, 0, 1, 1, 1])
        np.testing.assert_allclose(data['support_start_s'], [0, .1, .2, 1, 1.1, 1.2])

    def test_short_interval_has_no_fabricated_windows(self):
        sink = m.TileSink(2)
        sink.initialize(np.ones((2, 2), bool), [(0., .001), (1., 1.0045)], .001, 2)
        data = sink.finish()['evs']
        self.assertEqual(data['counts'].shape, (3, 1))
        np.testing.assert_array_equal(data['interval'], [1, 1, 1])

    def test_memory_budget_rejects_before_histogram_allocation(self):
        sink = m.TileSink(1, max_memory_mb=1)
        with self.assertRaisesRegex(ValueError, 'limit=1'):
            sink.initialize(np.ones((32, 32), bool), [(0., 10.)], .001, 2)

    def test_invalid_grid_and_empty_roi(self):
        with self.assertRaises(ValueError):
            m.TileSink(0)
        with self.assertRaisesRegex(ValueError, 'empty common ROI'):
            m.TileSink(2).initialize(np.zeros((2, 2), bool), [(0., 1.)], .1, 2)

    def test_observation_straddling_boundary_is_excluded(self):
        data = dict(support_start_s=np.array([.7, .9, 1.0, 1.1]),
                    time_s=np.array([.95, 1.0, 1.15, 1.2]),
                    counts=np.array([[100], [999], [2], [888]]), valid_pixels=np.array([4]))
        n, values = m.window_statistics(data, 1., 1.2)
        self.assertEqual(n, 1)
        np.testing.assert_array_equal(values, [.5])
        with self.assertRaisesRegex(ValueError, 'no complete'):
            m.window_statistics(data, 4., 5.)

    def test_review_uses_control_elapsed_time_not_its_record_time(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            sink = m.TileSink(32)
            sink.initialize(np.ones((480, 640), bool), [(0., 1.)], .1, 2)
            meta = dict(tiles=sink.tiles, output_size=sink.size)
            alignments = dict(none=dict(drive_start_s=20.), pos=dict(drive_start_s=10., rgb_first_visible_from_drive_s=1.5))
            for session, zero, count in [('none', 20., 1), ('pos', 10., 2)]:
                folder = out/session; folder.mkdir()
                (folder/'tiles.json').write_text(json.dumps(meta))
                ts = zero + np.array([1.38, 1.39, 1.55, 1.56])
                counts = np.full((4, len(sink.tiles)), count)
                for sensor in ('rgb', 'evs'):
                    np.savez_compressed(folder/f'{sensor}_tiles.npz', time_s=ts, support_start_s=ts-.001,
                                        counts=counts, valid_pixels=sink.pixels)
            self.assertEqual(m.make_reviews(out, ['none', 'pos'], 'none', alignments), [])
            rows = list(csv.DictReader(io.StringIO((out/'tile_diagnostics.csv').read_text())))
            self.assertEqual(len(rows), 2)
            for r in rows:
                self.assertAlmostEqual(float(r['after_p95']), 2/1024)
                self.assertAlmostEqual(float(r['none_after_p95']), 1/1024)
            self.assertTrue((out/'pos/tile_review.svg').is_file())
            # Geometric mismatch must not produce a comparison table.
            meta['tiles'][0]['valid_pixels'] = 1
            (out/'none/tiles.json').write_text(json.dumps(meta))
            self.assertIn('geometry differs', m.make_reviews(out, ['pos'], 'none', alignments)[0]['error'])

    def test_cli_keeps_evaluation_closed_and_checks_conservation(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            config = dict(common=dict(roi={}, sessions=[dict(session=s) for s in ['none', 'pos', 'held_out']]), parameters={})
            source = root/'run_config.json'; source.write_text(json.dumps(config))
            split = root/'split.json'; split.write_text(json.dumps(dict(groups=[
                dict(condition='none', development=['none'], evaluation=['held_out']),
                dict(condition='popout', development=['pos'], evaluation=['held_out_2'])])))
            def check(folder, source_dir):
                self.assertIn(folder.name, ['none', 'pos'])
                return dict(session=folder.name, drive_start_s=0., rgb_first_visible_from_drive_s=None if folder.name=='none' else .5), dict(annotation_sha256='a', time_sync_sha256='s')
            def analyze(config, entry, args, *, tile_sink):
                self.assertIn(entry['session'], ['none', 'pos'])
                tile_sink.initialize(np.ones((4, 4), bool), [(0., 1.)], .01, 2)
                tile_sink.rgb(0, .1, .2, np.ones((4, 4), bool))
                return dict(annotation_sha256='a', time_sync_sha256='s', rgb_first_visible_s=None if entry['session']=='none' else .5), [(0, .2, 1.)], [(0, i*.01, 0) for i in range(2, 100)]
            argv = ['--motion-dir', str(root), '--score-config', str(source), '--split', str(split), '--output', str(root/'out')]
            with patch.object(m, 'check_scene', side_effect=check), patch.object(m, 'analyze', side_effect=analyze), patch.object(m, 'make_reviews', return_value=[]), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(m.main(argv), 0)
                run = json.loads((root/'out/run_config.json').read_text())
                self.assertEqual(run['sessions'], ['none', 'pos'])
                self.assertFalse((root/'out/held_out').exists())
                with np.load(root/'out/none/rgb_tiles.npz', allow_pickle=False) as data:
                    self.assertEqual(data['counts'].sum(), 16)
                with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
                    m.main(argv)
            def inconsistent(*a, **kw):
                result, rgb, evs = analyze(*a, **kw)
                return result, [(0, .2, 0.)], evs
            argv[-1] = str(root/'bad')
            with patch.object(m, 'check_scene', side_effect=check), patch.object(m, 'analyze', side_effect=inconsistent), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(m.main(argv), 1)
                status = json.loads((root/'bad/summary.json').read_text())
                self.assertTrue(all('tile sum' in r['error'] for r in status))


if __name__ == '__main__':
    unittest.main()
