from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
import rc_popout_event_figure as event_figure


def fake_source():
    dtype = [('x', 'i4'), ('y', 'i4'), ('p', 'i4'), ('t', 'i8')]
    # The final batch contains delayed in-window events. Repeated events at one
    # pixel must count twice even though the visualization uses occupancy.
    events = [np.array([(0, 0, 0, 0), (0, 0, 0, 249999), (0, 0, 1, 250000),
                        (0, 0, 1, 250000), (1, 0, 0, 499999), (0, 0, 1, 500000),
                        (3, 0, 1, 375000), (2, 1, 1, 375000)], dtype=dtype),
              np.array([(1, 0, 1, 375000), (0, 0, 0, 900000)], dtype=dtype)]
    return SimpleNamespace(width=3, height=2,
        anchor=SimpleNamespace(reference_time_s=0., source_time_us=0., scale=1.),
        batches=lambda: (SimpleNamespace(events=e) for e in events))


class EventFigureTests(unittest.TestCase):
    def setUp(self):
        self.mask = np.ones((2, 3), bool)
        self.mask[1, 2] = False
        uy, ux = np.indices(self.mask.shape)
        self.mapping = (self.mask, self.mask, ux, uy, self.mask)

    def test_counts_keep_duplicates_late_events_and_detector_bin_boundaries(self):
        counts = event_figure.accumulate(fake_source(), SimpleNamespace(apply=lambda t: t),
                                         0., self.mapping, (0., 1.), 4, 2, .125)
        self.assertEqual(counts.sum(), 4)
        self.assertEqual(counts[0, 0].tolist(), [0, 2])
        self.assertEqual(counts[0, 1].tolist(), [1, 1])
        self.assertEqual(counts[1, 2].sum(), 0)
        meta = dict(tiles=[dict(x=0, y=0, width=1, height=2), dict(x=1, y=0, width=2, height=2)])
        event_figure.verify_counts(counts, self.mask, meta, np.array([2, 2]))
        with self.assertRaisesRegex(ValueError, 'tile by tile'):
            event_figure.verify_counts(counts, self.mask, meta, np.array([1, 3]))

    def test_missing_coverage_and_wrong_sensor_size_fail(self):
        with self.assertRaisesRegex(ValueError, 'does not cover'):
            event_figure.accumulate(fake_source(), SimpleNamespace(apply=lambda t: t),
                                   0., self.mapping, (0., 2.), 12, 2, .125)
        source = fake_source(); source.width = 4
        with self.assertRaisesRegex(ValueError, 'dimensions'):
            event_figure.accumulate(source, SimpleNamespace(apply=lambda t: t),
                                   0., self.mapping, (0., 1.), 4, 2, .125)

    def test_rendered_dots_exist_only_at_measured_event_pixels(self):
        counts = np.zeros((4, 5, 2), np.int64)
        counts[1, 1] = [4, 0]
        counts[1, 2] = [1, 2]
        counts[2, 1] = [0, 3]
        raw = event_figure.event_image(counts, np.ones((4, 5)))
        weighted = event_figure.event_image(counts, np.full((4, 5), .5))
        np.testing.assert_array_equal(raw[counts.sum(axis=2) == 0], 255)
        np.testing.assert_array_equal(weighted[counts.sum(axis=2) == 0], 255)
        np.testing.assert_array_equal(raw[1, 1], [43, 99, 160])
        np.testing.assert_array_equal(raw[2, 1], [210, 74, 67])
        self.assertTrue(np.all(weighted >= raw))
        np.testing.assert_array_equal(event_figure.event_image(counts, np.zeros((4, 5))), 255)

    def test_figure_is_text_free_and_preserves_pixel_counts_and_residual_weights(self):
        from PIL import Image
        from matplotlib.figure import Figure
        counts = np.zeros((32, 64, 2), np.int64)
        counts[8, 8, 1] = 3
        counts[9, 40, 0] = 4
        mask = np.ones((32, 64), bool)
        tiles = [dict(tile_id=i, x=i*32, y=0, width=32, height=32, valid_pixels=1024) for i in (0, 1)]
        evidence = dict(scene=dict(meta=dict(roi=dict(x=0, y=0, width=64, height=32), tiles=tiles)),
                        parameters=dict(settings=dict(z_clip=10.)))
        result = dict(tile_id=np.array([0, 1]), time_s=np.array([1.]))
        values = dict(observed_density=np.array([3., 4.])/1024,
            estimated_background=np.array([2., 1.])/1024, positive_residual=np.array([0., 5.]),
            integrated_residual=np.array([0., .1]), threshold_s=.06)
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            with mock.patch.object(event_figure.figure, 'map_values', return_value=values), \
                 mock.patch('matplotlib.figure.Figure.savefig', autospec=True, side_effect=Figure.savefig) as save:
                event_figure.draw(folder, counts, mask, evidence, result, 0)
                for call in save.call_args_list:
                    fig = call.args[0]
                    self.assertFalse(fig.texts)
                    self.assertTrue(all(not any(t.get_text() for t in ax.texts) for ax in fig.axes))
            with np.load(folder/'source_values.npz') as saved:
                np.testing.assert_array_equal(saved['counts_by_polarity'], counts)
                self.assertEqual(saved['display_weights'][8, 8], 0.)
                self.assertEqual(saved['display_weights'][9, 40], .5)
            with Image.open(folder/'residual_events.png') as image:
                self.assertEqual(image.size, (64, 32))
                self.assertEqual(image.getpixel((8, 8)), (255, 255, 255))
                self.assertNotEqual(image.getpixel((40, 9)), (255, 255, 255))
            with Image.open(folder/'detection_method_events.png') as image:
                self.assertEqual(image.size, (3600, 1200))
            self.assertIn('<svg', (folder/'detection_method_events.svg').read_text())

    def test_compact_replot_changes_display_only_and_checks_transferred_data(self):
        from PIL import Image
        counts = np.zeros((32, 64, 2), np.int64)
        counts[8, 8, 1], counts[9, 40, 0] = 3, 4
        mask = np.ones((32, 64), bool)
        meta = dict(roi=dict(x=0, y=0, width=64, height=32), output_size=[64, 32],
                    tiles=[dict(tile_id=i, x=i*32, y=0, width=32, height=32, valid_pixels=1024) for i in (0, 1)])
        candidate = dict(session='fixture', annotation_sha256='annotation', time_sync_sha256='sync')
        evidence = dict(scene=dict(meta=meta, data=dict(evs=dict(counts=np.array([[3, 4]])))),
            parameters=dict(settings=dict(z_clip=10.), input_definition=dict(spatial=dict(camchain_sha256='chain'))),
            candidate=candidate, provenance=dict(bundle_sha256='bundle'), frozen_manifest_sha256='frozen')
        result = dict(tile_id=np.array([0, 1]), time_s=np.array([1.]), support_start_s=np.array([.998]))
        values = dict(observed_density=np.array([3., 4.])/1024,
            estimated_background=np.array([2., 1.])/1024, positive_residual=np.array([0., 5.]),
            integrated_residual=np.array([0., .1]), threshold_s=.06)
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(event_figure.figure, 'map_values', return_value=values):
            folder = Path(tmp)
            appearance = event_figure.draw(folder, counts, mask, evidence, result, 0, layout='compact')
            with Image.open(folder/'detection_method_events.png') as image:
                self.assertEqual(image.size, (3600, 768))
            self.assertEqual(appearance['residual_display_max'], 5.)
            with np.load(folder/'source_values.npz') as saved:
                np.testing.assert_array_equal(saved['counts_by_polarity'], counts)
                np.testing.assert_array_equal(saved['positive_residual'], values['positive_residual'])
                self.assertEqual(saved['display_weights'][9, 40], 1.)
            report = dict(status='complete', source_mode='native_RAW_event_positions_and_archived_grid_model',
                session='fixture', candidate=candidate, bundle_sha256='bundle', frozen_manifest_sha256='frozen',
                annotation_sha256='annotation', time_sync_sha256='sync', camchain_sha256='chain',
                outputs_sha256={'source_values.npz': event_figure.figure.digest(folder/'source_values.npz')},
                timing=dict(events=7, start_s=.998, end_s=1., window_ms=2.))
            event_figure.figure.write_json(folder/'summary.json', report)
            reread, support, timing, _, history = event_figure.load_export(folder, evidence, result, 0)
            self.assertIsNone(history)
            np.testing.assert_array_equal(reread, counts)
            np.testing.assert_array_equal(support, mask)
            self.assertEqual(timing, report['timing'])
            path = folder/'source_values.npz'
            path.write_bytes(path.read_bytes()+b'changed')
            with self.assertRaisesRegex(ValueError, 'arrays changed'):
                event_figure.load_export(folder, evidence, result, 0)

    def test_long_accumulation_is_disjoint_and_keeps_late_events(self):
        identity = SimpleNamespace(apply=lambda t: t)
        frames = event_figure.accumulate(fake_source(), identity, 0., self.mapping,
                                         (0., 1.), 4, 2, .125, window_count=2)
        self.assertEqual(frames.shape, (2, 2, 3, 2))
        self.assertEqual(frames[0].sum(), 2)
        self.assertEqual(frames[1].sum(), 4)
        combined = event_figure.accumulate(fake_source(), identity, 0., self.mapping,
                                           (0., 1.), 4, 4, .125)
        np.testing.assert_array_equal(frames.sum(axis=0), combined)
        self.assertEqual(frames.sum(), 6)

    def test_display_samples_do_not_use_overlapping_or_future_windows(self):
        evidence = dict(parameters=dict(input_definition=dict(step_ms=1., window_bins=2)))
        times = np.arange(2, 42)*.001
        result = dict(time_s=times, support_start_s=times-.002,
                      ready=np.ones(40, bool), reset=np.zeros(40, bool), interval=np.zeros(40, int))
        indexes = event_figure.display_indexes(evidence, result, 29, 20.)
        np.testing.assert_array_equal(indexes, np.arange(11, 30, 2))
        self.assertAlmostEqual(result['time_s'][29]-result['support_start_s'][indexes[0]], .020)
        self.assertTrue(np.all(result['time_s'][indexes] <= result['time_s'][29]))
        for invalid in (0., -2., 3., 102., float('nan')):
            with self.assertRaisesRegex(ValueError, 'multiple'):
                event_figure.display_indexes(evidence, result, 29, invalid)
        result['reset'][12] = True  # Even between the selected endpoints, a reset invalidates the history.
        with self.assertRaisesRegex(ValueError, 'gap, reset'):
            event_figure.display_indexes(evidence, result, 29, 20.)
        with self.assertRaisesRegex(ValueError, 'coverage'):
            event_figure.display_indexes(evidence, result, 1, 20.)

    def test_past_events_use_their_own_residual_not_the_final_snapshot(self):
        mask = np.ones((1, 2), bool)
        meta = dict(tiles=[dict(x=0, y=0, width=1, height=1), dict(x=1, y=0, width=1, height=1)])
        frames = np.zeros((2, 1, 2, 2), np.int64)
        frames[0, 0, 0, 0], frames[1, 0, 0, 0] = 1, 3
        frames[0, 0, 1, 1], frames[1, 0, 1, 1] = 2, 2
        history = dict(counts_by_window=frames, residual_z=np.array([[10., 0.], [0., 10.]]))
        counts = frames.sum(axis=0)
        opacity = event_figure.display_weights(counts, mask, meta, history['residual_z'][-1], 10., history)
        self.assertEqual(opacity[0, 0, 0], .25)
        self.assertEqual(opacity[0, 1, 1], .5)
        self.assertEqual(opacity[0, 0, 1], 0.)
        image = event_figure.event_image(counts, opacity)
        self.assertNotEqual(image[0, 0].tolist(), [255, 255, 255])
        self.assertTrue(np.all(image >= 0))
        for requested in (10., 20., float('nan')):
            with self.assertRaisesRegex(ValueError, 'use --record-root'):
                event_figure.check_cached_window(dict(window_ms=2.), requested)
        event_figure.check_cached_window(dict(window_ms=20.), None)
        event_figure.check_cached_window(dict(window_ms=20.), 20.)

    def test_long_history_export_reloads_and_checks_each_block(self):
        mask = np.ones((2, 2), bool)
        frames = np.zeros((2, 2, 2, 2), np.int64)
        frames[0, 0, 0, 1], frames[1, 1, 1, 0] = 2, 5
        counts = frames.sum(axis=0)
        meta = dict(roi=dict(x=0, y=0, width=2, height=2), output_size=[2, 2],
                    tiles=[dict(tile_id=0, x=0, y=0, width=2, height=2, valid_pixels=4)])
        candidate = dict(session='fixture', annotation_sha256='ann', time_sync_sha256='sync')
        evidence = dict(scene=dict(meta=meta, data=dict(evs=dict(counts=np.array([[0], [2], [0], [5]])))),
            parameters=dict(settings=dict(z_clip=10.), input_definition=dict(step_ms=1., window_bins=2,
                spatial=dict(camchain_sha256='chain'))), candidate=candidate,
            provenance=dict(bundle_sha256='bundle'), frozen_manifest_sha256='frozen')
        times = np.array([.002, .003, .004, .005])
        result = dict(tile_id=np.array([0]), time_s=times, support_start_s=times-.002,
            ready=np.ones(4, bool), reset=np.zeros(4, bool), interval=np.zeros(4, int),
            residual_z=np.array([[0.], [1.], [2.], [5.]], dtype=np.float32))
        history = dict(counts_by_window=frames, source_indexes=np.array([1, 3]),
            time_s=times[[1, 3]], support_start_s=result['support_start_s'][[1, 3]],
            residual_z=result['residual_z'][[1, 3]])
        values = dict(observed_density=np.array([1.25]), estimated_background=np.array([.2]),
                      positive_residual=np.array([5.]), integrated_residual=np.array([.1]), threshold_s=.06)
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(event_figure.figure, 'map_values', return_value=values):
            folder = Path(tmp)
            appearance = event_figure.draw(folder, counts, mask, evidence, result, 3, layout='compact', history=history)
            self.assertEqual(appearance['residual_time_policy'], 'each disjoint detector window at its endpoint')
            report = dict(status='complete', source_mode='native_RAW_event_positions_and_archived_grid_model',
                session='fixture', candidate=candidate, bundle_sha256='bundle', frozen_manifest_sha256='frozen',
                annotation_sha256='ann', time_sync_sha256='sync', camchain_sha256='chain',
                outputs_sha256={'source_values.npz': event_figure.figure.digest(folder/'source_values.npz')},
                timing=dict(events=7, start_s=.001, end_s=.005, window_ms=4., detector_window_ms=2.))
            event_figure.figure.write_json(folder/'summary.json', report)
            reread, _, _, _, cached = event_figure.load_export(folder, evidence, result, 3)
            np.testing.assert_array_equal(reread, counts)
            np.testing.assert_array_equal(cached['counts_by_window'], frames)
            # Preserve the total image but assign an event to the wrong time block.
            path = folder/'source_values.npz'
            with np.load(path) as saved:
                arrays = {k:saved[k] for k in saved.files}
            arrays['history_counts_by_window'][0, 0, 0, 1] -= 1
            arrays['history_counts_by_window'][1, 0, 0, 1] += 1
            np.savez_compressed(path, **arrays)
            report['outputs_sha256']['source_values.npz'] = event_figure.figure.digest(path)
            event_figure.figure.write_json(folder/'summary.json', report)
            with self.assertRaisesRegex(ValueError, 'tile by tile'):
                event_figure.load_export(folder, evidence, result, 3)


if __name__ == '__main__':
    unittest.main()
