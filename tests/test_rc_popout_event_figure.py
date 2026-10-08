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


if __name__ == '__main__':
    unittest.main()
