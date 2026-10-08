import contextlib
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
import rc_popout_static_traces as traces
import evaluate_rc_popout_grid_static as static
import test_rc_popout_grid_static as fixture


class StaticTraceTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixture.StaticTransferTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.root, self.frozen = self.fixture.root, self.fixture.frozen
        self.folder = self.root/'static'
        self.session = 'popout-0928-static-100_01'
        with patch.object(static, 'analyze', side_effect=self.extract), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(static.evaluate(self.fixture.config, self.frozen, self.folder), 0)

    def extract(self, config, entry, args, *, tile_sink):
        result, rgb, _ = self.fixture.fake_extract(config, entry, args, tile_sink=tile_sink)
        data = tile_sink.finish()['evs']
        if entry['condition'] != 'none':
            # Strong EVS signal at different cells from the RGB pair.
            data['counts'][:] = 0
            data['counts'][data['time_s'] >= 1., 2:4] = 300
        evs = list(zip(data['interval'], data['time_s'], data['counts'].sum(axis=1)))
        return result, rgb, evs

    def test_native_scores_and_each_sensors_own_candidate_are_preserved(self):
        original = {p:static.digest(p) for p in self.folder.rglob('*') if p.is_file()}
        with patch.object(static, 'analyze', side_effect=AssertionError('must not decode recordings')):
            data, report = traces.prepare(self.folder, self.frozen, self.session)
        pairs = []
        for method in ('rgb', 'evs'):
            trace, sensor = data[method], report['sensors'][method]
            with np.load(self.folder/self.session/f'{method}_background_maps.npz') as saved:
                np.testing.assert_array_equal(trace['time_s'], saved['time_s'][trace['source_indexes']])
                np.testing.assert_allclose(trace['score_s'], saved['score'][trace['source_indexes']], atol=1e-12)
            c = sensor['candidates'][0]
            alarm = np.flatnonzero(trace['alarm'])
            self.assertEqual(len(alarm), 1)
            self.assertAlmostEqual(trace['time_s'][alarm[0]], c['start_relative_time_s'])
            self.assertAlmostEqual(trace['from_rgb_onset_ms'][alarm[0]], c['minus_onset_ms'])
            pair = [c['pair_tile_a'], c['pair_tile_b']]
            np.testing.assert_array_equal(trace['winner_pair_tile_ids'][alarm[0]], pair)
            pairs.append(pair)
            self.assertGreaterEqual(trace['normalized'][alarm[0]], 1.)
        self.assertNotEqual(pairs[0], pairs[1])
        self.assertGreater(len(data['evs']['time_s']), len(data['rgb']['time_s'])*10)
        self.assertEqual(original, {p:static.digest(p) for p in original})

        # A rendering integration check with synthetic data only. Verify that
        # circles are placed at saved alarms rather than final displayed samples.
        import matplotlib.axes
        scatter = matplotlib.axes.Axes.scatter
        marked = []
        def capture(ax, x, y, *args, **kwargs):
            marked.extend(x)
            return scatter(ax, x, y, *args, **kwargs)
        output = self.root/'plot'
        with patch.object(matplotlib.axes.Axes, 'scatter', capture):
            traces.export(output, data, report)
        self.assertEqual(sorted(marked), sorted(s['candidates'][0]['minus_onset_ms']
                                               for s in report['sensors'].values()))
        for name in ('traces_overlay.svg', 'traces_overlay.png', 'rgb_trace_values.csv', 'evs_trace_values.npz',
                     'summary.json', 'index.html'):
            self.assertGreater((output/name).stat().st_size, 100)
        with self.assertRaisesRegex(ValueError, 'already exists'):
            traces.export(output, data, report)

    def test_tampered_history_is_rejected_even_if_candidates_unchanged(self):
        path = self.folder/self.session/'evs_background_maps.npz'
        with np.load(path) as saved:
            maps = {k:saved[k] for k in saved.files}
        maps['score'][10] += 1e-3  # Not an alarm sample; candidate checks alone miss this.
        np.savez_compressed(path, **maps)
        with self.assertRaisesRegex(ValueError, 'saved score differs from frozen replay'):
            traces.prepare(self.folder, self.frozen, self.session)

    def test_coverage_and_missing_onset_are_not_silently_hidden(self):
        for before, after in ((-1, 500), (100, float('nan')), (0, 0)):
            with self.assertRaisesRegex(ValueError, 'before/after'):
                traces.prepare(self.folder, self.frozen, self.session, before, after)
        with self.assertRaisesRegex(ValueError, 'exceeds saved evaluation coverage'):
            traces.prepare(self.folder, self.frozen, self.session, 1100, 500)
        with self.assertRaisesRegex(ValueError, 'first-visible annotation required'):
            traces.prepare(self.folder, self.frozen, 'popout-0928-static-100_06')
        data, report = traces.prepare(self.folder, self.frozen, self.session, 100, 0)
        for method, sensor in report['sensors'].items():
            expected = sum(-100 <= c['minus_onset_ms'] <= 0 for c in sensor['candidates'])
            self.assertEqual(int(data[method]['alarm'].sum()), expected)
        self.assertTrue(all(s['full_evaluation_candidates'] == 1 for s in report['sensors'].values()))


if __name__ == '__main__':
    unittest.main()
