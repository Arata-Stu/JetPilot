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
import rc_popout_local_review as m
import test_rc_popout_local_detection as local_tests


def synthetic():
    times = np.arange(11)/10
    scores = np.zeros((len(times), 2))
    scores[2:6, 0] = .1; scores[5:9, 1] = .1
    _, meta = local_tests.fixture()
    pairs = np.array([[0, 1], [4, 5]])
    rows = []
    for i, t in enumerate(times):
        winner = int(np.argmax(scores[i]))
        rows.append(dict(from_drive_s=float(t), relative_time_s=float(t), interval=0, ready=True,
            active=bool(scores[i].max() >= .05), state_reset=i == 0, phase='drive',
            pair_tile_a=int(pairs[winner, 0]), pair_tile_b=int(pairs[winner, 1])))
    return rows, scores, pairs, meta


class LocalReviewTests(unittest.TestCase):
    def test_spatial_relay_is_not_one_independent_pair(self):
        rows, scores, pairs, meta = synthetic()
        episodes, boundary = m.replay_pairs(rows, scores, pairs, meta['tiles'], local_tests.m.DEFAULTS)
        self.assertEqual(len(episodes), 2)
        self.assertFalse(episodes[0]['global_active_before_start'])
        self.assertTrue(episodes[1]['global_active_before_start'])
        self.assertAlmostEqual(episodes[0]['start_from_drive_s'], .2)
        self.assertAlmostEqual(episodes[1]['start_from_drive_s'], .5)
        self.assertEqual(boundary, 0)
        summary = m.audit_summary(rows, episodes, dict(drive_start_s=0., phases=[dict(phase='drive', start_s=.3, end_s=.8)],
            rgb_first_visible_from_drive_s=.5), .03)
        self.assertAlmostEqual(summary['global_active_s'], .7)
        self.assertAlmostEqual(summary['drive_active_s'], .5)
        self.assertEqual(summary['winning_pair_changes_during_alarm'], 1)
        self.assertEqual(summary['pair_starts_during_existing_global_alarm'], 1)
        # Overlapping spatial pair durations must never be summed as alarm occupancy.
        self.assertAlmostEqual(sum(e['duration_s'] for e in episodes), .8)

    def test_occupancy_does_not_cross_gap_or_warmup_or_eof(self):
        rows, _, _, _ = synthetic()
        rows[6]['state_reset'] = True; rows[6]['ready'] = False
        observed, active = m.occupancy(rows)
        self.assertAlmostEqual(observed, .8)
        self.assertAlmostEqual(active, .5)
        self.assertEqual(m.occupancy(rows, start=2.), (0., 0.))
        rows[6]['state_reset'] = False
        self.assertEqual(m.occupancy(rows), (observed, active))

    def test_snapshot_uses_past_and_rejects_unobserved_time(self):
        rows, scores, _, _ = synthetic()
        when, state = m.snapshot(rows, scores, .45)
        self.assertEqual(when, .4)
        np.testing.assert_array_equal(state, [.1, 0])
        self.assertIsNone(m.snapshot(rows, scores, -.1))
        self.assertIsNone(m.snapshot(rows, scores, 1.1))
        rows[5]['state_reset'] = True
        self.assertIsNone(m.snapshot(rows, scores, .45))
        rows[5]['ready'] = False
        self.assertIsNone(m.snapshot(rows, scores, .5))

    def test_float32_boundary_is_flagged_not_called_a_true_detection(self):
        rows, scores, pairs, meta = synthetic()
        scores[3, 0] = float(np.float32(.05))
        _, boundary = m.replay_pairs(rows, scores, pairs, meta['tiles'], local_tests.m.DEFAULTS)
        self.assertEqual(boundary, 1)

    def test_full_cache_audit_preserves_original_and_checks_consistency(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            root = Path(tmp)
            argv = local_tests.LocalTests().prepare_cache(root)
            self.assertEqual(local_tests.m.main(argv), 0)
            before = m.digest(root/'out/summary.csv')
            review = ['--local-dir', str(root/'out'), '--split', str(root/'split.json'), '--output', str(root/'review')]
            self.assertEqual(m.main(review), 0)
            self.assertEqual(m.digest(root/'out/summary.csv'), before)
            with (root/'review/summary.csv').open() as f: rows = list(csv.DictReader(f))
            self.assertEqual(len(rows), 4)
            self.assertTrue(all(r['status'] == 'complete' for r in rows))
            self.assertTrue((root/'review/pos.svg').is_file())
            self.assertEqual(json.loads((root/'review/review_errors.json').read_text()), [])
            self.assertFalse((root/'review/held_out_pos').exists())
            with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
                m.main(review)
            npz = root/'out/pos/evs_local_state.npz'
            with np.load(npz, allow_pickle=False) as f: arrays = {k: f[k] for k in f.files}
            arrays['cusum_s'][:] = 0
            np.savez_compressed(npz, **arrays)
            self.assertEqual(m.main(review[:-1]+[str(root/'bad_review')]), 1)
            bad = json.loads((root/'bad_review/summary.json').read_text())
            self.assertTrue(any('disagree' in r.get('error', '') for r in bad))


if __name__ == '__main__':
    unittest.main()
