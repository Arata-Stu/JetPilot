"""Small causal/replay checks; real bundle reproduction is recorded separately."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from analyze_rc_popout_development_bundle import negative_thresholds, replay_threshold, smooth_activity


class BundleDiagnosticTests(unittest.TestCase):
    def test_hysteresis_and_reset(self):
        score = np.array([0., .3, .2, .3, .12, .3, .3])
        ready = np.array([True, True, True, True, True, False, True])
        reset = np.array([True, False, False, True, False, False, False])
        starts, active = replay_threshold(score, ready, reset, .25, .5)
        self.assertEqual(starts, [1, 3, 6])
        np.testing.assert_array_equal(active, [False, True, True, True, False, False, True])

    def test_prefix_invariance_and_gap(self):
        data = dict(time_s=np.array([0., .001, .002, .05, .051]),
                    support_start_s=np.array([-.002, -.001, 0., .048, .049]),
                    interval=np.zeros(5, dtype=int), counts=np.array([[0], [10], [10], [90], [10]]),
                    valid_pixels=np.array([100]))
        settings = {'evs_max_gap_s': .003}
        full = smooth_activity(data, [np.array([0])], settings, 'evs')
        short = {k: (v if k == 'valid_pixels' else v[:3]) for k, v in data.items()}
        np.testing.assert_array_equal(full[:3], smooth_activity(short, [np.array([0])], settings, 'evs'))
        self.assertGreater(full[2,0], 0.)
        self.assertEqual(full[3,0], 0.)

    def test_common_row_activity_removed(self):
        data = dict(time_s=np.arange(4)*.001, support_start_s=np.arange(4)*.001-.002,
                    interval=np.zeros(4, dtype=int), counts=np.full((4,3), 10),
                    valid_pixels=np.full(3,100))
        values = smooth_activity(data, [np.arange(3)], {'evs_max_gap_s': .003}, 'evs', True)
        np.testing.assert_array_equal(values, 0.)

    def test_calibration_uses_entire_control_and_ignores_positive_amplitude(self):
        def item(peak, onset):
            return dict(score=np.array([0., peak, 0.]), ready=np.ones(3, bool),
                reset=np.array([True, False, False]), time_s=np.array([0.,1.,3.]),
                cusum_s=np.array([[0.,0.],[peak,peak],[0.,0.]]), pairs=np.array([[0,1]]),
                scene=dict(alignment=dict(drive_start_s=0.,rgb_first_visible_from_drive_s=onset),
                    meta=dict(tiles=[dict(tile_id=0,x=0,y=0),dict(tile_id=1,x=32,y=0)])))
        results = {(scene, sensor):item(.23 if scene=='t_0.2-none' else 10., None if scene=='t_0.2-none' else 1.)
                   for scene in ('t_0.2-none','positive') for sensor in ('rgb','evs')}
        calibration, _ = negative_thresholds(results, dict(threshold_s=.05,release_ratio=.5,onset_guard_s=.03))
        self.assertEqual([r['threshold'] for r in calibration], [.25,.25])
        # A late peak in the negative must also contribute, not just driving.
        results['t_0.2-none','rgb']['score'][-1] = .4
        calibration, _ = negative_thresholds(results, dict(threshold_s=.05,release_ratio=.5,onset_guard_s=.03))
        self.assertEqual(calibration[0]['threshold'], .45)


if __name__ == '__main__':
    unittest.main()
