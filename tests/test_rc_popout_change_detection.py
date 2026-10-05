import contextlib
import csv
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import rc_popout_change_detection as m


def settings(**changes):
    return dict(m.DEFAULTS, evs_window_s=.002, **changes)


def rows(step=.01, until=1., jump_at=.6, high=100., interval=0):
    return [dict(interval=interval, relative_time_s=10+t, from_drive_s=t,
                 phase='drive', score=20. if t < jump_at else high)
            for i in range(round(until/step)+1) for t in [round(i*step, 8)]]


def fixture(root):
    motion = root/'motion'
    source = root/'scores'
    motion.mkdir(); source.mkdir()
    m.write_json(motion/'run_config.json', dict(scores_dir=str(source)))
    m.write_json(source/'run_config.json', dict(parameters=dict(step_ms=1, window_bins=2)))
    split = root/'split.json'
    m.write_json(split, dict(groups=[dict(development=['dev'], evaluation=['test'])]))
    for session in ('dev', 'test'):
        a = motion/session; a.mkdir()
        b = source/session; b.mkdir()
        sync = b/'sync.yaml'; sync.write_text('sync model')
        ann = b/'annotation.json'
        m.write_json(ann, dict(time_sync=str(sync)))
        result = b/'result.json'
        m.write_json(result, dict(annotation=str(ann), annotation_sha256=m.digest(ann),
                                 time_sync_sha256=m.digest(sync)))
        m.write_json(a/'alignment.json', dict(session=session, drive_start_s=10.,
                                             rgb_first_visible_from_drive_s=.6,
                                             input_result_sha256=m.digest(result),
                                             annotation_sha256=m.digest(ann)))
        for sensor, step, high in [('rgb', 1/60, .02), ('evs', .001, 100.)]:
            with (a/f'{sensor}_aligned_scores.csv').open('w') as f:
                fields = ['interval', 'relative_time_s', 'from_drive_s', 'phase', 'score']
                w = csv.DictWriter(f, fieldnames=fields); w.writeheader()
                data = rows(step=step, high=high)
                for r in data:
                    if sensor == 'rgb' and r['from_drive_s'] < .6:
                        r['score'] = .001
                w.writerows(data)
    return motion, source, split


class ChangeTests(unittest.TestCase):
    def test_constant_background_no_alarm_and_scale_floor(self):
        data = rows(high=20.)
        p = dict(settings(), evs_max_gap_s=.02)
        output, events = m.detect(data, 'evs', p)
        self.assertFalse(events)
        ready = [r for r in output if r['ready']]
        self.assertTrue(ready)
        self.assertTrue(all(r['z'] == 0 and r['background_scale'] == 20 for r in ready))
        self.assertFalse(output[0]['ready'])

    def test_positive_step_detects_without_backdating(self):
        p = dict(settings(), evs_max_gap_s=.02)
        output, events = m.detect(rows(), 'evs', p)
        self.assertTrue(events)
        self.assertGreaterEqual(events[0]['start_from_drive_s'], .6)
        self.assertLess(events[0]['start_from_drive_s'], .65)
        self.assertTrue(any(r['alarm'] for r in output))

    def test_negative_step_is_not_positive_change(self):
        _, events = m.detect(rows(high=0), 'evs', dict(settings(), evs_max_gap_s=.02))
        self.assertFalse(events)

    def test_one_sample_spike_is_clipped_and_does_not_alarm(self):
        data = rows(step=.001, high=20.)
        data[600]['score'] = 1e9
        output, events = m.detect(data, 'evs', settings())
        self.assertFalse(events)
        self.assertEqual(output[600]['z'], 10)

    def test_prefix_invariant_to_future_data(self):
        data = rows(step=.001)
        partial, _ = m.detect(data[:651], 'evs', settings())
        data[700]['score'] = 1e9
        full, _ = m.detect(data, 'evs', settings())
        self.assertEqual(partial, full[:651])

    def test_phase_labels_do_not_gate_detector(self):
        data = rows(step=.001)
        a, ea = m.detect(data, 'evs', settings())
        b, eb = m.detect([dict(r, phase='outside_phases') for r in data], 'evs', settings())
        self.assertEqual([r['cusum_s'] for r in a], [r['cusum_s'] for r in b])
        self.assertEqual([r['start_from_drive_s'] for r in ea], [r['start_from_drive_s'] for r in eb])

    def test_different_sample_rates_use_elapsed_time(self):
        first = []
        for step in (.001, .002):
            _, events = m.detect(rows(step=step), 'evs', settings())
            first.append(events[0]['start_from_drive_s'])
        self.assertLessEqual(abs(first[0]-first[1]), .002 + 1e-9)

    def test_gap_and_interval_boundary_reset_and_rewarm(self):
        for kind in ('gap', 'interval'):
            data = rows(step=.001)
            if kind == 'gap':
                data = [r for r in data if not .65 < r['from_drive_s'] < .8]
                reset_at = .8
            else:
                data = [dict(r, interval=int(r['from_drive_s'] >= .8)) for r in data]
                reset_at = .8
            output, events = m.detect(data, 'evs', settings())
            r = next(r for r in output if r['from_drive_s'] == reset_at)
            self.assertTrue(r['state_reset'])
            self.assertFalse(r['ready']); self.assertEqual(r['cusum_s'], 0)
            self.assertEqual(events[0]['end_reason'], 'interval_or_gap')

    def test_reference_excludes_current_event_window(self):
        data = rows(step=.001, jump_at=.4)
        p = dict(settings(), evs_window_s=.05)
        output, _ = m.detect(data, 'evs', p)
        # At .44, all reference score endpoints are <= .39, before the step.
        r = next(r for r in output if r['from_drive_s'] == .44)
        self.assertEqual(r['background_median'], 20)
        self.assertAlmostEqual(r['support_start_from_drive_s'], .39)

    def test_onset_changes_report_only_not_candidates(self):
        data, events = m.detect(rows(step=.001), 'evs', settings())
        a = m.summarize('s', 'evs', data, events, .6, .03)
        b = m.summarize('s', 'evs', data, events, .8, .03)
        self.assertEqual(a['first_candidate_from_drive_s'], b['first_candidate_from_drive_s'])
        self.assertNotEqual(a['candidates_before_guard'], b['candidates_before_guard'])

    def test_invalid_split_and_parameters_rejected(self):
        with self.assertRaises(ValueError):
            m.select_sessions(dict(groups=[dict(development=['a'], evaluation=['a'])]), 'development')
        for patch in ({'rgb_scale_floor': 0}, {'history_s': .1}, {'z_clip': float('nan')}, {'min_samples': 2.5},
                      {'decay_tau_s': -.1}, {'release_ratio': -1}, {'release_ratio': 1}):
            with self.assertRaises(ValueError):
                m.validate_settings(dict(settings(), **patch))

    def test_decay_integral_independent_of_rate_and_bounded(self):
        tau, rate = .1, 9.
        expected = tau * rate * (1-math.exp(-1/tau))
        for dt in (.001, .01, .05):
            value = 0.
            for _ in range(round(1/dt)):
                value = m.advance_cusum(value, rate, dt, tau)
            self.assertAlmostEqual(value, expected, places=12)
            self.assertLessEqual(value, tau*rate)
        # Background z=0 => rate=-k=-1; an old maximal transient must clear.
        value = tau*rate
        for _ in range(250):
            value = m.advance_cusum(value, -1., .001, tau)
        self.assertEqual(value, 0.)

    def test_two_changes_are_separated_without_hiding_first_alarm(self):
        for sensor, step, divisor in [('evs', .001, 1), ('rgb', 1/60, 20000)]:
            data = rows(step=step, until=2., jump_at=.6)
            for r in data:
                if r['from_drive_s'] >= 1.05:
                    r['score'] = 300.
                r['score'] /= divisor
            _, old = m.detect(data, sensor, settings())
            _, new = m.detect(data, sensor, dict(settings(), decay_tau_s=.1, release_ratio=.5))
            self.assertEqual(len(old), 1)
            self.assertEqual(len(new), 2)
            self.assertGreaterEqual(new[0]['start_from_drive_s'], .6)
            self.assertLess(new[0]['start_from_drive_s'], .65)
            self.assertLess(new[0]['end_from_drive_s'], 1.05)
            self.assertGreaterEqual(new[1]['start_from_drive_s'], 1.05)
            self.assertEqual(new[0]['end_reason'], 'cusum_below_release')

    def test_leaky_prefix_and_phase_invariance(self):
        data = rows(step=.001)
        p = dict(settings(), decay_tau_s=.1, release_ratio=.5)
        a, _ = m.detect(data[:651], 'evs', p)
        data[700]['score'] = 1e9
        b, _ = m.detect(data, 'evs', p)
        c, _ = m.detect([dict(r, phase='outside_phases') for r in data], 'evs', p)
        self.assertEqual(a, b[:651])
        self.assertEqual([r['alarm'] for r in b], [r['alarm'] for r in c])

    def test_release_can_rearm_before_cusum_reaches_zero(self):
        data = rows(step=.001)
        p = dict(settings(), decay_tau_s=.1, release_ratio=.5)
        series, events = m.detect(data, 'evs', p)
        end = next(r for r in series if r['from_drive_s'] == events[0]['end_from_drive_s'])
        self.assertFalse(end['active'])
        self.assertGreater(end['cusum_s'], 0)
        self.assertLessEqual(end['cusum_s'], p['threshold_s']*p['release_ratio'])
        report = m.summarize('s', 'evs', series, events, .9, .03)
        self.assertFalse(report['active_candidate_at_onset'])

    def test_cli_development_and_frozen_evaluation(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            motion, source, split = fixture(root)
            dev = root/'dev_out'; ev = root/'eval_out'
            base = ['--motion-dir', str(motion), '--split', str(split)]
            # Removing evaluation input must not affect development runs.
            eval_csv = motion/'test/evs_aligned_scores.csv'
            original = eval_csv.read_text(); eval_csv.unlink()
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(m.main(base + ['--output', str(dev), '--decay-tau-s', '.1', '--release-ratio', '.5']), 0)
            self.assertFalse((dev/'test').exists())
            summary = json.loads((dev/'summary.json').read_text())
            self.assertEqual({r['session'] for r in summary}, {'dev'})
            self.assertEqual({r['method'] for r in summary}, {'rgb', 'evs'})
            self.assertTrue((dev/'dev/rgb_change_scores.svg').exists())
            with (dev/'candidates.csv').open() as f:
                candidates = list(csv.DictReader(f))
            self.assertEqual({r['session'] for r in candidates}, {'dev'})
            self.assertTrue(all(float(r['duration_s']) >= 0 for r in candidates))
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                m.main(base + ['--subset', 'evaluation', '--output', str(ev)])
            self.assertFalse(ev.exists())
            frozen = ['--parameters', str(dev/'detector_parameters.json')]
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                m.main(base + frozen + ['--subset', 'evaluation', '--threshold-s', '.01', '--output', str(ev)])
            stale = root/'old_parameters.json'
            old = json.loads((dev/'detector_parameters.json').read_text())
            old['algorithm'] = 'rolling_median_mad_time_cusum_v1'
            m.write_json(stale, old)
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                m.main(base + ['--parameters', str(stale), '--subset', 'evaluation', '--output', str(ev)])
            eval_csv.write_text(original)
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(m.main(base + frozen + ['--subset', 'evaluation', '--output', str(ev)]), 0)
            self.assertFalse((ev/'dev').exists())
            self.assertEqual(json.loads((dev/'detector_parameters.json').read_text()),
                             json.loads((ev/'detector_parameters.json').read_text()))
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                m.main(base + ['--output', str(dev)])

    def test_cli_changed_sync_fails_instead_of_reporting_success(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            motion, source, split = fixture(root)
            (source/'dev/sync.yaml').write_text('changed')
            with contextlib.redirect_stdout(io.StringIO()):
                status = m.main(['--motion-dir', str(motion), '--split', str(split), '--output', str(root/'out')])
            self.assertEqual(status, 1)
            summary = json.loads((root/'out/summary.json').read_text())
            self.assertEqual(summary[0]['status'], 'failed')
            self.assertIn('time sync changed', summary[0]['error'])

    def test_csv_invalid_values_order_and_alignment(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)/'scores.csv'
            header = 'interval,relative_time_s,from_drive_s,phase,score\n'
            for body in ('0,10,0,drive,nan\n0,11,1,drive,0\n',
                         '0,10,0,drive,0\n0,10,0,drive,0\n',
                         '0,10,1,drive,0\n0,11,2,drive,0\n'):
                p.write_text(header+body)
                with self.assertRaises(ValueError):
                    m.read_scores(p, 10.)


if __name__ == '__main__':
    unittest.main()
