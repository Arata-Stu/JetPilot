"""Reproduce the static transfer summary from the supplied terminal transcript."""
import argparse
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import statistics

HERE = Path(__file__).resolve().parent
ROOT = Path(__file__).resolve().parents[4]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def save_csv(path, rows):
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader(); writer.writerows(rows)


def distribution(values):
    return dict(n=len(values), mean=statistics.mean(values), median=statistics.median(values),
                min=min(values), max=max(values))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('transcript', type=Path)
    parser.add_argument('--output', type=Path, default=HERE)
    args = parser.parse_args()
    text = args.transcript.read_text()
    lines = text[text.index('session,condition,'):].splitlines()
    boundary = lines.index('[]')
    table = '\n'.join(lines[:boundary])+'\n'
    errors = json.loads('\n'.join(lines[boundary:]))
    assert errors == [], 'Transcript must include the complete errors.json array'
    rows = list(csv.DictReader(io.StringIO(table)))
    expected = {}
    for prefix, limit, numbers in [('popout-0928-static','20pct',range(1,10)),
                                   ('popout-0928-static-100','100pct',range(9))]:
        for i, number in enumerate(numbers):
            expected[prefix+(f'_{number:02d}' if number else '')] = (('left','right','none')[i//3],limit)
    keys = {(r['session'],r['method']) for r in rows}
    assert len(rows) == len(keys) == 36
    assert keys == {(s,m) for s in expected for m in ('rgb','evs')}
    package = ROOT/'docs/evidence/rc_popout_20260930/grid_background_frozen_20261006'
    parameters = json.loads((package/'detector_parameters.json').read_text())
    for r in rows:
        assert (r['condition'],r['transmitter_limit']) == expected[r['session']]
        assert r['status'] == 'complete' and not r['error']
        assert int(r['samples']) == int(r['ready_samples']) > 0
        assert float(r['threshold_s']) == parameters['calibration'][r['method']]['threshold_s']
        active, ready, duration = (float(r[k]) for k in ('active_seconds','ready_observed_seconds','evaluation_seconds'))
        assert 0 <= active <= ready <= duration
        if r['condition'] == 'none':
            assert int(r['candidates']) == 0 and active == 0 and not r['rgb_first_visible_s']
            assert float(r['max_score_s']) < float(r['threshold_s'])
        else:
            assert int(r['candidates']) == int(r['candidates_onset_to_250ms']) == 1
            assert int(r['candidates_before_guard']) == int(r['candidates_before_onset']) == 0
            assert r['active_at_rgb_onset'] == 'False'
            first, onset, delta = (float(r[k]) for k in ('first_candidate_s','rgb_first_visible_s','first_candidate_minus_onset_ms'))
            assert math.isclose((first-onset)*1000, delta, abs_tol=1e-6)
            assert float(r['first_candidate_after_onset_s']) == first
            assert float(r['first_candidate_after_onset_minus_onset_ms']) == delta
            assert 0 <= delta <= 250 and float(r['max_score_s']) >= float(r['threshold_s'])
    sensors = {}
    for sensor in ('rgb','evs'):
        positive = [r for r in rows if r['method']==sensor and r['condition']!='none']
        negative = [r for r in rows if r['method']==sensor and r['condition']=='none']
        sensors[sensor] = dict(
            positive_records=len(positive), positive_candidates=sum(int(r['candidates']) for r in positive),
            positive_records_with_onset_to_250ms_candidate=sum(int(r['candidates_onset_to_250ms'])>0 for r in positive),
            positive_before_onset_candidates=sum(int(r['candidates_before_onset']) for r in positive),
            candidate_minus_rgb_onset_ms=distribution([float(r['first_candidate_minus_onset_ms']) for r in positive]),
            negative_records=len(negative), negative_candidates=sum(int(r['candidates']) for r in negative),
            negative_evaluation_seconds=sum(float(r['evaluation_seconds']) for r in negative),
            negative_ready_observed_seconds=sum(float(r['ready_observed_seconds']) for r in negative))
    by_key = {(r['session'],r['method']):r for r in rows}
    pairs = []
    for name,(condition,limit) in expected.items():
        if condition == 'none': continue
        rgb,evs = (by_key[name,m] for m in ('rgb','evs'))
        assert rgb['rgb_first_visible_s'] == evs['rgb_first_visible_s']
        assert rgb['evaluation_seconds'] == evs['evaluation_seconds']
        pairs.append(dict(session=name, condition=condition, transmitter_limit=limit,
            rgb_candidate_minus_onset_ms=float(rgb['first_candidate_minus_onset_ms']),
            evs_candidate_minus_onset_ms=float(evs['first_candidate_minus_onset_ms']),
            rgb_minus_evs_candidate_ms=(float(rgb['first_candidate_s'])-float(evs['first_candidate_s']))*1000))
    differences = [r['rgb_minus_evs_candidate_ms'] for r in pairs]
    groups = []
    for limit in ('20pct','100pct'):
        for condition in ('left','right'):
            selected = [r for r in pairs if r['condition']==condition and r['transmitter_limit']==limit]
            groups.append(dict(transmitter_limit=limit,condition=condition,n=len(selected),
                **{k:statistics.mean(r[k] for r in selected) for k in (
                    'rgb_candidate_minus_onset_ms','evs_candidate_minus_onset_ms','rgb_minus_evs_candidate_ms')}))
    aggregate = dict(status='summary_arithmetic_verified_spatial_review_pending', rows=len(rows),
        sensors=sensors, paired_difference_ms=distribution(differences),
        raw_numeric_order=dict(evs_earlier=sum(d>0 for d in differences),rgb_earlier=sum(d<0 for d in differences),
                              equal=sum(d==0 for d in differences), note='Raw signs, without uncertainty or spatial verification.'),
        direction_mean_difference_ms={c:statistics.mean(r['rgb_minus_evs_candidate_ms'] for r in pairs if r['condition']==c)
                                      for c in ('left','right')}, groups=groups)
    out = args.output; out.mkdir(parents=True,exist_ok=True)
    (out/'source_user_paste.txt').write_bytes(args.transcript.read_bytes())
    (out/'summary_user_paste.csv').write_text(table)
    save_json(out/'errors_user_paste.json',errors)
    save_csv(out/'paired_times.csv',pairs); save_csv(out/'condition_means.csv',groups)
    save_json(out/'aggregate.json',aggregate)
    save_json(out/'provenance.json',dict(received_date='2026-10-08',
        reported_remote_output='/workspaces/record/09-28/analysis/grid_background_static_trial01',
        input_kind='User terminal transcript, not the remote CSV original bytes',
        source_transcript_sha256=sha(args.transcript), summary_sha256=sha(out/'summary_user_paste.csv'),
        script_sha256=sha(Path(__file__)),
        local_frozen_parameters_sha256=sha(package/'detector_parameters.json'),
        verification='36 rows, complete population and conditions, thresholds, counts and timestamp arithmetic checked.',
        unverified=['Remote run_config and frozen-file provenance','Candidate coordinates and video correspondence',
                    'Re-extracted tile arrays and raw data','Same-input activity baseline results']))
    print(json.dumps(aggregate,ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
