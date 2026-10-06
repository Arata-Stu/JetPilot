"""Review saved development flow results, without RAW access or retuning.

Uses Python's standard library. An optional ZIP contains only the selected
development run's metadata, CSVs, tile arrays and already-rendered snapshots.
"""
import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path
from statistics import median
from zipfile import ZIP_DEFLATED, ZipFile

from rc_popout_change_detection import DEFAULT_SPLIT, digest, select_sessions, write_json


ALGORITHM = 'sparse_lk_affine_residual_adjacent_persistent_v1'


def load_rows(path):
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f'empty scores: {path}')
    for i, r in enumerate(rows):
        for k in ('ready', 'active', 'alarm', 'state_reset'):
            if r[k] not in ('True', 'False'):
                raise ValueError(f'invalid boolean {k}: {path}')
            r[k] = r[k] == 'True'
        for k in ('time_s', 'from_drive_s', 'tracks', 'inliers', 'inlier_tiles',
                  'inlier_ratio', 'span_x', 'span_y', 'score', 'ready_observed_step_s'):
            r[k] = None if r[k] == '' else float(r[k])
            if r[k] is not None and not math.isfinite(r[k]):
                raise ValueError(f'nonfinite {k}: {path}')
        r['interval'] = int(r['interval'])
        if r['time_s'] is None or r['from_drive_s'] is None:
            raise ValueError(f'missing timestamp: {path}')
        if i and (r['time_s'] <= rows[i-1]['time_s'] or r['interval'] < rows[i-1]['interval']):
            raise ValueError(f'non-increasing time/interval: {path}')
        if r['ready'] != (r['reason'] == 'ok') or r['ready'] != (r['score'] is not None):
            raise ValueError(f'quality/reason/score mismatch: {path}')
        if r['active'] and not r['ready'] or r['alarm'] and not r['active']:
            raise ValueError(f'alarm during invalid estimate: {path}')
    return rows


def clipped(spans, start, end):
    return [(max(a,start), min(b,end)) for a,b in spans if max(a,start) < min(b,end)]


def coverage(rows, spans):
    """Both estimates valid; left-endpoint alarm hold, no gap or EOF extension."""
    ready = active = longest = 0.
    for start, end in spans:
        run = 0.
        previous_end = None
        for a,b in zip(rows, rows[1:]):
            lo, hi = max(start,a['time_s']), min(end,b['time_s'])
            if hi <= lo:
                continue
            valid = a['ready'] and b['ready'] and not b['state_reset'] and a['interval'] == b['interval']
            if not valid:
                run = 0.; previous_end = None
                continue
            dt = hi-lo
            ready += dt
            active += dt if a['active'] else 0.
            run = run+dt if previous_end is not None and abs(lo-previous_end) < 1e-8 else dt
            longest = max(longest, run)
            previous_end = hi
    return ready, active, longest


def summarize(rows, spans, session, sensor, scope):
    selected = [r for r in rows if any(a <= r['time_s'] < b for a,b in spans)]
    counts = Counter(r['reason'] for r in selected)
    failed = counts.copy(); failed.pop('ok', None)
    duration = sum(b-a for a,b in spans)
    ready, active, longest = coverage(rows, spans)
    top, n = failed.most_common(1)[0] if failed else ('', 0)
    result = dict(session=session, method=sensor, scope=scope, evaluation_seconds=duration,
        samples=len(selected), ready_samples=sum(r['ready'] for r in selected),
        ready_seconds=ready, ready_fraction=ready/duration if duration else None,
        unobservable_seconds=max(0., duration-ready), active_seconds_causal_hold=active,
        longest_ready_run_ms=longest*1000, alarms=sum(r['alarm'] for r in selected),
        most_common_rejection=top, rejection_samples=n)
    for key in ('tracks', 'inliers', 'inlier_ratio', 'inlier_tiles', 'span_x', 'span_y'):
        values = [r[key] for r in selected if r[key] is not None]
        result[key+'_p50'] = median(values) if values else None
    return result, [dict(session=session, method=sensor, scope=scope, reason=k,
                        samples=v, sample_fraction=v/len(selected)) for k,v in counts.most_common()]


def write_csv(path, rows, fields=None):
    with path.open('w', newline='') as stream:
        writer=csv.DictWriter(stream, fieldnames=fields or list(rows[0]))
        writer.writeheader(); writer.writerows(rows)


def inspect_run(root, split):
    run = json.loads((root/'run_config.json').read_text())
    parameters = json.loads((root/'detector_parameters.json').read_text())
    if run['algorithm'] != ALGORITHM or run['subset'] != 'development':
        raise ValueError('only the saved development flow algorithm is supported')
    for k,v in parameters.items():
        if run.get(k) != v:
            raise ValueError(f'run/parameter mismatch: {k}')
    expected = select_sessions(json.loads(split.read_text()), 'development')
    if run['split_sha256'] != digest(split) or run['sessions'] != expected:
        raise ValueError('development split/session mismatch')
    summaries, reasons, pairs = [], [], []
    inputs = [root/name for name in ('run_config.json','detector_parameters.json','summary.csv','summary.json')]
    for session in expected:
        folder=root/session
        result=json.loads((folder/'result.json').read_text())
        if result['settings'] != run['settings'] or result['algorithm'] != run['algorithm']:
            raise ValueError(f'{session}: result/parameter mismatch')
        alignment=result['alignment']; zero=alignment['drive_start_s']
        if alignment['session'] != session or not math.isfinite(zero):
            raise ValueError(f'{session}: invalid alignment')
        spans=result['extraction']['intervals']
        if (not spans or any(not (math.isfinite(a) and math.isfinite(b) and 0 <= a < b) for a,b in spans)
                or any(b > c for (_,b),(c,_) in zip(spans,spans[1:]))):
            raise ValueError(f'{session}: invalid intervals')
        drives=[p for p in alignment['phases'] if p['phase']=='drive']
        if len(drives) != 1:
            raise ValueError(f'{session}: expected one drive phase')
        drive=drives[0]
        scopes=dict(all=spans, before_drive=clipped(spans,-math.inf,zero),
                    drive=[(lo,hi) for a,b in drive['intervals'] for lo,hi in clipped(spans,a,b)])
        onset=alignment.get('rgb_first_visible_from_drive_s')
        if onset is not None:
            scopes['near_onset']=clipped(spans,zero+onset-.030,zero+onset+.250)
        geometry=json.loads((folder/'geometry.json').read_text())
        tiles={t['tile_id']:t for t in geometry['tiles']}
        inputs += [folder/'result.json',folder/'geometry.json']
        for sensor in ('rgb','evs'):
            rows=load_rows(folder/f'{sensor}_flow_scores.csv')
            saved=next(r for r in result['summaries'] if r['method']==sensor)
            if (len(rows) != saved['samples'] or sum(r['ready'] for r in rows) != saved['ready_samples']
                    or dict(Counter(r['reason'] for r in rows)) != result['status_counts'][sensor]
                    or sum(r['alarm'] for r in rows) != saved['candidates']):
                raise ValueError(f'{session}/{sensor}: CSV/result counts differ')
            for r in rows:
                if abs(r['time_s']-zero-r['from_drive_s']) > 1e-6:
                    raise ValueError(f'{session}: time alignment mismatch')
            for scope, windows in scopes.items():
                summary, detail=summarize(rows, windows, session, sensor, scope)
                summaries.append(summary); reasons.extend(detail)
            candidates=json.loads((folder/f'{sensor}_candidates.json').read_text())
            alarm_times=[r['time_s'] for r in rows if r['alarm']]
            if len(candidates) != len(alarm_times) or any(abs(c['time_s']-t)>1e-8 for c,t in zip(candidates,alarm_times)):
                raise ValueError(f'{session}/{sensor}: candidate times differ from scores')
            for number, c in enumerate(candidates,1):
                for first, second in c['pairs']:
                    a,b=tiles[first],tiles[second]
                    pairs.append(dict(session=session,method=sensor,candidate=number,
                        time_s=c['time_s'],from_drive_s=c['time_s']-zero,
                        in_drive=any(x <= c['time_s'] < y for x,y in scopes['drive']),
                        minus_rgb_onset_ms=None if onset is None else 1000*(c['time_s']-zero-onset),
                        tile_a=first,ax=a['x'],ay=a['y'],tile_b=second,bx=b['x'],by=b['y']))
            inputs += [folder/f'{sensor}_{name}' for name in
                       ('flow_scores.csv','flow_tiles.npz','candidates.json','pair_starts.json','phases.json')]
            for snapshot in result['snapshots'][sensor].values():
                name=snapshot['file']
                if Path(name).name != name or not name.startswith(sensor+'_') or not name.endswith('.png'):
                    raise ValueError('invalid snapshot filename')
                inputs.append(folder/name)
    for path in inputs:
        if not path.is_file() or not path.resolve().is_relative_to(root.resolve()):
            raise ValueError(f'missing or external input: {path}')
    return summaries,reasons,pairs,inputs


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--flow-dir', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--split', default=DEFAULT_SPLIT, type=Path)
    p.add_argument('--bundle', action='store_true')
    args=p.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError('output already exists; choose a new directory')
        summaries,reasons,pairs,inputs=inspect_run(args.flow_dir,args.split)
        hashes={str(f.relative_to(args.flow_dir)):digest(f) for f in inputs}
        args.output.mkdir(parents=True)
        write_csv(args.output/'summary.csv',summaries)
        write_csv(args.output/'rejection_reasons.csv',reasons)
        write_csv(args.output/'candidate_locations.csv',pairs,fields=[
            'session','method','candidate','time_s','from_drive_s','in_drive','minus_rgb_onset_ms',
            'tile_a','ax','ay','tile_b','bx','by'])
        write_json(args.output/'provenance.json',dict(input_root=str(args.flow_dir.resolve()),
            review_code_sha256=digest(__file__),input_sha256=hashes,
            note='Saved development artifacts only; no RAW decode, new detector, retuning or current-input verification. '
                 'Active time uses left-endpoint hold; original run used right-endpoint approximation. '
                 'Tracks are after forward/backward filtering; initial corner count and filtering losses were not saved.'))
        if args.bundle:
            with ZipFile(args.output/'flow_debug_bundle.zip','x',compression=ZIP_DEFLATED) as archive:
                for f in inputs:
                    archive.write(f,Path('source')/f.relative_to(args.flow_dir))
                for name in ('summary.csv','rejection_reasons.csv','candidate_locations.csv','provenance.json'):
                    archive.write(args.output/name,Path('review')/name)
        for r in summaries:
            if r['scope'] != 'drive':
                continue
            ratio='n/a' if r['ready_fraction'] is None else f'{r["ready_fraction"]:.1%}'
            print(f'{r["session"]} / {r["method"]}: drive ready={ratio}, '
                  f'longest={r["longest_ready_run_ms"]:.1f}ms, '
                  f'reject={r["most_common_rejection"]} ({r["rejection_samples"]}/{r["samples"]}), '
                  f'tracks_p50={r["tracks_p50"]}, alarms={r["alarms"]}')
        print(f'Review: {args.output}')
        if args.bundle: print(f'Bundle: {args.output}/flow_debug_bundle.zip')
        return 0
    except (ValueError,OSError,KeyError,TypeError,StopIteration) as exc:
        p.exit(1,f'error: {exc}\n')


if __name__=='__main__':
    raise SystemExit(main())
