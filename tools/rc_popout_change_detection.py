"""Causal, robust activity-change candidates from existing RGB/EVS score CSVs.

No ROS/RAW access or neural-network dependencies. Labels are used only in reports.
The statistic is a time-integrated, CUSUM-like score, not a calibrated hypothesis
test: sliding event windows and video samples are temporally dependent.
"""
from __future__ import annotations

import argparse
from collections import deque
import csv
import hashlib
import html
import json
import math
from pathlib import Path
from statistics import median


ROOT = Path(__file__).resolve().parents[1]
ALGORITHM = 'rolling_median_mad_time_cusum_v1'
DEFAULT_SPLIT = ROOT / 'docs/evidence/rc_popout_20260930/development_evaluation_split_v1.json'
DEFAULTS = dict(history_s=0.30, min_history_s=0.20, min_samples=8,
                drift_k=1.0, threshold_s=0.05, z_clip=10.0,
                rgb_scale_floor=0.001, evs_scale_floor=20.0,
                rgb_max_gap_s=0.10, evs_max_gap_s=0.003,
                onset_guard_s=0.030)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def select_sessions(split, subset):
    roles = {name: [] for name in ('development', 'evaluation')}
    for group in split['groups']:
        for role in roles:
            for session in group[role]:
                if not isinstance(session, str) or Path(session).name != session or session in ('.', '..'):
                    raise ValueError('invalid session name in split')
                roles[role].append(session)
    combined = roles['development'] + roles['evaluation']
    if len(set(combined)) != len(combined) or any(not x for x in roles.values()):
        raise ValueError('split must contain disjoint, nonempty development/evaluation sets')
    return roles[subset]


def validate_settings(settings):
    if set(settings) != set(DEFAULTS) | {'evs_window_s'}:
        raise ValueError('unexpected or missing detector settings')
    for key, value in settings.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key} must be finite and positive')
    if not isinstance(settings['min_samples'], int) or settings['min_samples'] < 2:
        raise ValueError('min_samples must be an integer >= 2')
    if settings['min_history_s'] >= settings['history_s']:
        raise ValueError('min_history_s must be smaller than history_s')
    if settings['drift_k'] >= settings['z_clip']:
        raise ValueError('drift_k must be smaller than z_clip')


def read_scores(path, zero):
    rows = []
    with path.open() as stream:
        reader = csv.DictReader(stream)
        required = {'interval', 'relative_time_s', 'from_drive_s', 'phase', 'score'}
        if not required <= set(reader.fieldnames or []):
            raise ValueError(f'missing columns: {path}')
        for r in reader:
            row = dict(interval=int(r['interval']), relative_time_s=float(r['relative_time_s']),
                       from_drive_s=float(r['from_drive_s']), phase=r['phase'], score=float(r['score']))
            if any(not math.isfinite(row[k]) for k in ('relative_time_s', 'from_drive_s', 'score')):
                raise ValueError(f'nonfinite score/time: {path}')
            if row['score'] < 0 or row['interval'] < 0:
                raise ValueError(f'negative score/interval: {path}')
            if not math.isclose(row['relative_time_s'] - zero, row['from_drive_s'], abs_tol=1e-6, rel_tol=0):
                raise ValueError(f'alignment/CSV time mismatch: {path}')
            if rows and (row['relative_time_s'] <= rows[-1]['relative_time_s']
                         or row['interval'] < rows[-1]['interval']):
                raise ValueError(f'times must increase; intervals must not go backwards: {path}')
            rows.append(row)
    if len(rows) < 2:
        raise ValueError(f'insufficient scores: {path}')
    return rows


def detect(rows, sensor, settings):
    """No annotations, phase gates or future rows enter the state update.

    At t, compare x(t) to past score endpoints in [support_start-history,
    support_start]. For RGB support_start is the previous frame-pair endpoint;
    for EVS it is t-window. Thus even overlapping EVS windows cannot put
    current-window events into the reference. Reset and warm up on gaps and
    annotated-interval boundaries, not on command phase transitions.
    """
    validate_settings(settings)
    if sensor not in ('rgb', 'evs'):
        raise ValueError('unsupported sensor')
    pending, history = deque(), deque()
    previous = None
    cusum = 0.0
    active = None
    output, episodes = [], []
    max_gap = settings[f'{sensor}_max_gap_s']
    floor = settings[f'{sensor}_scale_floor']

    def close_episode(row, reason):
        nonlocal active
        if active is not None:
            active.update(end_from_drive_s=row['from_drive_s'], end_reason=reason)
            episodes.append(active)
            active = None

    for row in rows:
        t = row['from_drive_s']
        dt = 0.0 if previous is None else t - previous['from_drive_s']
        reset = (previous is None or row['interval'] != previous['interval']
                 or dt > max_gap + 1e-9)
        if reset:
            if previous is not None:
                close_episode(previous, 'interval_or_gap')
            pending.clear()
            history.clear()
            cusum = 0.0
        support = (None if reset else previous['from_drive_s']) if sensor == 'rgb' else t - settings['evs_window_s']
        baseline = scale = z = None
        ready = False
        alarm = False
        if support is not None:
            while pending and pending[0][0] <= support + 1e-9:
                history.append(pending.popleft())
            cutoff = support - settings['history_s']
            while history and history[0][0] < cutoff - 1e-9:
                history.popleft()
            ready = (len(history) >= settings['min_samples']
                     and history[-1][0] - history[0][0] >= settings['min_history_s'] - 1e-9)
            if ready:
                values = [v for _, v in history]
                baseline = median(values)
                scale = max(floor, 1.4826 * median(abs(v - baseline) for v in values))
                z = max(-settings['z_clip'], min(settings['z_clip'], (row['score'] - baseline) / scale))
                # Backward time quadrature, available at t; never backdate alarm to t-dt.
                cusum = max(0.0, cusum + (z - settings['drift_k']) * dt)
                if active is None and cusum >= settings['threshold_s']:
                    alarm = True
                    active = dict(interval=row['interval'], start_from_drive_s=t,
                                  start_relative_time_s=row['relative_time_s'], start_phase=row['phase'])
                if cusum <= 0:
                    close_episode(row, 'cusum_returned_to_zero')
        if not ready:
            close_episode(previous or row, 'warmup')
            cusum = 0.0
        output.append(dict(row, support_start_from_drive_s=support, ready=ready,
                           background_samples=len(history), background_median=baseline,
                           background_scale=scale, z=z, cusum_s=cusum, alarm=alarm,
                           active=active is not None, state_reset=reset,
                           observed_step_s=0.0 if reset else dt))
        # Always update background, including alarm periods; no label-dependent freezing.
        # RGB's first row after a gap has unknown support and cannot seed background.
        if sensor != 'rgb' or not reset:
            pending.append((t, row['score']))
        previous = row
    if previous is not None:
        close_episode(previous, 'end_of_recording')
    return output, episodes


def summarize(session, sensor, series, episodes, onset, guard):
    """Descriptive timing only; no inferred ground-truth matches or detection rate."""
    starts = [e['start_from_drive_s'] for e in episodes]
    ready_seconds = sum(r['observed_step_s'] for r in series if r['ready'])
    phases = {}
    for row in series:
        phase = phases.setdefault(row['phase'], dict(samples=0, ready_samples=0, alarms=0))
        phase['samples'] += 1
        phase['ready_samples'] += int(row['ready'])
        phase['alarms'] += int(row['alarm'])
    result = dict(session=session, method=sensor, status='complete', samples=len(series),
                  ready_samples=sum(r['ready'] for r in series), ready_observed_seconds=ready_seconds,
                  candidates=len(starts), first_candidate_from_drive_s=starts[0] if starts else None,
                  rgb_first_visible_from_drive_s=onset, phase_counts=phases)
    if onset is not None:
        result.update(candidates_before_guard=sum(t < onset - guard for t in starts),
                      candidates_near_onset=sum(onset - guard <= t <= onset + guard for t in starts),
                      candidates_after_guard=sum(t > onset + guard for t in starts),
                      first_candidate_minus_onset_s=starts[0] - onset if starts else None)
    else:
        result.update(candidates_before_guard=None, candidates_near_onset=None,
                      candidates_after_guard=None, first_candidate_minus_onset_s=None)
    return result


def plot(path, series, episodes, onset, settings, title):
    """Small dependency-free SVG. Downsample display only; CSV retains every point."""
    first, last = series[0]['from_drive_s'], series[-1]['from_drive_s']
    span = max(last - first, 1e-6)
    x = lambda t: 80 + 1000 * (t - first) / span
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="1160" height="690" viewBox="0 0 1160 690">',
             '<rect width="1160" height="690" fill="white"/>',
             f'<text x="20" y="25" font-family="sans-serif" font-size="18">{html.escape(title)}</text>']
    panels = [('score', 'Activity / past median', False), ('z', 'Robust deviation (clipped)', False),
              ('cusum_s', 'Time-integrated CUSUM [s]', True)]
    for i, (key, label, threshold) in enumerate(panels):
        top = 60 + i * 185
        lo = min(0, min((r[key] for r in series if r[key] is not None), default=0))
        hi = max(1e-9, max((r[key] for r in series if r[key] is not None), default=0),
                 settings['threshold_s'] if threshold else 0)
        y = lambda v: top + 135 - 135 * (v - lo) / (hi - lo)
        parts += [f'<text x="80" y="{top-8}" font-family="sans-serif">{label} ({lo:.3g} .. {hi:.3g})</text>',
                  f'<rect x="80" y="{top}" width="1000" height="135" fill="none" stroke="#aaa"/>']
        for field, color in [(key, '#215ca0')] + ([('background_median', '#da7c17')] if key == 'score' else []):
            # Retain extrema in each display bucket and break at gaps/warmup.
            segments, current = [], []
            for row in series:
                if row['state_reset'] or row[field] is None:
                    if current:
                        segments.append(current)
                    current = []
                if row[field] is not None:
                    current.append(row)
            if current:
                segments.append(current)
            for segment in segments:
                stride = max(1, math.ceil(len(segment) / 1500))
                selected = []
                for j in range(0, len(segment), stride):
                    block = segment[j:j + stride]
                    indexes = sorted({0, len(block)-1, min(range(len(block)), key=lambda k: block[k][field]),
                                      max(range(len(block)), key=lambda k: block[k][field])})
                    selected.extend(block[k] for k in indexes)
                points = ' '.join(f'{x(r["from_drive_s"]):.2f},{y(r[field]):.2f}' for r in selected)
                parts.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="1"/>')
        if threshold:
            yy = y(settings['threshold_s'])
            parts.append(f'<path d="M80 {yy} H1080" stroke="#b22" stroke-dasharray="5 4"/>')
        for e in episodes:
            if not first <= e['start_from_drive_s'] <= last:
                continue
            xx = x(e['start_from_drive_s'])
            parts.append(f'<path d="M{xx} {top} v135" stroke="#b22" opacity=".5"/>')
        if onset is not None and first <= onset <= last:
            xx = x(onset)
            parts.append(f'<path d="M{xx} {top} v135" stroke="#19834c" stroke-width="2" stroke-dasharray="5 4"/>')
        for j in range(6):
            t = first + span * j / 5
            parts.append(f'<text x="{x(t)}" y="{top+154}" font-size="12">{t:.2f}</text>')
    parts += ['<text x="80" y="645" font-family="sans-serif">Time from drive command [s]. Red: candidate; green: RGB onset (report only).</text>',
              '<text x="80" y="670" font-family="sans-serif">Blue: signal; orange: past median. Full annotated CSV; no command-phase gating.</text>', '</svg>']
    path.write_text('\n'.join(parts))


def score_config_path(motion_dir, override):
    if override:
        return override.resolve()
    run = json.loads((motion_dir / 'run_config.json').read_text())
    return Path(run['scores_dir']) / 'run_config.json'


def check_scene(folder, source_dir):
    alignment_path = folder / 'alignment.json'
    alignment = json.loads(alignment_path.read_text())
    if alignment['session'] != folder.name:
        raise ValueError('alignment session mismatch')
    if not math.isfinite(alignment['drive_start_s']):
        raise ValueError('invalid drive start')
    onset = alignment.get('rgb_first_visible_from_drive_s')
    if onset is not None and not math.isfinite(onset):
        raise ValueError('invalid onset')
    # Read only the chosen role's metadata. Never read test scenes while developing.
    source_result = source_dir / folder.name / 'result.json'
    if digest(source_result) != alignment['input_result_sha256']:
        raise ValueError('source result changed after alignment; regenerate motion analysis')
    result = json.loads(source_result.read_text())
    ann_path = Path(result['annotation'])
    if digest(ann_path) != result['annotation_sha256'] or result['annotation_sha256'] != alignment['annotation_sha256']:
        raise ValueError('annotation changed after scores/alignment; regenerate upstream analysis')
    ann = json.loads(ann_path.read_text())
    if digest(ann['time_sync']) != result['time_sync_sha256']:
        raise ValueError('time sync changed after score generation')
    return alignment, dict(alignment_sha256=digest(alignment_path),
                           input_result_sha256=digest(source_result),
                           annotation_sha256=digest(ann_path), time_sync_sha256=digest(ann['time_sync']))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--motion-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--split', type=Path, default=DEFAULT_SPLIT)
    parser.add_argument('--subset', choices=['development', 'evaluation'], default='development')
    parser.add_argument('--parameters', type=Path, help='Reuse detector_parameters.json; required for evaluation')
    parser.add_argument('--score-config', type=Path, help='Original detection run_config.json if its directory moved')
    for name, default in DEFAULTS.items():
        parser.add_argument('--' + name.replace('_', '-'), type=int if name == 'min_samples' else float,
                            help=f'Initial exploratory default: {default}; cannot combine with --parameters')
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError('output already exists; choose a new directory')
        split = json.loads(args.split.read_text())
        sessions = select_sessions(split, args.subset)
        code_sha = digest(__file__)
        split_sha = digest(args.split)
        source_config_path = score_config_path(args.motion_dir, args.score_config)
        source_config = json.loads(source_config_path.read_text())
        upstream = source_config['parameters']
        window_s = float(upstream['step_ms']) * int(upstream['window_bins']) / 1000
        overrides = {k: getattr(args, k) for k in DEFAULTS if getattr(args, k) is not None}
        if args.subset == 'evaluation' and args.parameters is None:
            raise ValueError('evaluation requires --parameters from the chosen development run')
        if args.parameters:
            if overrides:
                raise ValueError('--parameters cannot be combined with detector overrides')
            frozen = json.loads(args.parameters.read_text())
            if (frozen['algorithm'] != ALGORITHM or frozen['code_sha256'] != code_sha
                    or frozen['split_sha256'] != split_sha):
                raise ValueError('frozen algorithm/code/split mismatch; do not silently retune evaluation')
            settings = frozen['settings']
            if not math.isclose(settings['evs_window_s'], window_s, abs_tol=1e-12, rel_tol=0):
                raise ValueError('upstream EVS window differs from frozen settings')
        else:
            settings = dict(DEFAULTS, **overrides, evs_window_s=window_s)
        validate_settings(settings)
        expected_step = float(upstream['step_ms']) / 1000
        if settings['evs_max_gap_s'] < expected_step:
            raise ValueError('evs_max_gap_s is smaller than upstream score step')
        args.output.mkdir(parents=True)
        frozen = dict(schema_version=1, algorithm=ALGORITHM, code_sha256=code_sha,
                      split_sha256=split_sha, settings=settings)
        write_json(args.output / 'detector_parameters.json', frozen)
        write_json(args.output / 'run_config.json', dict(
            **frozen, subset=args.subset, sessions=sessions, split=split,
            motion_dir=str(args.motion_dir.resolve()), score_config=str(source_config_path),
            score_config_sha256=digest(source_config_path),
            parameters_source=None if args.parameters is None else str(args.parameters.resolve()),
            note='Exploratory change candidates, not vehicle classification or measured online latency. '
                 'All saved score intervals, including outside_phases, are processed. '
                 'Onset is reporting-only. Background always updates. No p-values or independent-sample claims.'))
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(1, f'error: {error}\n')

    summaries = []
    for session in sessions:
        print(f'[{args.subset}] {session}', flush=True)
        try:
            folder = args.motion_dir / session
            alignment, hashes = check_scene(folder, source_config_path.parent)
            out = args.output / session
            out.mkdir()
            scene_results = []
            for sensor in ('rgb', 'evs'):
                path = folder / f'{sensor}_aligned_scores.csv'
                rows = read_scores(path, alignment['drive_start_s'])
                series, episodes = detect(rows, sensor, settings)
                if not any(r['ready'] for r in series):
                    raise ValueError(f'{sensor}: no warmed-up samples; check gaps/history')
                hashes[f'{sensor}_aligned_scores_sha256'] = digest(path)
                with (out / f'{sensor}_change_scores.csv').open('w') as stream:
                    writer = csv.DictWriter(stream, fieldnames=list(series[0]))
                    writer.writeheader()
                    writer.writerows(series)
                onset = alignment.get('rgb_first_visible_from_drive_s')
                result = summarize(session, sensor, series, episodes, onset, settings['onset_guard_s'])
                scene_results.append(result)
                write_json(out / f'{sensor}_candidates.json', episodes)
                plot(out / f'{sensor}_change_scores.svg', series, episodes, onset, settings, f'{session} / {sensor}')
                # Display crop only: detector ran on the whole CSV without a phase reset.
                phase_end = max((p['end_from_drive_s'] for p in alignment.get('phases', [])), default=3.0)
                zoom = [r for r in series if -0.5 <= r['from_drive_s'] <= phase_end + 0.2]
                if len(zoom) >= 2:
                    plot(out / f'{sensor}_drive_zoom.svg', zoom, episodes, onset, settings,
                         f'{session} / {sensor} / drive zoom (display only)')
                print(f'  {sensor}: candidates={result["candidates"]}, '
                      f'first={result["first_candidate_from_drive_s"]}', flush=True)
            write_json(out / 'result.json', dict(alignment=alignment, input_hashes=hashes, results=scene_results))
            summaries.extend(scene_results)
        except (OSError, ValueError, KeyError, TypeError) as error:
            summaries.append(dict(session=session, method='', status='failed', error=str(error)))
            print(f'  FAILED: {error}', flush=True)
    write_json(args.output / 'summary.json', summaries)
    fields = ['session', 'method', 'status', 'error', 'samples', 'ready_samples', 'ready_observed_seconds',
              'candidates', 'first_candidate_from_drive_s', 'rgb_first_visible_from_drive_s',
              'first_candidate_minus_onset_s', 'candidates_before_guard', 'candidates_near_onset',
              'candidates_after_guard']
    with (args.output / 'summary.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(summaries)
    links = []
    for r in summaries:
        session = html.escape(r['session'], quote=True)
        if r['status'] == 'failed':
            links.append(f'<li>{session}: FAILED {html.escape(r["error"])}</li>')
        else:
            sensor = r['method']
            links.append(f'<li><a href="{session}/{sensor}_change_scores.svg">{session} / {sensor}</a>: '
                         f'{r["candidates"]} candidates; first={r["first_candidate_from_drive_s"]}'
                         + (f' / <a href="{session}/{sensor}_drive_zoom.svg">走行付近</a>'
                            if (args.output/r['session']/f'{sensor}_drive_zoom.svg').exists() else '') + '</li>')
    (args.output / 'index.html').write_text(
        '<!doctype html><meta charset="utf-8"><title>RC popout change detection</title>'
        f'<h1>Activity change candidates / {args.subset}</h1>'
        '<p>全保存区間を処理。赤線は候補、緑線はRGB初出現（判定には未使用）。'
        '候補数は検知成功数ではありません。初期値は未調整です。</p><ul>' + ''.join(links) + '</ul>')
    print(f'Summary: {args.output / "summary.csv"}', flush=True)
    return int(any(r['status'] == 'failed' for r in summaries))


if __name__ == '__main__':
    raise SystemExit(main())
