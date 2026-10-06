"""Audit a saved development local-detector run without changing its detector.

The original CSV is authoritative for the global alarm. Per-pair replay uses
saved float32 states and is diagnostic only, not a new detection result.
"""
import argparse
import csv
import html
import json
from pathlib import Path

import numpy as np

from rc_popout_change_detection import DEFAULT_SPLIT, digest, select_sessions, write_json
from rc_popout_local_detection import ALGORITHM, geometry, validate_settings, write_csv


def boolean(value):
    if value not in ('True', 'False'):
        raise ValueError(f'invalid boolean: {value}')
    return value == 'True'


def load_sensor(folder, sensor, meta, tile_px):
    with (folder/f'{sensor}_local_scores.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    for r in rows:
        for key in ('relative_time_s', 'from_drive_s', 'pair_score_s'):
            r[key] = float(r[key])
        for key in ('ready', 'active', 'state_reset'):
            r[key] = boolean(r[key])
        r['interval'] = int(r['interval'])
    with np.load(folder/f'{sensor}_local_state.npz', allow_pickle=False) as data:
        times, ids, states = data['time_s'], data['tile_id'], data['cusum_s'].astype(float)
    if (len(rows) < 2 or states.shape != (len(rows), len(meta['tiles']))
            or times.shape != (len(rows),) or not np.all(np.isfinite(states)) or np.any(states < 0)
            or not np.all(np.isfinite(times)) or np.any(np.diff(times) <= 0)
            or not np.array_equal(ids, [t['tile_id'] for t in meta['tiles']])
            or not np.allclose(times, [r['relative_time_s'] for r in rows], rtol=0, atol=1e-9)):
        raise ValueError('saved state/CSV/geometry mismatch')
    pairs, _ = geometry(meta, tile_px)
    scores = np.minimum(states[:, pairs[:, 0]], states[:, pairs[:, 1]])
    if not np.allclose(scores.max(axis=1), [r['pair_score_s'] for r in rows], rtol=5e-6, atol=1e-8):
        raise ValueError('saved float32 states disagree with original pair score')
    if any(r['active'] and not r['ready'] for r in rows):
        raise ValueError('active alarm during warmup')
    return rows, states, pairs, scores


def occupancy(rows, start=-np.inf, end=np.inf):
    """Causal hold on [t_i,t_(i+1)); no holding across a gap, reset or EOF."""
    observed = active = 0.
    for a, b in zip(rows, rows[1:]):
        if not a['ready'] or not b['ready'] or b['state_reset'] or a['interval'] != b['interval']:
            continue
        dt = max(0., min(end, b['from_drive_s'])-max(start, a['from_drive_s']))
        observed += dt
        active += dt*int(a['active'])
    return observed, active


def replay_pairs(rows, scores, pairs, tiles, settings):
    """Independent hysteresis per pair; labels and directions are absent."""
    live = np.full(len(pairs), -1, dtype=int)
    episodes = []
    threshold, release = settings['threshold_s'], settings['threshold_s']*settings['release_ratio']

    def close(indices, end, reason):
        for j in indices:
            start = int(live[j])
            a, b = (tiles[k] for k in pairs[j])
            r = rows[start]
            previous_active = (start > 0 and not r['state_reset'] and rows[start-1]['active'])
            episodes.append(dict(pair_tile_a=a['tile_id'], pair_tile_b=b['tile_id'],
                ax=a['x'], ay=a['y'], bx=b['x'], by=b['y'],
                start_from_drive_s=r['from_drive_s'], end_from_drive_s=rows[end]['from_drive_s'],
                duration_s=rows[end]['from_drive_s']-r['from_drive_s'],
                global_active_before_start=previous_active, start_phase=r['phase'], end_reason=reason))
        live[indices] = -1

    for i, r in enumerate(rows):
        if r['state_reset'] or not r['ready']:
            close(np.flatnonzero(live >= 0), max(0, i-1), 'reset_or_warmup')
        if not r['ready']:
            continue
        begin = np.flatnonzero((live < 0) & (scores[i] >= threshold))
        live[begin] = i
        close(np.flatnonzero((live >= 0) & (scores[i] <= release)), i, 'below_release')
    close(np.flatnonzero(live >= 0), len(rows)-1, 'end_of_recording')
    episodes.sort(key=lambda e: (e['start_from_drive_s'], e['pair_tile_a'], e['pair_tile_b']))
    boundary = int(np.count_nonzero((np.abs(scores-threshold) <= 1e-8) | (np.abs(scores-release) <= 1e-8)))
    return episodes, boundary


def audit_summary(rows, episodes, alignment, guard):
    observed, active = occupancy(rows)
    drive = next(p for p in alignment['phases'] if p['phase'] == 'drive')
    zero = alignment['drive_start_s']
    drive_observed, drive_active = occupancy(rows, drive['start_s']-zero, drive['end_s']-zero)
    onset = alignment.get('rgb_first_visible_from_drive_s')
    pre_observed, pre_active = (None, None) if onset is None else occupancy(rows, end=onset-guard)
    changes = sum(a['active'] and b['active'] and not b['state_reset'] and
                  (a['pair_tile_a'], a['pair_tile_b']) != (b['pair_tile_a'], b['pair_tile_b'])
                  for a, b in zip(rows, rows[1:]))
    near = None if onset is None else [e for e in episodes if onset-.15 <= e['start_from_drive_s'] <= onset+.25]
    return dict(ready_observed_s=observed, global_active_s=active,
        global_active_fraction=active/observed if observed else None,
        drive_ready_s=drive_observed, drive_active_s=drive_active,
        drive_active_fraction=drive_active/drive_observed if drive_observed else None,
        pre_onset_ready_s=pre_observed, pre_onset_active_s=pre_active,
        winning_pair_changes_during_alarm=changes,
        independent_pair_episodes=len(episodes),
        pair_starts_during_existing_global_alarm=sum(e['global_active_before_start'] for e in episodes),
        onset_window_pair_starts=None if near is None else len(near),
        onset_window_starts_during_global_alarm=None if near is None else sum(e['global_active_before_start'] for e in near))


def snapshot(rows, states, target):
    times = np.array([r['from_drive_s'] for r in rows])
    i = int(np.searchsorted(times, target+1e-9, side='right'))-1
    if i < 0 or not rows[i]['ready'] or (target > times[i]+1e-9 and
            (i+1 == len(rows) or rows[i+1]['state_reset'] or rows[i+1]['interval'] != rows[i]['interval'])):
        return None
    return times[i], states[i]


def spatial_review(path, session, control, meta, saved, alignments, threshold):
    onset = alignments[session]['rgb_first_visible_from_drive_s']
    width, height = meta['output_size']; scale = 320/width
    row_height = height*scale+85
    total_height = 70+2*row_height+35
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="1400" height="{total_height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="20" y="28" font-family="sans-serif" font-size="19">{html.escape(session)} vs {html.escape(control)} / saved local states</text>',
        '<text x="20" y="53" font-family="sans-serif">C / start threshold: dark=0, yellow=2 or more; red outline=C at or above threshold. Not object labels.</text>']
    for row, sensor in enumerate(('rgb', 'evs')):
        y = 108+row*row_height
        parts.append(f'<text x="20" y="{y-32}" font-family="sans-serif">{sensor.upper()}; last available sample at or before target time</text>')
        for col, (scene, delta) in enumerate([(session, -.1), (session, .1), (control, -.1), (control, .1)]):
            x = 20+345*col
            target = onset+delta
            rows, states = saved[scene, sensor]
            value = snapshot(rows, states, target)
            label = f'{scene}: drive +{target:.3f}s'
            parts.append(f'<text x="{x}" y="{y-10}" font-family="sans-serif" font-size="14">{html.escape(label)}</text>')
            parts.append(f'<rect x="{x}" y="{y}" width="320" height="{height*scale}" fill="#ddd"/>')
            if value is None:
                parts.append(f'<text x="{x+8}" y="{y+25}">No ready observation at target</text>')
                continue
            actual, values = value
            for tile, v in zip(meta['tiles'], values):
                q = min(2., float(v)/threshold)/2
                color = f'#{int(20+235*q):02x}{int(35+190*q):02x}{int(70-40*q):02x}'
                parts.append(f'<rect x="{x+tile["x"]*scale}" y="{y+tile["y"]*scale}" '
                    f'width="{tile["width"]*scale}" height="{tile["height"]*scale}" fill="{color}" '
                    f'stroke="{"#f33" if v >= threshold else "white"}" stroke-width="{1 if v >= threshold else .25}">'
                    f'<title>tile {tile["tile_id"]}: C={v:.6g}</title></rect>')
            parts.append(f'<text x="{x}" y="{y+height*scale+20}" font-family="sans-serif" font-size="12">sample: drive +{actual:.6f}s</text>')
    parts.append(f'<text x="20" y="{total_height-15}" font-family="sans-serif">Targets: positive RGB onset -100 / +100 ms. Control uses same elapsed drive time; not matched vehicle position.</text></svg>')
    path.write_text('\n'.join(parts))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--local-dir', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--split', type=Path, default=DEFAULT_SPLIT)
    args = p.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError('output already exists; choose a new directory')
        run = json.loads((args.local_dir/'run_config.json').read_text())
        params = json.loads((args.local_dir/'detector_parameters.json').read_text())
        split = json.loads(args.split.read_text()); sessions = select_sessions(split, 'development')
        if (run['algorithm'] != ALGORITHM or run['subset'] != 'development' or run['sessions'] != sessions
                or run['split_sha256'] != digest(args.split)
                or any(run[k] != params[k] for k in params)):
            raise ValueError('saved development run/parameters/split mismatch')
        settings = run['settings']; validate_settings(settings)
        tile_dir = Path(run['tile_dir'])
        if digest(tile_dir/'run_config.json') != run['tile_run_sha256']:
            raise ValueError('tile extraction metadata changed')
        control = [s for g in split['groups'] if g['condition'] == 'none' for s in g['development']]
        if len(control) != 1:
            raise ValueError('one development control required')
        complete = json.loads((args.local_dir/'summary.json').read_text())
        if any(not any(r['session'] == s and r.get('method') == sensor and r['status'] == 'complete'
                       for r in complete) for s in sessions for sensor in ('rgb', 'evs')):
            raise ValueError('local development run incomplete')
        args.output.mkdir(parents=True)
        write_json(args.output/'run_config.json', dict(source=str(args.local_dir.resolve()), settings=settings,
            source_run_sha256=digest(args.local_dir/'run_config.json'), review_code_sha256=digest(__file__),
            geometry_code_sha256=digest(Path(__file__).with_name('rc_popout_local_detection.py')),
            sessions=sessions, subset='development', numpy_version=np.__version__,
            note='Audit only. Original CSV global alarms are authoritative. Independent pair replay uses saved float32 states. No new detection-success count.'))
    except (ValueError, KeyError, OSError, TypeError) as error:
        p.exit(1, f'error: {error}\n')
    summaries, all_episodes, saved, metas, alignments = [], [], {}, {}, {}
    input_hashes = {}
    for session in sessions:
        try:
            folder = args.local_dir/session
            result = json.loads((folder/'result.json').read_text())
            meta_path = tile_dir/session/'tiles.json'
            if digest(meta_path) != result['input_hashes']['tile_metadata_sha256']:
                raise ValueError('tile geometry changed after detection')
            meta = json.loads(meta_path.read_text()); metas[session] = meta
            alignment = result['alignment']; alignments[session] = alignment
            if alignment['session'] != session:
                raise ValueError('alignment session mismatch')
            scene_rows, scene_episodes, scene_saved = [], [], {}
            hashes = dict(result_sha256=digest(folder/'result.json'), tiles_sha256=digest(meta_path))
            for sensor in ('rgb', 'evs'):
                rows, states, pairs, scores = load_sensor(folder, sensor, meta, run['input_definition']['tile_px'])
                if not np.allclose([r['relative_time_s']-alignment['drive_start_s'] for r in rows], [r['from_drive_s'] for r in rows], rtol=0, atol=1e-8):
                    raise ValueError('CSV/alignment mismatch')
                for suffix in ('local_scores.csv', 'local_state.npz', 'candidates.json'):
                    hashes[f'{sensor}_{suffix}_sha256'] = digest(folder/f'{sensor}_{suffix}')
                episodes, boundary = replay_pairs(rows, scores, pairs, meta['tiles'], settings)
                onset = alignment.get('rgb_first_visible_from_drive_s')
                for e in episodes:
                    e.update(session=session, method=sensor,
                             start_minus_onset_s=None if onset is None else e['start_from_drive_s']-onset)
                summary = dict(session=session, method=sensor, status='complete',
                    global_episodes=len(json.loads((folder/f'{sensor}_candidates.json').read_text())),
                    **audit_summary(rows, episodes, alignment, settings['onset_guard_s']),
                    near_float32_threshold_values=boundary)
                scene_rows.append(summary); scene_episodes.extend(episodes); scene_saved[session, sensor] = (rows, states)
                print(f'{session} / {sensor}: drive active={summary["drive_active_s"]:.3f}/{summary["drive_ready_s"]:.3f}s, '
                      f'winner changes={summary["winning_pair_changes_during_alarm"]}', flush=True)
            summaries.extend(scene_rows); all_episodes.extend(scene_episodes); saved.update(scene_saved); input_hashes[session] = hashes
        except (ValueError, KeyError, OSError, TypeError) as error:
            summaries.append(dict(session=session, method='', status='failed', error=str(error)))
            print(f'{session}: FAILED {error}', flush=True)
    fields = ['session', 'method', 'status', 'error', 'global_episodes', 'ready_observed_s', 'global_active_s', 'global_active_fraction',
              'drive_ready_s', 'drive_active_s', 'drive_active_fraction', 'pre_onset_ready_s', 'pre_onset_active_s',
              'winning_pair_changes_during_alarm', 'independent_pair_episodes', 'pair_starts_during_existing_global_alarm',
              'onset_window_pair_starts', 'onset_window_starts_during_global_alarm', 'near_float32_threshold_values']
    write_csv(args.output/'summary.csv', summaries, fields)
    write_json(args.output/'summary.json', summaries); write_json(args.output/'input_hashes.json', input_hashes)
    fields = ['session', 'method', 'pair_tile_a', 'pair_tile_b', 'ax', 'ay', 'bx', 'by', 'start_from_drive_s', 'end_from_drive_s',
              'duration_s', 'global_active_before_start', 'start_phase', 'end_reason', 'start_minus_onset_s']
    write_csv(args.output/'pair_episodes.csv', all_episodes, fields)
    near = [e for e in all_episodes if e['start_minus_onset_s'] is not None and -.15 <= e['start_minus_onset_s'] <= .25]
    write_csv(args.output/'onset_pair_starts.csv', near, fields)
    links, errors = [], []
    for session in sessions:
        if session not in alignments or alignments[session].get('rgb_first_visible_from_drive_s') is None:
            continue
        if any((s, sensor) not in saved for s in (session, control[0]) for sensor in ('rgb', 'evs')):
            continue
        if metas[session]['tiles'] != metas[control[0]]['tiles'] or metas[session]['output_size'] != metas[control[0]]['output_size']:
            errors.append(dict(session=session, error='control geometry differs')); continue
        spatial_review(args.output/f'{session}.svg', session, control[0], metas[session], saved, alignments, settings['threshold_s'])
        links.append(f'<li><a href="{html.escape(session, quote=True)}.svg">{html.escape(session)}：局所状態と負例</a></li>')
    write_json(args.output/'review_errors.json', errors)
    (args.output/'index.html').write_text('<!doctype html><meta charset="utf-8"><h1>局所検知の監査</h1>'
        '<p>検知器・閾値は変更していません。元CSVの全体警報と、保存float32状態から独立に再現したペア反応を区別します。</p>'
        '<p>ペア反応数は物体数・検知成功数ではありません。図の赤枠は各タイルの積分値が開始閾値以上。対象位置との照合は別途必要です。</p>'
        '<ul>'+''.join(links)+'</ul><p>詳細：summary.csv / pair_episodes.csv / onset_pair_starts.csv</p>' +
        ''.join(f'<p>FAILED: {html.escape(str(r))}</p>' for r in summaries if r['status'] != 'complete') +
        ''.join(f'<p>FAILED review: {html.escape(str(r))}</p>' for r in errors))
    print(f'Report: {args.output/"index.html"}', flush=True)
    return int(bool(errors) or any(r['status'] != 'complete' for r in summaries))


if __name__ == '__main__':
    raise SystemExit(main())
