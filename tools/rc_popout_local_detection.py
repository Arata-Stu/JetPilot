"""Development-only causal spatial change candidates from cached tile activity.

Per-tile past median/MAD, contemporaneous row-common subtraction, leaky
integration and adjacent-pair agreement. Labels enter reports only.
"""
import argparse
import csv
import html
import json
import math
from pathlib import Path

import numpy as np

from rc_popout_change_detection import DEFAULT_SPLIT, check_scene, digest, select_sessions, summarize, write_json


ALGORITHM = 'tile_past_mad_row_common_adjacent_leaky_v1'
DEFAULTS = dict(history_s=.30, min_history_s=.20, min_samples=8,
                rgb_scale_floor=.02, evs_scale_floor=.005,
                rgb_max_gap_s=.10, evs_max_gap_s=.003,
                z_clip=10., drift_k=1., decay_tau_s=.10, threshold_s=.05,
                release_ratio=.5, onset_guard_s=.030)


def validate_settings(settings):
    if set(settings) != set(DEFAULTS):
        raise ValueError('unexpected detector settings')
    for key, value in settings.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError(f'invalid setting: {key}')
        if value == 0 and key != 'release_ratio':
            raise ValueError(f'{key} must be positive')
    if not isinstance(settings['min_samples'], int) or settings['min_samples'] < 2:
        raise ValueError('min_samples must be integer >= 2')
    if (settings['min_history_s'] >= settings['history_s'] or settings['drift_k'] >= settings['z_clip']
            or settings['release_ratio'] >= 1):
        raise ValueError('invalid history, drift or release setting')


def geometry(meta, tile_px):
    """Use physical grid neighbors; never join across a row wrap or missing cell."""
    tiles = meta['tiles']
    width, height = meta['output_size']
    if not tiles or not isinstance(tile_px, int) or tile_px < 1:
        raise ValueError('empty or invalid tile geometry')
    cols = (width + tile_px - 1)//tile_px
    locations = {}
    for i, tile in enumerate(tiles):
        x, y = tile['x'], tile['y']
        if (any(not isinstance(tile[k], int) for k in ('x', 'y', 'width', 'height', 'tile_id', 'valid_pixels'))
                or x % tile_px or y % tile_px or not (0 <= x < width and 0 <= y < height)
                or tile['tile_id'] != y//tile_px*cols+x//tile_px
                or tile['width'] != min(tile_px, width-x) or tile['height'] != min(tile_px, height-y)
                or not 0 < tile['valid_pixels'] <= tile['width']*tile['height'] or (x, y) in locations):
            raise ValueError('inconsistent tile geometry')
        locations[x, y] = i
    pairs = [(i, locations[pos]) for (x, y), i in locations.items()
             for pos in ((x+tile_px, y), (x, y+tile_px)) if pos in locations]
    if not pairs:
        raise ValueError('ROI has no adjacent tile pair')
    rows = [np.array([i for i, tile in enumerate(tiles) if tile['y'] == y])
            for y in sorted({tile['y'] for tile in tiles})]
    # The same-row median includes every valid tile, including the current tile.
    # Do not silently substitute a different background rule for very narrow ROIs.
    if min(map(len, rows)) < 3:
        raise ValueError('at least 3 valid tiles per row required for row-common comparison')
    return np.asarray(pairs, dtype=int), rows


def validate_data(data, meta, sensor):
    required = {'interval', 'time_s', 'support_start_s', 'counts', 'tile_id', 'valid_pixels'}
    if not required <= data.keys():
        raise ValueError('missing tile arrays')
    t, start, interval, counts = (data[k] for k in ('time_s', 'support_start_s', 'interval', 'counts'))
    n, k = len(t), len(meta['tiles'])
    if (n < 2 or t.shape != (n,) or start.shape != (n,) or interval.shape != (n,)
            or counts.shape != (n, k) or data['tile_id'].shape != (k,) or data['valid_pixels'].shape != (k,)):
        raise ValueError('invalid tile array shapes')
    if (not np.issubdtype(counts.dtype, np.integer) or not np.issubdtype(interval.dtype, np.integer)
            or np.any(counts < 0) or np.any(interval < 0)
            or not np.all(np.isfinite(t)) or not np.all(np.isfinite(start))
            or np.any(np.diff(t) <= 0) or np.any(np.diff(interval) < 0) or np.any(start >= t)):
        raise ValueError('invalid tile counts/times/intervals')
    if (not np.array_equal(data['tile_id'], [tile['tile_id'] for tile in meta['tiles']])
            or not np.array_equal(data['valid_pixels'], [tile['valid_pixels'] for tile in meta['tiles']])):
        raise ValueError('tile columns differ from geometry')
    same = interval[1:] == interval[:-1]
    if np.any((np.diff(start) < -1e-9) & same):
        raise ValueError('observation supports go backwards')
    if sensor == 'rgb':
        if np.any(counts > data['valid_pixels'][None, :]):
            raise ValueError('RGB changed pixels exceed valid area')
        if not np.allclose(start[1:][same], t[:-1][same], rtol=0, atol=1e-9):
            raise ValueError('RGB frame-pair supports are not consecutive')
    elif sensor == 'evs':
        if not np.allclose(t-start, meta['event_window_s'], rtol=0, atol=1e-9):
            raise ValueError('EVS support differs from saved window')
    else:
        raise ValueError('unsupported sensor')


def detect(data, meta, sensor, settings, tile_px):
    """No onset, drive timestamp, direction or command phase is accepted here."""
    validate_settings(settings)
    validate_data(data, meta, sensor)
    pairs, row_groups = geometry(meta, tile_px)
    times, support, interval = (data[k] for k in ('time_s', 'support_start_s', 'interval'))
    density = data['counts']/data['valid_pixels'][None, :]
    # A one-count quantization floor prevents partial tiles from acquiring
    # arbitrarily large standardized values just because their area is small.
    floor = np.maximum(settings[f'{sensor}_scale_floor'], 1./data['valid_pixels'])
    max_gap = settings[f'{sensor}_max_gap_s']
    n, k = density.shape
    state = np.zeros(k)
    residuals = np.full((n, k), np.nan, dtype=np.float32)
    states = np.zeros((n, k), dtype=np.float32)
    output, episodes = [], []
    active = None
    segment_start = 0

    def close(index, reason):
        nonlocal active
        if active is not None:
            active.update(end_time_s=float(times[index]), end_reason=reason)
            episodes.append(active)
            active = None

    for i, t in enumerate(times):
        dt = 0. if i == 0 else float(t-times[i-1])
        bad_support = t-support[i] > max_gap+1e-9
        reset = i == 0 or interval[i] != interval[i-1] or dt > max_gap+1e-9 or bad_support
        if reset:
            if i:
                close(i-1, 'interval_or_gap')
            state[:] = 0
            segment_start = i+1 if bad_support else i
        # Only reference windows ending no later than the current support start.
        hi = min(i, int(np.searchsorted(times, support[i]+1e-9, side='right')))
        lo = max(segment_start, int(np.searchsorted(times, support[i]-settings['history_s']-1e-9)))
        size = max(0, hi-lo)
        ready = size >= settings['min_samples'] and times[hi-1]-times[lo] >= settings['min_history_s']-1e-9
        alarm = False
        pair_score, peak_residual, common_max = 0., None, None
        pair = None
        if ready:
            past = density[lo:hi]
            center = np.median(past, axis=0)
            scale = np.maximum(floor, 1.4826*np.median(np.abs(past-center), axis=0))
            z = np.clip((density[i]-center)/scale, -settings['z_clip'], settings['z_clip'])
            common = np.zeros(k)
            for group in row_groups:
                common[group] = max(0., float(np.median(z[group])))
            residual = np.clip(z-common, -settings['z_clip'], settings['z_clip'])
            tau = settings['decay_tau_s']
            state = np.maximum(0., math.exp(-dt/tau)*state-tau*math.expm1(-dt/tau)*(residual-settings['drift_k']))
            pair_scores = np.minimum(state[pairs[:, 0]], state[pairs[:, 1]])
            best = int(np.argmax(pair_scores))
            pair_score = float(pair_scores[best])
            if pair_score > 0:
                pair = [int(data['tile_id'][j]) for j in pairs[best]]
            peak_residual, common_max = float(residual.max()), float(common.max())
            residuals[i] = residual
            if active is None and pair_score >= settings['threshold_s']:
                alarm = True
                active = dict(interval=int(interval[i]), start_time_s=float(t), pair_tile_ids_at_start=pair)
            if pair_score <= settings['threshold_s']*settings['release_ratio']:
                close(i, 'pair_score_below_release')
        else:
            close(max(0, i-1), 'warmup')
            state[:] = 0
        states[i] = state
        output.append(dict(interval=int(interval[i]), relative_time_s=float(t),
            support_start_s=float(support[i]), ready=bool(ready), background_samples=size,
            background_end_s=float(times[hi-1]) if size else None,
            peak_residual_z=peak_residual, row_common_max_z=common_max, pair_score_s=pair_score,
            pair_tile_a=None if pair is None else pair[0], pair_tile_b=None if pair is None else pair[1],
            active_tiles=int(np.count_nonzero(state >= settings['threshold_s'])),
            alarm=alarm, active=active is not None, state_reset=bool(reset), observed_step_s=0. if reset else dt))
    close(n-1, 'end_of_recording')
    return output, episodes, dict(residual_z=residuals, cusum_s=states, time_s=times, tile_id=data['tile_id'])


def attach_reporting(series, episodes, alignment):
    zero = alignment['drive_start_s']
    def phase(t):
        return next((p['phase'] for p in alignment.get('phases', []) if p['start_s'] <= t < p['end_s']), 'outside_phases')
    series = [dict(r, from_drive_s=r['relative_time_s']-zero, phase=phase(r['relative_time_s'])) for r in series]
    episodes = [dict(e, start_from_drive_s=e['start_time_s']-zero,
                     end_from_drive_s=e['end_time_s']-zero, start_phase=phase(e['start_time_s'])) for e in episodes]
    return series, episodes


def plot(path, series, episodes, onset, settings, title):
    begin, end = series[0]['from_drive_s'], series[-1]['from_drive_s']
    x = lambda t: 65+1010*(t-begin)/max(end-begin, 1e-9)
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="1150" height="470" viewBox="0 0 1150 470">',
             '<rect width="1150" height="470" fill="white"/>',
             f'<text x="20" y="25" font-family="sans-serif" font-size="18">{html.escape(title)}</text>']
    for panel, (key, label) in enumerate([('peak_residual_z', 'Maximum local residual z'), ('pair_score_s', 'Adjacent-pair integrated score [s]')]):
        top = 65+panel*180
        vmax = max(settings['threshold_s'] if panel else 1., max((r[key] for r in series if r[key] is not None), default=0))
        y = lambda v: top+120-120*max(0., v)/vmax
        parts.append(f'<text x="65" y="{top-10}" font-family="sans-serif">{label} / 0 .. {vmax:.4g}</text>')
        parts.append(f'<rect x="65" y="{top}" width="1010" height="120" fill="none" stroke="#aaa"/>')
        # Min/max envelopes, display only. No time-series interpolation across gaps.
        buckets = {}
        for r in series:
            if r[key] is not None and r['ready']:
                px = int(x(r['from_drive_s']))
                lo, hi = buckets.get(px, (r[key], r[key]))
                buckets[px] = (min(lo, r[key]), max(hi, r[key]))
        for px, (lo, hi) in buckets.items():
            parts.append(f'<path d="M{px} {y(lo):.2f} V{y(hi):.2f}" stroke="#215ca0" stroke-width="1.4" stroke-linecap="round"/>')
        if panel:
            for value, color in [(settings['threshold_s'], '#b22'), (settings['threshold_s']*settings['release_ratio'], '#888')]:
                parts.append(f'<path d="M65 {y(value)} H1075" stroke="{color}" stroke-dasharray="4 3"/>')
        for e in episodes:
            if begin <= e['start_from_drive_s'] <= end:
                parts.append(f'<path d="M{x(e["start_from_drive_s"])} {top} v120" stroke="#b22" opacity=".6"/>')
        if onset is not None and begin <= onset <= end:
            parts.append(f'<path d="M{x(onset)} {top} v120" stroke="#19834c" stroke-dasharray="5 4"/>')
        for t in np.linspace(begin, end, 6):
            parts.append(f'<text x="{x(t)}" y="{top+138}" font-size="12">{t:.2f}</text>')
    parts += ['<text x="65" y="421" font-family="sans-serif">Time from drive command [s]. Red: candidate; green: RGB onset (report only).</text>',
              '<text x="65" y="446" font-family="sans-serif">All saved intervals processed; no onset, position or phase gating. Candidates are not object matches.</text>', '</svg>']
    path.write_text('\n'.join(parts))


def load_scene(tile_dir, session, run):
    folder = tile_dir/session
    saved = json.loads((folder/'result.json').read_text())
    source_path = Path(run['source_config_path'])
    alignment, hashes = check_scene(Path(run['motion_dir'])/session, source_path.parent)
    if hashes != saved['input_hashes'] or alignment != saved['alignment']:
        raise ValueError('alignment/source changed since tile extraction; regenerate tiles')
    meta = json.loads((folder/'tiles.json').read_text())
    params, config = run['source_config']['parameters'], run['source_config']['common']
    if (meta['roi'] != config['roi'] or meta['output_size'] != config['spatial']['output_size']
            or not math.isclose(meta['step_s'], params['step_ms']/1000, rel_tol=0, abs_tol=1e-12)
            or not math.isclose(meta['event_window_s'], params['step_ms']*params['window_bins']/1000, rel_tol=0, abs_tol=1e-12)):
        raise ValueError('tile metadata differs from source extraction settings')
    result = saved['source_result']
    if (result['session'] != session or result['annotation_sha256'] != hashes['annotation_sha256']
            or result['time_sync_sha256'] != hashes['time_sync_sha256']
            or result['valid_pixels'] != sum(t['valid_pixels'] for t in meta['tiles'])):
        raise ValueError('tile source result mismatch')
    hashes.update(tile_metadata_sha256=digest(folder/'tiles.json'), tile_result_sha256=digest(folder/'result.json'))
    return alignment, meta, hashes


def write_csv(path, rows, fields):
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader(); writer.writerows(rows)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tile-dir', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--subset', choices=['development'], default='development')
    parser.add_argument('--split', type=Path, default=DEFAULT_SPLIT)
    parser.add_argument('--parameters', type=Path, help='Reuse exact saved parameters without overrides')
    for key, value in DEFAULTS.items():
        parser.add_argument('--'+key.replace('_', '-'), type=type(value))
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError('output already exists; choose a new directory')
        run = json.loads((args.tile_dir/'run_config.json').read_text())
        split = json.loads(args.split.read_text())
        sessions = select_sessions(split, 'development')
        if run['subset'] != 'development' or run['split'] != split or run['sessions'] != sessions:
            raise ValueError('tile cache does not match development split')
        status = json.loads((args.tile_dir/'summary.json').read_text())
        if any(sum(r['session'] == s and r['status'] == 'complete' for r in status) != 1 for s in sessions):
            raise ValueError('development tile extraction incomplete')
        if digest(run['source_config_path']) != run['source_config_sha256']:
            raise ValueError('original score config changed since extraction')
        source = json.loads(Path(run['source_config_path']).read_text())
        if source != run['source_config']:
            raise ValueError('cached source config mismatch')
        upstream = source['parameters']
        definition = dict(tile_px=run['tile_px'], roi=source['common']['roi'], spatial=source['common']['spatial'],
            **{k: upstream[k] for k in ('rgb_topic', 'rgb_pixel_delta', 'step_ms', 'window_bins')})
        code = {name: digest(Path(__file__).with_name(name)) for name in ('rc_popout_local_detection.py', 'rc_popout_change_detection.py')}
        contract = dict(algorithm=ALGORITHM, code_sha256=code, split_sha256=digest(args.split), input_definition=definition)
        overrides = {k: getattr(args, k) for k in DEFAULTS if getattr(args, k) is not None}
        if args.parameters:
            frozen = json.loads(args.parameters.read_text())
            if overrides or any(frozen.get(k) != value for k, value in contract.items()):
                raise ValueError('frozen parameters/code/split/input mismatch or attempted override')
            settings = frozen['settings']
        else:
            settings = dict(DEFAULTS, **overrides)
        validate_settings(settings)
        if settings['evs_max_gap_s'] < max(upstream['step_ms'], upstream['step_ms']*upstream['window_bins'])/1000:
            raise ValueError('EVS gap limit smaller than observation step/window')
        args.output.mkdir(parents=True)
        write_json(args.output/'detector_parameters.json', dict(**contract, settings=settings))
        write_json(args.output/'run_config.json', dict(**contract, settings=settings, sessions=sessions,
            subset='development', tile_dir=str(args.tile_dir.resolve()), tile_run_sha256=digest(args.tile_dir/'run_config.json'),
            numpy_version=np.__version__,
            note='Exploratory local activity candidates, not vehicle classification or measured online latency. No phase gating.'))
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.exit(1, f'error: {error}\n')
    summaries, candidates = [], []
    for session in sessions:
        print(f'[development] {session}', flush=True)
        try:
            alignment, meta, hashes = load_scene(args.tile_dir, session, run)
            out = args.output/session; out.mkdir()
            scene_results, scene_candidates = [], []
            for sensor in ('rgb', 'evs'):
                print(f'  {sensor}: processing all cached samples', flush=True)
                path = args.tile_dir/session/f'{sensor}_tiles.npz'
                with np.load(path, allow_pickle=False) as arrays:
                    data = {k: arrays[k] for k in arrays.files}
                hashes[f'{sensor}_tiles_sha256'] = digest(path)
                series, episodes, states = detect(data, meta, sensor, settings, run['tile_px'])
                if not any(r['ready'] for r in series):
                    raise ValueError(f'{sensor}: no warmed-up samples')
                series, episodes = attach_reporting(series, episodes, alignment)
                onset = alignment.get('rgb_first_visible_from_drive_s')
                result = summarize(session, sensor, series, episodes, onset, settings['onset_guard_s'])
                result['drive_candidates'] = result['phase_counts'].get('drive', {}).get('alarms', 0)
                scene_results.append(result)
                for number, e in enumerate(episodes, 1):
                    a, b = e['pair_tile_ids_at_start']
                    scene_candidates.append(dict(session=session, method=sensor, candidate=number,
                        start_from_drive_s=e['start_from_drive_s'], end_from_drive_s=e['end_from_drive_s'],
                        duration_s=e['end_from_drive_s']-e['start_from_drive_s'], start_phase=e['start_phase'],
                        end_reason=e['end_reason'], pair_tile_a=a, pair_tile_b=b,
                        rgb_first_visible_from_drive_s=onset, start_minus_onset_s=None if onset is None else e['start_from_drive_s']-onset))
                write_csv(out/f'{sensor}_local_scores.csv', series, list(series[0]))
                np.savez_compressed(out/f'{sensor}_local_state.npz', **states)
                write_json(out/f'{sensor}_candidates.json', episodes)
                plot(out/f'{sensor}_local_scores.svg', series, episodes, onset, settings, f'{session} / {sensor}')
                phase_end = max((p['end_from_drive_s'] for p in alignment.get('phases', [])), default=3.)
                zoom = [r for r in series if -.5 <= r['from_drive_s'] <= phase_end+.2]
                if len(zoom) >= 2:
                    plot(out/f'{sensor}_drive_zoom.svg', zoom, episodes, onset, settings, f'{session} / {sensor} / drive zoom')
                print(f'  {sensor}: candidates={result["candidates"]}, drive={result["drive_candidates"]}, first={result["first_candidate_from_drive_s"]}', flush=True)
            write_json(out/'result.json', dict(alignment=alignment, input_hashes=hashes, results=scene_results))
            summaries.extend(scene_results); candidates.extend(scene_candidates)
        except (ValueError, OSError, KeyError, TypeError) as error:
            summaries.append(dict(session=session, method='', status='failed', error=str(error)))
            print(f'  FAILED: {error}', flush=True)
    fields = ['session', 'method', 'status', 'error', 'samples', 'ready_samples', 'ready_observed_seconds',
              'candidates', 'drive_candidates', 'first_candidate_from_drive_s', 'rgb_first_visible_from_drive_s',
              'first_candidate_minus_onset_s', 'candidates_before_guard', 'candidates_near_onset', 'candidates_after_guard',
              'first_candidate_end_from_drive_s', 'active_candidate_at_onset']
    write_csv(args.output/'summary.csv', summaries, fields)
    write_json(args.output/'summary.json', summaries)
    write_csv(args.output/'candidates.csv', candidates, ['session', 'method', 'candidate', 'start_from_drive_s', 'end_from_drive_s',
        'duration_s', 'start_phase', 'end_reason', 'pair_tile_a', 'pair_tile_b', 'rgb_first_visible_from_drive_s', 'start_minus_onset_s'])
    links = []
    for r in summaries:
        if r['status'] == 'complete':
            stem = f'{r["session"]}/{r["method"]}'
            links.append(f'<li><a href="{html.escape(stem, quote=True)}_local_scores.svg">{html.escape(stem)}</a>: {r["candidates"]} candidates'
                         + (f' / <a href="{html.escape(stem, quote=True)}_drive_zoom.svg">走行付近</a>' if (args.output/f'{stem}_drive_zoom.svg').exists() else '') + '</li>')
        else:
            links.append(f'<li>FAILED {html.escape(r["session"])}: {html.escape(r["error"])}</li>')
    (args.output/'index.html').write_text('<!doctype html><meta charset="utf-8"><h1>調整用・局所活動変化の候補</h1>'
        '<p>RGB初出現・走行phase・方向は判定に未使用。全保存時間・全タイルを処理。候補数は対象の検知成功数ではありません。</p>'
        '<p>初期設定は未調整。赤線＝候補開始、緑線＝RGB初出現（表示のみ）。</p><ul>'+''.join(links)+'</ul>')
    print(f'Summary: {args.output/"summary.csv"}', flush=True)
    return int(any(r['status'] == 'failed' for r in summaries))


if __name__ == '__main__':
    raise SystemExit(main())
