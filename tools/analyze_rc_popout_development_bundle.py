"""Inspect an archived development-only tile bundle; no ROS or remote access.

This validates archived hashes, not the current RAW/sync/annotation files.
Onset labels define diagnostic reporting windows only. Nothing here freezes a
detector or opens evaluation recordings.
"""
import argparse
import csv
import hashlib
import io
import json
import math
from pathlib import Path
from zipfile import ZipFile

import numpy as np

from rc_popout_change_detection import DEFAULT_SPLIT, digest, select_sessions, write_json
from rc_popout_local_detection import (ALGORITHM, attach_reporting, detect,
                                      geometry, validate_data, write_csv)


def load_bundle(path):
    split = json.loads(DEFAULT_SPLIT.read_text())
    sessions = select_sessions(split, 'development')
    expected = {'tile_dev_trial01/run_config.json'}
    expected.update('local_dev_trial01/'+n for n in (
        'run_config.json', 'detector_parameters.json', 'summary.csv', 'candidates.csv'))
    expected.update('local_review_trial01/'+n for n in ('summary.csv', 'review_errors.json'))
    for session in sessions:
        expected.update(f'tile_dev_trial01/{session}/{n}' for n in (
            'rgb_tiles.npz', 'evs_tiles.npz', 'tiles.json', 'result.json'))
        expected.add(f'local_dev_trial01/{session}/result.json')
    with ZipFile(path) as archive:
        if len(archive.namelist()) != len(expected) or set(archive.namelist()) != expected:
            raise ValueError('bundle must contain exactly the expected development files')
        if sum(i.file_size for i in archive.infolist()) > 128*1024**2:
            raise ValueError('unexpectedly large development bundle')
        raw = {n: archive.read(n) for n in expected}
    js = lambda n: json.loads(raw[n])
    hashes = {n: hashlib.sha256(v).hexdigest() for n, v in raw.items()}
    run = js('local_dev_trial01/run_config.json')
    tile_run = js('tile_dev_trial01/run_config.json')
    frozen = js('local_dev_trial01/detector_parameters.json')
    if (tile_run['split'] != split or tile_run['sessions'] != sessions
            or run['sessions'] != sessions or tile_run['subset'] != 'development'
            or run['subset'] != 'development' or frozen['split_sha256'] != digest(DEFAULT_SPLIT)
            or run['tile_run_sha256'] != hashes['tile_dev_trial01/run_config.json']
            or frozen['algorithm'] != ALGORITHM):
        raise ValueError('development split/algorithm/config hash mismatch')
    for key, value in frozen.items():
        if run.get(key) != value:
            raise ValueError(f'run/parameter mismatch: {key}')
    for name, value in frozen['code_sha256'].items():
        if name not in ('rc_popout_local_detection.py', 'rc_popout_change_detection.py'):
            raise ValueError('unexpected detector code')
        if digest(Path(__file__).with_name(name)) != value:
            raise ValueError(f'local detector code differs from archived run: {name}')
    source = tile_run['source_config']
    definition = dict(tile_px=tile_run['tile_px'], roi=source['common']['roi'],
                      spatial=source['common']['spatial'], **{k: source['parameters'][k]
                      for k in ('rgb_topic', 'rgb_pixel_delta', 'step_ms', 'window_bins')})
    if definition != frozen['input_definition']:
        raise ValueError('tile input definition differs from detector')
    scenes = {}
    for session in sessions:
        prefix = f'tile_dev_trial01/{session}/'
        saved = js(prefix+'result.json')
        local = js(f'local_dev_trial01/{session}/result.json')
        meta = js(prefix+'tiles.json')
        if saved['alignment'] != local['alignment'] or saved['alignment']['session'] != session:
            raise ValueError('archived alignment mismatch')
        source_result = saved['source_result']
        if (source_result['session'] != session
                or source_result['annotation_sha256'] != saved['input_hashes']['annotation_sha256']
                or source_result['time_sync_sha256'] != saved['input_hashes']['time_sync_sha256']
                or source_result['camchain_sha256'] != definition['spatial']['camchain_sha256']
                or source_result['valid_pixels'] != sum(t['valid_pixels'] for t in meta['tiles'])):
            raise ValueError('archived source result mismatch')
        for k, v in saved['input_hashes'].items():
            if local['input_hashes'].get(k) != v:
                raise ValueError('archived source hash metadata mismatch')
        for name, key in [('tiles.json', 'tile_metadata_sha256'), ('result.json', 'tile_result_sha256'),
                          ('rgb_tiles.npz', 'rgb_tiles_sha256'), ('evs_tiles.npz', 'evs_tiles_sha256')]:
            if hashes[prefix+name] != local['input_hashes'][key]:
                raise ValueError(f'archived input hash mismatch: {session}/{name}')
        if (meta['roi'] != definition['roi'] or meta['output_size'] != definition['spatial']['output_size']
                or not np.isclose(meta['step_s'], definition['step_ms']/1000)
                or not np.isclose(meta['event_window_s'], definition['step_ms']*definition['window_bins']/1000)):
            raise ValueError('geometry/window mismatch')
        data = {}
        for sensor in ('rgb', 'evs'):
            with np.load(io.BytesIO(raw[prefix+sensor+'_tiles.npz']), allow_pickle=False) as arrays:
                data[sensor] = {k: arrays[k] for k in arrays.files}
            validate_data(data[sensor], meta, sensor)
        scenes[session] = dict(meta=meta, alignment=saved['alignment'], data=data)
    candidates = list(csv.DictReader(io.StringIO(raw['local_dev_trial01/candidates.csv'].decode())))
    return scenes, frozen, hashes, candidates


def reproduce(scenes, frozen, candidates, output):
    results = {}
    for session, scene in scenes.items():
        folder = output/session
        folder.mkdir()
        meta, alignment = scene['meta'], scene['alignment']
        pairs, _ = geometry(meta, frozen['input_definition']['tile_px'])
        for sensor, data in scene['data'].items():
            print(f'{session}/{sensor}: reproducing archived detector', flush=True)
            series, episodes, state = detect(data, meta, sensor, frozen['settings'], frozen['input_definition']['tile_px'])
            series, episodes = attach_reporting(series, episodes, alignment)
            saved = [r for r in candidates if r['session'] == session and r['method'] == sensor]
            if len(saved) != len(episodes):
                raise ValueError('candidate count differs from archived run')
            for old, new in zip(saved, episodes):
                for key in ('start_from_drive_s', 'end_from_drive_s'):
                    if not np.isclose(float(old[key]), new[key], rtol=0, atol=1e-9):
                        raise ValueError('candidate time differs from archived run')
                if [int(old['pair_tile_a']), int(old['pair_tile_b'])] != new['pair_tile_ids_at_start']:
                    raise ValueError('candidate location differs from archived run')
            cache = dict(**state, ready=np.array([r['ready'] for r in series]),
                         reset=np.array([r['state_reset'] for r in series]),
                         score=np.array([r['pair_score_s'] for r in series]),
                         active=np.array([r['active'] for r in series]))
            np.savez_compressed(folder/f'{sensor}_reproduced.npz', **cache)
            write_csv(folder/f'{sensor}_scores.csv', series, list(series[0]))
            write_json(folder/f'{sensor}_candidates.json', episodes)
            results[session, sensor] = dict(scene=scene, data=data, pairs=pairs, **cache)
    return results


def distribution_rows(results):
    rows = []
    for (session, sensor), r in results.items():
        alignment = r['scene']['alignment']
        zero = alignment['drive_start_s']
        t = r['time_s']-zero
        start = r['data']['support_start_s']-zero
        onset = alignment.get('rgb_first_visible_from_drive_s')
        end = next(p['end_from_drive_s'] for p in alignment['phases'] if p['phase'] == 'drive')
        windows = [('all', -np.inf, np.inf), ('drive', 0, end)]
        if onset is not None:
            windows += [('pre_onset', -np.inf, onset-.03), ('drive_pre_onset', 0, onset-.03),
                        ('onset_0_100ms', onset, onset+.1), ('onset_0_250ms', onset, onset+.25)]
        for name, lo, hi in windows:
            mask = r['ready'] & (t <= hi) & (start >= lo)
            values = r['score'][mask]
            if not len(values):
                continue
            top = np.flatnonzero(mask)[np.argmax(values)]
            pair_scores = np.minimum(r['cusum_s'][top,r['pairs'][:,0]], r['cusum_s'][top,r['pairs'][:,1]])
            a, b = r['pairs'][int(np.argmax(pair_scores))]
            rows.append(dict(session=session, sensor=sensor, window=name, samples=int(len(values)),
                p50=float(np.quantile(values,.5)), p95=float(np.quantile(values,.95)),
                max=float(values.max()), peak_from_drive_s=float(t[top]),
                tile_a=int(r['tile_id'][a]), tile_b=int(r['tile_id'][b])))
    return rows


def replay_threshold(score, ready, reset, threshold, release_ratio):
    """Causal scalar alarm replay; labels/drive times are not accepted."""
    active = False
    starts = []
    flags = np.zeros(len(score), dtype=bool)
    for i, value in enumerate(score):
        if reset[i] or not ready[i]:
            active = False
        if not ready[i]:
            continue
        if not active and value >= threshold:
            starts.append(i)
            active = True
        if value <= threshold*release_ratio:
            active = False
        flags[i] = active
    return starts, flags


def negative_thresholds(results, settings):
    calibration, rows = [], []
    for sensor in ('rgb', 'evs'):
        negative = results['t_0.2-none', sensor]
        peak = float(negative['score'][negative['ready']].max())
        # A diagnostic candidate on a 0.05 grid, strictly above the entire
        # development negative. This is calibration, not validation.
        step = settings['threshold_s']
        threshold = (math.floor(peak/step)+1)*step
        calibration.append(dict(sensor=sensor, control='t_0.2-none',
                                control_max=peak, threshold=threshold, step=step))
        for (session, method), r in results.items():
            if method != sensor:
                continue
            starts, _ = replay_threshold(r['score'], r['ready'], r['reset'], threshold, settings['release_ratio'])
            alignment = r['scene']['alignment']
            t = r['time_s']-alignment['drive_start_s']
            onset = alignment.get('rgb_first_visible_from_drive_s')
            ids = []
            for i in starts:
                p = r['pairs']
                best = np.argmax(np.minimum(r['cusum_s'][i,p[:,0]], r['cusum_s'][i,p[:,1]]))
                a, b = (r['scene']['meta']['tiles'][j] for j in p[best])
                ids.append(dict(time=float(t[i]), tile_a=a['tile_id'], tile_b=b['tile_id'],
                                ax=a['x'], ay=a['y'], bx=b['x'], by=b['y']))
            first = ids[0] if ids else {}
            rows.append(dict(session=session, sensor=sensor, threshold=threshold, candidates=len(starts),
                before_onset=None if onset is None else sum(t[i] < onset for i in starts),
                before_guard=None if onset is None else sum(t[i] < onset-settings['onset_guard_s'] for i in starts),
                first_from_drive_s=first.get('time'), onset_from_drive_s=onset,
                first_recording_relative_s=None if not ids else first['time']+alignment['drive_start_s'],
                first_minus_onset_ms=None if onset is None or not ids else 1000*(first['time']-onset),
                **{k: first.get(k) for k in ('tile_a', 'tile_b', 'ax', 'ay', 'bx', 'by')}))
    return calibration, rows


def smooth_activity(data, row_groups, settings, sensor, subtract_row=False):
    """20 ms causal low-pass of per-area activity; diagnostic ablation only."""
    times, support, interval = (data[k] for k in ('time_s', 'support_start_s', 'interval'))
    density = data['counts']/data['valid_pixels']
    if subtract_row:
        for group in row_groups:
            density[:, group] = np.maximum(0., density[:, group]-np.median(density[:, group], axis=1)[:, None])
    states = np.zeros_like(density)
    state = np.zeros(density.shape[1])
    limit = settings[f'{sensor}_max_gap_s']
    for i, t in enumerate(times):
        dt = 0. if i == 0 else t-times[i-1]
        if i == 0 or interval[i] != interval[i-1] or dt > limit+1e-9 or t-support[i] > limit+1e-9:
            state[:] = 0.
        else:
            retain = math.exp(-dt/.02)
            state = retain*state+(1-retain)*density[i]
        states[i] = state
    return states


def ablations(results, frozen):
    """Report all exploratory variants, including failures, on development only."""
    details = []
    def add(variant, session, sensor, r, score):
        alignment = r['scene']['alignment']
        t = r['time_s']-alignment['drive_start_s']
        onset = alignment.get('rgb_first_visible_from_drive_s')
        pre = r['ready'] if onset is None else r['ready'] & (t < onset-frozen['settings']['onset_guard_s'])
        post = None if onset is None else r['ready'] & (t >= onset) & (t <= onset+.25)
        details.append(dict(variant=variant, session=session, sensor=sensor,
            background_max=float(score[pre].max()), background_samples=int(pre.sum()),
            post250_max=None if post is None else float(score[post].max()),
            post250_samples=None if post is None else int(post.sum())))
    for (session, sensor), r in results.items():
        add('current', session, sensor, r, r['score'])
        meta = r['scene']['meta']
        _, groups = geometry(meta, frozen['input_definition']['tile_px'])
        for subtract in (False, True):
            states = smooth_activity(r['data'], groups, frozen['settings'], sensor, subtract)
            p = r['pairs']
            score = np.minimum(states[:,p[:,0]], states[:,p[:,1]]).max(axis=1)
            add('row_excess_ema20ms' if subtract else 'density_ema20ms', session, sensor, r, score)
        locations = {(tile['x'], tile['y']): i for i, tile in enumerate(meta['tiles'])}
        px = frozen['input_definition']['tile_px']
        for shape, offsets in [('horizontal_pair', [(0,0), (px,0)]),
                               ('square_2x2', [(0,0), (px,0), (0,px), (px,px)])]:
            groups = np.array([[locations[x+dx, y+dy] for dx, dy in offsets]
                               for x, y in locations if all((x+dx, y+dy) in locations for dx, dy in offsets)])
            score = r['cusum_s'][:,groups].min(axis=2).max(axis=1)
            add(shape, session, sensor, r, score)
        if sensor == 'rgb':
            for floor in (.05, .10, .20, .40):
                settings = dict(frozen['settings'], rgb_scale_floor=floor)
                series, _, _ = detect(r['data'], meta, sensor, settings, px)
                add(f'rgb_floor_{floor:.2f}', session, sensor, r, np.array([v['pair_score_s'] for v in series]))
    summaries = []
    for variant, sensor in sorted({(r['variant'], r['sensor']) for r in details}):
        group = [r for r in details if r['variant'] == variant and r['sensor'] == sensor]
        background = max(r['background_max'] for r in group)
        minimum_post = min(r['post250_max'] for r in group if r['post250_max'] is not None)
        summaries.append(dict(variant=variant, sensor=sensor, background_max=background,
                              weakest_post250_peak=minimum_post, ratio=minimum_post/background,
                              amplitude_separable_in_these_windows=minimum_post > background))
    return details, summaries


def make_figures(results, calibration, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    h = {r['sensor']: r['threshold'] for r in calibration}
    names = ['t_0.2-none', 'test_01', 'test_05', 'test_11', 'test_14']
    conditions = ['No popout', '20% / left', '20% / right', '100% / left', '100% / right']
    fig, axes = plt.subplots(5, 2, figsize=(12, 11), sharex=True, sharey='col', layout='constrained')
    for row, (session, condition) in enumerate(zip(names, conditions)):
        for col, sensor in enumerate(('rgb', 'evs')):
            r = results[session, sensor]
            a = r['scene']['alignment']
            t = r['time_s']-a['drive_start_s']
            ax = axes[row, col]
            mask = (t >= -.2) & (t <= 3.2)
            ax.plot(t[mask], np.where(r['ready'][mask], r['score'][mask], np.nan), lw=1.2, color='#155f91', label='Current score')
            ax.axhline(.05, color='#aaa', ls=':', label='Original threshold 0.05')
            ax.axhline(h[sensor], color='#bd3d38', ls='--', label=f'Negative-derived threshold {h[sensor]:.2f}')
            onset = a.get('rgb_first_visible_from_drive_s')
            if onset is not None:
                ax.axvline(onset, color='#19854f', ls='--', label='Annotated RGB onset')
            ax.set_title(f'{session} | {condition} | {sensor.upper()}', loc='left', fontsize=10)
            ax.set_xlim(-.2, 3.2)
            ax.set_ylim(0, .75 if sensor == 'rgb' else .6)
            ax.grid(alpha=.18)
            if col == 0:
                ax.set_ylabel('Pair score [s]')
            if row == 4:
                ax.set_xlabel('Time from drive command [s]')
    axes[0,0].legend(loc='upper left', fontsize=7)
    axes[1,1].legend(loc='upper left', fontsize=7)
    fig.suptitle('Development recordings: local activity scores\nThresholds calibrated on the single negative; no onset gating or evaluation data', fontsize=14)
    fig.savefig(output/'score_overview.png', dpi=150)
    fig.savefig(output/'score_overview.pdf')
    plt.close(fig)


def diagnostics(results, frozen, output, plots):
    calibration, thresholds = negative_thresholds(results, frozen['settings'])
    write_json(output/'diagnostic_thresholds.json', calibration)
    write_csv(output/'threshold_probe.csv', thresholds, list(thresholds[0]))
    details, summary = ablations(results, frozen)
    write_csv(output/'ablation_details.csv', details, list(details[0]))
    write_csv(output/'ablation_summary.csv', summary, list(summary[0]))
    # Exploratory horizontal-only aggregation of saved float32 states. This
    # replay is deliberately separated from the original float64 score probe.
    horizontal = {}
    for key, r in results.items():
        tiles = r['scene']['meta']['tiles']
        pairs = np.array([p for p in r['pairs'] if tiles[p[0]]['y'] == tiles[p[1]]['y']])
        score = np.minimum(r['cusum_s'][:,pairs[:,0]], r['cusum_s'][:,pairs[:,1]]).max(axis=1)
        horizontal[key] = dict(r, pairs=pairs, score=score)
    hcal, hrows = negative_thresholds(horizontal, frozen['settings'])
    write_json(output/'horizontal_diagnostic_thresholds.json', dict(calibration=hcal,
        note='Post-hoc spatial ablation using float32 saved states; diagnostic candidates only.'))
    write_csv(output/'horizontal_threshold_probe.csv', hrows, list(hrows[0]))
    if plots:
        make_figures(results, calibration, output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--plots', action='store_true', help='Requires Matplotlib; saves PNG and PDF')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output exists; use a new directory')
    scenes, frozen, hashes, candidates = load_bundle(args.bundle)
    args.output.mkdir(parents=True)
    write_json(args.output/'provenance.json', dict(bundle=str(args.bundle.resolve()),
        bundle_sha256=digest(args.bundle), archived_file_sha256=hashes, parameters=frozen,
        analysis_code_sha256=digest(__file__), numpy_version=np.__version__,
        note='Archived development data only. Current RAW/sync/annotations unavailable; labels for reporting only.'))
    results = reproduce(scenes, frozen, candidates, args.output)
    rows = distribution_rows(results)
    write_csv(args.output/'score_distribution.csv', rows, list(rows[0]))
    diagnostics(results, frozen, args.output, args.plots)
    write_json(args.output/'completion.json', dict(status='complete', reproduced_cases=10,
        plots=args.plots, note='Exploratory development diagnostics, not a frozen or validated detector.'))
    print('Exact archived candidate counts/times/locations reproduced for all 10 cases.', flush=True)
    for r in rows:
        if r['window'] in ('drive', 'drive_pre_onset', 'onset_0_100ms', 'onset_0_250ms'):
            print(f"{r['session']:12} {r['sensor']:3} {r['window']:20} n={r['samples']:4} max={r['max']:.6f} p95={r['p95']:.6f}")


if __name__ == '__main__':
    main()
