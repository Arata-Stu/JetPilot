"""Development-only background-motion compensation from calibrated RGB and RAW.

Rereads source data through the existing calibrated extractor. EVS images use
past-only timestamp bins, built on disk so out-of-order batches are preserved.
Neither annotations nor the other sensor's motion estimate enter the detector.
"""
import argparse
from collections import Counter
import csv
import html
import json
import math
from pathlib import Path
import shutil
import tempfile
from types import SimpleNamespace

import cv2
import numpy as np

from rc_popout_change_detection import DEFAULT_SPLIT, check_scene, digest, score_config_path, select_sessions, write_json
from rc_popout_detection import analyze, evaluation_intervals
from rc_popout_flow import ALGORITHM, DEFAULTS, INTEGERS, FlowDetector, make_geometry, validate


class EventImages:
    """Timestamp histogram with bounded disk use. Read only after RAW EOF.

    Sample at t includes [t-window, t), regardless of input batch ordering.
    Empty windows produce empty images; poor motion observability remains unknown.
    """
    def __init__(self, directory, mask, spans, p, max_cache_mb):
        self.p, self.spans = p, spans
        self.step = p['event_step_ms']/1000
        self.window = round(p['event_window_ms']/p['event_step_ms'])
        self.geo = make_geometry(mask, p)
        self.shape = self.geo['mask'].shape
        self.lengths = [int(math.floor((b-a)/self.step)) for a, b in spans]
        if any(n < self.window for n in self.lengths):
            raise ValueError('evaluation interval too short for event image window')
        self.bytes = sum(self.lengths)*int(np.prod(self.shape))*4
        if self.bytes > max_cache_mb*1024**2:
            raise ValueError(f'event frame scratch needs {self.bytes/1024**2:.1f} MiB; '
                             f'limit={max_cache_mb} MiB (--max-cache-mb)')
        if self.bytes + 64*1024**2 > shutil.disk_usage(directory).free:
            raise ValueError('not enough free disk for event frame scratch')
        self.maps = []
        for i, n in enumerate(self.lengths):
            self.maps.append(np.memmap(Path(directory)/f'events_{i}.bin', dtype=np.uint32,
                                      mode='w+', shape=(n, *self.shape)))

    def add(self, interval, times, x, y):
        if len(times) == 0:
            return
        d = self.p['downsample']
        ix, iy = np.asarray(x)//d, np.asarray(y)//d
        bins = np.floor((np.asarray(times)-self.spans[interval][0])/self.step).astype(np.int64)
        valid = (bins >= 0) & (bins < self.lengths[interval]) & self.geo['mask'][iy, ix]
        keys = (bins[valid]*self.shape[0]+iy[valid])*self.shape[1]+ix[valid]
        keys, counts = np.unique(keys, return_counts=True)
        flat = self.maps[interval].reshape(-1)
        values = flat[keys].astype(np.uint64) + counts.astype(np.uint64)
        if (values > np.iinfo(np.uint32).max).any():
            raise ValueError('event pixel count exceeds uint32')
        flat[keys] = values.astype(np.uint32)

    def frames(self):
        for i, ((a, b), hist) in enumerate(zip(self.spans, self.maps)):
            running = np.zeros(self.shape, np.int64)
            for k in range(len(hist)):
                running += hist[k]
                if k >= self.window:
                    running -= hist[k-self.window]
                if k+1 < self.window:
                    continue
                t = a+(k+1)*self.step
                if t >= b:
                    continue
                gray = np.rint(np.minimum(running, self.p['event_count_clip']) *
                               (255/self.p['event_count_clip'])).astype(np.uint8)
                gray = cv2.GaussianBlur(gray, (3, 3), .7)
                gray[~self.geo['mask']] = 0
                yield i, t, t-self.window*self.step, gray

    def close(self):
        for array in self.maps:
            array.flush()
            array._mmap.close()
        self.maps.clear()


def save_snapshot(path, gray, geo, row, detail, p):
    """Native sensor representation; green=background, red=motion residual."""
    d = p['downsample']
    canvas = cv2.resize(gray, tuple(geo['original_size']), interpolation=cv2.INTER_NEAREST)
    canvas = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)
    if row['ready']:
        for q, prediction, outlier, inlier in zip(detail['points'], detail['prediction'], detail['outlier'], detail['inlier']):
            dest, origin = tuple(np.rint(q*d).astype(int)), tuple(np.rint(prediction*d).astype(int))
            color = (0, 0, 255) if outlier else (0, 180, 0) if inlier else (150,150,150)
            cv2.circle(canvas, dest, 2, color, -1)
            if outlier:
                cv2.line(canvas, origin, dest, color, 1)
        # Draw all currently persistent pairs, not just the largest raw score.
        for pair in detail['qualified_pairs']:
            for index in geo['pairs'][pair]:
                tile = geo['tiles'][index]
                x, y, w, h = (tile[k] for k in ('x', 'y', 'width', 'height'))
                cv2.rectangle(canvas, (x, y), (x+w-1, y+h-1), (0, 200, 255), 1)
    canvas = cv2.copyMakeBorder(canvas, 0, 84, 0, 0, cv2.BORDER_CONSTANT)
    score = 'unknown' if row['score'] is None else f'{row["score"]:.4g}'
    text = [f't={row["time_s"]:.6f}s  {row["reason"]}  tracks={row["tracks"]} inliers={row["inliers"]}',
            f'score={score} active_pairs={row["active_pairs"]}',
            'green=background red=residual orange=candidate']
    for j, line in enumerate(text):
        width = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 1., 1)[0][0]
        scale = min(.4, (canvas.shape[1]-12)/max(1, width))
        cv2.putText(canvas, line, (6, canvas.shape[0]-62+24*j), cv2.FONT_HERSHEY_SIMPLEX, scale, (255,255,255), 1)
    if not cv2.imwrite(str(path), canvas):
        raise OSError(f'failed to save {path}')


class FlowSink:
    def __init__(self, output, scratch, p, max_cache_mb):
        self.output, self.scratch, self.p, self.max_cache_mb = output, scratch, p, max_cache_mb
        self.cache = None
        self.rows = {s: [] for s in ('rgb', 'evs')}
        self.tiles = {s: [] for s in self.rows}
        self.pair_starts = {s: [] for s in self.rows}
        self.global_starts = {s: [] for s in self.rows}
        self.snapshots = {s: {} for s in self.rows}
        self.best = {s: -1. for s in self.rows}

    def initialize(self, mask, spans):
        self.geo = make_geometry(mask, self.p)
        self.spans = spans
        self.cache = EventImages(self.scratch, mask, spans, self.p, self.max_cache_mb)
        self.detectors = {s: FlowDetector(self.geo, self.p) for s in self.rows}
        write_json(self.output/'geometry.json', dict(output_size=self.geo['original_size'],
            tiles=self.geo['tiles'], adjacent_pairs=self.geo['ids'][self.geo['pairs']].tolist(),
            downsample=self.p['downsample'], original_roi_pixels=int(mask.sum()),
            used_roi_pixels=int(self.geo['mask'].sum())*self.p['downsample']**2,
            tracking_roi_pixels=int(self.geo['tracking_mask'].sum())*self.p['downsample']**2,
            event_scratch_bytes=self.cache.bytes,
            tracking_margin_original_px=(max(5, int(self.p['lk_window_px']/self.p['downsample']) | 1)//2)*self.p['downsample'],
            affine_matrix_coordinates='downsampled image pixels; tile IDs and residual thresholds use original EVS coordinates',
            mask_policy='Only downsample blocks wholly inside the original common ROI are used.'))
        print(f'  scratch={self.cache.bytes/1024**2:.1f} MiB; RGB flow...', flush=True)

    def process(self, sensor, interval, t, gray, support):
        row, detail = self.detectors[sensor].update(interval, t, gray, support)
        self.rows[sensor].append(row)
        n = len(self.geo['ids'])
        self.tiles[sensor].append((detail.get('tile_tracks', np.zeros(n, np.int32)),
                                   detail.get('tile_outliers', np.zeros(n, np.int32))))
        for pair in detail['started_pairs']:
            a, b = self.geo['ids'][self.geo['pairs'][pair]]
            self.pair_starts[sensor].append(dict(interval=interval, time_s=t, tile_a=int(a), tile_b=int(b)))
        if row['alarm']:
            self.global_starts[sensor].append(dict(interval=interval, time_s=t,
                pairs=self.geo['ids'][self.geo['pairs'][detail['qualified_pairs']]].tolist()))
        tags = []
        if len(self.rows[sensor]) == 1:
            tags.append('first_frame')
        if row['ready'] and row['score'] > self.best[sensor]:
            self.best[sensor] = row['score']
            tags.append('peak_score')
        if row['alarm'] and 'first_candidate' not in self.snapshots[sensor]:
            tags.append('first_candidate')
        for tag in tags:
            name = f'{sensor}_{tag}.png'
            save_snapshot(self.output/name, gray, self.geo, row, detail, self.p)
            self.snapshots[sensor][tag] = dict(file=name, time_s=t, score=row['score'], reason=row['reason'])

    def rgb_frame(self, interval, t, gray):
        size = (self.geo['mask'].shape[1], self.geo['mask'].shape[0])
        small = cv2.resize(gray, size, interpolation=cv2.INTER_AREA)
        small = cv2.GaussianBlur(small, (3, 3), .7)
        small[~self.geo['mask']] = 0
        self.process('rgb', interval, t, small, t)

    def events(self, interval, times, x, y):
        self.cache.add(interval, times, x, y)

    def finish_events(self):
        print('  RAW loaded; EVS flow...', flush=True)
        for interval, t, support, image in self.cache.frames():
            self.process('evs', interval, t, image, support)

    def close(self):
        if self.cache is not None:
            self.cache.close()


def series_plot(path, rows, zero, onset):
    first, last = rows[0]['time_s'], rows[-1]['time_s']
    top = max(1.2, max((r['score'] or 0 for r in rows))*1.05)
    def x(t): return 60+880*(t-first)/max(1e-6, last-first)
    def y(v): return 200-160*v/top
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="270">',
           '<rect width="1000" height="270" fill="white"/>',
           '<text x="60" y="20">Pair strength (before persistence). Red=alarm; gray=unobservable.</text>']
    # Per-pixel min/max envelopes, including uncertainty, retain short peaks.
    buckets = {}
    for r in rows:
        bucket = buckets.setdefault(int(x(r['time_s'])), [])
        bucket.append(r)
    for px, group in buckets.items():
        if any(not r['ready'] for r in group):
            svg.append(f'<path d="M{px},40 V200" stroke="#ddd"/>')
        values = [r['score'] for r in group if r['ready']]
        if values:
            svg.append(f'<path d="M{px},{y(min(values)):.2f} V{y(max(values)):.2f}" stroke="#1464a0" stroke-width="2" stroke-linecap="round"/>')
    svg.append(f'<path d="M60,{y(1):.2f} H940" stroke="#b44" stroke-dasharray="4 4"/>')
    for r in rows:
        if r['alarm']:
            svg.append(f'<path d="M{x(r["time_s"]):.2f},40 V200" stroke="#d22"/>')
    if onset is not None:
        svg.append(f'<path d="M{x(onset+zero):.2f},40 V200" stroke="#19834c" stroke-width="2"/>')
    for i in range(6):
        t = first+(last-first)*i/5
        svg.append(f'<text x="{x(t):.2f}" y="223">{t-zero:.2f}</text>')
    svg += ['<text x="60" y="252">Time from drive command [s]; green=RGB annotation (report only).</text>', '</svg>']
    path.write_text('\n'.join(svg))


def phase_summaries(rows, phases):
    """Coverage is reported against command phases, never used as a gate."""
    result = []
    for phase in phases:
        spans = phase['intervals']
        duration = sum(b-a for a,b in spans)
        ready = active = 0.
        for r in rows:
            # Conservative coverage: both endpoints must have a valid estimate.
            dt = r['ready_observed_step_s']
            overlap = sum(max(0., min(r['time_s'],b)-max(r['time_s']-dt,a)) for a,b in spans)
            ready += overlap
            active += overlap if r['active'] else 0.
        alarms = sum(r['alarm'] and any(a <= r['time_s'] < b for a,b in spans) for r in rows)
        result.append(dict(phase=phase['phase'], evaluation_seconds=duration, ready_observed_seconds=ready,
            unobservable_seconds=max(0.,duration-ready), ready_fraction=ready/duration if duration else None,
            active_seconds=active, candidates=alarms))
    return result


def save_sensor(sink, sensor, session, alignment):
    rows = sink.rows[sensor]
    if not rows:
        raise ValueError(f'no {sensor} image samples')
    zero = alignment['drive_start_s']
    onset = alignment.get('rgb_first_visible_from_drive_s')
    for row in rows:
        row['from_drive_s'] = row['time_s']-zero
    with (sink.output/f'{sensor}_flow_scores.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    starts = [dict(r, from_drive_s=r['time_s']-zero) for r in sink.global_starts[sensor]]
    write_json(sink.output/f'{sensor}_candidates.json', starts)
    write_json(sink.output/f'{sensor}_pair_starts.json', sink.pair_starts[sensor])
    phases = phase_summaries(rows, alignment.get('phases', []))
    write_json(sink.output/f'{sensor}_phases.json', phases)
    drive = next((r for r in phases if r['phase']=='drive'), {})
    tracks, outliers = zip(*sink.tiles[sensor])
    np.savez_compressed(sink.output/f'{sensor}_flow_tiles.npz', time_s=[r['time_s'] for r in rows],
        interval=[r['interval'] for r in rows], ready=[r['ready'] for r in rows],
        tile_id=sink.geo['ids'], tracks=np.asarray(tracks, np.int32), outliers=np.asarray(outliers, np.int32))
    series_plot(sink.output/f'{sensor}_scores.svg', rows, zero, onset)
    duration = sum(b-a for a, b in sink.spans)
    ready = sum(r['ready_observed_step_s'] for r in rows)
    first = starts[0]['from_drive_s'] if starts else None
    return dict(session=session, method=sensor, status='complete' if any(r['ready'] for r in rows) else 'unobservable',
        error='', samples=len(rows), ready_samples=sum(r['ready'] for r in rows),
        evaluation_seconds=duration, ready_observed_seconds=ready,
        unobservable_seconds=max(0., duration-ready), ready_fraction=ready/duration,
        drive_seconds=drive.get('evaluation_seconds'), drive_ready_seconds=drive.get('ready_observed_seconds'),
        drive_ready_fraction=drive.get('ready_fraction'), drive_active_seconds=drive.get('active_seconds'),
        candidates=len(starts), independent_pair_starts=len(sink.pair_starts[sensor]),
        first_candidate_from_drive_s=first, rgb_first_visible_from_drive_s=onset,
        first_candidate_minus_onset_ms=None if first is None or onset is None else 1000*(first-onset),
        candidates_before_guard=None if onset is None else sum(s['from_drive_s'] < onset-.030 for s in starts),
        candidates_near_onset=None if onset is None else sum(onset-.030 <= s['from_drive_s'] <= onset+.250 for s in starts))


def run_scene(config, entry, extraction_args, alignment, hashes, output, p, max_cache_mb):
    output.mkdir()
    with tempfile.TemporaryDirectory(prefix='.event_frames_', dir=output) as scratch:
        sink = FlowSink(output, scratch, p, max_cache_mb)
        try:
            result, _, _ = analyze(config, entry, extraction_args, frame_sink=sink)
            if any(result[k] != hashes[k] for k in ('annotation_sha256', 'time_sync_sha256')):
                raise ValueError('source changed between preflight and extraction')
            raw = Path(hashes['raw_identity']['path'])
            if (raw.stat().st_size != hashes['raw_identity']['size'] or raw.stat().st_mtime_ns != hashes['raw_identity']['mtime_ns']
                    or digest(str(raw)+'.metadata.yaml') != hashes['raw_sidecar_sha256']):
                raise ValueError('RAW or sidecar changed during extraction')
            onset = alignment.get('rgb_first_visible_from_drive_s')
            actual = result['rgb_first_visible_s']
            if ((onset is None) != (actual is None) or actual is not None and
                    not math.isclose(actual-alignment['drive_start_s'], onset, abs_tol=1e-6, rel_tol=0)):
                raise ValueError('onset differs from saved alignment')
            sink.finish_events()
            summaries = [save_sensor(sink, s, entry['session'], alignment) for s in ('rgb', 'evs')]
            write_json(output/'result.json', dict(algorithm=ALGORITHM, settings=p, alignment=alignment,
                input_hashes=hashes, extraction=result, summaries=summaries,
                status_counts={s:dict(Counter(r['reason'] for r in sink.rows[s])) for s in ('rgb', 'evs')},
                snapshots=sink.snapshots,
                note='Exploratory motion residuals. Unknown intervals are not negative detections. '
                     'No full-depth model, object classification or measured online latency.'))
            return summaries
        finally:
            sink.close()


SUMMARY_FIELDS = ['session', 'method', 'status', 'error', 'samples', 'ready_samples', 'evaluation_seconds',
    'ready_observed_seconds', 'unobservable_seconds', 'ready_fraction', 'candidates', 'independent_pair_starts',
    'drive_seconds', 'drive_ready_seconds', 'drive_ready_fraction', 'drive_active_seconds',
    'first_candidate_from_drive_s', 'rgb_first_visible_from_drive_s', 'first_candidate_minus_onset_ms',
    'candidates_before_guard', 'candidates_near_onset']


def report(output, summaries):
    with (output/'summary.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=SUMMARY_FIELDS); writer.writeheader(); writer.writerows(summaries)
    write_json(output/'summary.json', summaries)
    sections = []
    for r in summaries:
        s, sensor = r['session'], r['method']
        folder = output/s
        links = []
        for name in (f'{sensor}_scores.svg', f'{sensor}_first_candidate.png', f'{sensor}_peak_score.png', f'{sensor}_first_frame.png'):
            if (folder/name).is_file():
                links.append(f'<a href="{html.escape(s)}/{name}">{name}</a>')
        sections.append(f'<h2>{html.escape(s)} / {sensor}</h2><p>{html.escape(str(r))}</p>'+ ' | '.join(links))
    (output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>Background motion residuals</title>'
        '<h1>背景移動補償・調整用</h1><p>候補は車両検知の確定ではありません。'
        'ready_fractionと推定失敗理由も確認してください。灰色は判定不能、緑線はRGB初出現注釈です。'
        '画像の緑点は背景モデルに沿う点、赤点は残差、橙枠は持続条件を満たした隣接タイルです。'
        'peak_score画像は全保存区間の最大スコア時刻であり、対象の出現時刻で選んでいません。</p>'+''.join(sections), encoding='utf-8')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--motion-dir', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--split', default=DEFAULT_SPLIT, type=Path)
    parser.add_argument('--subset', choices=['development'], default='development')
    parser.add_argument('--score-config', type=Path)
    parser.add_argument('--parameters', type=Path, help='Development settings JSON; not a frozen evaluation contract')
    parser.add_argument('--max-cache-mb', type=int, default=2048)
    parser.add_argument('--dry-run', action='store_true')
    for k in DEFAULTS:
        parser.add_argument('--'+k.replace('_', '-'), type=int if k in INTEGERS else float)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError('output already exists; choose a new directory')
        overrides = {k:getattr(args, k) for k in DEFAULTS if getattr(args, k) is not None}
        if args.parameters and overrides:
            raise ValueError('cannot combine --parameters and detector overrides')
        if args.parameters:
            saved = json.loads(args.parameters.read_text())
            if saved['algorithm'] != ALGORITHM:
                raise ValueError('parameter algorithm mismatch')
            p = saved['settings']
        else:
            p = dict(DEFAULTS, **overrides)
        validate(p)
        if args.max_cache_mb <= 0:
            raise ValueError('max-cache-mb must be positive')
        split = json.loads(args.split.read_text())
        sessions = select_sessions(split, 'development')
        source_path = score_config_path(args.motion_dir, args.score_config)
        source = json.loads(source_path.read_text())
        config = source['common']
        entries = {e['session']: e for e in config['sessions']}
        if len(entries) != len(config['sessions']) or any(s not in entries for s in sessions):
            raise ValueError('missing/duplicate sessions in source config')
        if digest(config['spatial']['camchain']) != config['spatial']['camchain_sha256']:
            raise ValueError('calibration changed after source configuration')
        plan = {}
        for s in sessions:
            alignment, hashes = check_scene(args.motion_dir/s, source_path.parent)
            if digest(entries[s]['annotation']) != hashes['annotation_sha256']:
                raise ValueError(f'{s}: config/analysis annotation mismatch')
            ann = json.loads(Path(entries[s]['annotation']).read_text())
            spans = evaluation_intervals(ann)
            w, h = config['spatial']['output_size']
            if w % p['downsample'] or h % p['downsample']:
                raise ValueError('output dimensions must divide by downsample')
            cache = sum(int(math.floor((b-a)/(p['event_step_ms']/1000))) for a,b in spans)*(w//p['downsample'])*(h//p['downsample'])*4
            if cache > args.max_cache_mb*1024**2:
                raise ValueError(f'{s}: scratch requires {cache/1024**2:.1f} MiB, above --max-cache-mb')
            session_dir = Path(entries[s]['annotation']).parents[3]/s
            raw = list(session_dir.glob('*.raw'))
            if len(raw) != 1 or not Path(str(raw[0])+'.metadata.yaml').is_file():
                raise ValueError(f'{s}: exactly one RAW with sidecar required')
            hashes['raw_sidecar_sha256'] = digest(str(raw[0])+'.metadata.yaml')
            hashes['raw_identity'] = dict(path=str(raw[0]), size=raw[0].stat().st_size, mtime_ns=raw[0].stat().st_mtime_ns,
                                         note='File identity only, not a RAW content hash')
            plan[s] = (alignment, hashes)
            print(f'[preflight] {s}: estimated scratch {cache/1024**2:.1f} MiB', flush=True)
        if args.dry_run:
            print('Preflight OK; no RAW decode or outputs. Runtime mask/coverage checks still run during extraction.')
            return 0
        args.output.mkdir(parents=True)
        cv2.setNumThreads(1)
        code = {name:digest(Path(__file__).with_name(name)) for name in
                ('analyze_rc_popout_flow.py', 'rc_popout_flow.py', 'rc_popout_detection.py', 'rc_popout_change_detection.py')}
        parameters = dict(schema_version=1, algorithm=ALGORITHM, settings=p, development_only=True,
                          code_sha256=code, split_sha256=digest(args.split))
        write_json(args.output/'detector_parameters.json', parameters)
        write_json(args.output/'run_config.json', dict(**parameters, subset='development', sessions=sessions,
            source_config_path=str(source_path), source_config_sha256=digest(source_path), source_config=source,
            motion_dir=str(args.motion_dir.resolve()), split=split, opencv_version=cv2.__version__, numpy_version=np.__version__,
            max_cache_mb=args.max_cache_mb, input_provenance={s:h for s, (_,h) in plan.items()},
            report_windows=dict(before_guard_s=.030, near_onset_after_s=.250, reporting_only=True),
            note='New image-flow prototype; EVS window/cadence differ from the existing activity detector. '
                 'Timestamp-causal offline processing; RAW file availability/processing latency is not modeled.'))
    except (ValueError, OSError, KeyError, TypeError) as exc:
        parser.exit(1, f'error: {exc}\n')
    summaries = []
    extraction_args = SimpleNamespace(**dict(source['parameters'], spatial=False))
    for s in sessions:
        print(f'[development] {s}: calibrated RGB / RAW motion compensation', flush=True)
        try:
            alignment, hashes = plan[s]
            rows = run_scene(config, entries[s], extraction_args, alignment, hashes, args.output/s, p, args.max_cache_mb)
            for r in rows:
                print(f'  {r["method"]}: ready={r["ready_fraction"]:.1%}, candidates={r["candidates"]}, '
                      f'first-onset={r["first_candidate_minus_onset_ms"]} ms', flush=True)
            summaries.extend(rows)
        except Exception as exc:
            print(f'  FAILED: {exc}', flush=True)
            summaries.extend(dict(session=s, method=sensor, status='error', error=str(exc)) for sensor in ('rgb','evs'))
        report(args.output, summaries)
    print(f'Report: {args.output}/index.html')
    return int(any(r['status'] != 'complete' for r in summaries))


if __name__ == '__main__':
    raise SystemExit(main())
