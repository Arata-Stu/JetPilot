"""Export all ROI tile activity for development-only spatial diagnostics.

Keeps the existing calibrated RGB/EVS score definitions, windows and sampling.
Onset-aligned panels are descriptive only; this tool does not detect objects.
"""
import argparse
import csv
import html
import json
import math
from pathlib import Path
from types import SimpleNamespace

from rc_popout_change_detection import (DEFAULT_SPLIT, check_scene, digest,
                                       score_config_path, select_sessions, write_json)
from rc_popout_detection import analyze


class TileSink:
    def __init__(self, tile_px, max_memory_mb=512):
        if not isinstance(tile_px, int) or tile_px < 1:
            raise ValueError('tile_px must be a positive integer')
        self.tile_px, self.max_memory_mb = tile_px, max_memory_mb

    def initialize(self, mask, spans, step, bins):
        import numpy as np
        self.mask, self.spans, self.step, self.bins = mask, spans, step, bins
        h, w = mask.shape
        self.size = [w, h]
        cols = (w + self.tile_px - 1) // self.tile_px
        yy, xx = np.indices(mask.shape)
        grid = (yy // self.tile_px) * cols + xx // self.tile_px
        self.ids, self.pixels = np.unique(grid[mask], return_counts=True)
        self.n = len(self.ids)
        if not self.n:
            raise ValueError('empty common ROI')
        if not math.isfinite(step) or step <= 0 or not isinstance(bins, int) or bins < 1:
            raise ValueError('invalid event step/window')
        self.tiles = [dict(tile_id=int(i), x=int(i % cols)*self.tile_px,
                           y=int(i // cols)*self.tile_px, width=min(self.tile_px, w-int(i % cols)*self.tile_px),
                           height=min(self.tile_px, h-int(i // cols)*self.tile_px), valid_pixels=int(n))
                      for i, n in zip(self.ids, self.pixels)]
        self.lookup = np.full(mask.shape, -1, dtype=np.int32)
        self.lookup[mask] = np.searchsorted(self.ids, grid[mask])
        sizes = [int(math.floor((b-a)/step)) for a, b in spans]
        # Histograms + prefix sum/window output + RGB arrays. Reject rather than silently shrink time/ROI.
        estimate = 4 * sum(sizes) * self.n * 8
        if estimate > self.max_memory_mb * 1024**2:
            raise ValueError(f'tile arrays need about {estimate/1024**2:.1f} MiB; '
                             f'limit={self.max_memory_mb} MiB (raise --max-memory-mb if appropriate)')
        self.hist = [np.zeros((length, self.n), dtype=np.int64) for length in sizes]
        self.rgb_rows, self.rgb_counts = [], []

    def rgb(self, interval, start, end, changed):
        import numpy as np
        self.rgb_rows.append((interval, start, end))
        self.rgb_counts.append(np.bincount(self.lookup[self.mask & changed], minlength=self.n))

    def events(self, interval, indices, xs, ys):
        import numpy as np
        tile = self.lookup[ys, xs]
        if np.any(tile < 0):
            raise ValueError('event outside common ROI reached tile exporter')
        # add.at preserves repeated events at one pixel/bin and out-of-order batches.
        np.add.at(self.hist[interval], (indices, tile), 1)

    def finish(self):
        import numpy as np
        rgb_rows = np.asarray(self.rgb_rows, dtype=float).reshape(-1, 3)
        result = {'rgb': dict(interval=rgb_rows[:, 0].astype(int), support_start_s=rgb_rows[:, 1],
                              time_s=rgb_rows[:, 2], counts=np.asarray(self.rgb_counts, dtype=np.int64).reshape(-1, self.n))}
        counts, times, intervals = [], [], []
        for i, ((a, b), hist) in enumerate(zip(self.spans, self.hist)):
            prefix = np.vstack([np.zeros((1, self.n), dtype=np.int64), np.cumsum(hist, axis=0)])
            ends = np.arange(self.bins, len(hist)+1)
            t = a + ends*self.step
            keep = t < b
            counts.append((prefix[self.bins:] - prefix[:-self.bins])[keep])
            times.append(t[keep]); intervals.append(np.full(int(keep.sum()), i, dtype=int))
            self.hist[i] = None
        ts = np.concatenate(times)
        result['evs'] = dict(interval=np.concatenate(intervals), time_s=ts,
                             support_start_s=ts-self.step*self.bins, counts=np.concatenate(counts))
        for data in result.values():
            data.update(tile_id=self.ids, valid_pixels=self.pixels)
        return result


def window_statistics(data, start, end):
    import numpy as np
    selected = (data['support_start_s'] >= start) & (data['time_s'] < end)
    n = int(selected.sum())
    if n == 0:
        raise ValueError(f'no complete observation windows in [{start:.6f}, {end:.6f})')
    density = data['counts'][selected] / data['valid_pixels'][None, :]
    return n, np.percentile(density, 95, axis=0)


def review_svg(path, tiles, size, panels, title):
    """Four maps per sensor with shared within-row color limits; no inference."""
    import numpy as np
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="1400" height="725" viewBox="0 0 1400 725">',
             '<rect width="1400" height="725" fill="white"/>',
             f'<text x="20" y="26" font-family="sans-serif" font-size="19">{html.escape(title)}</text>']
    scale = 320/size[0]
    for row, sensor in enumerate(('rgb', 'evs')):
        vectors = panels[sensor]
        vmax = max(float(np.max(v)) for _, _, v in vectors)
        y0 = 85 + row*310
        parts.append(f'<text x="20" y="{y0-32}" font-family="sans-serif" font-size="15">'
                     f'{sensor.upper()}: tile p95 density; shared color 0 .. {vmax:.6g}; '
                     'dark = low, yellow = high; gray = no valid ROI pixels</text>')
        for col, (label, n, values) in enumerate(vectors):
            x0 = 20 + col*345
            parts.append(f'<text x="{x0}" y="{y0-10}" font-family="sans-serif" font-size="14">'
                         f'{html.escape(label)} (n={n})</text>')
            parts.append(f'<rect x="{x0}" y="{y0}" width="320" height="{size[1]*scale}" fill="#ddd"/>')
            for tile, v in zip(tiles, values):
                level = float(v)/vmax if vmax > 0 else 0.
                # Linear, shared scale. Values/units differ between sensor rows.
                color = f'#{int(20+235*level):02x}{int(35+190*level):02x}{int(70-40*level):02x}'
                parts.append(f'<rect x="{x0+tile["x"]*scale}" y="{y0+tile["y"]*scale}" '
                             f'width="{tile["width"]*scale}" height="{tile["height"]*scale}" '
                             f'fill="{color}" stroke="white" stroke-width=".25">'
                             f'<title>tile {tile["tile_id"]}: {float(v):.6g}; valid={tile["valid_pixels"]}</title></rect>')
    parts.extend(['<text x="20" y="690" font-family="sans-serif">Before: onset -130..-30 ms; after: onset +30..+130 ms. Control: same elapsed command time.</text>',
                  '<text x="20" y="712" font-family="sans-serif">RGB: changed pixels / valid pixels. EVS: events / valid pixels per original time window. No detection claim.</text>', '</svg>'])
    path.write_text('\n'.join(parts))


def make_reviews(output, sessions, control, alignments):
    import numpy as np
    summary, errors = [], []
    for session in sessions:
        onset = alignments[session].get('rgb_first_visible_from_drive_s')
        if onset is None:
            continue
        panels = {}
        try:
            meta = json.loads((output/session/'tiles.json').read_text())
            negative_meta = json.loads((output/control/'tiles.json').read_text())
            if meta['tiles'] != negative_meta['tiles'] or meta['output_size'] != negative_meta['output_size']:
                raise ValueError('control/positive tile geometry differs')
            scene_summary = []
            for sensor in ('rgb', 'evs'):
                vectors = []
                for scene, name in [(session, 'popout'), (control, 'none')]:
                    with np.load(output/scene/f'{sensor}_tiles.npz', allow_pickle=False) as data:
                        zero = alignments[scene]['drive_start_s']
                        for label, a, b in [('before', onset-.13, onset-.03), ('after', onset+.03, onset+.13)]:
                            n, values = window_statistics(data, zero+a, zero+b)
                            vectors.append((f'{name} {label}', n, values))
                panels[sensor] = vectors
                peak = int(np.argmax(vectors[1][2]))
                tile = meta['tiles'][peak]
                scene_summary.append(dict(session=session, method=sensor, control=control,
                    peak_after_tile_id=tile['tile_id'], x=tile['x'], y=tile['y'], valid_pixels=tile['valid_pixels'],
                    before_p95=float(vectors[0][2][peak]), after_p95=float(vectors[1][2][peak]),
                    none_before_p95=float(vectors[2][2][peak]), none_after_p95=float(vectors[3][2][peak]),
                    before_samples=vectors[0][1], after_samples=vectors[1][1]))
            review_svg(output/session/'tile_review.svg', meta['tiles'], meta['output_size'], panels,
                       f'{session} vs {control} / descriptive tile activity')
            summary.extend(scene_summary)
        except (ValueError, OSError, KeyError) as error:
            errors.append(dict(session=session, stage='review', error=str(error)))
    fields = ['session', 'method', 'control', 'peak_after_tile_id', 'x', 'y', 'valid_pixels',
              'before_p95', 'after_p95', 'none_before_p95', 'none_after_p95', 'before_samples', 'after_samples']
    with (output/'tile_diagnostics.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); writer.writerows(summary)
    return errors


def main(argv=None):
    import numpy as np
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--motion-dir', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--split', default=DEFAULT_SPLIT, type=Path)
    parser.add_argument('--subset', choices=['development'], default='development')
    parser.add_argument('--score-config', type=Path)
    parser.add_argument('--tile-px', default=32, type=int)
    parser.add_argument('--max-memory-mb', default=512, type=int)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError('output already exists; choose a new directory')
        if args.tile_px < 1 or args.max_memory_mb < 1:
            raise ValueError('tile size and memory limit must be positive')
        split = json.loads(args.split.read_text()); sessions = select_sessions(split, 'development')
        source_path = score_config_path(args.motion_dir, args.score_config)
        source = json.loads(source_path.read_text()); config = source['common']
        entries = {e['session']: e for e in config['sessions']}
        if len(entries) != len(config['sessions']) or any(s not in entries for s in sessions):
            raise ValueError('duplicate or missing sessions in source config')
        controls = [s for g in split['groups'] if g['condition']=='none' for s in g['development']]
        if len(controls) != 1:
            raise ValueError('one development no-popout control required')
        detector_args = SimpleNamespace(**dict(source['parameters'], spatial=False))
        args.output.mkdir(parents=True)
        write_json(args.output/'run_config.json', dict(
            version=1, tile_px=args.tile_px, subset='development', sessions=sessions,
            motion_dir=str(args.motion_dir.resolve()), source_config_path=str(source_path),
            source_config=source, source_config_sha256=digest(source_path), split=split,
            exporter_sha256=digest(__file__), extraction_code_sha256=digest(Path(__file__).with_name('rc_popout_detection.py')),
            note='Descriptive spatial analysis. Same original RGB/EVS samples and windows; no detector tuning or evaluation data.'))
    except (ValueError, OSError, KeyError, TypeError) as error:
        parser.exit(1, f'error: {error}\n')
    status, alignments = [], {}
    for session in sessions:
        print(f'[development] {session}: extracting calibrated RGB and RAW event tiles', flush=True)
        try:
            alignment, hashes = check_scene(args.motion_dir/session, source_path.parent)
            sink = TileSink(args.tile_px, args.max_memory_mb)
            result, rgb, evs = analyze(config, entries[session], detector_args, tile_sink=sink)
            if (result['annotation_sha256'] != hashes['annotation_sha256']
                    or result['time_sync_sha256'] != hashes['time_sync_sha256']):
                raise ValueError('annotation/sync mismatch between alignment and source config')
            actual_onset = result['rgb_first_visible_s']
            aligned_onset = alignment.get('rgb_first_visible_from_drive_s')
            if ((actual_onset is None) != (aligned_onset is None)
                    or actual_onset is not None and not math.isclose(
                        actual_onset-alignment['drive_start_s'], aligned_onset, rel_tol=0, abs_tol=1e-6)):
                raise ValueError('RGB onset mismatch between alignment and extraction')
            arrays = sink.finish()
            # Conservation checks: spatial extraction must not change old whole-ROI scores.
            for sensor, whole in [('rgb', rgb), ('evs', evs)]:
                data = arrays[sensor]
                totals = data['counts'].sum(axis=1)
                if sensor == 'rgb':
                    totals = totals / data['valid_pixels'].sum()
                if (len(whole) != len(totals) or not np.array_equal(data['interval'], [r[0] for r in whole])
                        or not np.allclose(data['time_s'], [r[1] for r in whole], atol=1e-9, rtol=0)
                        or not np.allclose(totals, [r[2] for r in whole], atol=1e-12, rtol=0)):
                    raise ValueError(f'{sensor}: tile sum does not match whole-ROI score')
            out = args.output/session; out.mkdir()
            for sensor, data in arrays.items():
                np.savez_compressed(out/f'{sensor}_tiles.npz', **data)
            write_json(out/'tiles.json', dict(tiles=sink.tiles, output_size=sink.size, roi=config['roi'],
                step_s=sink.step, event_window_s=sink.step*sink.bins,
                tile_anchor='EVS image origin, intersect original common ROI; partial tiles use actual valid area'))
            write_json(out/'result.json', dict(source_result=result, alignment=alignment, input_hashes=hashes))
            alignments[session] = alignment
            status.append(dict(session=session, status='complete', tiles=len(sink.tiles)))
            print(f'  complete: tiles={len(sink.tiles)}, RGB={len(rgb)}, EVS={len(evs)}', flush=True)
        except Exception as error:
            status.append(dict(session=session, status='failed', stage='extraction', error=str(error)))
            print(f'  FAILED: {error}', flush=True)
        write_json(args.output/'summary.json', status)
    if controls[0] in alignments:
        errors = make_reviews(args.output, [s for s in sessions if s in alignments], controls[0], alignments)
    else:
        errors = [dict(session=controls[0], stage='review', error='no control extraction; cannot compare')]
    write_json(args.output/'review_errors.json', errors)
    links = []
    for r in status:
        session = r['session']; review = args.output/session/'tile_review.svg'
        links.append(f'<li>{html.escape(session)}: {r["status"]}' +
                     (f' / <a href="{html.escape(session, quote=True)}/tile_review.svg">タイル比較</a>' if review.exists() else '') +
                     (': '+html.escape(r['error']) if 'error' in r else '') + '</li>')
    (args.output/'index.html').write_text('<!doctype html><meta charset="utf-8"><h1>調整用・タイル活動量</h1>'
        '<p>記述的比較です。初出現前後の図は検知結果ではありません。RGBとEVSの色・数値の単位は別です。</p>'
        '<p>CSVは出現後p95が最大のタイルを選んだ診断値であり、車両位置や未知時刻の検知を意味しません。</p>'
        '<ul>'+''.join(links)+'</ul>' + ''.join(f'<p>FAILED review: {html.escape(str(e))}</p>' for e in errors))
    print(f'Report: {args.output/"index.html"}', flush=True)
    return int(bool(errors) or any(r['status']=='failed' for r in status))


if __name__ == '__main__':
    raise SystemExit(main())
