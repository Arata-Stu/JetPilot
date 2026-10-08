"""Text-free event-view illustration of the frozen grid background residual.

Event coordinates come from RAW, not from RGB or synthesized tile textures.
The residual view weights these points by the tile residual; it is not pixel
segmentation or a background-removed event stream.
"""
import argparse
import html
import importlib
from pathlib import Path
import sys

import numpy as np

import rc_popout_detection_figure as figure
from rc_popout_detection import apply_common_mask, evaluation_intervals


def geometry(sources, evidence):
    import cv2
    sys.path.insert(0, str(figure.CALIBRATION))
    from multi_sensor_calibration.scenario_overlay import _camera
    from multi_sensor_calibration.io import load_yaml

    spatial = evidence['parameters']['input_definition']['spatial']
    if not np.allclose(spatial['event_to_view_homography'], np.eye(3), atol=1e-9, rtol=0):
        raise ValueError('native undistorted EVS coordinate system required')
    chain = load_yaml(sources['camchain'])
    evs, rgb = _camera(chain, 'cam0', np), _camera(chain, 'cam1', np)
    size = tuple(spatial['output_size'])
    if size != tuple(evs['size']):
        raise ValueError('EVS calibration dimensions differ from the archived grid')
    supports = []
    for camera, transform in ((evs, np.eye(3)), (rgb, np.asarray(spatial['rgb_to_view_homography']))):
        remap = cv2.initUndistortRectifyMap(camera['matrix'], camera['distortion'], None,
            camera['matrix'], camera['size'], cv2.CV_32FC1)
        valid = cv2.remap(np.ones((camera['size'][1], camera['size'][0]), np.float32),
                         *remap, cv2.INTER_LINEAR)
        supports.append(cv2.warpPerspective(valid, transform, size, flags=cv2.INTER_LINEAR) >= .999)
    common = supports[0] & supports[1]
    meta = evidence['scene']['meta']
    figure.validate_support(common, meta)
    roi = meta['roi']
    x, y, w, h = (roi[k] for k in ('x', 'y', 'width', 'height'))
    mask = np.zeros_like(common)
    mask[y:y+h, x:x+w] = True
    mask = apply_common_mask(mask, common, roi['mask_policy'])
    yy, xx = np.indices(mask.shape)
    points = np.stack((xx, yy), axis=-1).astype(np.float32).reshape(-1, 1, 2)
    mapped = cv2.undistortPoints(points, evs['matrix'], evs['distortion'], P=evs['matrix']).reshape(*mask.shape, 2)
    ux, uy = np.rint(mapped[..., 0]).astype(int), np.rint(mapped[..., 1]).astype(int)
    inside = (ux >= 0) & (ux < size[0]) & (uy >= 0) & (uy < size[1])
    keep = np.zeros_like(mask)
    keep[inside] = mask[uy[inside], ux[inside]]
    return common, mask, ux, uy, keep


def accumulate(source, clock, origin, mapping, span, end_bin, bins, step, *, window_count=1):
    """Use the detector's floor-bin membership, including delayed RAW batches."""
    _, mask, ux, uy, keep = mapping
    h, w = mask.shape
    counts = np.zeros((window_count, h, w, 2), dtype=np.int64)
    first_bin = end_bin-bins*window_count
    if first_bin < 0:
        raise ValueError('display window extends before the evaluation interval')
    first, last = None, None
    a, b = span
    for batch in source.batches():
        if (source.width, source.height) != (w, h):
            raise ValueError('RAW dimensions differ from calibrated EVS grid')
        events = batch.events
        if not len(events):
            continue
        provisional = source.anchor.reference_time_s + source.anchor.scale * (
            events['t'].astype(np.float64)-source.anchor.source_time_us)/1e6
        times = clock.apply(provisional)-origin
        if not np.all(np.isfinite(times)):
            raise ValueError('nonfinite event timestamps')
        first = float(times.min()) if first is None else min(first, float(times.min()))
        last = float(times.max()) if last is None else max(last, float(times.max()))
        good = ((events['x'] >= 0) & (events['x'] < w) & (events['y'] >= 0) & (events['y'] < h)
                & (times >= a) & (times < b))
        ix = np.flatnonzero(good)
        ix = ix[keep[events['y'][ix], events['x'][ix]]]
        event_bins = np.floor((times[ix]-a)/step).astype(np.int64)
        selected = (event_bins >= first_bin) & (event_bins < end_bin)
        ix, event_bins = ix[selected], event_bins[selected]
        if np.any((events['p'][ix] != 0) & (events['p'][ix] != 1)):
            raise ValueError('expected RAW polarity values 0 or 1')
        ex, ey = events['x'][ix], events['y'][ix]
        slot = (event_bins-first_bin)//bins
        np.add.at(counts, (slot, uy[ey, ex], ux[ey, ex], events['p'][ix].astype(int)), 1)
        # Do not stop after passing the window: RAW may contain out-of-order
        # events in subsequent batches, as preserved by the original extractor.
    if first is None or first > a+first_bin*step or last < a+end_bin*step:
        raise ValueError('RAW timestamp extent does not cover the selected detector window')
    return counts[0] if window_count == 1 else counts


def verify_counts(counts, mask, meta, expected):
    measured = []
    for tile in meta['tiles']:
        x, y, w, h = (tile[k] for k in ('x', 'y', 'width', 'height'))
        measured.append(counts[y:y+h, x:x+w][mask[y:y+h, x:x+w]].sum())
    if not np.array_equal(measured, expected):
        raise ValueError('RAW pixel counts do not match the archived detector window tile by tile')


def display_indexes(evidence, result, index, display_window_ms):
    """Use disjoint detector windows; overlapping 1 ms updates must not double-count events."""
    definition = evidence['parameters']['input_definition']
    bins = definition['window_bins']
    window_ms = definition['step_ms']*bins
    duration = window_ms if display_window_ms is None else display_window_ms
    if (not np.isfinite(duration) or duration <= 0 or duration > 100
            or not np.isclose(duration/window_ms, round(duration/window_ms), rtol=0, atol=1e-9)):
        raise ValueError(f'display-window-ms must be a positive multiple of {window_ms:g}, at most 100 ms')
    n = int(round(duration/window_ms))
    indexes = index-np.arange(n-1, -1, -1)*bins
    if indexes[0] < 0:
        raise ValueError('display window exceeds saved evaluation coverage')
    ends = result['time_s'][indexes]
    starts = result['support_start_s'][indexes]
    full_span = slice(int(indexes[0]), index+1)
    if (not np.all(result['ready'][full_span]) or np.any(result['reset'][full_span])
            or not np.all(result['interval'][full_span] == result['interval'][index])
            or not np.allclose(ends-starts, window_ms/1000, rtol=0, atol=1e-8)
            or not np.allclose(ends[:-1], starts[1:], rtol=0, atol=1e-8)):
        raise ValueError('display history crosses a gap, reset, or noncontiguous detector windows')
    return indexes


def extract_events(sources, evidence, result, index, display_window_ms=None):
    sys.path.insert(0, str(figure.CALIBRATION))
    from multi_sensor_calibration.evs_sources import MetavisionFileSource
    from multi_sensor_calibration.io import load_yaml
    from multi_sensor_calibration.models import ClockEstimate

    raw = list(sources['bag'].glob('*.raw'))
    if len(raw) != 1:
        raise ValueError('exactly one source RAW required')
    sidecar = Path(str(raw[0])+'.metadata.yaml')
    identity = dict(raw=str(raw[0]), bytes=raw[0].stat().st_size,
                    mtime_ns=raw[0].stat().st_mtime_ns, metadata_sha256=figure.digest(sidecar))
    source = MetavisionFileSource(raw[0])
    if source.anchor is None:
        raise ValueError('RAW time anchor missing')
    clock = ClockEstimate.from_dict(load_yaml(sources['time_sync'])['models']['evs'])
    if not np.isfinite(clock.drift) or clock.drift <= -1:
        raise ValueError('invalid EVS clock scale')
    ann = figure.read_json(sources['annotation'])
    span = evaluation_intervals(ann)[int(result['interval'][index])]
    definition = evidence['parameters']['input_definition']
    step, bins = definition['step_ms']/1000., definition['window_bins']
    end = float(result['time_s'][index])
    end_bin = int(round((end-span[0])/step))
    indexes = display_indexes(evidence, result, index, display_window_ms)
    start = span[0]+(end_bin-bins)*step
    if (end_bin < bins or abs(end-span[0]-end_bin*step) > 1e-8
            or abs(start-result['support_start_s'][index]) > 1e-8):
        raise ValueError('archived EVS endpoint/support differs from annotation-aligned bins')
    mapping = geometry(sources, evidence)
    pixels = accumulate(source, clock, evidence['candidate']['reference_origin_s'], mapping,
                        span, end_bin, bins, step, window_count=len(indexes))
    windows = pixels[None] if len(indexes) == 1 else pixels
    for frame, saved_index in zip(windows, indexes):
        verify_counts(frame, mapping[1], evidence['scene']['meta'],
                      evidence['scene']['data']['evs']['counts'][saved_index])
    counts = windows.sum(axis=0)
    history = None if len(indexes) == 1 else dict(counts_by_window=windows, source_indexes=indexes,
        time_s=result['time_s'][indexes], support_start_s=result['support_start_s'][indexes],
        residual_z=result['residual_z'][indexes])
    if (raw[0].stat().st_size != identity['bytes'] or raw[0].stat().st_mtime_ns != identity['mtime_ns']
            or figure.digest(sidecar) != identity['metadata_sha256']):
        raise ValueError('RAW or its metadata changed during export')
    return counts, mapping[1], dict(start_s=float(result['support_start_s'][indexes[0]]), end_s=end,
        window_ms=step*bins*len(indexes)*1000, detector_window_ms=step*bins*1000,
        detector_start_s=start, detector_events=int(windows[-1].sum()),
        display_windows=len(indexes), events=int(counts.sum()), raw=identity,
        verification='Each disjoint detector window matched every archived tile'), history


def event_image(counts, weights):
    """Polarity occupancy only: never invent dots at tile centers or from RGB."""
    present = counts > 0
    n = present.sum(axis=2)
    colors = np.array([[43, 99, 160], [210, 74, 67]], dtype=float)  # OFF / ON.
    weights = weights[..., None] if weights.ndim == 2 else weights
    opacity = present*weights
    ink = np.einsum('hwp,pc->hwc', opacity, 255-colors)/np.maximum(n, 1)[..., None]
    return np.rint(255-ink).astype(np.uint8)


def display_weights(counts, mask, meta, residuals, maximum, history=None):
    def tile_weights(vector):
        weights = np.zeros(mask.shape)
        for tile, z in zip(meta['tiles'], vector):
            x, y, w, h = (tile[k] for k in ('x', 'y', 'width', 'height'))
            weights[y:y+h, x:x+w] = np.clip(z/maximum, 0, 1)
        return weights*mask
    if history is None:
        return tile_weights(residuals)
    # Every event contributes once with its own 2 ms block's residual. Repeated
    # events at a pixel/polarity receive their occurrence-weighted mean opacity.
    weighted = np.zeros(counts.shape, dtype=float)
    for frame, z in zip(history['counts_by_window'], history['residual_z']):
        weighted += frame*tile_weights(z)[..., None]
    return np.divide(weighted, counts, out=np.zeros_like(weighted), where=counts > 0)


def draw(output, counts, mask, evidence, result, index, *, layout='model', history=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, FancyArrowPatch
    from PIL import Image

    meta = evidence['scene']['meta']
    x, y, w, h = (meta['roi'][k] for k in ('x', 'y', 'width', 'height'))
    values = figure.map_values(evidence, result, index)
    if layout not in ('model', 'compact'):
        raise ValueError('unknown figure layout')
    if history is not None and layout != 'compact':
        raise ValueError('long accumulation uses --layout compact; the background inset is a single-window model')
    # Compact poster view maps the largest residual in this snapshot to full
    # opacity. This changes display contrast only, never detector inputs/state.
    all_residuals = values['positive_residual'] if history is None else history['residual_z']
    residual_max = (max(float(all_residuals.max()), 1e-9) if layout == 'compact'
                    else evidence['parameters']['settings']['z_clip'])
    weights = display_weights(counts, mask, meta, values['positive_residual'], residual_max, history)
    raw, weighted = event_image(counts, mask.astype(float)), event_image(counts, weights)
    scale = max(1e-9, float(max(values['observed_density'].max(), values['estimated_background'].max())))

    def panel(ax, kind):
        ax.imshow(np.full_like(raw, 255) if kind == 'background' else
                  raw if kind == 'events' else weighted, interpolation='nearest')
        for tile, value in zip(meta['tiles'], values['estimated_background']):
            tx, ty, tw, th = (tile[k] for k in ('x', 'y', 'width', 'height'))
            if kind == 'background':
                ax.add_patch(Rectangle((tx-.5, ty-.5), tw, th, color='#2d659b',
                                      lw=0, alpha=.85*min(1., float(value)/scale)))
            if layout == 'model' or kind != 'residual':
                ax.add_patch(Rectangle((tx-.5, ty-.5), tw, th, fill=False,
                    ec='#607f94', lw=.25 if layout == 'compact' else .35,
                    alpha=.10 if layout == 'compact' else .22))
        ax.set(xlim=(x-.5, x+w-.5), ylim=(y+h-.5, y-.5))
        ax.axis('off')

    if layout == 'compact':
        fig = plt.figure(figsize=(15, 3.2), facecolor='white')
        panel(fig.add_axes([.02, .08, .43, .84]), 'events')
        panel(fig.add_axes([.55, .08, .43, .84]), 'residual')
        fig.add_artist(FancyArrowPatch((.466, .5), (.534, .5), transform=fig.transFigure,
            arrowstyle='-|>', mutation_scale=22, lw=1.8, color='#2d659b'))
    else:
        fig = plt.figure(figsize=(15, 5), facecolor='white')
        panel(fig.add_axes([.02, .39, .42, .58]), 'events')
        panel(fig.add_axes([.56, .39, .42, .58]), 'residual')
        panel(fig.add_axes([.285, .025, .28, .315]), 'background')
        fig.add_artist(FancyArrowPatch((.455, .68), (.545, .68), transform=fig.transFigure,
            arrowstyle='-|>', mutation_scale=24, lw=2, color='#2d659b'))
        fig.add_artist(FancyArrowPatch((.58, .185), (.70, .395), transform=fig.transFigure,
            connectionstyle='angle,angleA=0,angleB=90,rad=10', arrowstyle='-|>',
            mutation_scale=20, lw=1.8, color='#2d659b'))
    for ext in ('png', 'svg'):
        fig.savefig(output/f'detection_method_events.{ext}', dpi=240, facecolor='white')
    plt.close(fig)
    for kind, name in (('events', 'events_grid'), ('residual', 'residual_events_grid'),
                       ('background', 'estimated_background_grid')):
        fig = plt.figure(figsize=(8, 8*h/w), facecolor='white')
        panel(fig.add_axes([0, 0, 1, 1]), kind)
        for ext in ('png', 'svg'):
            fig.savefig(output/f'{name}.{ext}', dpi=160, facecolor='white')
        plt.close(fig)
    for name, pixels in (('events', raw), ('residual_events', weighted)):
        Image.fromarray(pixels[y:y+h, x:x+w]).save(output/f'{name}.png')
    # This is an ROI support mask, not a denoised event mask.
    np.savez_compressed(output/'source_values.npz', counts_by_polarity=counts,
        roi_support=mask, display_weights=weights, **values,
        **({f'history_{k}': v for k, v in history.items()} if history is not None else {}),
        tile_ids=result['tile_id'], relative_time_s=result['time_s'][index])
    for path in output.glob('*.svg'):
        path.write_text('\n'.join(line.rstrip() for line in path.read_text().splitlines())+'\n')
    return dict(roi=meta['roi'], polarity_colors=dict(off='#2b63a0', on='#d24a43'),
        layout=layout, residual_display_max=residual_max,
        event_display='occupied native event pixels; no dilation or RGB background',
        residual_opacity='clip(positive standardized tile residual / residual_display_max, 0, 1)',
        residual_scale_policy=('display history maximum' if history is not None else 'snapshot maximum')
            if layout == 'compact' else 'frozen z_clip',
        residual_time_policy='each disjoint detector window at its endpoint' if history is not None else 'candidate snapshot',
        repeated_pixel_policy='event-count weighted mean opacity per pixel and polarity',
        map_values_reference='Candidate-endpoint detector window; history arrays retain each display block separately',
        background_display_max_density=scale,
        background_opacity='0.85 * clip(predicted tile density / shared density max, 0, 1)',
        note='Tile-weighted event visualization, not pixelwise background removal or segmentation')


def load_export(folder, evidence, result, index):
    """Replot a transferred measured export without RAW/ROS access."""
    report = figure.read_json(folder/'summary.json')
    if (report.get('status') != 'complete'
            or report.get('source_mode') != 'native_RAW_event_positions_and_archived_grid_model'
            or report['session'] != evidence['candidate']['session']
            or report['candidate'] != evidence['candidate']
            or report['bundle_sha256'] != evidence['provenance']['bundle_sha256']
            or report['frozen_manifest_sha256'] != evidence['frozen_manifest_sha256']):
        raise ValueError('transferred export differs from the frozen recording/candidate')
    for key in ('annotation_sha256', 'time_sync_sha256'):
        if report[key] != evidence['candidate'][key]:
            raise ValueError(f'transferred export {key} mismatch')
    if report['camchain_sha256'] != evidence['parameters']['input_definition']['spatial']['camchain_sha256']:
        raise ValueError('transferred export calibration mismatch')
    path = folder/'source_values.npz'
    if figure.digest(path) != report['outputs_sha256']['source_values.npz']:
        raise ValueError('transferred event arrays changed since RAW extraction')
    with np.load(path, allow_pickle=False) as saved:
        counts, mask = saved['counts_by_polarity'], saved['roi_support']
        history_keys = ('counts_by_window', 'source_indexes', 'time_s', 'support_start_s', 'residual_z')
        history = ({k: saved[f'history_{k}'] for k in history_keys}
                   if any(k.startswith('history_') for k in saved.files) else None)
        for name, expected in figure.map_values(evidence, result, index).items():
            if not np.array_equal(saved[name], expected):
                raise ValueError(f'transferred {name} differs from frozen replay')
        if (not np.array_equal(saved['tile_ids'], result['tile_id'])
                or abs(float(saved['relative_time_s'])-result['time_s'][index]) > 1e-9):
            raise ValueError('transferred event timestamp or grid differs')
    width, height = evidence['scene']['meta']['output_size']
    if (counts.shape != (height, width, 2) or mask.shape != (height, width)
            or mask.dtype != bool or counts.dtype.kind not in 'iu' or np.any(counts < 0)
            or np.any(counts[~mask])):
        raise ValueError('invalid transferred event pixel arrays')
    figure.validate_support(mask, evidence['scene']['meta'])
    timing = report['timing']
    expected_start = result['support_start_s'][index]
    if history is None:
        verify_counts(counts, mask, evidence['scene']['meta'], evidence['scene']['data']['evs']['counts'][index])
    else:
        indexes = display_indexes(evidence, result, index, timing['window_ms'])
        frames = history['counts_by_window']
        if (len(indexes) < 2 or frames.shape != (len(indexes), *counts.shape)
                or frames.dtype.kind not in 'iu' or np.any(frames < 0)
                or np.any(frames[:, ~mask]) or not np.array_equal(frames.sum(axis=0), counts)
                or not np.array_equal(history['source_indexes'], indexes)):
            raise ValueError('invalid transferred display history counts/indexes')
        for k, expected in (('time_s', result['time_s'][indexes]),
                            ('support_start_s', result['support_start_s'][indexes]),
                            ('residual_z', result['residual_z'][indexes])):
            if not np.array_equal(history[k], expected):
                raise ValueError(f'transferred history {k} differs from frozen replay')
        for frame, saved_index in zip(frames, indexes):
            verify_counts(frame, mask, evidence['scene']['meta'],
                          evidence['scene']['data']['evs']['counts'][saved_index])
        expected_start = result['support_start_s'][indexes[0]]
    if (timing['events'] != int(counts.sum()) or abs(timing['end_s']-result['time_s'][index]) > 1e-9
            or abs(timing['start_s']-expected_start) > 1e-9
            or abs(timing['window_ms']-(timing['end_s']-expected_start)*1000) > 1e-6
            or ('detector_window_ms' in timing and abs(timing['detector_window_ms']-
                (result['time_s'][index]-result['support_start_s'][index])*1000) > 1e-6)):
        raise ValueError('transferred RAW time window/count mismatch')
    return counts, mask, timing, report, history


def check_cached_window(timing, requested):
    if requested is not None and (not np.isfinite(requested)
            or not np.isclose(requested, timing['window_ms'], rtol=0, atol=1e-9)):
        raise ValueError(f'cached figure contains only its {timing["window_ms"]:g} ms window; '
                         'use --record-root to read RAW for a different display-window-ms')


def write_page(output, session, timing, appearance):
    compact = appearance['layout'] == 'compact'
    description = ('左：実際のイベント点と薄い32×32画素グリッド。右：背景との差が大きい区画ほどイベント点を濃く表示。'
                   if compact else '左：実際のイベント点と32×32画素のグリッド。下：同時刻の背景活動推定。右：区画の残差で濃さを重み付け。')
    scale = ('右図は表示対象の最大残差を最も濃く表示するよう正規化しています。'
             'これは表示コントラストの変更で、検知器の入力・スコア・しきい値は変更していません。'
             if compact else '右図は固定済みz_clipを濃さの上限として表示しています。')
    (output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>Measured EVS background residual</title>'
        '<style>body{font:18px sans-serif;color:#24374b;margin:32px;max-width:1400px}img{width:100%}</style>'
        f'<h1>{html.escape(session)}：イベント像による背景差分の説明</h1>'
        '<img src="detection_method_events.png">'
        f'<p>{description}図内に文章や折れ線はありません。</p>'
        f'<p>可視化は時刻 {timing["end_s"]:.6f} sまでの過去 {timing["window_ms"]:g} ms、'
        f'検知器の窓は {timing.get("detector_window_ms", timing["window_ms"]):g} ms。'
        f'実イベント {timing["events"]} 件。区画ごとの個数は元の検知窓ごとに保存済み活動量と照合しています。</p>'
        '<p>長い蓄積では、重複しない各検知窓のイベントを、その窓末尾の解析済み残差で重み付けして表示します。'
        '同じ画素・極性に複数回発火がある場合は、発火回数で重み付けした平均の濃さを使います。'
        '候補時刻より後のイベントは含めません。表示用の蓄積時間から検知遅延を読み取る図ではありません。</p>'
        f'<p>{scale} 表示上限：標準化残差 {appearance["residual_display_max"]:.6g}。</p>'
        '<p>青はOFF、赤はONイベント。両極性が同じ画素にある場合は混色。計算では重複イベントも保持し、画像は発火有無で表示します。</p>'
        '<p>右側は区画の正の標準化残差に応じた濃淡表示で、画素単位の背景除去や車両セグメンテーションではありません。'
        '別出力の背景推定は区画ごとの統計値です。背景のイベント点を合成していません。</p>'
        '<p><a href="detection_method_events.svg">全体 SVG</a> / <a href="events_grid.svg">入力図 SVG</a> / '
        '<a href="residual_events_grid.svg">背景との差 SVG</a> / <a href="summary.json">数値・出典</a></p>', encoding='utf-8')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument('--record-root', type=Path)
    inputs.add_argument('--from-figure', type=Path, help='Replot an exported summary.json + source_values.npz without RAW/ROS')
    parser.add_argument('--session', default='test_05')
    parser.add_argument('--bundle', type=Path)
    parser.add_argument('--frozen-dir', type=Path, default=figure.FROZEN)
    parser.add_argument('--camchain', type=Path, default=figure.CALIBRATION/'config/calibrations/rc_popout_default/kalibr-camchain.yaml')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--layout', choices=('compact', 'model'), default='compact')
    parser.add_argument('--display-window-ms', type=float,
        help='Visualization only: a multiple of the detector window, up to 100 ms; default 2 ms for RAW or the exported window for --from-figure')
    parser.add_argument('--preflight', action='store_true')
    parser.add_argument('--debug', action='store_true')
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError('output already exists; choose a new directory')
        bundle = args.bundle or ((args.record_root.parent if args.record_root else figure.ROOT/'record/09-30')
                                 /'analysis/development_debug_bundle01.zip')
        evidence = figure.load_evidence(bundle, args.frozen_dir, args.session)
        result, index = figure.replay(evidence)
        if args.from_figure:
            counts, mask, timing, original, history = load_export(args.from_figure, evidence, result, index)
            check_cached_window(timing, args.display_window_ms)
            if history is not None and args.layout != 'compact':
                raise ValueError('long accumulation requires --layout compact')
            if args.preflight:
                print('Transferred measured arrays and frozen scores verified; no RAW access or output.')
                return 0
            importlib.import_module('matplotlib')
            importlib.import_module('PIL')
            args.output.mkdir(parents=True)
            appearance = draw(args.output, counts, mask, evidence, result, index, layout=args.layout, history=history)
            original.update(appearance=appearance, renderer_sha256=figure.digest(__file__),
                replot_source=dict(path=str(args.from_figure.resolve()),
                    summary_sha256=figure.digest(args.from_figure/'summary.json'),
                    arrays_sha256=figure.digest(args.from_figure/'source_values.npz')),
                outputs_sha256={p.name: figure.digest(p) for p in args.output.iterdir() if p.is_file()})
            figure.write_json(args.output/'summary.json', original)
            write_page(args.output, args.session, timing, appearance)
            print(f'Saved: {args.output}/detection_method_events.png / reused {timing["events"]} verified real events')
            return 0
        indexes = display_indexes(evidence, result, index, args.display_window_ms)
        if len(indexes) > 1 and args.layout != 'compact':
            raise ValueError('long accumulation requires --layout compact')
        sources = figure.validate_sources(args.record_root, args.camchain, evidence)
        raw = list(sources['bag'].glob('*.raw'))
        if len(raw) != 1 or not Path(str(raw[0])+'.metadata.yaml').is_file():
            raise ValueError('one RAW recording and its time-anchor metadata are required')
        if args.preflight:
            print('Preflight passed: archived model/sync/calibration and RAW paths checked; no decode or output.')
            return 0
        for module in ('matplotlib', 'PIL', 'cv2', 'yaml', 'metavision_core.event_io'):
            importlib.import_module(module)
        print(f'{args.session}: frozen candidate reproduced; reading measured RAW event positions...', flush=True)
        counts, mask, timing, history = extract_events(sources, evidence, result, index, args.display_window_ms)
        figure.validate_sources(args.record_root, args.camchain, evidence)
        args.output.mkdir(parents=True)
        appearance = draw(args.output, counts, mask, evidence, result, index, layout=args.layout, history=history)
        figure.write_json(args.output/'summary.json', dict(status='complete', session=args.session,
            subset='development', timing=timing, appearance=appearance, candidate=evidence['candidate'],
            source_mode='native_RAW_event_positions_and_archived_grid_model',
            purpose='Background residual method illustration; no additional detection evaluation',
            bundle_sha256=figure.digest(bundle), frozen_manifest_sha256=evidence['frozen_manifest_sha256'],
            annotation_sha256=figure.digest(sources['annotation']), time_sync_sha256=figure.digest(sources['time_sync']),
            camchain_sha256=figure.digest(sources['camchain']), renderer_sha256=figure.digest(__file__),
            outputs_sha256={p.name: figure.digest(p) for p in args.output.iterdir() if p.is_file()}))
        write_page(args.output, args.session, timing, appearance)
        print(f'Saved: {args.output}/detection_method_events.png / events={timing["events"]}; '
              f'display={timing["window_ms"]:g} ms / detector={timing["detector_window_ms"]:g} ms; '
              f'{timing["display_windows"]} disjoint windows verified')
        return 0
    except (OSError, ValueError, KeyError, RuntimeError, ImportError) as error:
        if args.debug:
            raise
        print(f'error: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
