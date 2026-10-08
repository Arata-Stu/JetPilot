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


def accumulate(source, clock, origin, mapping, span, end_bin, bins, step):
    """Use the detector's floor-bin membership, including delayed RAW batches."""
    _, mask, ux, uy, keep = mapping
    h, w = mask.shape
    counts = np.zeros((h, w, 2), dtype=np.int64)
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
        ix = ix[(event_bins >= end_bin-bins) & (event_bins < end_bin)]
        if np.any((events['p'][ix] != 0) & (events['p'][ix] != 1)):
            raise ValueError('expected RAW polarity values 0 or 1')
        ex, ey = events['x'][ix], events['y'][ix]
        np.add.at(counts, (uy[ey, ex], ux[ey, ex], events['p'][ix].astype(int)), 1)
        # Do not stop after passing the window: RAW may contain out-of-order
        # events in subsequent batches, as preserved by the original extractor.
    if first is None or first > a+(end_bin-bins)*step or last < a+end_bin*step:
        raise ValueError('RAW timestamp extent does not cover the selected detector window')
    return counts


def verify_counts(counts, mask, meta, expected):
    measured = []
    for tile in meta['tiles']:
        x, y, w, h = (tile[k] for k in ('x', 'y', 'width', 'height'))
        measured.append(counts[y:y+h, x:x+w][mask[y:y+h, x:x+w]].sum())
    if not np.array_equal(measured, expected):
        raise ValueError('RAW pixel counts do not match the archived detector window tile by tile')


def extract_events(sources, evidence, result, index):
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
    start = span[0]+(end_bin-bins)*step
    if (end_bin < bins or abs(end-span[0]-end_bin*step) > 1e-8
            or abs(start-result['support_start_s'][index]) > 1e-8):
        raise ValueError('archived EVS endpoint/support differs from annotation-aligned bins')
    mapping = geometry(sources, evidence)
    counts = accumulate(source, clock, evidence['candidate']['reference_origin_s'], mapping,
                        span, end_bin, bins, step)
    verify_counts(counts, mapping[1], evidence['scene']['meta'],
                  evidence['scene']['data']['evs']['counts'][index])
    if (raw[0].stat().st_size != identity['bytes'] or raw[0].stat().st_mtime_ns != identity['mtime_ns']
            or figure.digest(sidecar) != identity['metadata_sha256']):
        raise ValueError('RAW or its metadata changed during export')
    return counts, mapping[1], dict(start_s=start, end_s=end, window_ms=step*bins*1000,
        events=int(counts.sum()), raw=identity, verification='Exact event counts matched every archived tile')


def event_image(counts, weights):
    """Polarity occupancy only: never invent dots at tile centers or from RGB."""
    present = counts > 0
    n = present.sum(axis=2)
    colors = np.array([[43, 99, 160], [210, 74, 67]], dtype=float)  # OFF / ON.
    ink = np.einsum('hwp,pc->hwc', present.astype(float), colors)
    ink /= np.maximum(n, 1)[..., None]
    alpha = (n > 0)*weights
    return np.rint(255*(1-alpha[..., None])+ink*alpha[..., None]).astype(np.uint8)


def draw(output, counts, mask, evidence, result, index):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, FancyArrowPatch
    from PIL import Image

    meta = evidence['scene']['meta']
    x, y, w, h = (meta['roi'][k] for k in ('x', 'y', 'width', 'height'))
    values = figure.map_values(evidence, result, index)
    # Keep the same fixed residual scale as the existing method illustration.
    weights = np.zeros(mask.shape)
    for tile, z in zip(meta['tiles'], values['positive_residual']):
        tx, ty, tw, th = (tile[k] for k in ('x', 'y', 'width', 'height'))
        weights[ty:ty+th, tx:tx+tw] = np.clip(z/evidence['parameters']['settings']['z_clip'], 0, 1)
    weights *= mask
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
            ax.add_patch(Rectangle((tx-.5, ty-.5), tw, th, fill=False,
                                  ec='#607f94', lw=.35, alpha=.22))
        ax.set(xlim=(x-.5, x+w-.5), ylim=(y+h-.5, y-.5))
        ax.axis('off')

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
        tile_ids=result['tile_id'], relative_time_s=result['time_s'][index])
    for path in output.glob('*.svg'):
        path.write_text('\n'.join(line.rstrip() for line in path.read_text().splitlines())+'\n')
    return dict(roi=meta['roi'], polarity_colors=dict(off='#2b63a0', on='#d24a43'),
        event_display='occupied native event pixels; no dilation or RGB background',
        residual_opacity='clip(positive standardized tile residual / frozen z_clip, 0, 1)',
        background_display_max_density=scale,
        background_opacity='0.85 * clip(predicted tile density / shared density max, 0, 1)',
        note='Tile-weighted event visualization, not pixelwise background removal or segmentation')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record-root', type=Path, required=True)
    parser.add_argument('--session', default='test_05')
    parser.add_argument('--bundle', type=Path)
    parser.add_argument('--frozen-dir', type=Path, default=figure.FROZEN)
    parser.add_argument('--camchain', type=Path, default=figure.CALIBRATION/'config/calibrations/rc_popout_default/kalibr-camchain.yaml')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--preflight', action='store_true')
    parser.add_argument('--debug', action='store_true')
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError('output already exists; choose a new directory')
        bundle = args.bundle or args.record_root.parent/'analysis/development_debug_bundle01.zip'
        evidence = figure.load_evidence(bundle, args.frozen_dir, args.session)
        sources = figure.validate_sources(args.record_root, args.camchain, evidence)
        raw = list(sources['bag'].glob('*.raw'))
        if len(raw) != 1 or not Path(str(raw[0])+'.metadata.yaml').is_file():
            raise ValueError('one RAW recording and its time-anchor metadata are required')
        if args.preflight:
            print('Preflight passed: archived model/sync/calibration and RAW paths checked; no decode or output.')
            return 0
        for module in ('matplotlib', 'PIL', 'cv2', 'yaml', 'metavision_core.event_io'):
            importlib.import_module(module)
        result, index = figure.replay(evidence)
        print(f'{args.session}: frozen candidate reproduced; reading measured RAW event positions...', flush=True)
        counts, mask, timing = extract_events(sources, evidence, result, index)
        figure.validate_sources(args.record_root, args.camchain, evidence)
        args.output.mkdir(parents=True)
        appearance = draw(args.output, counts, mask, evidence, result, index)
        figure.write_json(args.output/'summary.json', dict(status='complete', session=args.session,
            subset='development', timing=timing, appearance=appearance, candidate=evidence['candidate'],
            source_mode='native_RAW_event_positions_and_archived_grid_model',
            purpose='Background residual method illustration; no additional detection evaluation',
            bundle_sha256=figure.digest(bundle), frozen_manifest_sha256=evidence['frozen_manifest_sha256'],
            annotation_sha256=figure.digest(sources['annotation']), time_sync_sha256=figure.digest(sources['time_sync']),
            camchain_sha256=figure.digest(sources['camchain']), renderer_sha256=figure.digest(__file__),
            outputs_sha256={p.name: figure.digest(p) for p in args.output.iterdir() if p.is_file()}))
        (args.output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>Measured EVS background residual</title>'
            '<style>body{font:18px sans-serif;color:#24374b;margin:32px;max-width:1400px}img{width:100%}</style>'
            f'<h1>{html.escape(args.session)}：イベント像による背景差分の説明</h1>'
            '<img src="detection_method_events.png">'
            '<p>左：実際のイベント点と32×32画素のグリッド。下：同時刻の背景活動推定。'
            '右：背景との差が大きい区画ほどイベント点を濃く表示。図内に文章や折れ線はありません。</p>'
            f'<p>対象窓は元の検知器と同じ過去 {timing["window_ms"]:g} ms、時刻 {timing["end_s"]:.6f} s。'
            f'実イベント {timing["events"]} 件をRAWから取得し、区画ごとの個数が保存済み活動量と一致することを確認しました。</p>'
            '<p>青はOFF、赤はONイベント。両極性が同じ画素にある場合は混色します。重複イベントは計算では保持し、画像は画素の発火有無で表示します。'
            '下の背景推定は区画ごとの統計値であり、架空の背景イベント点は作っていません。</p>'
            '<p>右側は区画の正の標準化残差に応じた濃淡表示です。画素単位の背景除去や車両セグメンテーションの結果ではありません。'
            '実際の判定では、この区画残差を時間積算し、隣接区画を評価します。</p>'
            '<p><a href="detection_method_events.svg">全体 SVG</a> / <a href="events_grid.svg">入力図 SVG</a> / '
            '<a href="residual_events_grid.svg">背景との差 SVG</a> / <a href="summary.json">数値・出典</a></p>', encoding='utf-8')
        print(f'Saved: {args.output}/detection_method_events.png / events={timing["events"]}; tile counts verified')
        return 0
    except (OSError, ValueError, KeyError, RuntimeError, ImportError) as error:
        if args.debug:
            raise
        print(f'error: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
