"""Text-free RGB/EVS histories at the same locations and native sample times.

The two locations are selected by the saved EVS candidate, not by RGB. This is
a development illustration, not a new sensor-performance evaluation.
"""
import argparse
import csv
import html
from pathlib import Path
import sys

import numpy as np

from rc_popout_detection_figure import FROZEN, ROOT, digest, load_evidence, replay, write_json
from rc_popout_grid_background import alarm_episodes, score_maps


def prepare(bundle, frozen, session):
    evidence = load_evidence(bundle, frozen, session)
    evs, index = replay(evidence)
    p, scene = evidence['parameters'], evidence['scene']
    rgb = score_maps(scene['data']['rgb'], scene['meta'], 'rgb', p['settings'],
                     evidence['models']['rgb'], p['input_definition']['tile_px'])
    results = dict(rgb=rgb, evs=evs)
    with (frozen/'development_summary.csv').open() as stream:
        saved = {r['method']: r for r in csv.DictReader(stream) if r['session'] == session}
    candidate = evidence['candidate']
    end = float(evs['time_s'][index])
    start = end - .150
    ids = [tile['tile_id'] for tile in candidate['tiles']]
    traces = {}
    for method, result in results.items():
        threshold = p['calibration'][method]['threshold_s']
        episodes, _, _ = alarm_episodes(result, threshold, p['settings']['release_ratio'])
        row = saved[method]
        if len(episodes) != int(row['candidates']):
            raise ValueError(f'{method}: replayed candidate count differs from frozen summary')
        if episodes and abs(episodes[0]['start_time_s'] - candidate['drive_start_s']
                            - float(row['first_candidate_from_drive_s'])) > 1e-7:
            raise ValueError(f'{method}: first candidate differs from frozen summary')
        times = result['time_s']
        selected = np.flatnonzero((times >= start) & (times <= end))
        columns = [int(np.flatnonzero(result['tile_id'] == tile)[0]) for tile in ids]
        if len(selected) < 2:
            raise ValueError(f'{method}: fewer than two native samples in the display window')
        # Do not draw a continuous history through missing/invalid observations.
        if (not np.all(result['ready'][selected]) or np.any(result['reset'][selected])
                or len(set(result['interval'][selected])) != 1):
            raise ValueError(f'{method}: selected history contains a reset or unobservable gap')
        states = result['state_s'][selected][:, columns]
        traces[method] = dict(time_s=times[selected], from_candidate_ms=(times[selected]-end)*1000,
            state_s=states, normalized=states/threshold, threshold_s=threshold,
            source_indexes=selected, full_record_candidates=len(episodes),
            source_median_step_ms=float(np.median(np.diff(times)))*1000)
    return evidence, traces, dict(start_s=start, end_s=end, duration_ms=150.,
        tile_ids=ids, reference='saved EVS first candidate',
        pair_selection='Fixed EVS-selected pair; RGB did not select these locations',
        rgb_onset_relative_s=candidate['rgb_onset_recording_s'])


def draw(output, traces):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    colors = dict(rgb=('#245e91', '#74afd3'), evs=('#c45800', '#edaa5d'))
    ymax = max(1.2, max(float(t['normalized'].max()) for t in traces.values())*1.10)

    def panel(ax, method):
        t = traces[method]
        for column, color in enumerate(colors[method]):
            ax.plot(t['from_candidate_ms'], t['normalized'][:, column], color=color,
                    lw=2.3, marker='o' if method == 'rgb' else None, ms=3.8,
                    clip_on=False)
        ax.axhline(1., color='#697986', ls=(0, (4, 3)), lw=1.1)
        if method == 'evs':
            ax.plot(t['from_candidate_ms'][-1], t['normalized'][-1].min(),
                    'o', color=colors[method][0], ms=6, clip_on=False)
        ax.set(xlim=(-150., 0.), ylim=(0., ymax))
        ax.axis('off')
        ax.annotate('', xy=(1.01, -.025), xytext=(-.01, -.025), xycoords='axes fraction',
                    arrowprops=dict(arrowstyle='->', lw=1.4, color='#2d659b'))

    for name, methods in (('trace_rgb', ['rgb']), ('trace_evs', ['evs']),
                           ('traces_rgb_evs', ['rgb', 'evs'])):
        fig = plt.figure(figsize=(6*len(methods), 3), facecolor='white')
        for i, method in enumerate(methods):
            if len(methods) == 1:
                pos = [.055, .11, .89, .83]
            else:
                pos = [.025+i*.5, .11, .45, .83]
            panel(fig.add_axes(pos), method)
        for ext in ('png', 'svg'):
            path = output/f'{name}.{ext}'
            fig.savefig(path, dpi=240, facecolor='white')
            if ext == 'svg':
                path.write_text('\n'.join(line.rstrip() for line in path.read_text().splitlines())+'\n')
        plt.close(fig)
    return dict(y_axis='integrated residual / sensor-specific frozen threshold',
        common_y_limits=[0., ymax], common_x_limits_ms=[-150., 0.],
        threshold_line=1., rgb_native_samples_marked=True, image_text=False,
        combined_panel_order=['rgb', 'evs'])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, default=ROOT/'record/09-30/analysis/development_debug_bundle01.zip')
    parser.add_argument('--frozen-dir', type=Path, default=FROZEN)
    parser.add_argument('--session', default='test_05')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError('output already exists; choose a new directory')
        import matplotlib  # Check plotting dependency before creating output.
        evidence, traces, window = prepare(args.bundle, args.frozen_dir, args.session)
        args.output.mkdir(parents=True)
        appearance = draw(args.output, traces)
        sensors = {}
        for method, t in traces.items():
            np.savez_compressed(args.output/f'{method}_trace_values.npz',
                **{k: v for k, v in t.items() if isinstance(v, np.ndarray)},
                tile_ids=window['tile_ids'], threshold_s=t['threshold_s'])
            with (args.output/f'{method}_trace_values.csv').open('w', newline='') as stream:
                writer = csv.writer(stream, lineterminator='\n')
                writer.writerow(['relative_time_s', 'from_evs_candidate_ms', 'tile_a_state_s',
                                 'tile_b_state_s', 'tile_a_over_threshold', 'tile_b_over_threshold'])
                for i, time in enumerate(t['time_s']):
                    writer.writerow([time, t['from_candidate_ms'][i], *t['state_s'][i], *t['normalized'][i]])
            sensors[method] = dict(samples=len(t['time_s']), threshold_s=t['threshold_s'],
                first_time_s=float(t['time_s'][0]), last_time_s=float(t['time_s'][-1]),
                full_record_candidates=t['full_record_candidates'],
                source_median_step_ms=t['source_median_step_ms'],
                last_values_over_threshold=t['normalized'][-1].tolist())
        report = dict(status='complete', session=args.session, subset='development', window=window,
            appearance=appearance, sensors=sensors,
            source_bundle_sha256=digest(args.bundle),
            frozen_manifest_sha256=evidence['frozen_manifest_sha256'],
            renderer_sha256=digest(Path(__file__)),
            candidate=evidence['candidate'],
            purpose='Text-free method illustration of the same EVS-selected pair, not an independent RGB/EVS evaluation',
            limitations=['RGB and EVS use their own frozen background model and threshold.',
                         'Y is threshold-normalized; it is not absolute event count, reaction time, or probability.',
                         'Lines connect native samples without resampling; no future RGB frame is included.',
                         'The selected pair is a spatial reference, not a bounding box.',
                         'RGB may alarm at other locations/times; full-record counts are reported separately.'],
            outputs_sha256={p.name: digest(p) for p in args.output.iterdir() if p.is_file()})
        write_json(args.output/'summary.json', report)
        (args.output/'index.html').write_text('<!doctype html><meta charset="utf-8">'
            '<title>RGB / EVS measured traces</title>'
            '<style>body{font:18px sans-serif;color:#24374b;margin:32px}img{width:100%;max-width:1300px}p{max-width:1100px}</style>'
            f'<h1>{html.escape(args.session)}：左 RGB ／ 右 EVS</h1>'
            '<img src="traces_rgb_evs.png"><p>同じ隣接2区画、同じ150 ms。横軸はEVS候補時刻を0とした時間、'
            '縦軸は各センサの積算スコアをそのセンサの固定しきい値で割った値。点線は1。</p>'
            '<p>RGBの小さな点は元のフレームごとの値。EVS右端の丸は検出候補時刻。RGBには検出を示す丸を追加していない。</p>'
            f'<p>区画ID {window["tile_ids"]}。表示サンプル数 RGB={sensors["rgb"]["samples"]} / EVS={sensors["evs"]["samples"]}。'
            f'全評価区間の候補数 RGB={sensors["rgb"]["full_record_candidates"]} / EVS={sensors["evs"]["full_record_candidates"]}。</p>'
            '<p>EVSで選ばれた区画の履歴をRGBでも表示した方法説明用の図。RGB自身が選んだ候補区画や、検出性能の独立評価を表すものではない。</p>'
            '<p><a href="trace_rgb.svg">RGB SVG</a> / <a href="trace_evs.svg">EVS SVG</a> / '
            '<a href="summary.json">数値と出典</a></p>', encoding='utf-8')
        print(f"Saved: {args.output/'traces_rgb_evs.png'} / RGB samples={sensors['rgb']['samples']} / EVS samples={sensors['evs']['samples']}")
        return 0
    except (OSError, ValueError, KeyError, RuntimeError, ImportError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
