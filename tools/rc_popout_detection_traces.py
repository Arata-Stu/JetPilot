"""RGB/EVS histories at the same locations, with a shared time axis.

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


def prepare(bundle, frozen, session, before_ms=100., after_ms=500.):
    if (not np.isfinite(before_ms) or not np.isfinite(after_ms)
            or before_ms < 0 or after_ms < 0 or before_ms+after_ms <= 0):
        raise ValueError('before/after ms must be finite, nonnegative and span a positive duration')
    evidence = load_evidence(bundle, frozen, session)
    evs, index = replay(evidence)
    p, scene = evidence['parameters'], evidence['scene']
    rgb = score_maps(scene['data']['rgb'], scene['meta'], 'rgb', p['settings'],
                     evidence['models']['rgb'], p['input_definition']['tile_px'])
    results = dict(rgb=rgb, evs=evs)
    with (frozen/'development_summary.csv').open() as stream:
        saved = {r['method']: r for r in csv.DictReader(stream) if r['session'] == session}
    candidate = evidence['candidate']
    candidate_time = float(evs['time_s'][index])
    onset = candidate['rgb_onset_recording_s']
    start, end = onset-before_ms/1000., onset+after_ms/1000.
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
        if start < times[0] or end > times[-1]:
            raise ValueError(f'{method}: requested window exceeds saved evaluation coverage')
        selected = np.flatnonzero((times >= start) & (times <= end))
        columns = [int(np.flatnonzero(result['tile_id'] == tile)[0]) for tile in ids]
        if len(selected) < 2:
            raise ValueError(f'{method}: fewer than two native samples in the display window')
        # Do not draw a continuous history through missing/invalid observations.
        if (not np.all(result['ready'][selected]) or np.any(result['reset'][selected])
                or len(set(result['interval'][selected])) != 1):
            raise ValueError(f'{method}: selected history contains a reset or unobservable gap')
        states = result['state_s'][selected][:, columns]
        marker = np.flatnonzero(selected == index) if method == 'evs' else np.array([], dtype=int)
        ready = result['ready']
        traces[method] = dict(time_s=times[selected], from_candidate_ms=(times[selected]-candidate_time)*1000,
            state_s=states, normalized=states/threshold, threshold_s=threshold,
            source_indexes=selected, full_record_candidates=len(episodes),
            candidate_marker_index=int(marker[0]) if len(marker) else None,
            full_evaluation_from_rgb_onset_ms=[float((times[0]-onset)*1000), float((times[-1]-onset)*1000)],
            full_evaluation_observed_s=float(result['observed_step_s'][ready].sum()),
            full_evaluation_max_global_score_ratio=float(result['score'][ready].max()/threshold),
            source_median_step_ms=float(np.median(np.diff(times)))*1000)
    return evidence, traces, dict(start_s=start, end_s=end, duration_ms=before_ms+after_ms,
        before_ms=before_ms, after_ms=after_ms,
        saved_evs_candidate_s=candidate_time, saved_evs_candidate_from_rgb_onset_ms=(candidate_time-onset)*1000,
        tile_ids=ids, reference='annotated RGB first-visible time',
        pair_selection='Fixed EVS-selected pair; RGB did not select these locations',
        rgb_onset_relative_s=candidate['rgb_onset_recording_s'])


def draw(output, traces, window):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    colors = dict(rgb='#245e91', evs='#c45800')
    styles = ('-', (0, (5, 2.5)))
    ymax = max(1.2, max(float(t['normalized'].max()) for t in traces.values())*1.10)
    onset = window['rgb_onset_relative_s']
    start_ms, end_ms = -window['before_ms'], window['after_ms']
    xmin = float(np.floor(start_ms/50)*50)
    xmax = float(np.ceil(end_ms/10)*10)
    ticks = np.arange(xmin, xmax+1e-9, 50.)

    def panel(ax, methods):
        for method in methods:
            t = traces[method]
            times = (t['time_s']-onset)*1000
            for column, style in enumerate(styles):
                ax.plot(times, t['normalized'][:, column], color=colors[method],
                        ls=style, lw=2.3, marker='o' if method == 'rgb' else None, ms=4,
                        label=f'{method.upper()} / {"AB"[column]}', clip_on=False)
            marker = t['candidate_marker_index']
            if marker is not None:
                ax.plot(times[marker], t['normalized'][marker].min(),
                        'o', color=colors[method], ms=6, clip_on=False)
        ax.axhline(1., color='#697986', ls=(0, (4, 3)), lw=1.1, zorder=1.5)
        ax.axvline(0., color='#aab8c2', lw=.8, ls=':', zorder=0)
        ax.set(xlim=(xmin, xmax), ylim=(-.025*ymax, ymax), xticks=ticks,
               yticks=np.arange(0, ymax+.001, .5))
        ax.set_xlabel('Time from RGB onset [ms]', fontsize=12, labelpad=9)
        ax.set_ylabel('Score / threshold', fontsize=12, labelpad=9)
        ax.spines[['top', 'right']].set_visible(False)
        for side in ('left', 'bottom'):
            ax.spines[side].set_color('#758898')
        ax.tick_params(labelsize=11, colors='#24374b', length=4)
        ax.grid(axis='y', color='#e6ebef', lw=.6)
        ax.set_axisbelow(True)
        ax.legend(loc='lower left', bbox_to_anchor=(0, 1.02), ncol=2*len(methods), frameon=False,
                  fontsize=11, handlelength=2.7, columnspacing=1.6)

    for name, groups in (('trace_rgb', [['rgb']]), ('trace_evs', [['evs']]),
                         ('traces_rgb_evs', [['rgb'], ['evs']]),
                         ('traces_overlay', [['rgb', 'evs']])):
        size = (8.5, 4.7) if name == 'traces_overlay' else (6.5*len(groups), 4.)
        fig, axes = plt.subplots(1, len(groups), figsize=size, squeeze=False,
                                 facecolor='white')
        for ax, methods in zip(axes[0], groups):
            panel(ax, methods)
        fig.tight_layout(pad=1.5, w_pad=2.)
        for ext in ('png', 'svg'):
            path = output/f'{name}.{ext}'
            fig.savefig(path, dpi=240, facecolor='white')
            if ext == 'svg':
                path.write_text('\n'.join(line.rstrip() for line in path.read_text().splitlines())+'\n')
        plt.close(fig)
    return dict(y_axis='integrated residual / sensor-specific frozen threshold',
        common_y_limits=[-.025*ymax, ymax], common_x_limits_ms=[xmin, xmax],
        x_axis='milliseconds from annotated RGB first-visible time',
        data_window_from_rgb_onset_ms=[start_ms, end_ms], x_ticks_ms=ticks.tolist(),
        threshold_line=1., rgb_native_samples_marked=True,
        image_text='axis labels, ticks and legend only',
        colors=colors, line_styles={'A': 'solid', 'B': 'dashed'},
        tile_labels=dict(zip('AB', window['tile_ids'])),
        combined_panel_order=['rgb', 'evs'], overlay_order=['rgb', 'evs'])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, default=ROOT/'record/09-30/analysis/development_debug_bundle01.zip')
    parser.add_argument('--frozen-dir', type=Path, default=FROZEN)
    parser.add_argument('--session', default='test_05')
    parser.add_argument('--before-ms', type=float, default=100., help='display time before RGB onset (default: 100)')
    parser.add_argument('--after-ms', type=float, default=500., help='display time after RGB onset (default: 500)')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError('output already exists; choose a new directory')
        import matplotlib  # Check plotting dependency before creating output.
        evidence, traces, window = prepare(args.bundle, args.frozen_dir, args.session,
                                           args.before_ms, args.after_ms)
        args.output.mkdir(parents=True)
        appearance = draw(args.output, traces, window)
        sensors = {}
        for method, t in traces.items():
            np.savez_compressed(args.output/f'{method}_trace_values.npz',
                **{k: v for k, v in t.items() if isinstance(v, np.ndarray)},
                tile_ids=window['tile_ids'], threshold_s=t['threshold_s'],
                from_rgb_onset_ms=(t['time_s']-window['rgb_onset_relative_s'])*1000)
            with (args.output/f'{method}_trace_values.csv').open('w', newline='') as stream:
                writer = csv.writer(stream, lineterminator='\n')
                writer.writerow(['relative_time_s', 'from_evs_candidate_ms', 'from_rgb_onset_ms', 'tile_a_state_s',
                                 'tile_b_state_s', 'tile_a_over_threshold', 'tile_b_over_threshold'])
                for i, time in enumerate(t['time_s']):
                    writer.writerow([time, t['from_candidate_ms'][i],
                        (time-window['rgb_onset_relative_s'])*1000, *t['state_s'][i], *t['normalized'][i]])
            sensors[method] = dict(samples=len(t['time_s']), threshold_s=t['threshold_s'],
                first_time_s=float(t['time_s'][0]), last_time_s=float(t['time_s'][-1]),
                full_record_candidates=t['full_record_candidates'],
                full_evaluation_from_rgb_onset_ms=t['full_evaluation_from_rgb_onset_ms'],
                full_evaluation_observed_s=t['full_evaluation_observed_s'],
                full_evaluation_max_global_score_ratio=t['full_evaluation_max_global_score_ratio'],
                displayed_max_tile_ratios=t['normalized'].max(axis=0).tolist(),
                displayed_max_pair_score_ratio=float(t['normalized'].min(axis=1).max()),
                candidate_marker_index=t['candidate_marker_index'],
                source_median_step_ms=t['source_median_step_ms'],
                last_values_over_threshold=t['normalized'][-1].tolist())
        report = dict(status='complete', session=args.session, subset='development', window=window,
            appearance=appearance, sensors=sensors,
            source_bundle_sha256=digest(args.bundle),
            frozen_manifest_sha256=evidence['frozen_manifest_sha256'],
            renderer_sha256=digest(Path(__file__)),
            candidate=evidence['candidate'],
            purpose='Method illustration of the same EVS-selected pair, not an independent RGB/EVS evaluation',
            limitations=['RGB and EVS use their own frozen background model and threshold.',
                         'Y is threshold-normalized; it is not absolute event count, reaction time, or probability.',
                         'Lines connect native samples at their acquisition timestamps without resampling.',
                         'The selected pair is a spatial reference, not a bounding box.',
                         'RGB may alarm at other locations/times; full-record counts are reported separately.'],
            outputs_sha256={p.name: digest(p) for p in args.output.iterdir() if p.is_file()})
        write_json(args.output/'summary.json', report)
        (args.output/'index.html').write_text('<!doctype html><meta charset="utf-8">'
            '<title>RGB / EVS measured traces</title>'
            '<style>body{font:18px sans-serif;color:#24374b;margin:32px}img{width:100%;max-width:1300px}p{max-width:1100px}</style>'
            f'<h1>{html.escape(args.session)}：RGB・EVSの重ね描き</h1>'
            f'<img src="traces_overlay.png"><p>同じ隣接2区画、RGB初出現の−{window["before_ms"]:g}〜+{window["after_ms"]:g} ms。'
            '横軸はRGBで見え始めた時刻を0 msとした時間、50 ms刻み。'
            '縦軸は各センサの積算スコアをそのセンサの固定しきい値で割った値。点線は1。</p>'
            '<p>青：RGB、橙：EVS。実線：区画A、破線：区画B。同じA/Bは同じ画像位置を示す。</p>'
            '<p>RGBの小さな点は元のフレームごとの値。橙の丸は保存済みEVS候補時刻（表示区間内の場合のみ）。RGBには検出を示す丸を追加していない。</p>'
            f'<p>区画ID {window["tile_ids"]}。表示サンプル数 RGB={sensors["rgb"]["samples"]} / EVS={sensors["evs"]["samples"]}。'
            f'全評価区間の候補数 RGB={sensors["rgb"]["full_record_candidates"]} / EVS={sensors["evs"]["full_record_candidates"]}。</p>'
            f'<p>RGBの評価対象データは初出現から {sensors["rgb"]["full_evaluation_from_rgb_onset_ms"][0]/1000:.3f}〜'
            f'{sensors["rgb"]["full_evaluation_from_rgb_onset_ms"][1]/1000:.3f} s。全評価区間・全隣接ペアの最大検知スコア比は '
            f'{sensors["rgb"]["full_evaluation_max_global_score_ratio"]:.3f}。上図の固定2区画の値とは集計範囲が異なる。</p>'
            '<p>EVSで選ばれた区画の履歴をRGBでも表示した方法説明用の図。RGB自身が選んだ候補区画や、検出性能の独立評価を表すものではない。</p>'
            '<p><a href="traces_overlay.svg">重ね描き SVG</a> / <a href="trace_rgb.svg">RGB SVG</a> / '
            '<a href="trace_evs.svg">EVS SVG</a> / <a href="summary.json">数値と出典</a></p>'
            '<h2>左右に並べた表示：左 RGB ／ 右 EVS</h2><img src="traces_rgb_evs.png">', encoding='utf-8')
        print(f"Saved: {args.output/'traces_overlay.png'} / RGB samples={sensors['rgb']['samples']} / EVS samples={sensors['evs']['samples']}")
        return 0
    except (OSError, ValueError, KeyError, RuntimeError, ImportError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
