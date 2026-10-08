"""Plot saved static RGB/EVS detector outputs on the annotated RGB-onset axis.

Read measured tile arrays, reproduce frozen inference, and verify saved results.
No bag decoding, fitting, threshold tuning, or annotation changes.
"""
import argparse
import csv
import html
from pathlib import Path
import sys

import numpy as np

import evaluate_rc_popout_grid_static as static
from rc_popout_grid_static_videos import load_review


def prepare(folder, frozen, session, before_ms=100., after_ms=500.):
    if (not np.isfinite(before_ms) or not np.isfinite(after_ms)
            or min(before_ms, after_ms) < 0 or before_ms+after_ms <= 0):
        raise ValueError('before/after ms must be finite, nonnegative and span a positive duration')
    # Reuse the static review contract: complete run, source identities, frozen
    # parameters, tile hashes, candidate times/pairs, and CSV/JSON agreement.
    manifest = load_review(folder, frozen)
    summaries = {r['method']: r for r in manifest['summaries'] if r['session'] == session}
    if set(summaries) != {'rgb', 'evs'}:
        raise ValueError(f'{session}: not in the completed static analysis')
    onset = summaries['rgb']['rgb_first_visible_s']
    if onset is None or not np.isfinite(onset):
        raise ValueError('finite RGB first-visible annotation required')
    parameters, models, _ = static.load_frozen(frozen)
    scene_dir = folder/session
    meta = static.read_json(scene_dir/'tiles.json')
    source = static.read_json(scene_dir/'result.json')['scene']
    start, end = onset-before_ms/1000., onset+after_ms/1000.
    traces, sensors = {}, {}
    for method in ('rgb', 'evs'):
        with np.load(scene_dir/f'{method}_tiles.npz', allow_pickle=False) as data:
            result = static.score_maps(data, meta, method, parameters['settings'],
                models[method], parameters['input_definition']['tile_px'])
        # The lines must reproduce the original full history, not just its
        # crossing timestamps. Float comparisons tolerate roundoff only.
        with np.load(scene_dir/f'{method}_background_maps.npz', allow_pickle=False) as saved:
            for key in ('time_s', 'interval', 'tile_id', 'ready', 'reset', 'winner_pair', 'background_index'):
                if not np.array_equal(result[key], saved[key]):
                    raise ValueError(f'{method}: saved {key} differs from frozen replay')
            for key in ('score', 'state_s'):
                if not np.allclose(result[key], saved[key], rtol=1e-6, atol=1e-9):
                    raise ValueError(f'{method}: saved {key} differs from frozen replay')
            summary, candidates, _, alarm, active = static.summarize(
                source, method, result, parameters['calibration'][method], parameters['settings'])
            if not np.array_equal(alarm, saved['alarm']) or not np.array_equal(active, saved['active']):
                raise ValueError(f'{method}: saved alarms differ from frozen replay')
        if (len(candidates) != summaries[method]['candidates'] or
                len(candidates) != len(static.read_json(scene_dir/f'{method}_candidates.json'))):
            raise ValueError(f'{method}: replayed candidate count differs from saved results')
        times = result['time_s']
        if start < times[0] or end > times[-1]:
            raise ValueError(f'{method}: requested window exceeds saved evaluation coverage')
        selected = np.flatnonzero((times >= start) & (times <= end))
        if len(selected) < 2:
            raise ValueError(f'{method}: fewer than two native samples in the display window')
        if (not np.all(result['ready'][selected]) or np.any(result['reset'][selected])
                or len(set(result['interval'][selected])) != 1):
            raise ValueError(f'{method}: selected history contains a reset or unobservable gap')
        threshold = summary['threshold_s']
        pair_ids = result['tile_id'][result['winner_pair'][selected]]
        traces[method] = dict(time_s=times[selected], source_indexes=selected,
            from_rgb_onset_ms=(times[selected]-onset)*1000,
            score_s=result['score'][selected], normalized=result['score'][selected]/threshold,
            alarm=alarm[selected], active=active[selected], winner_pair_tile_ids=pair_ids)
        sensors[method] = dict(threshold_s=threshold, samples=len(selected),
            full_evaluation_candidates=len(candidates), candidates=candidates,
            first_candidate_minus_onset_ms=summary['first_candidate_minus_onset_ms'],
            full_evaluation_from_rgb_onset_ms=[(float(times[0])-onset)*1000, (float(times[-1])-onset)*1000],
            full_evaluation_max_score_ratio=summary['max_score_s']/threshold,
            source_median_step_ms=float(np.median(np.diff(times)))*1000)
    # Guard against changes while reading/replaying the inputs.
    for name, expected in manifest['evaluation_source_sha256'].items():
        if static.digest(folder/name) != expected:
            raise ValueError(f'static analysis changed during export: {name}')
    report = dict(session=session, condition=source['condition'], transmitter_limit=source['transmitter_limit'],
        rgb_onset_relative_s=onset, before_ms=before_ms, after_ms=after_ms, sensors=sensors,
        frozen_manifest_sha256=manifest['frozen_manifest_sha256'],
        evaluation_source_sha256=manifest['evaluation_source_sha256'],
        score_definition='Maximum over all adjacent pairs of the minimum of their two integrated tile states',
        y_axis='Detector score / sensor-specific frozen threshold',
        purpose='Illustration of this selected static recording; not an aggregate performance result',
        limitations=['RGB and EVS may select different adjacent pairs, and the winning pair may change over time.',
                    'Background models and thresholds are sensor-specific; the normalized score is not a probability.',
                    'RGB onset is an annotated frame time, not a measured physical appearance time.',
                    'Native sample times are retained; lines do not imply measurements between RGB frames.',
                    'Candidate times are offline detector outputs, not measured processing or braking latencies.'])
    return traces, report


def draw(output, traces, report):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    colors = dict(rgb='#245e91', evs='#c45800')
    fig, ax = plt.subplots(figsize=(8.5, 4.7), facecolor='white')
    for method, trace in traces.items():
        ax.plot(trace['from_rgb_onset_ms'], trace['normalized'], color=colors[method],
                lw=2.3, marker='o' if method == 'rgb' else None, ms=3.5,
                label=method.upper())
        alarms = np.flatnonzero(trace['alarm'])
        ax.scatter(trace['from_rgb_onset_ms'][alarms], trace['normalized'][alarms],
                   s=75, facecolor='white', edgecolor=colors[method], linewidth=2, zorder=5)
        for index in alarms:
            ax.vlines(trace['from_rgb_onset_ms'][index], 0, trace['normalized'][index],
                      color=colors[method], ls=':', lw=1.1)
    ax.axhline(1., color='#697986', ls=(0, (4, 3)), lw=1.1)
    ax.axvline(0., color='#aab8c2', lw=.8, ls=':', zorder=0)
    ymax = max(1.2, max(float(t['normalized'].max()) for t in traces.values())*1.10)
    lo, hi = -report['before_ms'], report['after_ms']
    tick_step = 50. if hi-lo >= 200 else 20.
    ticks = np.arange(np.ceil(lo/tick_step)*tick_step, hi+1e-9, tick_step)
    ax.set(xlim=(lo, hi), ylim=(-.025*ymax, ymax), xticks=ticks)
    ax.set_xlabel('Time from RGB onset [ms]', fontsize=12, labelpad=9)
    ax.set_ylabel('Detection score / threshold', fontsize=12, labelpad=9)
    ax.spines[['top', 'right']].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color('#758898')
    ax.tick_params(labelsize=11, colors='#24374b', length=4)
    ax.grid(axis='y', color='#e6ebef', lw=.6)
    ax.set_axisbelow(True)
    ax.legend(loc='lower left', bbox_to_anchor=(0, 1.02), ncol=2, frameon=False,
              fontsize=11, handlelength=2.7, columnspacing=1.6)
    fig.tight_layout(pad=1.5)
    for ext in ('png', 'svg'):
        path = output/f'traces_overlay.{ext}'
        fig.savefig(path, dpi=240, facecolor='white')
        if ext == 'svg':
            path.write_text('\n'.join(line.rstrip() for line in path.read_text().splitlines())+'\n')
    plt.close(fig)
    return dict(colors=colors, threshold_line=1., x_ticks_ms=ticks.tolist(),
        x_limits_ms=[lo, hi], y_limits=[-.025*ymax, ymax],
        hollow_circles='All saved candidate starts inside the requested display window',
        rgb_dots='Native RGB samples; no temporal upsampling')


def export(output, traces, report):
    if output.exists():
        raise ValueError('output already exists; choose a new directory')
    import matplotlib  # Validate dependency before creating output.
    output.mkdir(parents=True)
    report['appearance'] = draw(output, traces, report)
    for method, trace in traces.items():
        np.savez_compressed(output/f'{method}_trace_values.npz', **trace)
        with (output/f'{method}_trace_values.csv').open('w', newline='') as stream:
            writer = csv.writer(stream, lineterminator='\n')
            writer.writerow(['relative_time_s', 'from_rgb_onset_ms', 'score_s', 'score_over_threshold',
                             'candidate_start', 'active', 'winner_tile_a', 'winner_tile_b'])
            for i, timestamp in enumerate(trace['time_s']):
                writer.writerow([timestamp, trace['from_rgb_onset_ms'][i], trace['score_s'][i],
                    trace['normalized'][i], bool(trace['alarm'][i]), bool(trace['active'][i]),
                    *trace['winner_pair_tile_ids'][i]])
    report.update(status='complete', renderer_sha256=static.digest(__file__),
                  outputs_sha256={p.name: static.digest(p) for p in output.iterdir() if p.is_file()})
    static.write_json(output/'summary.json', report)
    rows = []
    for method, sensor in report['sensors'].items():
        for c in sensor['candidates']:
            rows.append(f'<tr><td>{method.upper()}</td><td>{c["candidate"]}</td>'
                f'<td>{c["minus_onset_ms"]:+.3f}</td><td>{c["pair_tile_a"]}, {c["pair_tile_b"]}</td></tr>')
    (output/'index.html').write_text('<!doctype html><meta charset="utf-8">'
        '<title>Static RGB / EVS detector traces</title>'
        '<style>body{font:18px sans-serif;color:#24374b;margin:32px;max-width:1100px}img{width:100%}td,th{padding:6px 16px}</style>'
        f'<h1>{html.escape(report["session"])}：RGB・EVSの判定スコア</h1>'
        '<img src="traces_overlay.png">'
        '<p>青：RGB、橙：EVS。横軸は注釈済みRGB初出現を0 msとする時間。縦軸は各センサの判定スコアを、'
        'それぞれの固定しきい値で割った値。水平破線の1が候補を出すしきい値です。</p>'
        '<p>小さな青点はRGBの実サンプル。白抜きの丸と色付き縦線は保存済み候補開始時刻です。'
        '候補時刻が表示区間外なら丸を表示しません。</p>'
        '<p>各隣接2区画の積算値の小さい方をペアのスコアとし、全ペアの最大値を描いています。'
        'RGBとEVSは別の区画を選ぶ場合があり、同じ固定2区画の履歴を表す図ではありません。</p>'
        '<table><tr><th>センサ</th><th>候補</th><th>初出現から [ms]</th><th>区画ID</th></tr>'
        +''.join(rows)+'</table>'
        '<p>モデル・しきい値・同期・注釈は解析時の条件を保持。元のサンプル間を線で結んでおり、RGBの時間分解能を補間で増やしてはいません。'
        '特定の1試行の説明用で、全試行の平均やセンサ固有の遅延を示すものではありません。</p>'
        '<p><a href="traces_overlay.svg">ポスター用 SVG</a> / '
        '<a href="summary.json">数値・出典</a></p>', encoding='utf-8')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--static-dir', type=Path, required=True)
    parser.add_argument('--frozen-dir', type=Path, default=static.DEFAULT_FROZEN)
    parser.add_argument('--session', default='popout-0928-static-100_01')
    parser.add_argument('--before-ms', type=float, default=100.)
    parser.add_argument('--after-ms', type=float, default=500.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError('output already exists; choose a new directory')
        print(f'{args.session}: checking static inputs and reproducing frozen scores...', flush=True)
        traces, report = prepare(args.static_dir, args.frozen_dir, args.session, args.before_ms, args.after_ms)
        export(args.output, traces, report)
        for method, sensor in report['sensors'].items():
            delta = sensor['first_candidate_minus_onset_ms']
            text = 'none' if delta is None else f'{delta:+.3f} ms'
            print(f'{method.upper()}: first candidate={text}; samples={sensor["samples"]}')
        print(f'Saved: {args.output}/traces_overlay.png\nReport: {args.output}/index.html')
        return 0
    except (OSError, ValueError, KeyError, TypeError, RuntimeError, ImportError) as error:
        print(f'error: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
