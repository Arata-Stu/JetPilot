"""Re-segment existing activity scores by recorded vehicle commands, without changing annotations."""
import argparse
import csv
import html
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws/src/tool/multi_sensor_calibration'))
from rc_popout_detection import digest, evaluation_intervals, write_plot


def command_state(throttle, brake, reverse):
    values = (throttle, brake, reverse)
    if any(not math.isfinite(x) or not 0 <= x <= 1 for x in values):
        raise ValueError('invalid vehicle command')
    if reverse > 1e-4 or (throttle > 1e-4 and brake > 1e-4):
        return 'other'
    return 'drive' if throttle > 1e-4 else 'brake' if brake > 1e-4 else 'neutral'


def intersect(spans, start, end):
    return [(max(a, start), min(b, end)) for a, b in spans if max(a, start) < min(b, end)]


def find_phases(commands, spans, pre_seconds, max_gap):
    """Require a bounded, observed command run. Never substitute configured duration."""
    if len(commands) < 2:
        raise ValueError('vehicle command topic missing or too short')
    if any(b[0] <= a[0] for a, b in zip(commands, commands[1:])):
        raise ValueError('vehicle command timestamps are not strictly increasing')
    states = [command_state(*row[1:]) for row in commands]
    runs = []
    i = 0
    while i < len(commands):
        j = i + 1
        while j < len(commands) and states[j] == states[i]:
            j += 1
        runs.append((states[i], i, j))
        i = j
    drives = [(i, j) for state, i, j in runs if state == 'drive'
              and intersect(spans, commands[i][0], commands[j][0] if j < len(commands) else commands[-1][0])]
    if len(drives) != 1:
        raise ValueError(f'expected one drive run in evaluation, found {len(drives)}')
    i, j = drives[0]
    if i == 0 or states[i-1] != 'neutral' or j == len(commands):
        raise ValueError('neutral-before-drive or end transition was not observed')
    start, drive_end = commands[i][0], commands[j][0]
    if not any(a <= start < b for a, b in spans):
        raise ValueError('drive start is outside the annotated evaluation interval')
    phases = [('pre_drive', start-pre_seconds, start), ('drive', start, drive_end)]
    end_index = j
    if states[j] == 'brake':
        k = j + 1
        while k < len(commands) and states[k] == 'brake':
            k += 1
        if k == len(commands) or states[k] != 'neutral':
            raise ValueError('neutral after braking was not observed')
        phases.append(('brake', drive_end, commands[k][0]))
        end_index = k
    elif states[j] != 'neutral':
        raise ValueError('drive ended with an unsupported command')
    if any(commands[k+1][0]-commands[k][0] > max_gap for k in range(i-1, end_index)):
        raise ValueError('command gap around driving/braking exceeds limit')
    pre_begin = start-pre_seconds
    before = [k for k in range(i) if commands[k][0] <= pre_begin]
    pre_ok = bool(before)
    if before:
        k0 = before[-1]
        pre_ok = (all(s == 'neutral' for s in states[k0:i])
                  and all(commands[k+1][0]-commands[k][0] <= max_gap for k in range(k0, i)))
    result = []
    for name, a, b in phases:
        active = intersect(spans, a, b)
        duration = sum(y-x for x, y in active)
        result.append(dict(phase=name, start_s=a, end_s=b, start_from_drive_s=a-start,
                           end_from_drive_s=b-start, requested_seconds=b-a,
                           evaluation_seconds=duration, intervals=active,
                           complete=math.isclose(duration, b-a, abs_tol=1e-6) and (name != 'pre_drive' or pre_ok),
                           command_coverage=(name != 'pre_drive' or pre_ok)))
    return start, result


def phase_scores(rows, phase, sensor, window_s, max_gap):
    """Keep only scores whose observation window lies fully inside this phase."""
    selected = []
    for i, (interval, t, score) in enumerate(rows):
        if sensor == 'rgb':
            if i == 0 or rows[i-1][0] != interval:
                continue  # The first frame-pair start is not present in legacy CSVs.
            support_start = rows[i-1][1]
            if t-support_start > max_gap:
                continue
        else:
            support_start = t-window_s
        for segment, (a, b) in enumerate(phase['intervals']):
            if a <= support_start and t < b:
                # Causal hold, capped at the allowed sample gap; missing time is not silence.
                next_t = rows[i+1][1] if i+1 < len(rows) and rows[i+1][0] == interval else t
                stop = min(b, next_t, t+max_gap)
                selected.append((segment, t, score, max(0.0, stop-t)))
                break
    return selected


def statistics(rows, threshold, duration, np):
    values = [r[2] for r in rows]
    observed = sum(r[3] for r in rows)
    high_seconds = sum(r[3] for r in rows if r[2] >= threshold)
    episodes = 0
    previous = None
    for segment, t, score, held in rows:
        high = score >= threshold
        contiguous = previous is not None and segment == previous[0] and t <= previous[1]+previous[3]+1e-8
        if high and (not contiguous or previous[2] < threshold):
            episodes += 1
        previous = (segment, t, score, held)
    dist = dict(zip(('p50','p95','p99','p99_9','max'), map(float, np.percentile(values,[50,95,99,99.9,100])))) if values else {}
    return dict(samples=len(rows), observed_score_seconds=observed,
                threshold=threshold, above_threshold_seconds=high_seconds,
                above_threshold_fraction=high_seconds/observed if observed else None,
                episodes=episodes, episodes_per_second=episodes/observed if observed else None,
                score_coverage_fraction=observed/duration if duration else None, **dist)


def analyze_scene(folder, pre_seconds, max_command_gap, np):
    from multi_sensor_calibration.rosbag import iter_messages, selected_time_ns
    result = json.loads((folder/'result.json').read_text())
    ann_path = Path(result['annotation'])
    ann = json.loads(ann_path.read_text())
    if digest(ann_path) != result['annotation_sha256']:
        raise ValueError('annotations changed since score generation; regenerate activity scores')
    if digest(ann['time_sync']) != result['time_sync_sha256']:
        raise ValueError('time sync changed since score generation')
    spans = evaluation_intervals(ann)
    origin = ann['reference_origin_s']
    session = ann_path.parents[3]/ann['session']
    source = ann.get('rgb_timestamp_source','bag')
    commands = []
    for item in iter_messages(session, {'/vehicle/control_cmd'}):
        msg = item.message
        commands.append((selected_time_ns(item, source)/1e9-origin,
                         float(msg.throttle),float(msg.brake),float(msg.reverse)))
    zero, phases = find_phases(commands, spans, pre_seconds, max_command_gap)
    config = json.loads((folder.parent/'run_config.json').read_text())['parameters']
    window_s = config['step_ms']*config['window_bins']/1000
    methods = {}
    summaries = []
    for sensor in ('rgb','evs','evs_spatial'):
        path = folder/f'{sensor}_scores.csv'
        if not path.is_file():
            if sensor != 'evs_spatial':
                raise ValueError(f'missing scores: {path}')
            continue
        with path.open() as stream:
            rows = [(int(r['interval']),float(r['relative_time_s']),float(r['score'])) for r in csv.DictReader(stream)]
        if any(not math.isfinite(t) or not math.isfinite(v) for _,t,v in rows):
            raise ValueError('nonfinite activity scores')
        if any(b[1] <= a[1] for a,b in zip(rows,rows[1:])):
            raise ValueError('activity times are not increasing')
        methods[sensor] = rows
        for phase in phases:
            selected = phase_scores(rows, phase, sensor, window_s,
                                    0.1 if sensor == 'rgb' else config['step_ms']/1000*1.5)
            summaries.append(dict(session=ann['session'], phase=phase['phase'], method=sensor,
                                  complete=phase['complete'], command_coverage=phase['command_coverage'],
                                  evaluation_seconds=phase['evaluation_seconds'],
                                  start_from_drive_s=phase['start_from_drive_s'], end_from_drive_s=phase['end_from_drive_s'],
                                  **statistics(selected,result[sensor]['threshold'],phase['evaluation_seconds'],np)))
    drive = next(p for p in phases if p['phase'] == 'drive')
    throttle_values = [thr for t,thr,_,_ in commands if drive['start_s'] <= t < drive['end_s']]
    onset = result.get('rgb_first_visible_s')
    alignment = dict(session=ann['session'], drive_start_s=zero, command_topic='/vehicle/control_cmd',
                     timestamp_source=source, reference_origin_s=origin, phases=phases,
                     drive_throttle_min=min(throttle_values), drive_throttle_max=max(throttle_values),
                     rgb_first_visible_from_drive_s=None if onset is None else onset-zero,
                     semantics='command phases, not measured vehicle motion; score windows crossing phase boundaries excluded',
                     input_result_sha256=digest(folder/'result.json'), annotation_sha256=result['annotation_sha256'])
    return result, alignment, summaries, methods, commands


def main():
    import numpy as np
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scores-dir', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--pre-drive-s', type=float, default=1.0)
    parser.add_argument('--max-command-gap-s', type=float, default=0.1)
    args = parser.parse_args()
    if any(not math.isfinite(x) or x <= 0 for x in (args.pre_drive_s,args.max_command_gap_s)):
        parser.error('durations must be finite and positive')
    if args.output.exists():
        parser.error('output already exists; choose a new directory')
    folders = sorted(p.parent for p in args.scores_dir.glob('*/result.json'))
    if not folders:
        parser.error('no per-session result.json found')
    args.output.mkdir(parents=True)
    (args.output/'run_config.json').write_text(json.dumps(dict(scores_dir=str(args.scores_dir.resolve()),
        pre_drive_s=args.pre_drive_s,max_command_gap_s=args.max_command_gap_s,
        timing='same bag/header source as RGB annotations',
        occupancy='causal score hold until next sample, capped; divide by observed_score_seconds',
        rgb_max_sample_gap_s=0.1),indent=2))
    summaries, report = [], []
    for folder in folders:
        try:
            result, alignment, stats, methods, commands = analyze_scene(folder,args.pre_drive_s,args.max_command_gap_s,np)
            out = args.output/folder.name
            out.mkdir()
            zero = alignment['drive_start_s']
            (out/'alignment.json').write_text(json.dumps(alignment,indent=2))
            with (out/'commands.csv').open('w') as stream:
                writer=csv.writer(stream);writer.writerow(['relative_time_s','from_drive_s','throttle','brake','reverse'])
                writer.writerows((t,t-zero,thr,brk,rev) for t,thr,brk,rev in commands)
            for sensor, rows in methods.items():
                with (out/f'{sensor}_aligned_scores.csv').open('w') as stream:
                    writer=csv.writer(stream);writer.writerow(['interval','relative_time_s','from_drive_s','phase','score'])
                    for interval,t,score in rows:
                        phase=next((p['phase'] for p in alignment['phases'] if any(a<=t<b for a,b in p['intervals'])),'outside_phases')
                        writer.writerow([interval,t,t-zero,phase,score])
            # Plot the fixed pre-drive plus drive/brake interval on a start-aligned axis.
            a,b=alignment['phases'][0]['start_s'],alignment['phases'][-1]['end_s']
            plot_result=dict(result,intervals=[(x-zero,y-zero) for x,y in intersect(result['intervals'],a,b)],
                             rgb_first_visible_s=alignment['rgb_first_visible_from_drive_s'],
                             time_axis_label='Time from drive command',
                             phase_markers=[dict(time_s=p['start_from_drive_s'],label=p['phase']) for p in alignment['phases']]
                             + [dict(time_s=alignment['phases'][-1]['end_from_drive_s'],label='neutral')])
            plot_rows={s:[(i,t-zero,v) for i,t,v in rows if a<=t<b] for s,rows in methods.items()}
            if plot_rows['rgb'] and plot_rows['evs']:
                write_plot(out/'aligned_scores.svg',plot_result,plot_rows['rgb'],plot_rows['evs'])
            summaries.extend(stats)
            report.append(dict(session=folder.name,status='complete',phases=alignment['phases']))
            print(f'{folder.name}: complete; drive_start={zero:.6f}s',flush=True)
        except (ValueError,OSError,KeyError,RuntimeError) as exc:
            report.append(dict(session=folder.name,status='failed',error=str(exc)))
            print(f'{folder.name}: FAILED: {exc}',flush=True)
    fields=['session','phase','method','complete','command_coverage','evaluation_seconds','start_from_drive_s','end_from_drive_s',
            'samples','observed_score_seconds','score_coverage_fraction','threshold','above_threshold_seconds',
            'above_threshold_fraction','episodes','episodes_per_second','p50','p95','p99','p99_9','max']
    with (args.output/'phase_summary.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(summaries)
    (args.output/'summary.json').write_text(json.dumps(report,indent=2))
    links=''.join(f'<li>{html.escape(r["session"])}: '+(f'<a href="{html.escape(r["session"],quote=True)}/aligned_scores.svg">発進基準の波形</a>' if r['status']=='complete' else html.escape(r['error']))+'</li>' for r in report)
    (args.output/'index.html').write_text('<meta charset="utf-8"><h1>車両指令による区間別解析</h1><p>時刻0はスロットル指令開始。実測運動とは区別します。区間の詳細はalignment.json、集計はphase_summary.csvを参照。</p><ul>'+links+'</ul>')
    return int(any(r['status']=='failed' for r in report))


if __name__=='__main__':
    raise SystemExit(main())
