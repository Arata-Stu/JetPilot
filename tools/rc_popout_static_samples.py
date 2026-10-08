"""Presentation stills and event-timeline movies from saved static annotations.

No detector fitting, inference, annotation edits, or automatic sync selection.
Use the source calibration and sync pinned by the completed static analysis.
"""
import argparse
import bisect
import csv
import hashlib
import html
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
CALIBRATION = ROOT/'ros2_ws/src/tool/multi_sensor_calibration'
PREFIX = 'popout-0928-static-100'
SESSIONS = [PREFIX] + [f'{PREFIX}_{i:02d}' for i in range(1, 6)]


def read_json(path):
    return json.loads(Path(path).read_text())


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def same_matrix(a, b):
    return (isinstance(a, list) and isinstance(b, list) and len(a) == len(b)
            and all(same_matrix(x, y) if isinstance(x, list) else
                    finite(x) and finite(y) and math.isclose(x, y, rel_tol=1e-7, abs_tol=1e-7)
                    for x, y in zip(a, b)))


def prepare(static_dir, sessions, before_s, after_s):
    run = read_json(static_dir/'run_config.json')
    plan = read_json(static_dir/'preflight.json')
    if run.get('status') != 'complete' or run.get('preflight_sha256') != digest(static_dir/'preflight.json'):
        raise ValueError('static analysis must be complete and preflight.json unchanged')
    spatial = plan['input_definition']['spatial']
    if (spatial['view_frame'] != 'evs' or spatial['projection'] != 'rotation-only'
            or spatial['output_size'] != [640, 480]):
        raise ValueError('expected the saved 640x480 EVS rotation-only geometry')
    if digest(spatial['camchain']) != spatial['camchain_sha256']:
        raise ValueError('calibration changed since static analysis')
    scenes = {s['session']: s for s in plan['scenes']}
    result = []
    for session in sessions:
        source = scenes[session]
        expected_condition = 'left' if SESSIONS.index(session) < 3 else 'right'
        if source['transmitter_limit'] != '100pct' or source['condition'] != expected_condition:
            raise ValueError(f'{session}: static 100% population/condition mismatch')
        identity = source['source_identity']
        annotation = Path(source['annotation'])
        ann = read_json(annotation)
        if digest(annotation) != identity['annotation_sha256'] or ann['session'] != session:
            raise ValueError(f'{session}: annotation changed since static analysis')
        sync = Path(ann['time_sync'])
        if digest(sync) != identity['time_sync_sha256'] or ann['time_sync_sha256'] != identity['time_sync_sha256']:
            raise ValueError(f'{session}: pinned sync changed; do not substitute manual/auto sync')
        origin, onset = source['reference_origin_s'], source['rgb_first_visible_s']
        if not all(finite(x) for x in (origin, onset)):
            raise ValueError(f'{session}: finite RGB onset/origin required')
        if (abs(ann['reference_origin_s']-origin) > 1e-6
                or abs(ann['onset']['first_visible']['rgb_time_s']-origin-onset) > 1e-6):
            raise ValueError(f'{session}: saved onset/origin disagree with annotation')
        if ann.get('rgb_timestamp_source', 'bag') != 'bag':
            raise ValueError(f'{session}: this static analysis uses bag timestamps')
        start, end = onset-before_s, onset+after_s
        if start < 0 or not any(a <= start and end <= b for a, b in source['intervals']):
            raise ValueError(f'{session}: requested clip exceeds the annotated evaluation interval')
        bag = Path(identity['recording_directory'])
        raw = list(bag.glob('*.raw'))
        if len(raw) != 1 or not list(bag.glob('*.mcap')):
            raise ValueError(f'{session}: exactly one RAW and MCAP recording required: {bag}')
        for item in identity['small_files']:
            if digest(item['path']) != item['sha256']:
                raise ValueError(f"source metadata changed: {item['path']}")
        for item in identity['recording_files']:
            if Path(item['path']).stat().st_size != item['bytes']:
                raise ValueError(f"source recording byte size changed: {item['path']}")
        result.append(dict(session=session, condition=expected_condition, bag=str(bag),
            raw=str(raw[0]), annotation=str(annotation), annotation_sha256=digest(annotation),
            time_sync=str(sync), time_sync_sha256=digest(sync), reference_origin_s=origin,
            rgb_onset_s=onset, start_s=start, duration_s=before_s+after_s,
            source_identity=identity))
    return spatial, result


def command(scene, spatial, output, step_ms, fps):
    return [sys.executable, '-m', 'multi_sensor_calibration.cli', 'scenario-overlay',
        '--bag', scene['bag'], '--event-file', scene['raw'], '--time-sync', scene['time_sync'],
        '--camchain', spatial['camchain'], '--view-frame', 'evs', '--projection', 'rotation-only',
        '--rgb-timestamp-source', 'bag', '--timeline', 'event', '--step-ms', str(step_ms),
        '--event-window-ms', '2', '--event-window-position', 'before', '--event-dilate-px', '1',
        '--fps', str(fps), '--start-s', str(scene['start_s']), '--duration-s', str(scene['duration_s']),
        '--output-dir', str(output/'video')]


def validate_render(folder, scene, spatial, step_ms, fps):
    summary = read_json(folder/'summary.json')
    expected = dict(view_frame='evs', output_size=[640, 480], projection='rotation-only',
        timeline='event', step_ms=step_ms, fps=fps, event_window_ms=2., event_window_position='before',
        rgb_timestamp_source='bag', rgb_display_policy='latest_at_or_before',
        time_sync_sha256=scene['time_sync_sha256'], camchain_sha256=spatial['camchain_sha256'])
    for key, value in expected.items():
        if summary.get(key) != value:
            raise ValueError(f'{key}: expected={value!r}, actual={summary.get(key)!r}')
    for key in ('rgb_to_view_homography', 'event_to_view_homography'):
        if not same_matrix(summary.get(key), spatial[key]):
            raise ValueError(f'{key}: rendered geometry differs from saved analysis')
    if digest(scene['time_sync']) != scene['time_sync_sha256'] or digest(spatial['camchain']) != spatial['camchain_sha256']:
        raise ValueError('sync/calibration changed during rendering')
    if abs(summary['reference_origin_s']-scene['reference_origin_s']) > 1e-6:
        raise ValueError('rendered recording origin differs from annotation')
    if summary['truncated']:
        raise ValueError('RAW/RGB ended before the requested clip; do not use incomplete samples')
    with (folder/'frames.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != summary['rendered_frames'] or not rows:
        raise ValueError('frame manifest/count mismatch')
    origin = scene['reference_origin_s']
    for i, row in enumerate(rows):
        for key in ('video_time_s', 'reference_time_s', 'relative_time_s', 'rgb_time_s', 'rgb_age_ms'):
            row[key] = float(row[key])
            if not math.isfinite(row[key]):
                raise ValueError('nonfinite frame timestamp')
        expected_time = origin+scene['start_s']+i*step_ms/1000
        if (int(row['frame']) != i or abs(row['reference_time_s']-expected_time) > 1e-6
                or abs(row['relative_time_s']-(row['reference_time_s']-origin)) > 1e-6
                or abs(row['video_time_s']-i/fps) > 1e-6
                or row['rgb_time_s'] > row['reference_time_s']
                or abs(row['rgb_age_ms']-1000*(row['reference_time_s']-row['rgb_time_s'])) > 1e-6):
            raise ValueError('event timeline/causal RGB hold mismatch')
    if rows[-1]['relative_time_s'] < scene['start_s']+scene['duration_s']-step_ms/1000-1e-6:
        raise ValueError('RGB coverage is shorter than requested clip')
    return summary, rows


def select_stills(rows, scene, offsets_ms, step_ms):
    """At onset require the annotated RGB frame, never the preceding held image."""
    selected = []
    t0 = scene['reference_origin_s']+scene['rgb_onset_s']
    times = [r['reference_time_s'] for r in rows]
    for label, offset in zip(('before', 'onset', 'after'), offsets_ms):
        target = t0+offset/1000
        index = bisect.bisect_left(times, target)
        if index == len(rows) or times[index]-target > step_ms/1000+1e-6:
            raise ValueError(f'{label}: no event-timeline sample near requested time')
        row = rows[index]
        if label == 'onset' and abs(row['rgb_time_s']-t0) > 1e-6:
            raise ValueError('annotated first-visible RGB frame is absent from the new timeline')
        selected.append(dict(label=label, requested_offset_ms=offset, frame=row,
            actual_offset_ms=1000*(row['reference_time_s']-t0),
            rgb_offset_ms=1000*(row['rgb_time_s']-t0)))
    return selected


def write_stills(folder, scene, spatial, summary, selected):
    """Render PNGs directly from RGB/RAW, without recompressing video screenshots."""
    import cv2
    import numpy as np
    from multi_sensor_calibration import scenario_overlay as renderer
    from multi_sensor_calibration.io import load_yaml

    chain = load_yaml(spatial['camchain'])
    evs, rgb = renderer._camera(chain, 'cam0', np), renderer._camera(chain, 'cam1', np)
    size = tuple(summary['output_size'])
    maps = [cv2.initUndistortRectifyMap(c['matrix'], c['distortion'], None, c['matrix'], c['size'], cv2.CV_32FC1)
            for c in (evs, rgb)]
    common = cv2.imread(str(folder/'video/common_valid_mask.png'), cv2.IMREAD_GRAYSCALE)
    if common is None or common.shape != (size[1], size[0]):
        raise ValueError('common valid mask missing or wrong size')
    common = common > 0
    origin, _, rgb_times = renderer._selected_rgb_times(scene['bag'], '/realsense/color/image_raw', 'bag',
        start_s=0., duration_s=None, every_n=1, max_frames=None)
    if abs(origin-scene['reference_origin_s']) > 1e-6:
        raise ValueError('RGB origin changed during still extraction')
    refs = [s['frame']['reference_time_s'] for s in selected]
    indices = [bisect.bisect_right(rgb_times, t)-1 for t in refs]
    if any(i < 0 for i in indices) or any(abs(rgb_times[i]-s['frame']['rgb_time_s']) > 1e-6 for i, s in zip(indices, selected)):
        raise ValueError('RAW/RGB still timing differs from rendered timeline')
    rgb_frames = renderer._hold_rgb_frames(renderer._rgb_frames(scene['bag'], '/realsense/color/image_raw', 'bag', sorted(set(indices))), refs)
    events, _ = renderer._event_frames(scene['raw'], scene['time_sync'], refs, 2., 'before')
    panels = []
    for sample, (rgb_time, image), event in zip(selected, rgb_frames, events):
        if (image.shape[1], image.shape[0]) != rgb['size']:
            raise ValueError('RGB dimensions differ from calibration')
        if (event.image.shape[1], event.image.shape[0]) != evs['size']:
            raise ValueError('EVS dimensions differ from calibration')
        row = sample['frame']
        if (event.window.start_us != int(row['event_start_us'])
                or event.window.end_us != int(row['event_end_us'])
                or event.window.event_count != int(row['event_count'])):
            raise ValueError('RAW still event window/count differs from video')
        corrected = cv2.remap(image, maps[1][0], maps[1][1], cv2.INTER_LINEAR)
        rgb_view = cv2.warpPerspective(corrected, np.asarray(summary['rgb_to_view_homography']), size)
        rgb_view[~common] = (35, 35, 35)
        evs_view, event_mask = renderer._polarity_images(event.image, maps[0],
            np.asarray(summary['event_to_view_homography']), size, 1, cv2, np)
        event_mask &= common
        evs_view[~common] = (35, 35, 35)
        overlay = renderer._overlay_rgb(rgb_view, evs_view, event_mask, summary['alpha'], np)
        directory = folder/'stills'/sample['label']
        directory.mkdir(parents=True)
        sample['outputs'] = {}
        column = []
        for name, pixels in (('rgb', rgb_view), ('evs', evs_view), ('overlay', overlay)):
            path = directory/f'{name}.png'
            if not cv2.imwrite(str(path), pixels):
                raise RuntimeError(f'PNG write failed: {path}')
            sample['outputs'][name] = dict(path=str(path.relative_to(folder)), sha256=digest(path))
            panel = cv2.copyMakeBorder(pixels, 44, 0, 0, 0, cv2.BORDER_CONSTANT, value=(250, 250, 250))
            line = f"{sample['label']} | {name.upper()} | t-t0={sample['actual_offset_ms']:+.2f} ms"
            if name == 'rgb':
                line += f" | RGB={sample['rgb_offset_ms']:+.2f} ms"
            cv2.putText(panel, line, (8, 28), cv2.FONT_HERSHEY_SIMPLEX, .43, (20, 20, 20), 1, cv2.LINE_AA)
            column.append(panel)
        panels.append(np.concatenate(column, axis=0))
    if len(panels) != 3:
        raise ValueError('RAW ended before all three stills were rendered')
    sheet = np.concatenate(panels, axis=1)
    if not cv2.imwrite(str(folder/'contact_sheet.png'), sheet):
        raise RuntimeError('contact sheet write failed')


def write_index(output, results, slowdown):
    parts = ['<!doctype html><meta charset="utf-8"><title>静止・プロポ100%のサンプル</title>',
        '<style>body{font:16px sans-serif;margin:24px;max-width:1400px}img{max-width:100%}video{width:80%}section{margin:40px 0}</style>',
        '<h1>自車静止・プロポ100%：出現前／RGB初出現／出現後</h1>',
        f'<p>EVS座標640×480・EVS時間軸。過去2 msのイベント。{slowdown:.2f}倍スロー。RGBは直前フレームを保持し、補間しません。</p>',
        '<p>t0は既存のRGB初出現注釈です。初出現は車体が一部見えた時刻で、車体全体が出た時刻ではありません。PNGは元RGB/RAWから直接生成しています。</p>']
    for result in results:
        name = html.escape(result['session'])
        parts.append(f'<section><h2>{name}</h2>')
        if result['status'] != 'complete':
            parts.append('<p>'+html.escape(result['error'])+'</p></section>')
            continue
        parts += [f'<a href="{name}/contact_sheet.png"><img src="{name}/contact_sheet.png"></a>',
            f'<video controls preload="metadata" src="{name}/video/rgb_vs_overlay.mp4"></video>',
            f'<p><a href="{name}/video/polarity_only.mp4">EVS単独スロー</a> / <a href="{name}/video/overlay_polarity.mp4">重畳スロー</a> / <a href="{name}/summary.json">時刻・生成条件</a></p>']
        for sample in result['stills']:
            label = sample['label']
            links = ' / '.join(f'<a href="{name}/stills/{label}/{kind}.png">{kind}</a>' for kind in ('rgb', 'evs', 'overlay'))
            parts.append(f'<p>{label}：{links}</p>')
        parts.append('</section>')
    (output/'index.html').write_text('\n'.join(parts))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--static-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sessions', nargs='+', choices=SESSIONS, default=SESSIONS)
    parser.add_argument('--before-s', type=float, default=.15)
    parser.add_argument('--after-s', type=float, default=.35)
    parser.add_argument('--sample-offset-ms', type=float, nargs=3, default=[-100., 0., 100.], metavar=('BEFORE', 'ONSET', 'AFTER'))
    parser.add_argument('--step-ms', type=float, default=1.)
    parser.add_argument('--fps', type=float, default=60.)
    parser.add_argument('--preflight', action='store_true', help='Validate inputs and print commands; no ROS/RAW decoding or writes')
    args = parser.parse_args(argv)
    try:
        values = [args.before_s, args.after_s, args.step_ms, args.fps]
        if not all(finite(x) and x > 0 for x in values) or args.step_ms < .001:
            raise ValueError('durations, step and fps must be positive and finite; step >= 0.001 ms')
        offsets = args.sample_offset_ms
        if not all(finite(x) for x in offsets) or not (-args.before_s*1000 <= offsets[0] < offsets[1] == 0 < offsets[2] < args.after_s*1000):
            raise ValueError('still offsets must be before < 0, onset = 0, after > 0, within the clip')
        if args.step_ms*args.fps >= 1000:
            raise ValueError('choose step_ms * fps < 1000 for slow playback')
        if len(set(args.sessions)) != len(args.sessions):
            raise ValueError('duplicate sessions')
        if args.output.exists():
            raise ValueError(f'output exists; choose a new directory: {args.output}')
        spatial, scenes = prepare(args.static_dir.resolve(), args.sessions, args.before_s, args.after_s)
        slowdown = 1000/(args.step_ms*args.fps)
        print(f'Static 100%: {len(scenes)} sessions / EVS coordinates + event timeline / {slowdown:.2f}x slow')
        commands = []
        for scene in scenes:
            cmd = command(scene, spatial, args.output.resolve()/scene['session'], args.step_ms, args.fps)
            commands.append(cmd)
            print(f"{scene['session']} ({scene['condition']}): RGB onset={scene['rgb_onset_s']:.6f}s / pinned {Path(scene['time_sync']).name}")
            if args.preflight:
                print(shlex.join(cmd))
        if args.preflight:
            print('Preflight passed: no RGB/RAW decoded and no files written.')
            return 0
        # Prefer the checkout implementation consistently for CLI and stills.
        sys.path.insert(0, str(CALIBRATION))
        import cv2  # noqa: F401
        import numpy  # noqa: F401
        env = dict(os.environ)
        env['PYTHONPATH'] = str(CALIBRATION)+os.pathsep+env.get('PYTHONPATH', '')
        args.output.mkdir(parents=True)
        write_json(args.output/'run_config.json', dict(settings=vars(args) | {'static_dir': str(args.static_dir), 'output': str(args.output)},
            spatial=spatial, scenes=scenes, script_sha256=digest(__file__),
            renderer_sha256=digest(CALIBRATION/'multi_sensor_calibration/scenario_overlay.py'),
            note='Presentation samples only; no fitting, inference or annotation changes. RAW/MCAP byte sizes checked, not content hashed.'))
        results = []
        for scene, cmd in zip(scenes, commands):
            folder = args.output/scene['session']
            folder.mkdir()
            try:
                print(f"Rendering {scene['session']} ...", flush=True)
                with (folder/'render.log').open('w') as stream:
                    subprocess.run(cmd, stdout=stream, stderr=subprocess.STDOUT, env=env, check=True)
                summary, rows = validate_render(folder/'video', scene, spatial, args.step_ms, args.fps)
                selected = select_stills(rows, scene, offsets, args.step_ms)
                write_stills(folder, scene, spatial, summary, selected)
                for path, expected in ((scene['annotation'], scene['annotation_sha256']),
                                       (scene['time_sync'], scene['time_sync_sha256']),
                                       (spatial['camchain'], spatial['camchain_sha256'])):
                    if digest(path) != expected:
                        raise ValueError(f'input changed during still extraction: {path}')
                result = dict(session=scene['session'], status='complete', condition=scene['condition'],
                    source=scene, render=summary, stills=selected,
                    limitations='Existing RGB onset retained. Rotation-only retains parallax. Offline visualization, not physical latency or new detector evidence.')
                write_json(folder/'summary.json', result)
                print(f"  complete: {len(rows)} frames / PNG 9枚 / contact_sheet.png", flush=True)
            except (OSError, ValueError, KeyError, RuntimeError, cv2.error, subprocess.CalledProcessError) as error:
                result = dict(session=scene['session'], status='failed', error=str(error), log=str(folder/'render.log'))
                print(f"  FAILED: {error}; see {folder/'render.log'}", file=sys.stderr, flush=True)
            results.append(result)
            write_json(args.output/'summary.json', results)
            write_index(args.output, results, slowdown)
        failures = sum(r['status'] != 'complete' for r in results)
        print(f'Report: {args.output}/index.html / sessions={len(results)} failed={failures}')
        return int(bool(failures))
    except (OSError, ValueError, KeyError, TypeError, ImportError) as error:
        print(f'error: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
