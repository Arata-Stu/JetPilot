"""Frame-accurate spatial review clips from saved EVS-coordinate previews.

Use the preview's frames.csv, not recording time as MP4 seek time. Candidate
tiles are fixed reference boxes, not tracks or a replay of alarm duration.
"""
import argparse
import bisect
import csv
import hashlib
import html
import json
import math
from pathlib import Path
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT/'docs/evidence/rc_popout_20260930'
DEFAULT_MANIFEST = EVIDENCE/'development_bundle_analysis_20261006/candidate_review_manifest.json'
SPLIT = EVIDENCE/'development_evaluation_split_v1.json'


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            value.update(chunk)
    return value.hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def same_numbers(a, b):
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(same_numbers(x, y) for x, y in zip(a, b))
    if isinstance(a, (float, int)) and isinstance(b, (float, int)):
        return math.isclose(a, b, rel_tol=1e-7, abs_tol=1e-7)
    return a == b


def load_manifest(path):
    manifest = json.loads(path.read_text())
    if manifest['schema_version'] != 1 or manifest['split_sha256'] != digest(SPLIT):
        raise ValueError('candidate manifest schema/split mismatch')
    split = json.loads(SPLIT.read_text())
    allowed = {s for group in split['groups'] if group['condition'] != 'none' for s in group['development']}
    scenes = manifest['scenes']
    if len(scenes) != len(allowed) or {s['session'] for s in scenes} != allowed:
        raise ValueError('only the four fixed development positive recordings are supported')
    spatial = manifest['spatial']
    if spatial['view_frame'] != 'evs':
        raise ValueError('EVS-coordinate candidate manifest required')
    width, height = spatial['output_size']
    for scene in scenes:
        for key in ('candidate_recording_s', 'candidate_from_drive_s', 'drive_start_s',
                    'rgb_onset_recording_s', 'reference_origin_s'):
            if not math.isfinite(scene[key]):
                raise ValueError('invalid candidate timestamp')
        if abs(scene['candidate_recording_s']-scene['drive_start_s']-scene['candidate_from_drive_s']) > 1e-6:
            raise ValueError('candidate recording/drive time mismatch')
        if len(scene['tiles']) != 2:
            raise ValueError('expected the two tiles at candidate onset')
        for tile in scene['tiles']:
            x, y, w, h = (tile[k] for k in ('x', 'y', 'width', 'height'))
            if any(type(v) is not int for v in (x,y,w,h)) or not (0 <= x < x+w <= width and 0 <= y < y+h <= height):
                raise ValueError('candidate tile outside EVS view')
    return manifest


def read_frames(path, summary):
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    fps, origin = float(summary['fps']), float(summary['reference_origin_s'])
    if not math.isfinite(fps) or fps <= 0 or not math.isfinite(origin) or not rows or len(rows) != summary['rendered_frames']:
        raise ValueError('invalid preview fps/frame count')
    for i, row in enumerate(rows):
        row['frame'] = int(row['frame'])
        for key in ('video_time_s', 'reference_time_s', 'relative_time_s', 'rgb_time_s'):
            row[key] = float(row[key])
            if not math.isfinite(row[key]):
                raise ValueError('nonfinite preview timestamp')
        if (row['frame'] != i or abs(row['video_time_s']-i/fps) > 1e-5
                or abs(row['reference_time_s']-origin-row['relative_time_s']) > 1e-6
                or abs(row['rgb_time_s']-row['reference_time_s']) > 1e-6):
            raise ValueError('preview frame/time mapping mismatch (RGB timeline required)')
        if i and row['relative_time_s'] <= rows[i-1]['relative_time_s']:
            raise ValueError('preview time must increase')
    return rows


def select_span(rows, candidate, before, after):
    times = [r['relative_time_s'] for r in rows]
    if times[0] > candidate-before or times[-1] < candidate+after:
        raise ValueError('preview does not cover the full requested clip')
    lo = bisect.bisect_left(times, candidate-before)
    hi = bisect.bisect_right(times, candidate+after)
    crossing = bisect.bisect_left(times, candidate)
    if not lo < crossing < hi:
        raise ValueError('clip needs frames strictly before and at/after candidate')
    return lo, hi, crossing


def find_preview(root, scene, spatial, before, after):
    annotation_path = root/'analysis/led_sync'/scene['session']/'sequence_annotations.json'
    if digest(annotation_path) != scene['annotation_sha256']:
        raise ValueError(f'{annotation_path}: 注釈が候補解析時から変わっています')
    annotation = json.loads(annotation_path.read_text())
    if annotation['session'] != scene['session']:
        raise ValueError('annotation session mismatch')
    manual = annotation_path.parent/'time_sync_led.yaml'
    sync = manual if manual.exists() or manual.is_symlink() else annotation_path.parent/'time_sync_led_auto.yaml'
    if digest(sync) != scene['time_sync_sha256']:
        raise ValueError(f'{sync}: 同期が候補解析時から変わっています。候補の再解析が必要です')
    parent = root/'analysis/scenario_overlay'/scene['session']
    valid, errors = [], []
    for path in sorted(parent.glob('*/summary.json')):
        try:
            summary = json.loads(path.read_text())
            for key in ('view_frame', 'output_size', 'projection', 'depth_m', 'camchain_sha256',
                        'rgb_to_view_homography', 'event_to_view_homography'):
                if key not in summary or not same_numbers(summary[key], spatial[key]):
                    raise ValueError(f'coordinate/calibration mismatch: {key}')
            if (summary['time_sync_sha256'] != scene['time_sync_sha256']
                    or abs(summary['reference_origin_s']-scene['reference_origin_s']) > 1e-6
                    or summary.get('rgb_timestamp_source', 'bag') != scene['rgb_timestamp_source']
                    or Path(summary['bag']).name != scene['session']):
                raise ValueError('sync, recording origin or timestamp source mismatch')
            if summary.get('timeline', 'rgb') != 'rgb':
                raise ValueError(f"RGB-timeline preview required; timeline={summary.get('timeline')!r}, "
                                 f"truncated={summary.get('truncated', False)!r}")
            if (not math.isfinite(summary['event_window_ms']) or summary['event_window_ms'] <= 0
                    or summary['event_window_position'] not in ('before', 'center', 'after')):
                raise ValueError('invalid preview event window')
            if not (path.parent/'rgb_vs_overlay.mp4').is_file():
                raise ValueError('comparison video missing')
            frames = read_frames(path.parent/'frames.csv', summary)
            # A RAW stream can finish before the RGB recording, so a complete
            # usable prefix may legitimately have truncated=true. Require the
            # requested clip's coverage, not the unneeded recording tail.
            span = select_span(frames, scene['candidate_recording_s'], before, after)
            valid.append((path.parent, summary, frames, span))
        except (ValueError, KeyError, TypeError, OSError) as error:
            errors.append(f'{path.parent.name}: {error}')
    if not valid:
        detail = '; '.join(errors[:5]) or 'no preview summaries'
        raise ValueError(f'解析時と一致する共通視野動画がありません。generate_rc_popout_common_views.shで {scene["session"]} を生成してください。 '+detail)
    # Prefer the annotated preview; otherwise choose matching coordinates/sync
    # with the highest frame rate. Never select just by modification time.
    valid.sort(key=lambda v: (v[0].name != annotation.get('preview_id'), -v[1]['fps'], v[0].name))
    return dict(folder=valid[0][0], summary=valid[0][1], frames=valid[0][2], span=valid[0][3],
                annotation_path=annotation_path, sync_path=sync)


def decorate(frame, row, scene, manifest, summary, slow, cv2):
    width, height = manifest['spatial']['output_size']
    t = row['relative_time_s']
    delta = t-scene['candidate_recording_s']
    color = (0, 150, 255) if delta >= 0 else (255, 220, 0)
    output = cv2.copyMakeBorder(frame, 0, 128, 0, 0, cv2.BORDER_CONSTANT, value=(24,24,24))
    for shift in (0, width):
        for tile in scene['tiles']:
            x, y, w, h = (tile[k] for k in ('x', 'y', 'width', 'height'))
            cv2.rectangle(output, (shift+x,y), (shift+x+w-1,y+h-1), color, 2)
    method = scene.get('method', 'evs')
    onset = scene['rgb_onset_recording_s']
    onset_label = ('RGB onset=none' if onset is None else
                   f'RGB onset={onset:.6f}s | From onset={1000*(t-onset):+.2f}ms')
    detector_label = (f"past {manifest['detector_window_ms']:g}ms" if method == 'evs'
                      else 'consecutive RGB frame pair')
    text = [
        f"{scene['session']} | {method.upper()} candidate {scene.get('candidate_id', 1)} | Fixed tiles {','.join(str(t['tile_id']) for t in scene['tiles'])} | Playback {1/slow:g}x",
        f"Recording t={t:.6f}s | From drive={t-scene['drive_start_s']:.6f}s | From candidate={1000*delta:+.2f}ms",
        f"Candidate={scene['candidate_recording_s']:.6f}s | {onset_label}",
        f"Preview EVS: {summary['event_window_ms']:g}ms / {summary['event_window_position']} | Detector: {detector_label}",
        'Fixed reference boxes, NOT tracking/alarm duration. Cyan: before candidate; orange: at/after candidate.',
    ]
    for n, line in enumerate(text):
        cv2.putText(output, line, (10, height+22+n*23), cv2.FONT_HERSHEY_SIMPLEX, .48, (240,240,240), 1, cv2.LINE_AA)
    return output


def render(scene, manifest, plan, destination, slow, ffmpeg, *, require_onset=True):
    import cv2
    import numpy as np
    source = plan['folder']/'rgb_vs_overlay.mp4'
    summary, rows = plan['summary'], plan['frames']
    lo, hi, crossing = plan['span']
    width, height = manifest['spatial']['output_size']
    fps = summary['fps']/slow
    cap = cv2.VideoCapture(str(source))
    writer = None
    with tempfile.TemporaryDirectory(prefix='.candidate-', dir=destination.parent) as tmp:
        folder = Path(tmp)
        try:
            if not cap.isOpened():
                raise ValueError(f'cannot decode {source}')
            if (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) != width*2
                    or int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) != height
                    or int(round(cap.get(cv2.CAP_PROP_FRAME_COUNT))) != len(rows)
                    or not math.isclose(cap.get(cv2.CAP_PROP_FPS), summary['fps'], rel_tol=.002)):
                raise ValueError('decoded video dimensions/frame count/fps differ from summary')
            # Some H.264/OpenCV combinations report the requested frame after a
            # seek but return the next frame's pixels. Decode from the beginning
            # so CSV row indices remain tied to sequentially decoded frames.
            for skipped in range(lo):
                ok, _ = cap.read()
                if not ok or abs(cap.get(cv2.CAP_PROP_POS_FRAMES)-(skipped+1)) > .1:
                    raise ValueError(f'video decode/frame index mismatch before clip at {skipped}')
            temporary = folder/'intermediate.mp4'
            writer = cv2.VideoWriter(str(temporary), cv2.VideoWriter_fourcc(*'mp4v'), fps, (width*2,height+128))
            if not writer.isOpened():
                raise ValueError('cannot open MP4 writer')
            times = [r['relative_time_s'] for r in rows]
            onset = scene['rgb_onset_recording_s']
            if require_onset:
                onset_index = -1 if onset is None else bisect.bisect_left(times, onset)
                if not lo < onset_index < hi:
                    raise ValueError('clip must include frames before and at/after RGB onset; increase --before-s/--after-s')
                snapshots = {
                    'before_onset': max(lo, bisect.bisect_right(times, onset-.1)-1),
                    'onset_after': onset_index,
                    'candidate_before': crossing-1, 'candidate_after': crossing,
                }
            else:
                snapshots = {
                    'candidate_minus_100ms': max(lo, bisect.bisect_right(times, scene['candidate_recording_s']-.1)-1),
                    'candidate_before': crossing-1, 'candidate_after': crossing,
                    'candidate_plus_100ms': min(hi-1, bisect.bisect_left(times, scene['candidate_recording_s']+.1)),
                }
            if any(not lo <= i < hi for i in snapshots.values()):
                raise ValueError('clip does not include RGB onset; increase --before-s')
            stills, mapping = {}, []
            for index in range(lo, hi):
                ok, frame = cap.read()
                if not ok or abs(cap.get(cv2.CAP_PROP_POS_FRAMES)-(index+1)) > .1:
                    raise ValueError(f'video decode/frame index mismatch at {index}')
                marked = decorate(frame, rows[index], scene, manifest, summary, slow, cv2)
                writer.write(marked)
                for name, target in snapshots.items():
                    if index == target:
                        stills[name] = marked.copy()
                        if not cv2.imwrite(str(folder/f'{name}.png'), marked):
                            raise ValueError('snapshot write failed')
                mapping.append(dict(output_frame=index-lo, output_video_time_s=(index-lo)/fps,
                    source_frame=index, source_video_time_s=rows[index]['video_time_s'],
                    recording_relative_s=rows[index]['relative_time_s'], reference_time_s=rows[index]['reference_time_s'],
                    candidate_delta_ms=1000*(rows[index]['relative_time_s']-scene['candidate_recording_s']),
                    onset_delta_ms=None if onset is None else 1000*(rows[index]['relative_time_s']-onset)))
        finally:
            cap.release()
            if writer is not None:
                writer.release()
        command = [ffmpeg, '-nostdin', '-v', 'error', '-i', str(temporary), '-an', '-c:v', 'libx264',
                   '-preset', 'veryfast', '-crf', '18', '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
                   '-fps_mode', 'passthrough', str(folder/'candidate_review.mp4')]
        subprocess.run(command, check=True, capture_output=True, text=True)
        temporary.unlink()
        # Decode the encoded result to ensure that frame correspondence survived.
        check = cv2.VideoCapture(str(folder/'candidate_review.mp4'))
        count = 0
        try:
            if not check.isOpened():
                raise ValueError('cannot decode output MP4')
            if not math.isclose(check.get(cv2.CAP_PROP_FPS), fps, rel_tol=.002):
                raise ValueError('output playback rate changed during encoding')
            while True:
                ok, frame = check.read()
                if not ok:
                    break
                if frame.shape[:2] != (height+128, width*2):
                    raise ValueError('output dimensions changed')
                count += 1
        finally:
            check.release()
        if count != hi-lo:
            raise ValueError('output frame count changed during encoding')
        if (digest(plan['annotation_path']) != scene['annotation_sha256']
                or digest(plan['sync_path']) != scene['time_sync_sha256']):
            raise ValueError('annotation or sync changed while rendering')
        panels = []
        for name in snapshots:
            panel = cv2.copyMakeBorder(stills[name], 32, 0, 0, 0, cv2.BORDER_CONSTANT, value=(45,45,45))
            cv2.putText(panel, name, (10,23), cv2.FONT_HERSHEY_SIMPLEX, .6, (255,255,255), 1, cv2.LINE_AA)
            panels.append(panel)
        sheet = np.vstack((np.hstack(panels[:2]), np.hstack(panels[2:])))
        if not cv2.imwrite(str(folder/'contact_sheet.png'), sheet):
            raise ValueError('contact sheet write failed')
        with (folder/'frames.csv').open('w', newline='') as stream:
            csv_writer = csv.DictWriter(stream, fieldnames=list(mapping[0]))
            csv_writer.writeheader(); csv_writer.writerows(mapping)
        report = dict(session=scene['session'], method=scene.get('method', 'evs'),
            candidate_id=scene.get('candidate_id', 1), require_onset=require_onset,
            source_decode_strategy='sequential_from_start',
            status='complete', source_preview=str(plan['folder']),
            source_truncated=bool(summary.get('truncated', False)),
            source_requested_frames=summary.get('requested_frames'), source_rendered_frames=summary['rendered_frames'],
            source_summary_sha256=digest(plan['folder']/'summary.json'), source_frames_sha256=digest(plan['folder']/'frames.csv'),
            source_video_sha256=digest(source), annotation_sha256=digest(plan['annotation_path']),
            time_sync_sha256=digest(plan['sync_path']), candidate=scene, frames=count, fps=fps, slowdown=slow,
            recording_start_s=rows[lo]['relative_time_s'], recording_end_s=rows[hi-1]['relative_time_s'],
            first_at_or_after_candidate=rows[crossing], last_before_candidate=rows[crossing-1],
            first_display_after_candidate_ms=1000*(rows[crossing]['relative_time_s']-scene['candidate_recording_s']),
            max_source_frame_gap_ms=1000*max(rows[i]['relative_time_s']-rows[i-1]['relative_time_s'] for i in range(lo+1,hi)),
            preview_event_window_ms=summary['event_window_ms'], preview_event_window_position=summary['event_window_position'],
            detector_window_ms=manifest['detector_window_ms'] if scene.get('method', 'evs') == 'evs' else None,
            note='Spatial review only. Fixed onset tiles, not tracking. Source RGB cadence and event visualization window retained.')
        write_json(folder/'summary.json', report)
        Path(tmp).rename(destination)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument('--camchain', type=Path, help='Override calibration file location, not its hash')
    parser.add_argument('--before-s', type=float, default=.3)
    parser.add_argument('--after-s', type=float, default=.3)
    parser.add_argument('--slow-factor', type=float, default=4.)
    parser.add_argument('--dry-run', action='store_true', help='Validate current metadata and print clips; no video dependencies')
    args = parser.parse_args(argv)
    try:
        for value in (args.before_s, args.after_s, args.slow_factor):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('clip durations and slow factor must be finite and positive')
        if args.output.exists():
            raise ValueError('出力先が既に存在します。新しい --output を指定してください')
        manifest = load_manifest(args.manifest)
        chain = args.camchain or Path(manifest['spatial']['camchain'])
        if digest(chain) != manifest['spatial']['camchain_sha256']:
            raise ValueError('current calibration differs from candidate analysis')
        plans = []
        for scene in manifest['scenes']:
            plan = find_preview(args.record_root, scene, manifest['spatial'], args.before_s, args.after_s)
            plans.append(plan)
            lo, hi, crossing = plan['span']
            delay = 1000*(plan['frames'][crossing]['relative_time_s']-scene['candidate_recording_s'])
            print(f"{scene['session']}: {plan['folder'].name}, source frames={lo}..{hi-1}, first display=+{delay:.3f}ms", flush=True)
            if plan['summary'].get('truncated', False):
                print('  元動画の末尾は未生成ですが、候補前後の指定区間は含まれています', flush=True)
        if args.dry_run:
            return 0
        ffmpeg = shutil.which('ffmpeg')
        if not ffmpeg:
            raise ValueError('ffmpeg is required (H.264 encoding)')
        import cv2  # Fail before creating an output tree if the runtime is missing.
        args.output.mkdir(parents=True)
        write_json(args.output/'review_manifest.json', manifest)
        write_json(args.output/'run_config.json', dict(manifest_sha256=digest(args.manifest),
            code_sha256=digest(__file__), camchain_sha256=digest(chain), opencv_version=cv2.__version__,
            before_s=args.before_s, after_s=args.after_s, slow_factor=args.slow_factor))
        results = []
        for scene, plan in zip(manifest['scenes'], plans):
            try:
                report = render(scene, manifest, plan, args.output/scene['session'], args.slow_factor, ffmpeg)
                results.append(report)
                print(f"  complete: {scene['session']} / frames={report['frames']}", flush=True)
            except (ValueError, OSError, subprocess.CalledProcessError, cv2.error) as error:
                detail = str(error) + (f' / {error.stderr}' if isinstance(error, subprocess.CalledProcessError) else '')
                results.append(dict(session=scene['session'], status='failed', error=detail))
                print(f"  FAILED: {scene['session']}: {detail}", flush=True)
        write_json(args.output/'summary.json', results)
        page = ['<!doctype html><html lang="ja"><meta charset="utf-8"><title>候補領域の映像確認</title>',
            '<style>body{font:16px sans-serif;max-width:1300px;margin:30px auto;padding:0 16px}video,img{width:100%}section{margin:40px 0}code{background:#eee}</style>',
            '<h1>EVS候補領域の映像確認（調整用4記録）</h1>',
            f'<p>左：RGB、右：RGB＋EVS。前後各{args.before_s:g}／{args.after_s:g}秒、{args.slow_factor:g}倍スロー。</p>',
            '<p>枠は候補開始時の2タイルを前後にも固定表示したものです。車両追跡や警報の継続状態ではありません。水色は候補時刻より前、橙色は候補時刻以降です。</p>',
            '<p>録画基準時刻・候補との時刻差を画面下に表示しています。表示はRGBフレーム刻みです。既存動画のEVS蓄積窓と検知の過去2 ms窓は異なるため、ここでは対象位置を確認してください。</p>']
        for report in results:
            name = html.escape(report['session'])
            if report['status'] != 'complete':
                page.append(f'<section><h2>{name}: failed</h2><pre>{html.escape(report["error"])}</pre></section>')
                continue
            tail_note = ('元動画の末尾は未生成です。ここで使用する候補前後の区間と動画フレームの整合は確認済みです。 '
                         if report['source_truncated'] else '')
            page.append(f'<section><h2>{name}</h2><p>候補時刻から最初の表示フレームまで：+{report["first_display_after_candidate_ms"]:.3f} ms。'
                        f'元動画のEVS表示窓：{report["preview_event_window_ms"]:g} ms / {html.escape(report["preview_event_window_position"])}。{tail_note}</p>'
                        f'<video controls loop preload="metadata" src="{name}/candidate_review.mp4"></video>'
                        f'<p><a href="{name}/frames.csv">フレーム時刻対応表</a> / <a href="{name}/summary.json">入力・候補情報</a></p>'
                        f'<a href="{name}/contact_sheet.png"><img loading="lazy" src="{name}/contact_sheet.png" alt="初出現前後と候補直前直後"></a></section>')
        (args.output/'index.html').write_text('\n'.join(page)+'</html>')
        print(f'Report: {args.output/"index.html"}')
        return int(any(r['status'] != 'complete' for r in results))
    except (ValueError, KeyError, TypeError, OSError, ImportError) as error:
        parser.exit(1, f'error: {error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
