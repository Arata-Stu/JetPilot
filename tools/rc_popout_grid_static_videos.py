"""Spatial review of every saved static-transfer RGB/EVS candidate.

Read recording-time outputs without fitting or running the detector. The frozen
contract, source identities, tile arrays, saved alarms and CSVs must agree.
"""
import argparse
import html
import json
import math
from pathlib import Path
import shlex
import shutil
import subprocess

import numpy as np

import evaluate_rc_popout_grid_static as static
import rc_popout_candidate_videos as video
from rc_popout_grid_evaluation_videos import csv_matches_json
from rc_popout_local_detection import geometry, validate_data


def load_review(folder, frozen_dir):
    run = static.read_json(folder/'run_config.json')
    plan = static.read_json(folder/'preflight.json')
    if (run['protocol'] != static.PROTOCOL or run['status'] != 'complete'
            or run['preflight_sha256'] != video.digest(folder/'preflight.json')
            or run['sessions'] != [r['session'] for r in static.population()]):
        raise ValueError('static run is incomplete or differs from the saved preflight/population')
    fresh, parameters, models, _ = static.preflight(Path(plan['source_config']), frozen_dir)
    if json.loads(json.dumps(fresh)) != plan:
        raise ValueError('static source, annotation, sync, calibration or code changed since inference')
    if (run['adapter_sha256'] != plan['adapter_sha256']
            or run['frozen_runtime_sha256'] != plan['frozen_runtime_sha256']
            or run['frozen_manifest_sha256'] != plan['frozen_manifest_sha256']):
        raise ValueError('static run provenance differs from preflight')
    for name in ('freeze.json', 'detector_parameters.json', 'background_models.json'):
        if video.digest(folder/name) != video.digest(frozen_dir/name):
            raise ValueError(f'static copied frozen file differs: {name}')
    if (static.read_json(folder/'errors.json') != [] or
            static.read_json(folder/'effective_common_detection_roi.json') != plan['effective_config']):
        raise ValueError('static errors or effective configuration mismatch')
    summaries = static.read_json(folder/'summary.json')
    expected = [(s, method) for s in run['sessions'] for method in ('rgb', 'evs')]
    if [(r['session'], r['method']) for r in summaries] != expected:
        raise ValueError('summary must contain all 18 static recordings and both sensors')
    csv_matches_json(folder/'summary.csv', summaries, static.SUMMARY_FIELDS)
    by_key = {(r['session'], r['method']): r for r in summaries}
    hashes = {name: video.digest(folder/name) for name in
              ('run_config.json', 'preflight.json', 'summary.json', 'summary.csv', 'candidates.csv')}
    definition = parameters['input_definition']
    scenes, all_candidates = [], []
    for source in plan['scenes']:
        session = source['session']
        scene_dir = folder/session
        result = static.read_json(scene_dir/'result.json')
        meta = static.read_json(scene_dir/'tiles.json')
        if (result['status'] != 'complete' or result['protocol'] != static.PROTOCOL
                or result['algorithm'] != parameters['algorithm'] or result['scene'] != source
                or result['geometry'] != meta
                or result['source_result'] != static.read_json(scene_dir/'source_result.json')
                or result['parameters_sha256'] != video.digest(folder/'detector_parameters.json')):
            raise ValueError(f'{session}: saved static result/provenance mismatch')
        if (meta['output_size'] != definition['spatial']['output_size'] or meta['roi'] != definition['roi']
                or meta['step_s'] != definition['step_ms']/1000
                or meta['event_window_s'] != definition['step_ms']*definition['window_bins']/1000):
            raise ValueError(f'{session}: saved geometry differs from frozen inputs')
        pairs, _ = geometry(meta, definition['tile_px'])
        valid_pairs = {tuple(int(meta['tiles'][i]['tile_id']) for i in pair) for pair in pairs}
        tiles = {t['tile_id']: t for t in meta['tiles']}
        onset = source['rgb_first_visible_s']
        annotation = Path(source['annotation'])
        ann = static.read_json(annotation)
        for method in ('rgb', 'evs'):
            tile_path = scene_dir/f'{method}_tiles.npz'
            if video.digest(tile_path) != result['tile_arrays_sha256'][method]:
                raise ValueError(f'{session}: {method} tile arrays changed')
            with np.load(tile_path, allow_pickle=False) as data:
                validate_data(data, meta, method)
                static.validate_model(models[method], data, method)
                times, intervals, tile_ids = data['time_s'], data['interval'], data['tile_id']
            candidates = static.read_json(scene_dir/f'{method}_candidates.json')
            summary = by_key[session, method]
            if (summary['status'] != 'complete' or summary['candidates'] != len(candidates)
                    or summary['condition'] != source['condition']
                    or summary['transmitter_limit'] != source['transmitter_limit']
                    or summary['rgb_first_visible_s'] != onset
                    or summary['threshold_s'] != parameters['calibration'][method]['threshold_s']):
                raise ValueError(f'{session}: candidate count/threshold/summary mismatch')
            with np.load(scene_dir/f'{method}_background_maps.npz', allow_pickle=False) as maps:
                starts = np.flatnonzero(maps['alarm'])
                if (len(starts) != len(candidates) or len(times) != summary['samples']
                        or not np.array_equal(maps['time_s'], times)
                        or not np.array_equal(maps['interval'], intervals)
                        or not np.array_equal(maps['tile_id'], tile_ids)):
                    raise ValueError(f'{session}: saved alarms/timeline differ from candidates or extraction')
                for number, (candidate, index) in enumerate(zip(candidates, starts), 1):
                    pair = (candidate['pair_tile_a'], candidate['pair_tile_b'])
                    start, end = candidate['start_relative_time_s'], candidate['end_relative_time_s']
                    score = candidate['score_s']
                    background_index = int(maps['background_index'][index])
                    if (candidate['session'] != session or candidate['method'] != method
                            or candidate['candidate'] != number or pair not in valid_pairs
                            or not all(math.isfinite(v) for v in (start, end, score)) or end < start
                            or abs(start-float(times[index])) > 1e-9
                            or not maps['ready'][index] or score < summary['threshold_s']
                            or abs(score-float(maps['score'][index])) > 1e-9
                            or tuple(tile_ids[maps['winner_pair'][index]]) != pair
                            or background_index not in (0, 1)
                            or candidate['selected_background_model'] != ('pre_drive', 'drive')[background_index]):
                        raise ValueError(f'{session}: candidate time/score/tile/model differs from saved alarm')
                    delta = None if onset is None else (start-onset)*1000
                    if ((delta is None) != (candidate['minus_onset_ms'] is None)
                            or delta is not None and (not math.isfinite(candidate['minus_onset_ms'])
                                or abs(delta-candidate['minus_onset_ms']) > 1e-7)):
                        raise ValueError(f'{session}: candidate onset delta mismatch')
                    scenes.append(dict(session=session, method=method, candidate_id=number,
                        output_relative=f'{session}/{method}_c{number:03d}',
                        record_root=str(annotation.parents[3]), annotation_path=str(annotation),
                        time_sync_path=ann['time_sync'], candidate_recording_s=start,
                        drive_start_s=None, rgb_onset_recording_s=onset,
                        reference_origin_s=source['reference_origin_s'], rgb_timestamp_source='bag',
                        annotation_sha256=source['source_identity']['annotation_sha256'],
                        time_sync_sha256=source['source_identity']['time_sync_sha256'],
                        condition=source['condition'], transmitter_limit=source['transmitter_limit'],
                        tiles=[tiles[i] for i in pair], source_candidate=candidate,
                        time_class='negative_recording' if delta is None else
                            'before_guard' if delta < -parameters['settings']['onset_guard_s']*1000 else
                            'onset_to_250ms' if 0 <= delta <= 250 else 'other'))
            all_candidates.extend(candidates)
            for name in (f'{method}_candidates.json', f'{method}_background_maps.npz', f'{method}_tiles.npz'):
                hashes[f'{session}/{name}'] = video.digest(scene_dir/name)
        for name in ('result.json', 'source_result.json', 'tiles.json'):
            hashes[f'{session}/{name}'] = video.digest(scene_dir/name)
    csv_matches_json(folder/'candidates.csv', all_candidates, static.CANDIDATE_FIELDS)
    return dict(schema_version=2, protocol=static.PROTOCOL,
        purpose='All saved static RGB/EVS candidates; spatial review only, no detector changes',
        spatial=definition['spatial'], detector_window_ms=definition['step_ms']*definition['window_bins'],
        frozen_manifest_sha256=plan['frozen_manifest_sha256'], evaluation_source_sha256=hashes,
        summaries=summaries, scenes=scenes)


def plan_previews(manifest, before, after):
    plans = []
    for scene in manifest['scenes']:
        try:
            plans.append(video.find_preview(Path(scene['record_root']), scene, manifest['spatial'], before, after))
        except (ValueError, OSError, KeyError, TypeError) as error:
            plans.append(dict(error=str(error)))
    return plans


def preview_commands(manifest, plans):
    groups = {}
    for scene, plan in zip(manifest['scenes'], plans):
        if 'error' in plan:
            groups.setdefault(scene['record_root'], set()).add(scene['session'])
    script = static.ROOT/'scripts/experiments/generate_rc_popout_common_views.sh'
    return [['bash', str(script), '--record-root', root,
             '--camchain', manifest['spatial']['camchain'], '--projection', 'rotation-only',
             '--sessions', *sorted(sessions)] for root, sessions in sorted(groups.items())]


def write_page(output, manifest, results):
    esc = lambda value: html.escape(str(value))
    page = ['<!doctype html><html lang="ja"><meta charset="utf-8"><title>静止・全候補の映像照合</title>',
        '<style>body{font:16px sans-serif;max-width:1300px;margin:30px auto;padding:0 16px}video,img{width:100%}section{margin:32px 0}td,th{padding:5px}</style>',
        '<h1>静止条件・RGB/EVS全候補の映像照合</h1>',
        '<p>左はRGB、右はRGB＋EVS。枠は候補開始時の固定2タイルで、追跡や警報継続を表しません。水色は候補時刻より前、橙色は以降です。</p>',
        '<p>時刻は元記録のRGB原点基準です。発進時刻はありません。pre_drive/driveは選択された背景モデル名で、実際の走行状態ではありません。</p>',
        '<p>投影は解析と同じrotation-onlyです。車体・遮蔽物・床との対応、RGB/EVSの位置ずれ、初出現が遮蔽物端か画像端かを確認してください。</p>',
        '<p>プレビューのEVS表示窓と検知の過去2 ms窓は異なる場合があります。この動画から厳密なEVS初出現時刻は測定しません。候補位置は未判定です。</p>',
        '<table border="1"><tr><th>記録</th><th>方向</th><th>プロポ設定</th><th>センサ</th><th>候補数</th></tr>']
    for row in manifest['summaries']:
        page.append('<tr>'+''.join(f'<td>{esc(row[k])}</td>' for k in
                    ('session', 'condition', 'transmitter_limit', 'method', 'candidates'))+'</tr>')
    page.append('</table>')
    for scene, report in zip(manifest['scenes'], results):
        name = esc(scene['output_relative'])
        page.append(f'<section><h2>{name} / {esc(scene["condition"])}</h2>')
        page.append(f'<p>候補情報: {esc(json.dumps(scene["source_candidate"]))}</p>')
        if report['status'] != 'complete':
            page.append(f'<pre>FAILED: {esc(report["error"])}</pre></section>')
            continue
        page.append(f'<p>候補から最初の橙色フレームまで +{report["first_display_after_candidate_ms"]:.3f} ms。'
                    f'EVS表示窓 {report["preview_event_window_ms"]:g} ms / {esc(report["preview_event_window_position"])}。</p>'
                    f'<video controls loop preload="none" src="{name}/candidate_review.mp4"></video>'
                    f'<p><a href="{name}/frames.csv">時刻対応</a> / <a href="{name}/summary.json">候補情報</a></p>'
                    f'<a href="{name}/contact_sheet.png"><img loading="lazy" src="{name}/contact_sheet.png" alt="候補前後の確認画像"></a></section>')
    (output/'index.html').write_text('\n'.join(page)+'</html>')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--static-dir', type=Path, required=True)
    parser.add_argument('--frozen-dir', type=Path, default=static.DEFAULT_FROZEN)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--before-s', type=float, default=.3)
    parser.add_argument('--after-s', type=float, default=.3)
    parser.add_argument('--slow-factor', type=float, default=4.)
    parser.add_argument('--prepare-previews', action='store_true', help='Generate missing matching common-view previews (ROS/RAW runtime required)')
    parser.add_argument('--dry-run', action='store_true', help='Read/validate only; print missing-preview commands without writing or decoding')
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError('output already exists; choose a new directory')
        if any(not math.isfinite(v) or v <= 0 for v in (args.before_s, args.after_s, args.slow_factor)):
            raise ValueError('durations and slow factor must be positive and finite')
        manifest = load_review(args.static_dir, args.frozen_dir)
        plans = plan_previews(manifest, args.before_s, args.after_s)
        commands = preview_commands(manifest, plans)
        for command in commands:
            print('Missing matching previews: '+shlex.join(command), flush=True)
        if args.prepare_previews and not args.dry_run:
            for command in commands:
                subprocess.run(command, check=True)
            plans = plan_previews(manifest, args.before_s, args.after_s)
        for scene, plan in zip(manifest['scenes'], plans):
            print(f'{scene["output_relative"]}: '+
                  ('FAILED preview: '+plan['error'] if 'error' in plan else str(plan['folder'])), flush=True)
        print(f'Static review: recordings={len(manifest["summaries"])//2} candidates={len(plans)}', flush=True)
        if args.dry_run:
            return int(any('error' in p for p in plans))
        ffmpeg = shutil.which('ffmpeg')
        if not ffmpeg:
            raise ValueError('ffmpeg is required')
        import cv2
        args.output.mkdir(parents=True)
        video.write_json(args.output/'review_manifest.json', manifest)
        video.write_json(args.output/'run_config.json', dict(static_dir=str(args.static_dir.resolve()),
            before_s=args.before_s, after_s=args.after_s, slow_factor=args.slow_factor,
            prepare_previews=args.prepare_previews, opencv_version=cv2.__version__,
            review_code_sha256={name: video.digest(static.ROOT/'tools'/name) for name in
                ('rc_popout_grid_static_videos.py', 'rc_popout_candidate_videos.py', 'rc_popout_grid_evaluation_videos.py')},
            note='Static recording times; no fitting or inference. All saved candidates included.'))
        results = []
        for scene, plan in zip(manifest['scenes'], plans):
            try:
                if 'error' in plan:
                    raise ValueError(plan['error'])
                destination = args.output/scene['output_relative']
                destination.parent.mkdir(exist_ok=True)
                # Show the retained onset when covered; do not reject early or negative alarms.
                lo, hi, _ = plan['span']
                onset = scene['rgb_onset_recording_s']
                require_onset = (onset is not None and plan['frames'][lo]['relative_time_s'] < onset
                                 <= plan['frames'][hi-1]['relative_time_s'])
                report = video.render(scene, manifest, plan, destination, args.slow_factor, ffmpeg,
                                      require_onset=require_onset)
                report['output_relative'] = scene['output_relative']
                print(f'complete: {scene["output_relative"]}', flush=True)
            except (ValueError, OSError, subprocess.CalledProcessError, cv2.error) as error:
                detail = str(error)+(f' / {error.stderr}' if isinstance(error, subprocess.CalledProcessError) else '')
                report = dict(session=scene['session'], method=scene['method'], candidate_id=scene['candidate_id'],
                              output_relative=scene['output_relative'], status='failed', error=detail)
                print(f'FAILED: {scene["output_relative"]}: {detail}', flush=True)
            results.append(report)
            video.write_json(args.output/'summary.json', results)
        errors = [r for r in results if r['status'] != 'complete']
        video.write_json(args.output/'review_errors.json', errors)
        write_page(args.output, manifest, results)
        print(f'Report: {args.output}/index.html / candidates={len(results)} failed={len(errors)}', flush=True)
        return int(bool(errors))
    except (ValueError, OSError, KeyError, TypeError, IndexError, ImportError, subprocess.CalledProcessError) as error:
        parser.exit(1, f'error: {error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
