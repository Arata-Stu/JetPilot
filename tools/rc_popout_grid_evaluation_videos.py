"""Review every fixed-evaluation candidate, including early and negative alarms.

Read saved inference outputs and current provenance; never rerun or tune detection.
Reuse calibrated preview frame mappings. Candidate-centered clips need no RGB onset.
"""
import argparse
import csv
import html
import json
import math
from pathlib import Path
import shutil
import subprocess

import numpy as np

import rc_popout_candidate_videos as video
from evaluate_rc_popout_grid_background import (ROOT, DEFAULT_SPLIT, CANDIDATE_FIELDS,
    check_extraction_contract, load_frozen, read_json)
from rc_popout_local_detection import load_scene


DEFAULT_FROZEN = ROOT/'docs/evidence/rc_popout_20260930/grid_background_frozen_20261006'


def csv_matches_json(path, expected, fields):
    with path.open() as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != fields:
            raise ValueError(f'{path}: unexpected CSV columns')
        actual = list(reader)
    rendered = [{k: '' if row[k] is None else str(row[k]) for k in fields} for row in expected]
    if actual != rendered:
        raise ValueError(f'{path}: CSV differs from saved per-scene results')


def load_review(evaluation_dir, frozen_dir):
    parameters, _, freeze = load_frozen(frozen_dir)
    run = read_json(evaluation_dir/'run_config.json')
    if (run['subset'] != 'evaluation' or run['status'] != 'complete'
            or run['sessions'] != freeze['evaluation_sessions']
            or run['code_sha256'] != freeze['code_sha256']
            or run['frozen_manifest_sha256'] != video.digest(frozen_dir/'freeze.json')):
        raise ValueError('evaluation run differs from frozen contract or is incomplete')
    for name in ('freeze.json', 'detector_parameters.json', 'background_models.json'):
        if video.digest(evaluation_dir/name) != video.digest(frozen_dir/name):
            raise ValueError(f'evaluation copied frozen file differs: {name}')
    parameter_hash = video.digest(frozen_dir/'detector_parameters.json')
    if run['parameters_sha256'] != parameter_hash:
        raise ValueError('evaluation parameter hash mismatch')
    tile_dir = Path(run['provenance']['tile_dir'])
    tile_run_path = tile_dir/'run_config.json'
    if video.digest(tile_run_path) != run['provenance']['tile_run_sha256']:
        raise ValueError('tile run changed after evaluation')
    tile_run = read_json(tile_run_path)
    if (video.digest(tile_run['source_config_path']) != tile_run['source_config_sha256']
            or read_json(tile_run['source_config_path']) != tile_run['source_config']):
        raise ValueError('source config changed after extraction')
    check_extraction_contract(frozen_dir, tile_run['source_config'], tile_run['tile_px'], DEFAULT_SPLIT)
    split = read_json(DEFAULT_SPLIT)
    roots = {s: g['record_root_relative'] for g in split['groups'] for s in g['evaluation']}
    negatives = {s for g in split['groups'] if g['condition'] == 'none' for s in g['evaluation']}
    summaries = read_json(evaluation_dir/'summary.json')
    expected_keys = [(s, method) for s in run['sessions'] for method in ('rgb', 'evs')]
    if [(r['session'], r['method']) for r in summaries] != expected_keys:
        raise ValueError('evaluation summary must contain all ten recordings and both sensors')
    csv_matches_json(evaluation_dir/'summary.csv', summaries, list(summaries[0]))
    by_key = {(r['session'], r['method']): r for r in summaries}
    source_hashes = {name: video.digest(evaluation_dir/name)
                     for name in ('run_config.json', 'summary.csv', 'summary.json', 'candidates.csv')}
    scenes, all_candidates = [], []
    for session in run['sessions']:
        alignment, geometry, hashes = load_scene(tile_dir, session, tile_run)
        hashes.update({sensor+'_tiles_sha256': video.digest(tile_dir/session/f'{sensor}_tiles.npz')
                       for sensor in ('rgb', 'evs')})
        if hashes != run['provenance']['input_sha256'][session]:
            raise ValueError(f'{session}: input changed after evaluation')
        result = read_json(evaluation_dir/session/'result.json')
        if (result['alignment'] != alignment or result['geometry'] != geometry
                or result['input_hashes'] != hashes or result['parameters_sha256'] != parameter_hash):
            raise ValueError(f'{session}: saved result differs from source/provenance')
        onset = alignment.get('rgb_first_visible_from_drive_s')
        if (onset is None) != (session in negatives):
            raise ValueError('onset annotation and evaluation condition disagree')
        tiles = {t['tile_id']: t for t in geometry['tiles']}
        for method in ('rgb', 'evs'):
            candidate_path = evaluation_dir/session/f'{method}_candidates.json'
            candidates = read_json(candidate_path)
            summary = by_key[session, method]
            if (summary['status'] != 'complete' or summary['candidates'] != len(candidates)
                    or summary['threshold_s'] != parameters['calibration'][method]['threshold_s']):
                raise ValueError('candidate count/threshold/summary mismatch')
            maps_path = evaluation_dir/session/f'{method}_background_maps.npz'
            with np.load(maps_path, allow_pickle=False) as maps:
                starts = np.flatnonzero(maps['alarm'])
                if len(starts) != len(candidates) or len(maps['time_s']) != summary['samples']:
                    raise ValueError('saved alarms differ from candidate list')
                for number, (c, index) in enumerate(zip(candidates, starts), 1):
                    if (c['session'] != session or c['method'] != method or c['candidate'] != number
                            or abs(c['start_relative_time_s']-float(maps['time_s'][index])) > 1e-9
                            or abs(c['score_s']-float(maps['score'][index])) > 1e-9
                            or c['score_s'] < summary['threshold_s']
                            or list(maps['tile_id'][maps['winner_pair'][index]]) != [c['pair_tile_a'], c['pair_tile_b']]):
                        raise ValueError('candidate time/score/tile differs from saved alarm')
                    start = c['start_relative_time_s']
                    if abs(start-alignment['drive_start_s']-c['start_from_drive_s']) > 1e-9:
                        raise ValueError('candidate recording/drive timestamp mismatch')
                    delta = None if onset is None else (c['start_from_drive_s']-onset)*1000
                    if ((delta is None) != (c['minus_onset_ms'] is None)
                            or delta is not None and abs(delta-c['minus_onset_ms']) > 1e-7):
                        raise ValueError('candidate onset delta mismatch')
                    scenes.append(dict(
                        session=session, method=method, candidate_id=number,
                        output_relative=f'{session}/{method}_c{number:03d}', record_root_relative=roots[session],
                        candidate_recording_s=start, candidate_from_drive_s=c['start_from_drive_s'],
                        drive_start_s=alignment['drive_start_s'],
                        rgb_onset_recording_s=None if onset is None else alignment['drive_start_s']+onset,
                        reference_origin_s=alignment['reference_origin_s'],
                        rgb_timestamp_source=alignment['timestamp_source'],
                        annotation_sha256=hashes['annotation_sha256'], time_sync_sha256=hashes['time_sync_sha256'],
                        tiles=[tiles[c['pair_tile_a']], tiles[c['pair_tile_b']]],
                        source_candidate=c,
                        time_class='negative_recording' if delta is None else
                            'before_guard' if delta < -parameters['settings']['onset_guard_s']*1000 else
                            'onset_to_250ms' if 0 <= delta <= 250 else 'other'))
            for path in (candidate_path, maps_path, evaluation_dir/session/'result.json'):
                source_hashes[str(path.relative_to(evaluation_dir))] = video.digest(path)
            all_candidates.extend(candidates)
    csv_matches_json(evaluation_dir/'candidates.csv', all_candidates, CANDIDATE_FIELDS)
    return dict(schema_version=2, purpose='Every RGB/EVS fixed-evaluation candidate; no candidate selection',
                spatial=parameters['input_definition']['spatial'],
                detector_window_ms=parameters['input_definition']['step_ms']*parameters['input_definition']['window_bins'],
                frozen_manifest_sha256=video.digest(frozen_dir/'freeze.json'),
                evaluation_source_sha256=source_hashes, summaries=summaries, scenes=scenes)


def write_page(output, manifest, results):
    page = ['<!doctype html><html lang="ja"><meta charset="utf-8"><title>内部評価・全候補の映像照合</title>',
            '<style>body{font:16px sans-serif;max-width:1300px;margin:30px auto;padding:0 16px}video,img{width:100%}section{margin:32px 0}td,th{padding:5px}</style>',
            '<h1>内部評価・RGB/EVS全候補の映像照合</h1>',
            '<p>左はRGB、右はRGB＋EVS。両センサの候補を同じEVS座標上に描きます。枠は候補開始時の固定2タイルで、車両追跡や警報の継続状態を表しません。</p>',
            '<p>水色は候補時刻より前、橙色は以降。表示はRGBフレーム刻みで、EVS描画窓と検知の過去2 ms窓は異なります。RGB検知は連続フレーム対です。</p>',
            '<p>早期・出現後・負例の候補を全て掲載します。初出現を含まない短いクリップもあります。ゼロ候補の記録・センサは下表に残します。</p>',
            '<table border="1"><tr><th>記録</th><th>センサ</th><th>候補数</th></tr>']
    for r in manifest['summaries']:
        page.append(f'<tr><td>{html.escape(r["session"])}</td><td>{r["method"]}</td><td>{r["candidates"]}</td></tr>')
    page.append('</table>')
    for scene, report in zip(manifest['scenes'], results):
        name = html.escape(scene['output_relative'])
        page.append(f'<section><h2>{name} / {scene["time_class"]}</h2>')
        if report['status'] != 'complete':
            page.append(f'<pre>FAILED: {html.escape(report["error"])}</pre></section>')
            continue
        page.append(f'<p>候補から最初の橙色フレームまで +{report["first_display_after_candidate_ms"]:.3f} ms。'
                    f'EVS表示窓 {report["preview_event_window_ms"]:g} ms / {html.escape(report["preview_event_window_position"])}。</p>'
                    f'<video controls loop preload="none" src="{name}/candidate_review.mp4"></video>'
                    f'<p><a href="{name}/frames.csv">時刻対応</a> / <a href="{name}/summary.json">候補情報</a></p>'
                    f'<a href="{name}/contact_sheet.png"><img loading="lazy" src="{name}/contact_sheet.png" alt="候補前後4フレーム"></a></section>')
    (output/'index.html').write_text('\n'.join(page)+'</html>')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evaluation-dir', required=True, type=Path)
    parser.add_argument('--record-base', required=True, type=Path, help='Parent containing dynamic and 2026-09-30')
    parser.add_argument('--frozen-dir', type=Path, default=DEFAULT_FROZEN)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--before-s', type=float, default=.3)
    parser.add_argument('--after-s', type=float, default=.3)
    parser.add_argument('--slow-factor', type=float, default=4.)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError('output already exists; choose a new directory')
        if any(not math.isfinite(v) or v <= 0 for v in (args.before_s, args.after_s, args.slow_factor)):
            raise ValueError('durations and slow factor must be positive and finite')
        manifest = load_review(args.evaluation_dir, args.frozen_dir)
        plans = []
        for scene in manifest['scenes']:
            try:
                plan = video.find_preview(args.record_base/scene['record_root_relative'], scene,
                                          manifest['spatial'], args.before_s, args.after_s)
                print(f'{scene["output_relative"]}: {scene["time_class"]}, {plan["folder"].name}', flush=True)
            except (ValueError, OSError, KeyError, TypeError) as error:
                plan = dict(error=str(error))
                print(f'{scene["output_relative"]}: FAILED preview: {error}', flush=True)
            plans.append(plan)
        if args.dry_run:
            return int(any('error' in plan for plan in plans))
        ffmpeg = shutil.which('ffmpeg')
        if not ffmpeg:
            raise ValueError('ffmpeg is required')
        import cv2
        args.output.mkdir(parents=True)
        video.write_json(args.output/'review_manifest.json', manifest)
        video.write_json(args.output/'run_config.json', dict(
            evaluation_dir=str(args.evaluation_dir.resolve()), record_base=str(args.record_base.resolve()),
            review_code_sha256={name: video.digest(ROOT/'tools'/name) for name in
                               ('rc_popout_grid_evaluation_videos.py', 'rc_popout_candidate_videos.py')},
            before_s=args.before_s, after_s=args.after_s, slow_factor=args.slow_factor,
            opencv_version=cv2.__version__, note='All candidates; no changes to inference/frozen settings.'))
        results = []
        for scene, plan in zip(manifest['scenes'], plans):
            try:
                if 'error' in plan:
                    raise ValueError(plan['error'])
                destination = args.output/scene['output_relative']
                destination.parent.mkdir(exist_ok=True)
                report = video.render(scene, manifest, plan, destination, args.slow_factor, ffmpeg, require_onset=False)
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
    except (ValueError, OSError, KeyError, TypeError, ImportError) as error:
        parser.exit(1, f'error: {error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
