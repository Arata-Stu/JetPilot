"""Apply the frozen moving-ego grid detector to the 18 static-ego recordings.

Re-extract original RGB/RAW using the frozen geometry. Never fit a background,
change thresholds, invent a drive timestamp, or modify source annotations.
"""
import argparse
import copy
import html
import math
from pathlib import Path
import shutil
from types import SimpleNamespace

import numpy as np

from analyze_rc_popout_grid_background import write_csv
from evaluate_rc_popout_grid_background import ROOT, load_frozen, read_json, current_code
from rc_popout_change_detection import digest, write_json
from rc_popout_detection import analyze, evaluation_intervals
from rc_popout_grid_background import ALGORITHM, alarm_episodes, score_maps, validate_model
from rc_popout_local_detection import validate_data
from rc_popout_tile_activity import TileSink


DEFAULT_FROZEN = ROOT/'docs/evidence/rc_popout_20260930/grid_background_frozen_20261006'
PROTOCOL = 'static_cross_condition_transfer_v1'
BASELINE_THRESHOLDS = dict(rgb=.001, evs=20.)
SUMMARY_FIELDS = [
    'session', 'condition', 'transmitter_limit', 'method', 'status', 'error',
    'samples', 'ready_samples', 'threshold_s', 'evaluation_seconds',
    'ready_observed_seconds', 'active_seconds', 'candidates',
    'rgb_first_visible_s', 'first_candidate_s', 'first_candidate_minus_onset_ms',
    'candidates_before_guard', 'candidates_before_onset', 'candidates_onset_to_250ms',
    'first_candidate_after_onset_s', 'first_candidate_after_onset_minus_onset_ms',
    'active_at_rgb_onset', 'onset_state_age_ms', 'max_score_s',
]
CANDIDATE_FIELDS = [
    'session', 'method', 'candidate', 'start_relative_time_s', 'end_relative_time_s',
    'end_reason', 'minus_onset_ms', 'pair_tile_a', 'pair_tile_b',
    'selected_background_model', 'score_s',
]


def population():
    """Acquisition inventory, not selected by detector output or onset timing."""
    records = []
    for prefix, limit, numbers in (
        ('popout-0928-static', '20pct', range(1, 10)),
        ('popout-0928-static-100', '100pct', range(9)),
    ):
        for index, number in enumerate(numbers):
            records.append(dict(session=prefix+(f'_{number:02d}' if number else ''),
                condition=('left', 'right', 'none')[index//3], transmitter_limit=limit,
                repetition=index % 3+1))
    return records


def file_identity(path, *, hash_content=False):
    path = Path(path)
    stat = path.stat()
    if not path.is_file() or stat.st_size == 0:
        raise ValueError(f'missing or empty input: {path}')
    value = dict(path=str(path.resolve()), bytes=stat.st_size, mtime_ns=stat.st_mtime_ns)
    if hash_content:
        value['sha256'] = digest(path)
    return value


def scene_identity(annotation_path):
    ann = read_json(annotation_path)
    sync = Path(ann['time_sync'])
    if not sync.is_absolute():
        raise ValueError('annotation time_sync must be an absolute path')
    record = annotation_path.parents[3]/ann['session']
    raw = sorted(record.glob('*.raw'))
    bags = sorted(record.glob('*.mcap'))
    if len(raw) != 1 or not bags:
        raise ValueError(f'{ann["session"]}: exactly one RAW and at least one MCAP required: {record}')
    small = [annotation_path, sync, record/'metadata.yaml', Path(str(raw[0])+'.metadata.yaml')]
    return dict(annotation_sha256=digest(annotation_path), time_sync_sha256=digest(sync),
        small_files=[file_identity(p, hash_content=True) for p in small],
        recording_files=[file_identity(p) for p in raw+bags],
        recording_directory=str(record.resolve()),
        large_file_check='Path, byte size and mtime only; RAW/MCAP content is not hashed.')


def preflight(config_path, frozen_dir):
    parameters, models, freeze = load_frozen(frozen_dir)
    definition = parameters['input_definition']
    spatial = definition['spatial']
    if spatial['view_frame'] != 'evs' or spatial['projection'] != 'rotation-only':
        raise ValueError('this transfer requires the frozen EVS rotation-only input')
    chain = Path(spatial['camchain'])
    if digest(chain) != spatial['camchain_sha256']:
        raise ValueError('current calibration differs from frozen calibration')
    original = read_json(config_path)
    old_spatial = original['spatial']
    if (old_spatial['camchain_sha256'] != spatial['camchain_sha256']
            or digest(old_spatial['camchain']) != spatial['camchain_sha256']):
        raise ValueError('static calibration differs from frozen calibration; do not override it')
    records = population()
    entries = {e['session']: e for e in original['sessions']}
    wanted = {r['session'] for r in records}
    if len(entries) != len(original['sessions']) or not wanted <= set(entries):
        raise ValueError(f'duplicate/missing static sessions; missing={sorted(wanted-set(entries))}')
    extras = set(entries)-wanted
    if extras - {'popout-0928-static'}:
        raise ValueError(f'unexpected sessions in static config: {sorted(extras)}')
    effective = dict(schema_version=1, roi=copy.deepcopy(definition['roi']),
                     spatial=copy.deepcopy(spatial), sessions=[])
    scenes = []
    for record in records:
        name = record['session']
        path = Path(entries[name]['annotation'])
        path = (path if path.is_absolute() else config_path.parent/path).resolve()
        ann = read_json(path)
        if ann['session'] != name or '/' in name or len(path.parents) < 4:
            raise ValueError(f'{name}: invalid annotation session/path')
        if ann.get('rgb_timestamp_source', 'bag') != 'bag':
            raise ValueError(f'{name}: bag timestamps required for this static protocol')
        spans = evaluation_intervals(ann)
        origin = ann['reference_origin_s']
        if isinstance(origin, bool) or not isinstance(origin, (int, float)) or not math.isfinite(origin):
            raise ValueError(f'{name}: invalid RGB time origin')
        annotated = ann['spatial']
        if (annotated['view_frame'] != 'evs' or annotated['output_size'] != spatial['output_size']
                or annotated['camchain_sha256'] != spatial['camchain_sha256']
                or digest(annotated['camchain']) != spatial['camchain_sha256']):
            raise ValueError(f'{name}: annotation calibration/EVS coordinates differ')
        if not isinstance(ann.get('onset', {}), dict):
            raise ValueError(f'{name}: onset must be an object, not null')
        first = ann.get('onset', {}).get('first_visible')
        if first is not None and (not isinstance(first, dict) or
                isinstance(first.get('rgb_time_s'), bool) or
                not isinstance(first.get('rgb_time_s'), (int, float))):
            raise ValueError(f'{name}: first_visible requires a numeric rgb_time_s')
        onset = None if first is None else first['rgb_time_s']-origin
        if ((onset is None) != (record['condition'] == 'none')
                or onset is not None and (not math.isfinite(onset) or
                    not any(a <= onset < b for a, b in spans))):
            raise ValueError(f'{name}: onset must match condition and lie inside evaluation intervals')
        identity = scene_identity(path)
        if identity['time_sync_sha256'] != ann['time_sync_sha256']:
            raise ValueError(f'{name}: time sync changed since annotation')
        scenes.append(dict(record, annotation=str(path), source_identity=identity,
            intervals=spans, reference_origin_s=origin, rgb_first_visible_s=onset,
            annotation_spatial=annotated,
            annotation_geometry_changed=any(annotated.get(k) != spatial.get(k) for k in
                ('view_frame', 'output_size', 'projection', 'depth_m', 'rgb_to_view_homography')),
            onset_review='Retained RGB acquisition timestamp; recheck first visibility in the new projection.'))
        effective['sessions'].append(dict(session=name, annotation=str(path)))
    plan = dict(protocol=PROTOCOL, population=records, excluded_config_sessions=sorted(extras),
        source_config=str(config_path.resolve()), source_config_sha256=digest(config_path),
        original_roi=original['roi'], original_spatial=old_spatial,
        input_definition=definition, effective_config=effective, scenes=scenes,
        frozen_manifest_sha256=digest(frozen_dir/'freeze.json'),
        adapter_sha256=digest(__file__), frozen_runtime_sha256=current_code(),
        baseline=dict(thresholds=BASELINE_THRESHOLDS,
            note='Historical whole-ROI activity thresholds on re-extracted identical inputs; '
                 'not the earlier distinct-pixel spatial baseline or its old projection results.'),
        policy='No fitting, threshold recalibration, static phase forcing, or onset gating. '
               'Original annotations retained. Static observations use recording time, not drive time.')
    return plan, parameters, models, freeze


def check_unchanged(plan, scene, frozen_dir):
    if digest(plan['source_config']) != plan['source_config_sha256']:
        raise ValueError('static source config changed since preflight')
    load_frozen(frozen_dir)
    if digest(frozen_dir/'freeze.json') != plan['frozen_manifest_sha256']:
        raise ValueError('frozen package changed since preflight')
    if digest(plan['input_definition']['spatial']['camchain']) != plan['input_definition']['spatial']['camchain_sha256']:
        raise ValueError('calibration changed since preflight')
    if scene_identity(Path(scene['annotation'])) != scene['source_identity']:
        raise ValueError(f'{scene["session"]}: recording/annotation/sync changed since preflight')


def extraction_args(definition):
    return SimpleNamespace(**{k:definition[k] for k in
        ('rgb_topic', 'rgb_pixel_delta', 'step_ms', 'window_bins')},
        rgb_threshold=BASELINE_THRESHOLDS['rgb'], evs_threshold=BASELINE_THRESHOLDS['evs'], spatial=False)


def check_extraction(scene, result, arrays, meta, whole_rows, models, definition):
    expected = dict(session=scene['session'], annotation_sha256=scene['source_identity']['annotation_sha256'],
        time_sync_sha256=scene['source_identity']['time_sync_sha256'],
        camchain_sha256=definition['spatial']['camchain_sha256'], roi=definition['roi'])
    if any(result[k] != v for k, v in expected.items()):
        raise ValueError('extracted provenance differs from preflight/frozen definition')
    if (result['intervals'] != scene['intervals'] or
            result['rgb_first_visible_s'] != scene['rgb_first_visible_s']):
        raise ValueError('annotation times changed during extraction')
    if result['valid_pixels'] != sum(t['valid_pixels'] for t in meta['tiles']):
        raise ValueError('extracted valid area differs from tile geometry')
    for sensor in ('rgb', 'evs'):
        data = arrays[sensor]
        validate_data(data, meta, sensor)
        validate_model(models[sensor], data, sensor)
        for i, (a, b) in enumerate(scene['intervals']):
            chosen = data['interval'] == i
            if not chosen.any():
                raise ValueError(f'{sensor}: no samples in evaluation interval {i}')
            if np.any(data['support_start_s'][chosen] < a-1e-9) or np.any(data['time_s'][chosen] >= b):
                raise ValueError(f'{sensor}: samples extend outside evaluation intervals')
        if np.any(data['interval'] >= len(scene['intervals'])):
            raise ValueError('unknown evaluation interval')
        rows = whole_rows[sensor]
        total = data['counts'].sum(axis=1)
        if sensor == 'rgb':
            total = total/data['valid_pixels'].sum()
        if (len(rows) != len(total) or not np.array_equal(data['interval'], [r[0] for r in rows])
                or not np.allclose(data['time_s'], [r[1] for r in rows], rtol=0, atol=1e-9)
                or not np.allclose(total, [r[2] for r in rows], rtol=0, atol=1e-12)):
            raise ValueError(f'{sensor}: tile counts do not conserve whole-ROI activity')


def summarize(scene, sensor, result, calibration, settings):
    """Labels are used only here, after inference, to report recording-time metrics."""
    episodes, alarm, active = alarm_episodes(result, calibration['threshold_s'], settings['release_ratio'])
    t = result['time_s']
    onset = scene['rgb_first_visible_s']
    candidates = []
    for number, event in enumerate(episodes, 1):
        i = event['start_index']
        candidates.append(dict(session=scene['session'], method=sensor, candidate=number,
            start_relative_time_s=event['start_time_s'], end_relative_time_s=event['end_time_s'],
            end_reason=event['end_reason'], minus_onset_ms=None if onset is None else (t[i]-onset)*1000,
            pair_tile_a=event['pair_tile_ids_at_start'][0], pair_tile_b=event['pair_tile_ids_at_start'][1],
            selected_background_model=('pre_drive', 'drive')[result['background_index'][i]],
            score_s=float(result['score'][i])))
    after = [] if onset is None else [c for c in candidates if c['start_relative_time_s'] >= onset]
    i = -1 if onset is None else int(np.searchsorted(t, onset, side='right'))-1
    observable = (i >= 0 and result['ready'][i] and onset-t[i] <= settings[f'{sensor}_max_gap_s'])
    ready_scores = result['score'][result['ready']]
    # Charge each observed interval to the alarm state at its LEFT endpoint.
    active_seconds = float(np.sum(result['observed_step_s'][1:]*active[:-1]))
    summary = dict(session=scene['session'], condition=scene['condition'],
        transmitter_limit=scene['transmitter_limit'], method=sensor, status='complete', error='',
        samples=len(t), ready_samples=int(result['ready'].sum()), threshold_s=calibration['threshold_s'],
        evaluation_seconds=sum(b-a for a,b in scene['intervals']),
        ready_observed_seconds=float(result['observed_step_s'].sum()), active_seconds=active_seconds,
        candidates=len(candidates), rgb_first_visible_s=onset,
        first_candidate_s=candidates[0]['start_relative_time_s'] if candidates else None,
        first_candidate_minus_onset_ms=candidates[0]['minus_onset_ms'] if candidates else None,
        candidates_before_guard=None if onset is None else int(alarm[t < onset-settings['onset_guard_s']].sum()),
        candidates_before_onset=None if onset is None else int(alarm[t < onset].sum()),
        candidates_onset_to_250ms=None if onset is None else int(alarm[(t >= onset)&(t <= onset+.250)].sum()),
        first_candidate_after_onset_s=after[0]['start_relative_time_s'] if after else None,
        first_candidate_after_onset_minus_onset_ms=after[0]['minus_onset_ms'] if after else None,
        active_at_rgb_onset=bool(active[i]) if observable else None,
        onset_state_age_ms=float((onset-t[i])*1000) if observable else None,
        max_score_s=float(ready_scores.max()) if len(ready_scores) else None)
    rows = []
    for i, timestamp in enumerate(t):
        a,b = result['winner_pair'][i]
        rows.append(dict(relative_time_s=float(timestamp), interval=int(result['interval'][i]),
            ready=bool(result['ready'][i]), reset=bool(result['reset'][i]),
            selected_background_model=('pre_drive', 'drive')[result['background_index'][i]],
            background_rmse=float(result['background_rmse'][i]), score_s=float(result['score'][i]),
            threshold_s=calibration['threshold_s'], alarm=bool(alarm[i]), active=bool(active[i]),
            pair_tile_a=int(result['tile_id'][a]), pair_tile_b=int(result['tile_id'][b])))
    return summary, candidates, rows, alarm, active


def write_index(output, summaries, failures):
    rows = ''.join('<tr>'+''.join(f'<td>{html.escape(str(r.get(k,"")))}</td>' for k in
        ('session', 'condition', 'method', 'status', 'candidates', 'candidates_before_onset',
         'candidates_onset_to_250ms', 'first_candidate_after_onset_minus_onset_ms', 'error'))+'</tr>' for r in summaries)
    (output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>Static transfer</title>'
        '<h1>固定grid背景モデル・静止条件への適用</h1>'
        '<p>背景モデル・閾値は再適合していません。候補は車両検知の確定ではありません。</p>'
        '<p>時刻は記録のRGB原点からの秒です。発進時刻は設定していません。'
        '背景モデル名pre_drive/driveは、実際の走行状態を意味しません。</p>'
        '<p>RGB初出現は既存注釈を保持しています。投影変更後の映像で再確認が必要です。</p>'
        f'<p>処理失敗記録: {len(failures)}。失敗を候補0件として扱わないでください。</p>'
        '<p><a href="summary.csv">Grid summary</a> / <a href="candidates.csv">全候補</a> / '
        '<a href="baseline_summary.csv">同じ入力の単純活動量baseline</a> / '
        '<a href="preflight.json">入力・保持した注釈</a></p><table border="1"><tr>'
        '<th>session</th><th>condition</th><th>sensor</th><th>status</th><th>candidates</th>'
        '<th>before onset</th><th>0..250 ms</th><th>first after onset [ms]</th><th>error</th></tr>'
        +rows+'</table>')


def evaluate(config_path, frozen_dir, output, max_memory_mb=512, *, preflight_only=False):
    if output.exists():
        raise ValueError('output already exists; choose a new directory')
    if not isinstance(max_memory_mb, int) or max_memory_mb < 1:
        raise ValueError('max-memory-mb must be positive')
    plan, parameters, models, _ = preflight(config_path, frozen_dir)
    definition = parameters['input_definition']
    print(f'Static transfer: {len(plan["scenes"])} recordings / 12 popout + 6 none; '
          f'ROI={definition["roi"]}; projection={definition["spatial"]["projection"]}', flush=True)
    for sensor in ('rgb', 'evs'):
        print(f'  frozen {sensor} threshold={parameters["calibration"][sensor]["threshold_s"]}', flush=True)
    print('  RGB onset annotations retained; recheck visibility in the new projection.', flush=True)
    if preflight_only:
        print('Preflight passed: no RAW/RGB decoded and no files written.', flush=True)
        return 0
    output.mkdir(parents=True)
    for name in ('detector_parameters.json', 'background_models.json', 'freeze.json'):
        shutil.copyfile(frozen_dir/name, output/name)
    write_json(output/'preflight.json', plan)
    write_json(output/'effective_common_detection_roi.json', plan['effective_config'])
    run = dict(protocol=PROTOCOL, status='running', frozen_dir=str(frozen_dir.resolve()),
        sessions=[s['session'] for s in plan['scenes']], preflight_sha256=digest(output/'preflight.json'),
        frozen_manifest_sha256=plan['frozen_manifest_sha256'], adapter_sha256=digest(__file__),
        frozen_runtime_sha256=current_code(), numpy_version=np.__version__,
        prior_exposure='These static recordings were already used for activity-baseline exploration.',
        policy=plan['policy'])
    write_json(output/'run_config.json', run)
    summaries, candidates, baselines, failures = [], [], [], []
    baseline_fields = ['session','condition','transmitter_limit','method','status','error','threshold',
        'evaluation_seconds','episodes','first_trigger_s','first_trigger_minus_rgb_onset_ms']
    for number, scene in enumerate(plan['scenes'], 1):
        name = scene['session']
        print(f'[{number}/{len(plan["scenes"])}] {name}: calibrated RGB and RAW tiles', flush=True)
        folder = output/name
        folder.mkdir()
        stage = 'extraction'
        try:
            check_unchanged(plan, scene, frozen_dir)
            sink = TileSink(definition['tile_px'], max_memory_mb)
            source, rgb, evs = analyze(plan['effective_config'], scene, extraction_args(definition), tile_sink=sink)
            check_unchanged(plan, scene, frozen_dir)
            arrays = sink.finish()
            meta = dict(tiles=sink.tiles, output_size=sink.size, roi=definition['roi'],
                step_s=sink.step, event_window_s=sink.step*sink.bins,
                tile_anchor='EVS image origin; frozen common ROI, actual valid pixels')
            whole = dict(rgb=rgb, evs=evs)
            check_extraction(scene, source, arrays, meta, whole, models, definition)
            write_json(folder/'tiles.json', meta)
            write_json(folder/'source_result.json', source)
            for sensor, data in arrays.items():
                np.savez_compressed(folder/f'{sensor}_tiles.npz', **data)
                write_csv(folder/f'{sensor}_activity_scores.csv',
                    [dict(interval=i, relative_time_s=t, score=v) for i,t,v in whole[sensor]])
            stage = 'inference'
            scene_summaries, scene_candidates, scene_baselines = [], [], []
            for sensor in ('rgb', 'evs'):
                result = score_maps(arrays[sensor], meta, sensor, parameters['settings'],
                                    models[sensor], definition['tile_px'])
                summary, found, rows, alarm, active = summarize(
                    scene, sensor, result, parameters['calibration'][sensor], parameters['settings'])
                scene_summaries.append(summary); scene_candidates.extend(found)
                result.update(alarm=alarm, active=active)
                np.savez_compressed(folder/f'{sensor}_background_maps.npz', **result)
                write_csv(folder/f'{sensor}_background_scores.csv', rows)
                write_json(folder/f'{sensor}_candidates.json', found)
                baseline = source[sensor]
                scene_baselines.append(dict(session=name, condition=scene['condition'],
                    transmitter_limit=scene['transmitter_limit'], method=sensor, status='complete', error='',
                    evaluation_seconds=source['evaluation_seconds'],
                    **{k:baseline[k] for k in ('threshold','episodes','first_trigger_s','first_trigger_minus_rgb_onset_ms')}))
                print(f'  {sensor}: candidates={len(found)}, '
                      f'first-after-onset={summary["first_candidate_after_onset_minus_onset_ms"]} ms', flush=True)
            check_unchanged(plan, scene, frozen_dir)
            write_json(folder/'result.json', dict(status='complete', protocol=PROTOCOL, algorithm=ALGORITHM,
                scene=scene, geometry=meta, source_result=source,
                parameters_sha256=digest(output/'detector_parameters.json'),
                tile_arrays_sha256={s:digest(folder/f'{s}_tiles.npz') for s in ('rgb','evs')},
                note='Static scene; recording-time outputs. Spatial review pending.'))
            summaries.extend(scene_summaries); candidates.extend(scene_candidates); baselines.extend(scene_baselines)
        except Exception as error:
            failures.append(dict(session=name, stage=stage, error=str(error)))
            write_json(folder/'result.json', dict(status='failed', **failures[-1]))
            for sensor in ('rgb','evs'):
                failed = dict(session=name, condition=scene['condition'], transmitter_limit=scene['transmitter_limit'],
                              method=sensor, status='failed', error=str(error))
                summaries.append(failed); baselines.append(failed)
            print(f'  FAILED ({stage}): {error}', flush=True)
        write_csv(output/'summary.csv', summaries, SUMMARY_FIELDS)
        write_json(output/'summary.json', summaries)
        write_csv(output/'candidates.csv', candidates, CANDIDATE_FIELDS)
        write_csv(output/'baseline_summary.csv', baselines, baseline_fields)
        write_json(output/'errors.json', failures)
    run['status'] = 'failed' if failures else 'complete'
    write_json(output/'run_config.json', run)
    write_index(output, summaries, failures)
    print(f'Report: {output}/index.html / scenes={len(plan["scenes"])} failed={len(failures)}', flush=True)
    return int(bool(failures))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True, help='Existing static common_detection_roi.json')
    parser.add_argument('--frozen-dir', type=Path, default=DEFAULT_FROZEN)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-memory-mb', type=int, default=512)
    parser.add_argument('--preflight', action='store_true', help='Validate metadata and input files only; do not write/decode')
    args = parser.parse_args(argv)
    try:
        return evaluate(args.config.resolve(), args.frozen_dir.resolve(), args.output.resolve(),
                        args.max_memory_mb, preflight_only=args.preflight)
    except (OSError, ValueError, KeyError, TypeError, ImportError) as error:
        parser.exit(1, f'error: {error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
