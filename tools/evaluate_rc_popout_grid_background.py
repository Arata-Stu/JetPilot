"""Freeze an existing grid-background development result, then run fixed inference.

Evaluation never fits a background, recalibrates a threshold, or selects a margin.
The development scorer/reporting code is reused without modifying its saved hashes.
"""
import argparse
import csv
import html
import json
import math
from pathlib import Path
import shutil

import numpy as np

from analyze_rc_popout_grid_background import report_result, write_csv
from rc_popout_change_detection import DEFAULT_SPLIT, digest, select_sessions, write_json
from rc_popout_grid_background import ALGORITHM, score_maps, validate_model, validate_settings
from rc_popout_local_detection import load_scene, validate_data


ROOT = Path(__file__).resolve().parents[1]
DEVELOPMENT_CODE = (
    'rc_popout_grid_background.py', 'analyze_rc_popout_grid_background.py',
    'rc_popout_local_detection.py', 'rc_popout_change_detection.py',
    'analyze_rc_popout_development_bundle.py',
)
RUNTIME_CODE = DEVELOPMENT_CODE + (
    'evaluate_rc_popout_grid_background.py', 'rc_popout_tile_activity.py',
    'rc_popout_detection.py',
)
PACKAGE_FILES = ('detector_parameters.json', 'background_models.json',
                 'development_run_config.json', 'development_summary.csv', 'split.json')
CANDIDATE_FIELDS = ['session', 'method', 'candidate', 'start_relative_time_s',
                    'start_from_drive_s', 'end_from_drive_s', 'end_reason', 'minus_onset_ms',
                    'pair_tile_a', 'pair_tile_b', 'background_phase', 'score_s']


def read_json(path):
    return json.loads(Path(path).read_text())


def current_code():
    return {name: digest(ROOT/'tools'/name) for name in RUNTIME_CODE}


def input_definition(source, tile_px):
    return dict(tile_px=tile_px, roi=source['common']['roi'], spatial=source['common']['spatial'],
                **{k: source['parameters'][k]
                   for k in ('rgb_topic', 'rgb_pixel_delta', 'step_ms', 'window_bins')})


def validate_contract(parameters, models):
    if parameters['algorithm'] != ALGORITHM or parameters['split_sha256'] != digest(DEFAULT_SPLIT):
        raise ValueError('frozen algorithm/split mismatch')
    if set(parameters['code_sha256']) != set(DEVELOPMENT_CODE):
        raise ValueError('unexpected development code contract')
    for name, value in parameters['code_sha256'].items():
        if digest(ROOT/'tools'/name) != value:
            raise ValueError(f'development code changed: {name}; do not silently refreeze')
    validate_settings(parameters['settings'])
    definition = parameters['input_definition']
    if definition['tile_px'] != 32 or definition['spatial']['view_frame'] != 'evs':
        raise ValueError('32 px EVS-coordinate grid required')
    split = read_json(DEFAULT_SPLIT)
    negatives = [s for g in split['groups'] if g['condition'] == 'none' for s in g['development']]
    if negatives != [parameters['background_session']]:
        raise ValueError('background must be the single development negative')
    if set(models) != {'rgb', 'evs'} or set(parameters['calibration']) != {'rgb', 'evs'}:
        raise ValueError('both frozen sensor models and thresholds required')
    for sensor in ('rgb', 'evs'):
        validate_model(models[sensor], {k: np.asarray(models['rgb'][k])
                                       for k in ('tile_id', 'valid_pixels')}, sensor)
        calibration = parameters['calibration'][sensor]
        peak, threshold = calibration['negative_max'], calibration['threshold_s']
        if (not math.isfinite(peak) or peak < 0 or not math.isfinite(threshold)
                or threshold != max(parameters['settings']['threshold_floor_s'],
                                    peak * parameters['settings']['threshold_margin'])):
            raise ValueError('invalid frozen threshold calibration')


def freeze(development_dir, output):
    if output.exists():
        raise ValueError('output already exists; choose a new directory')
    parameters = read_json(development_dir/'detector_parameters.json')
    models = read_json(development_dir/'background_models.json')
    validate_contract(parameters, models)
    if digest(development_dir/'background_models.json') != parameters['model_sha256']:
        raise ValueError('development model hash mismatch')
    run = read_json(development_dir/'run_config.json')
    split = read_json(DEFAULT_SPLIT)
    if (run['subset'] != 'development' or run['sessions'] != select_sessions(split, 'development')
            or any(run.get(k) != v for k, v in parameters.items())):
        raise ValueError('development run/parameter/split mismatch')
    with (development_dir/'summary.csv').open() as stream:
        summary = list(csv.DictReader(stream))
    expected = {(s, sensor) for s in run['sessions'] for sensor in ('rgb', 'evs')}
    if len(summary) != len(expected) or {(r['session'], r['method']) for r in summary} != expected:
        raise ValueError('incomplete development summary')
    for row in summary:
        if (row['status'] != 'complete' or float(row['threshold_s']) !=
                parameters['calibration'][row['method']]['threshold_s']):
            raise ValueError('development summary threshold/status mismatch')
    output.mkdir(parents=True)
    for src, dst in [('detector_parameters.json', 'detector_parameters.json'),
                     ('background_models.json', 'background_models.json'),
                     ('run_config.json', 'development_run_config.json'),
                     ('summary.csv', 'development_summary.csv')]:
        shutil.copyfile(development_dir/src, output/dst)
    shutil.copyfile(DEFAULT_SPLIT, output/'split.json')
    write_json(output/'freeze.json', dict(
        schema_version=1, status='frozen_for_internal_evaluation', algorithm=ALGORITHM,
        source_development_dir=str(development_dir.resolve()),
        files_sha256={name: digest(output/name) for name in PACKAGE_FILES},
        code_sha256=current_code(), evaluation_sessions=select_sessions(split, 'evaluation'),
        selection_note='Development positives were inspected when choosing the settings; '
                       'this freeze records that chosen model, not an independent development result.',
        prior_exposure=split['prior_exposure'],
        policy='No refitting, threshold recalibration, parameter overrides, or onset/phase gating.'))
    print(f'Frozen: {output}/freeze.json', flush=True)


def load_frozen(folder):
    manifest = read_json(folder/'freeze.json')
    if (manifest['schema_version'] != 1 or manifest['status'] != 'frozen_for_internal_evaluation'
            or manifest['algorithm'] != ALGORITHM or set(manifest['files_sha256']) != set(PACKAGE_FILES)):
        raise ValueError('invalid frozen package')
    for name, value in manifest['files_sha256'].items():
        if digest(folder/name) != value:
            raise ValueError(f'frozen file changed: {name}')
    if manifest['code_sha256'] != current_code():
        raise ValueError('frozen runtime code mismatch; do not retune evaluation')
    if digest(folder/'split.json') != digest(DEFAULT_SPLIT):
        raise ValueError('frozen split changed')
    parameters = read_json(folder/'detector_parameters.json')
    models = read_json(folder/'background_models.json')
    validate_contract(parameters, models)
    if digest(folder/'background_models.json') != parameters['model_sha256']:
        raise ValueError('frozen model/parameter hash mismatch')
    if manifest['evaluation_sessions'] != select_sessions(read_json(DEFAULT_SPLIT), 'evaluation'):
        raise ValueError('unexpected evaluation sessions')
    return parameters, models, manifest


def check_extraction_contract(frozen_dir, source, tile_px, split_path):
    """Called before opening evaluation RAW; also checked on cached input."""
    parameters, _, manifest = load_frozen(frozen_dir)
    if digest(split_path) != parameters['split_sha256']:
        raise ValueError('extraction split differs from frozen split')
    if input_definition(source, tile_px) != parameters['input_definition']:
        raise ValueError('ROI/calibration/tile/window/RGB extraction differs from frozen input')
    spatial = parameters['input_definition']['spatial']
    if digest(spatial['camchain']) != spatial['camchain_sha256']:
        raise ValueError('current calibration file differs from frozen input')
    return manifest


def load_evaluation_tiles(tile_dir, frozen_dir):
    parameters, models, manifest = load_frozen(frozen_dir)
    run = read_json(tile_dir/'run_config.json')
    sessions = manifest['evaluation_sessions']
    if (run['subset'] != 'evaluation' or run['sessions'] != sessions
            or run['split'] != read_json(DEFAULT_SPLIT)
            or run.get('frozen_manifest_sha256') != digest(frozen_dir/'freeze.json')):
        raise ValueError('tile cache does not match frozen evaluation split/package')
    if (run['exporter_sha256'] != manifest['code_sha256']['rc_popout_tile_activity.py']
            or run['extraction_code_sha256'] != manifest['code_sha256']['rc_popout_detection.py']):
        raise ValueError('tile extraction code differs from frozen runtime')
    source_path = Path(run['source_config_path'])
    if digest(source_path) != run['source_config_sha256'] or read_json(source_path) != run['source_config']:
        raise ValueError('source config changed since extraction')
    check_extraction_contract(frozen_dir, run['source_config'], run['tile_px'], DEFAULT_SPLIT)
    statuses = read_json(tile_dir/'summary.json')
    if (len(statuses) != len(sessions)
            or any(sum(r['session'] == s and r['status'] == 'complete' for r in statuses) != 1 for s in sessions)):
        raise ValueError('evaluation tile extraction incomplete; inspect tile summary.json')
    scenes, hashes = {}, {}
    negative_sessions = {s for g in run['split']['groups'] if g['condition'] == 'none' for s in g['evaluation']}
    for session in sessions:
        alignment, meta, checked = load_scene(tile_dir, session, run)
        saved = read_json(tile_dir/session/'result.json')
        if (saved['source_result']['camchain_sha256'] !=
                parameters['input_definition']['spatial']['camchain_sha256']):
            raise ValueError('cached calibration differs from frozen input')
        if ((alignment.get('rgb_first_visible_from_drive_s') is None) != (session in negative_sessions)):
            raise ValueError(f'{session}: onset annotation does not match fixed split condition')
        arrays = {}
        for sensor in ('rgb', 'evs'):
            path = tile_dir/session/f'{sensor}_tiles.npz'
            if digest(path) != saved['tile_arrays_sha256'][sensor]:
                raise ValueError(f'{session}: cached {sensor} tiles changed after extraction')
            with np.load(path, allow_pickle=False) as data:
                arrays[sensor] = {k: data[k] for k in data.files}
            validate_data(arrays[sensor], meta, sensor)
            validate_model(models[sensor], arrays[sensor], sensor)
            checked[sensor+'_tiles_sha256'] = digest(path)
        if scenes and meta != next(iter(scenes.values()))['meta']:
            raise ValueError('evaluation tile geometry differs across sessions')
        scenes[session] = dict(meta=meta, alignment=alignment, data=arrays, source_hashes=checked)
        hashes[session] = checked
    provenance = dict(tile_dir=str(tile_dir.resolve()), tile_run_sha256=digest(tile_dir/'run_config.json'),
                      input_sha256=hashes, verification='Current annotation/sync/source checked; cached tiles used.')
    return scenes, parameters, models, manifest, provenance


def run_inference(scenes, parameters, models, output):
    """Shared fixed inference for evaluation and local development replay checks."""
    settings = parameters['settings']
    summaries, all_candidates = [], []
    for name, scene in scenes.items():
        out = output/name
        out.mkdir()
        for sensor in ('rgb', 'evs'):
            result = score_maps(scene['data'][sensor], scene['meta'], sensor, settings,
                                models[sensor], parameters['input_definition']['tile_px'])
            summary, candidates, rows, alarm, active = report_result(
                name, sensor, scene, result, parameters['calibration'][sensor], settings)
            onset = scene['alignment'].get('rgb_first_visible_from_drive_s')
            times = result['time_s'] - scene['alignment']['drive_start_s']
            # Extra diagnostics are reporting only. Early alarms are retained.
            summary['ready_observed_seconds'] = float(result['observed_step_s'].sum())
            summary['candidates_before_onset'] = None if onset is None else int(alarm[times < onset].sum())
            i = -1 if onset is None else int(np.searchsorted(times, onset, side='right'))-1
            observable = (i >= 0 and result['ready'][i]
                          and onset-times[i] <= settings[f'{sensor}_max_gap_s'])
            summary['active_at_rgb_onset'] = bool(active[i]) if observable else None
            summary['onset_state_age_ms'] = (onset-times[i])*1000 if observable else None
            # Do not make a hit/miss label from proximity alone. List all candidates for spatial review.
            summaries.append(summary)
            all_candidates.extend(candidates)
            result.update(alarm=alarm, active=active)
            write_csv(out/f'{sensor}_background_scores.csv', rows)
            np.savez_compressed(out/f'{sensor}_background_maps.npz', **result)
            write_json(out/f'{sensor}_candidates.json', candidates)
            print(f'{name} / {sensor}: candidates={summary["candidates"]}, '
                  f'before_guard={summary["candidates_before_guard"]}, '
                  f'first-onset={summary["first_candidate_minus_onset_ms"]} ms', flush=True)
        write_json(out/'result.json', dict(alignment=scene['alignment'], geometry=scene['meta'],
                                         input_hashes=scene.get('source_hashes', {}), algorithm=ALGORITHM,
                                         parameters_sha256=digest(output/'detector_parameters.json')))
    write_csv(output/'summary.csv', summaries)
    write_json(output/'summary.json', summaries)
    write_csv(output/'candidates.csv', all_candidates, fields=CANDIDATE_FIELDS)
    return summaries, all_candidates


def evaluate(tile_dir, frozen_dir, output):
    if output.exists():
        raise ValueError('output already exists; choose a new directory')
    scenes, parameters, models, manifest, provenance = load_evaluation_tiles(tile_dir, frozen_dir)
    output.mkdir(parents=True)
    for name in ('detector_parameters.json', 'background_models.json', 'freeze.json'):
        shutil.copyfile(frozen_dir/name, output/name)
    write_json(output/'run_config.json', dict(
        subset='evaluation', sessions=list(scenes), frozen_dir=str(frozen_dir.resolve()),
        frozen_manifest_sha256=digest(frozen_dir/'freeze.json'), code_sha256=current_code(),
        parameters_sha256=digest(output/'detector_parameters.json'), provenance=provenance,
        numpy_version=np.__version__, status='running',
        note='Internal evaluation with prior aggregate exposure. No fitting or tuning. '
             'All annotated intervals, both sensors, all candidates retained; spatial review pending.'))
    summaries, _ = run_inference(scenes, parameters, models, output)
    run = read_json(output/'run_config.json')
    run['status'] = 'complete'
    write_json(output/'run_config.json', run)
    rows = ''.join('<tr>'+''.join(f'<td>{html.escape(str(r[k]))}</td>' for k in
                   ('session', 'method', 'candidates', 'candidates_before_guard',
                    'first_candidate_minus_onset_ms'))+'</tr>' for r in summaries)
    (output/'index.html').write_text('<!doctype html><meta charset="utf-8">'
        '<title>Grid background internal evaluation</title>'
        '<h1>固定設定による内部評価</h1><p>候補は車両検知の確定ではありません。空間的な対応を別途確認します。</p>'
        '<p><a href="summary.csv">Summary</a> | <a href="candidates.csv">全候補の時刻・タイル</a></p>'
        '<table border="1"><tr><th>session</th><th>sensor</th><th>candidates</th>'
        '<th>before guard</th><th>first − RGB onset [ms]</th></tr>'+rows+'</table>')
    print(f'Report: {output}/index.html', flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    freeze_parser = commands.add_parser('freeze', help='Copy and lock a chosen development result; no fitting')
    freeze_parser.add_argument('--development-dir', required=True, type=Path)
    freeze_parser.add_argument('--output', required=True, type=Path)
    run_parser = commands.add_parser('run', help='Evaluate exactly the ten fixed internal evaluation records')
    run_parser.add_argument('--tile-dir', required=True, type=Path)
    run_parser.add_argument('--frozen-dir', required=True, type=Path)
    run_parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        if args.command == 'freeze':
            freeze(args.development_dir, args.output)
        else:
            evaluate(args.tile_dir, args.frozen_dir, args.output)
        return 0
    except (OSError, ValueError, KeyError, TypeError, ImportError, StopIteration) as error:
        parser.exit(1, f'error: {error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
