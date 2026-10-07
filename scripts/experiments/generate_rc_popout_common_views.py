#!/usr/bin/env python3
"""Batch EVS-coordinate annotation previews using saved manual/automatic sync."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shlex
import subprocess
import sys
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools/led_sync_viewer'))
from serve import annotation_time_sync


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate_pinned_preview(output, settings):
    """Check actual renderer metadata, including on cache reuse."""
    if digest(Path(settings['sync'])) != settings['sync_sha256']:
        raise ValueError('Pinned time sync changed while preparing preview')
    summary = json.loads((output/'summary.json').read_text())
    for key, expected in (('time_sync_sha256', settings['sync_sha256']),
                          ('rgb_timestamp_source', settings['rgb_timestamp_source'])):
        if summary.get(key) != expected:
            raise ValueError(f'{output}: {key} mismatch: expected={expected!r}, actual={summary.get(key)!r}')
    expected = settings.get('expected_reference_origin_s')
    if expected is not None:
        actual = summary.get('reference_origin_s')
        if (isinstance(actual, bool) or not isinstance(actual, (int, float))
                or not math.isfinite(actual) or abs(actual-expected) > 1e-6):
            raise ValueError(f'{output}: reference_origin_s mismatch: expected={expected!r}, actual={actual!r}')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record-root', type=Path, required=True)
    parser.add_argument('--sessions', nargs='+', help='Only these immediate session folder names')
    parser.add_argument('--camchain', type=Path, required=True)
    parser.add_argument('--projection', choices=['rotation-only', 'fixed-depth'], default='rotation-only')
    parser.add_argument('--depth-m', type=float, help='Required for fixed-depth; plane assumption, not measured depth')
    parser.add_argument('--time-sync', type=Path, help='Pin the saved analysis sync; single session only, no manual/auto fallback')
    parser.add_argument('--time-sync-sha256', help='Require the pinned sync to match this analysis hash')
    parser.add_argument('--rgb-timestamp-source', choices=['bag', 'header'], help='Override LED-export timestamp source with the saved analysis source')
    parser.add_argument('--reference-origin-s', type=float, help='Verify generated RGB recording origin against analysis; single session only')
    parser.add_argument('--dry-run', action='store_true', help='Print commands without ROS or writing files')
    args = parser.parse_args(argv)
    if args.projection == 'fixed-depth' and args.depth_m is None:
        parser.error('--depth-m is required for fixed-depth')
    if args.depth_m is not None and (not math.isfinite(args.depth_m) or args.depth_m <= 0):
        parser.error('--depth-m must be positive and finite')
    if args.reference_origin_s is not None and not math.isfinite(args.reference_origin_s):
        parser.error('--reference-origin-s must be finite')
    if args.time_sync_sha256 is not None and args.time_sync is None:
        parser.error('--time-sync-sha256 requires --time-sync')
    root, chain = args.record_root.resolve(), args.camchain.resolve()
    if not root.is_dir() or not chain.is_file():
        parser.error('record-root directory and camchain file must exist')
    sessions = sorted(p for p in root.iterdir() if p.is_dir() and p.name != 'analysis'
                      and '_calibration_' not in p.name and any(p.glob('*.mcap')))
    if args.sessions:
        requested = set(args.sessions)
        missing = requested - {p.name for p in sessions}
        if missing:
            parser.error(f'Unknown MCAP sessions: {", ".join(sorted(missing))}')
        sessions = [p for p in sessions if p.name in requested]
    if not sessions:
        parser.error('No MCAP sessions found in record-root')
    if (args.time_sync is not None or args.reference_origin_s is not None) and len(sessions) != 1:
        parser.error('Pinned sync/origin requires exactly one session')
    failures, results = 0, []
    for i, session in enumerate(sessions, 1):
        print(f'[{i}/{len(sessions)}] {session.name}', flush=True)
        try:
            dataset = root / 'analysis/led_sync' / session.name / 'led_sync_data.json'
            sync = args.time_sync.resolve() if args.time_sync is not None else annotation_time_sync(dataset)
            if not sync.is_file():
                raise ValueError('保存済み同期YAMLがありません。先にLED同期を保存してください')
            sync_hash = digest(sync)
            if args.time_sync_sha256 is not None and sync_hash != args.time_sync_sha256:
                raise ValueError(f'{sync}: pinned time sync hash differs from saved analysis')
            raw = list(session.glob('*.raw'))
            if len(raw) != 1:
                raise ValueError('Exactly one RAW file is required')
            timestamp_source = args.rgb_timestamp_source
            if timestamp_source is None:
                meta = json.loads(dataset.read_text())['meta']
                timestamp_source = meta.get('rgb_timestamp_source', 'bag')
            if timestamp_source not in ('bag', 'header'):
                raise ValueError('Unknown RGB timestamp source')
            settings = dict(schema_version=1, session=str(session), sync=str(sync),
                            sync_sha256=sync_hash, camchain=str(chain), camchain_sha256=digest(chain),
                            projection=args.projection, depth_m=args.depth_m,
                            rgb_timestamp_source=timestamp_source, view_frame='evs',
                            event_window_ms=10, event_dilate_px=1,
                            inputs=[(str(p), p.stat().st_size, p.stat().st_mtime_ns)
                                    for p in sorted([*session.glob('*.mcap'), *raw])])
            if args.reference_origin_s is not None:
                settings['expected_reference_origin_s'] = args.reference_origin_s
            # JSON round-trip makes stored tuples comparable to lists on the next run.
            settings = json.loads(json.dumps(settings))
            signature = hashlib.sha256(json.dumps(settings, sort_keys=True).encode()).hexdigest()[:16]
            parent = root / 'analysis/scenario_overlay' / session.name
            output = parent / f'common_evs_batch_{signature}'
            marker = output / 'batch_settings.json'
            complete_files = ('rgb_vs_overlay.mp4', 'frames.csv', 'summary.json', 'summary.yaml')
            if (marker.is_file() and json.loads(marker.read_text()) == settings
                    and all((output / name).is_file() and (output / name).stat().st_size for name in complete_files)):
                if args.time_sync is not None or args.reference_origin_s is not None:
                    validate_pinned_preview(output, settings)
                print(f'  reuse: {output}', flush=True)
                results.append(dict(session=session.name, status='reused', output=str(output), time_sync=str(sync)))
                continue
            if output.exists():
                output = parent / f'common_evs_batch_{signature}_{uuid.uuid4().hex[:6]}'
            command = [sys.executable, '-m', 'multi_sensor_calibration.cli', 'scenario-overlay',
                       '--bag', str(session), '--event-file', str(raw[0]), '--time-sync', str(sync),
                       '--camchain', str(chain), '--view-frame', 'evs', '--projection', args.projection,
                       '--rgb-timestamp-source', timestamp_source,
                       '--event-window-ms', '10', '--event-dilate-px', '1', '--alpha', '0.85',
                       '--output-dir', str(output)]
            if args.depth_m is not None:
                command += ['--depth-m', str(args.depth_m)]
            print(f'  sync: {sync.name}'+(' (pinned to analysis)' if args.time_sync is not None else ''), flush=True)
            if args.rgb_timestamp_source is not None:
                print(f'  RGB timestamps: {timestamp_source}; expected origin: {args.reference_origin_s}', flush=True)
            if args.dry_run:
                print(shlex.join(command))
                continue
            parent.mkdir(parents=True, exist_ok=True)
            log = parent / f'{output.name}.log'
            print(f'  rendering: {output}\n  log: {log}', flush=True)
            with log.open('w') as stream:
                result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT)
            if result.returncode:
                raise RuntimeError(f'動画生成失敗: {log}')
            if not all((output / name).is_file() and (output / name).stat().st_size for name in complete_files):
                raise RuntimeError(f'動画または時刻対応表がありません: {output}')
            if args.time_sync is not None or args.reference_origin_s is not None:
                validate_pinned_preview(output, settings)
            (output / 'batch_settings.json').write_text(json.dumps(settings, indent=2) + '\n')
            results.append(dict(session=session.name, status='complete', output=str(output), time_sync=str(sync)))
            print('  complete', flush=True)
        except (OSError, ValueError, KeyError, TypeError, RuntimeError) as error:
            failures += 1
            print(f'  FAILED: {error}', file=sys.stderr, flush=True)
            results.append(dict(session=session.name, status='failed', error=str(error)))
    if not args.dry_run:
        report = root / 'analysis/common_views_summary.json'
        report.parent.mkdir(parents=True, exist_ok=True)
        report.write_text(json.dumps(results, ensure_ascii=False, indent=2) + '\n')
        print(f'Summary: {report}')
    print(f'Finished: sessions={len(sessions)} failed={failures}')
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())
