#!/usr/bin/env python3
"""Apply a reviewed common ROI with annotation backups and create a detection config."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools/led_sync_viewer'))
import annotations as ann
from serve import annotation_time_sync
from review_rc_popout_roi_candidate import GEOMETRY


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepare(report):
    if not isinstance(report, list) or not report:
        raise ValueError('empty review report')
    jobs, entries, names = [], [], set()
    common_roi = common_geometry = common_spatial = None
    for row in report:
        if row.get('status') != 'OK':
            raise ValueError(f'{row.get("session")}: review did not pass')
        dataset = Path(row['dataset'])
        path = ann.annotation_file(dataset)
        revision = ann.revision(path)
        saved = json.loads(path.read_text())
        name = saved['session']
        if name in names or name != row['session']:
            raise ValueError(f'{name}: duplicate or mismatched session')
        names.add(name)
        _, summary, _ = ann.preview(dataset, saved['preview_id'])
        geometry = {k: summary.get(k) for k in GEOMETRY}
        if geometry != row['geometry']:
            raise ValueError(f'{name}: geometry changed after review')
        sync = annotation_time_sync(dataset)
        if digest(sync) != saved['time_sync_sha256'] or digest(sync) != summary['time_sync_sha256']:
            raise ValueError(f'{name}: stale synchronization')
        if digest(summary['camchain']) != summary['camchain_sha256']:
            raise ValueError(f'{name}: calibration changed')
        value = copy.deepcopy(saved)
        value['onset'] = {k: v['preview_frame'] for k, v in saved.get('onset', {}).items()}
        value['rois'] = [row['candidate']]
        checked = ann.validate(dataset, value)
        if checked['intervals'] != saved['intervals']:
            raise ValueError(f'{name}: evaluation intervals would change')
        if any(checked['onset'][k]['rgb_time_s'] != v['rgb_time_s']
               for k, v in saved.get('onset', {}).items()):
            raise ValueError(f'{name}: onset would change')
        if checked['rois'][0]['valid_pixel_count'] != row['valid_pixel_count']:
            raise ValueError(f'{name}: common mask changed after review')
        if common_roi is None:
            common_roi, common_geometry, common_spatial = row['candidate'], geometry, checked['spatial']
        elif common_roi != row['candidate'] or common_geometry != geometry:
            raise ValueError(f'{name}: ROI/geometry differs between scenes')
        jobs.append((dataset, value, revision))
        entries.append(dict(session=name, annotation=str(path)))
    config = dict(schema_version=1, roi=common_roi, spatial=common_spatial, sessions=entries)
    return jobs, config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    try:
        if args.output.exists():
            raise ValueError('output already exists; choose a new config filename')
        jobs, config = prepare(json.loads(args.report.read_text()))
        for dataset, value, revision in jobs:
            if not args.dry_run:
                ann.save(dataset, dict(annotation=value, revision=revision))
            print(f'{"CHECK" if args.dry_run else "SAVED"}: {dataset.parent.name}')
        if not args.dry_run:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open('x') as stream:
                json.dump(config, stream, ensure_ascii=False, indent=2)
                stream.write('\n')
        print(f'Scenes={len(jobs)} / ROI={config["roi"]}\nConfig: {args.output}')
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(1, f'error: {error}\n')


if __name__ == '__main__':
    main()
