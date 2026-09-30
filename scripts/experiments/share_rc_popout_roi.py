#!/usr/bin/env python3
"""Copy saved ROIs to compatible annotation previews, preserving each scene's timing."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools/led_sync_viewer'))
import annotations as ann

GEOMETRY = ('view_frame', 'output_size', 'projection', 'depth_m', 'camchain_sha256',
            'rgb_to_view_homography', 'event_to_view_homography')


def input_value(saved):
    result = copy.deepcopy(saved)
    result['onset'] = {key: value['preview_frame'] if isinstance(value, dict) else value
                       for key, value in saved.get('onset', {}).items()}
    return result


def check_clock(saved, summary):
    path = Path(saved['time_sync'])
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != saved['time_sync_sha256'] or digest != summary['time_sync_sha256']:
        raise ValueError('保存後に同期が変更されています。動画と時間注釈を確認してください')


def prepare(root, source, targets):
    dataset = root / 'analysis/led_sync' / source / 'led_sync_data.json'
    source_saved = json.loads(ann.annotation_file(dataset).read_text())
    _, source_summary, _ = ann.preview(dataset, source_saved['preview_id'])
    check_clock(source_saved, source_summary)
    source_valid = ann.validate(dataset, input_value(source_saved))
    if not source_valid['rois']:
        raise ValueError('コピー元に保存済みROIがありません')
    if any(source_summary.get(key) is None for key in GEOMETRY if key != 'depth_m'):
        raise ValueError('コピー元の座標・校正情報が不足しています')
    jobs = []
    for target in targets:
        if target == source:
            continue
        data = root / 'analysis/led_sync' / target / 'led_sync_data.json'
        saved = json.loads(ann.annotation_file(data).read_text())
        revision = ann.revision(ann.annotation_file(data))
        current = ann.context(data)
        candidates = sorted(current['previews'], key=lambda p: p['id'] != saved['preview_id'])
        selected = None
        for candidate in candidates:
            _, summary, _ = ann.preview(data, candidate['id'])
            if all(summary.get(key) == source_summary.get(key) for key in GEOMETRY):
                selected = candidate['id']
                break
        if selected is None:
            raise ValueError(f'{target}: 同じ校正・投影のEVS座標動画がありません')
        context = ann.context(data, selected)  # Validates time-preserving preview migration.
        value = input_value(context['annotation'])
        check_clock(saved, context['summary'])
        value['rois'] = copy.deepcopy(source_valid['rois'])
        validated = ann.validate(data, value)
        if validated['intervals'] != saved['intervals']:
            raise ValueError(f'{target}: 時間区間を維持できません')
        for key, onset in saved.get('onset', {}).items():
            if validated['onset'][key]['rgb_time_s'] != onset['rgb_time_s']:
                raise ValueError(f'{target}: 出現時刻を維持できません')
        jobs.append((data, value, revision))
    return jobs, source_valid['rois']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record-root', required=True, type=Path)
    parser.add_argument('--source', required=True, help='Source session folder name, e.g. t_0.2-none')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    root = args.record_root.resolve()
    targets = sorted(p.name for p in root.iterdir() if p.is_dir() and p.name != 'analysis'
                     and '_calibration_' not in p.name and any(p.glob('*.mcap')))
    if args.source not in targets:
        parser.error('source must be an MCAP session directly inside record-root')
    try:
        jobs, rois = prepare(root, args.source, targets)
        print(f'Source: {args.source}\nROIs: {json.dumps(rois, ensure_ascii=False)}')
        print(f'Scenes: {len(targets)} (source unchanged, destinations={len(jobs)})')
        # All destinations are validated before the first write.
        for data, value, revision in jobs:
            if args.dry_run:
                print(f'[dry-run] {data.parent.name}: preview={value["preview_id"]}')
            else:
                result = ann.save(data, dict(annotation=value, revision=revision))
                print(f'Saved: {result["path"]} (previous JSON backed up)')
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(1, f'error: {error}\n')


if __name__ == '__main__':
    main()
