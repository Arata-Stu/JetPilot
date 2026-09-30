#!/usr/bin/env python3
"""Build a common-ROI analysis config from saved per-session annotations."""
import argparse
import hashlib
import json
from pathlib import Path


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def build(root, roi_name):
    sessions = sorted(p for p in root.iterdir() if p.is_dir() and p.name != 'analysis'
                      and '_calibration_' not in p.name and any(p.glob('*.mcap')))
    if not sessions:
        raise ValueError('MCAPシーケンスがありません')
    entries, common_roi, common_spatial = [], None, None
    geometry = ('view_frame', 'output_size', 'projection', 'depth_m', 'camchain_sha256',
                'rgb_to_view_homography', 'event_to_view_homography')
    for session in sessions:
        path = root / 'analysis/led_sync' / session.name / 'sequence_annotations.json'
        annotation = json.loads(path.read_text())
        if annotation['session'] != session.name:
            raise ValueError(f'{session.name}: session不一致')
        evaluation = [x for x in annotation['intervals'] if x['label'] == 'evaluation']
        if not evaluation or any(not x['start_s'] < x['end_s'] for x in evaluation):
            raise ValueError(f'{session.name}: 評価区間がありません／不正です')
        selected = [x for x in annotation['rois'] if x['name'] == roi_name]
        if len(selected) != 1:
            raise ValueError(f'{session.name}: ROI {roi_name!r} が一意に保存されていません')
        roi = {k: selected[0][k] for k in ('name', 'x', 'y', 'width', 'height', 'mask_policy')}
        spatial = annotation['spatial']
        if not spatial or spatial['view_frame'] != 'evs' or roi['mask_policy'] != 'intersect_common':
            raise ValueError(f'{session.name}: EVS座標の共通視野と交差するROIが必要です')
        if digest(spatial['camchain']) != spatial['camchain_sha256']:
            raise ValueError(f'{session.name}: 校正ファイルが変更されています')
        if digest(annotation['time_sync']) != annotation['time_sync_sha256']:
            raise ValueError(f'{session.name}: 注釈後に同期ファイルが変更されています')
        if common_roi is None:
            common_roi, common_spatial = roi, spatial
        elif roi != common_roi or any(spatial.get(k) != common_spatial.get(k) for k in geometry):
            raise ValueError(f'{session.name}: ROIまたは投影条件が他シーンと異なります')
        entries.append(dict(session=session.name, annotation=str(path)))
    return dict(schema_version=1, roi=common_roi, spatial=common_spatial, sessions=entries)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record-root', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--roi-name', default='band')
    args = parser.parse_args()
    try:
        config = build(args.record_root.resolve(), args.roi_name)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x') as stream:
            json.dump(config, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(1, f'error: {error}\n')
    print(f'Saved: {args.output} / scenes={len(config["sessions"])} / ROI={config["roi"]}')


if __name__ == '__main__':
    main()
