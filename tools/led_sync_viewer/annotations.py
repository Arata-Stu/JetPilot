"""Sequence annotations and validated video/source-time manifests."""
from __future__ import annotations
import csv
import hashlib
import json
import math
import threading
import time
import uuid
from pathlib import Path

_LOCK = threading.Lock()


def contained(path, root):
    path = Path(path).resolve()
    if not path.is_relative_to(Path(root).resolve()):
        raise ValueError('許可された記録フォルダ外です')
    return path


def scene_root(dataset):
    if dataset.parents[1].name != 'led_sync' or dataset.parents[2].name != 'analysis':
        raise ValueError('標準の analysis/led_sync/<session> 配置が必要です')
    return contained(dataset.parents[2] / 'scenario_overlay' / dataset.parent.name, dataset.parents[3])


def preview(dataset, preview_id):
    root = scene_root(dataset)
    if not preview_id or Path(preview_id).name != preview_id:
        raise ValueError('動画IDが不正です')
    folder = contained(root / preview_id, root)
    for name in ('summary.yaml', 'frames.csv', 'rgb_vs_overlay.mp4'):
        if not contained(folder / name, root).is_file():
            raise ValueError('時刻対応表付き動画が必要です。確認動画を生成してください')
    summary_json = contained(folder / 'summary.json', root)
    if summary_json.is_file():
        summary = json.loads(summary_json.read_text())
    else:
        import yaml
        summary = yaml.safe_load((folder / 'summary.yaml').read_text())
    if Path(summary['bag']).name != dataset.parent.name:
        raise ValueError('動画とシーケンスが一致しません')
    fps = float(summary['fps'])
    if not math.isfinite(fps) or fps <= 0:
        raise ValueError('動画FPSが不正です')
    rows = []
    with (folder / 'frames.csv').open() as stream:
        for i, row in enumerate(csv.DictReader(stream)):
            entry = {key: float(row[key]) for key in (
                'video_time_s', 'reference_time_s', 'relative_time_s', 'rgb_time_s')}
            if not all(math.isfinite(v) for v in entry.values()):
                raise ValueError('動画時刻が不正です')
            if int(row['frame']) != i or (rows and entry['video_time_s'] <= rows[-1]['video_time_s']):
                raise ValueError('フレーム対応表が不正です')
            rows.append(entry)
    if not rows:
        raise ValueError('動画時刻がありません')
    return folder, summary, rows


def annotation_file(dataset):
    return contained(dataset.parent / 'sequence_annotations.json', dataset.parent)


def revision(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def context(dataset, selected=None):
    root = scene_root(dataset)
    previews = []
    if root.is_dir():
        for folder in sorted(root.iterdir(), key=lambda p: p.name, reverse=True):
            if not folder.is_dir() or '.before_' in folder.name:
                continue
            try:
                _, summary, _ = preview(dataset, folder.name)
            except (ValueError, OSError, KeyError, TypeError):
                continue
            previews.append({'id': folder.name, 'view_frame': summary.get('view_frame', 'rgb'),
                             'projection': summary.get('projection'), 'depth_m': summary.get('depth_m')})
    path = annotation_file(dataset)
    saved = json.loads(path.read_text()) if path.is_file() else None
    if not selected and saved and any(p['id'] == saved['preview_id'] for p in previews):
        selected = saved['preview_id']
    if not selected and previews:
        selected = next((p['id'] for p in previews if p['view_frame'] == 'evs'), previews[0]['id'])
    result = dict(session=dataset.parent.name, previews=previews, annotation=saved,
                  revision=revision(path), selected=selected)
    if selected:
        folder, summary, rows = preview(dataset, selected)
        result.update(summary=summary, frames=rows, video=str(folder / 'rgb_vs_overlay.mp4'))
    return result


def number(value):
    if isinstance(value, bool):
        raise ValueError('数値が必要です')
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('有限の数値が必要です')
    return value


def validate(dataset, value):
    if not isinstance(value, dict) or value.get('session') != dataset.parent.name:
        raise ValueError('注釈のシーケンスが一致しません')
    folder, summary, frames = preview(dataset, value.get('preview_id'))
    low, high = frames[0]['relative_time_s'], frames[-1]['relative_time_s']
    intervals = []
    if not isinstance(value.get('intervals'), list) or len(value['intervals']) > 1000:
        raise ValueError('区間一覧が不正です')
    for item in value['intervals']:
        if not isinstance(item, dict):
            raise ValueError('区間の形式が不正です')
        start, end = number(item['start_s']), number(item['end_s'])
        if not low <= start < end <= high:
            raise ValueError('区間は動画の元時刻範囲内で、開始 < 終了にしてください')
        if item['label'] not in ('evaluation', 'exclude', 'clip'):
            raise ValueError('区間ラベルが不正です')
        intervals.append(dict(start_s=start, end_s=end, label=item['label'], note=str(item.get('note', ''))[:1000]))
    if not isinstance(value.get('onset', {}), dict):
        raise ValueError('出現注釈が不正です')
    onset = {}
    for key in ('last_hidden', 'first_visible'):
        index = value.get('onset', {}).get(key)
        if index is not None:
            if isinstance(index, bool) or not isinstance(index, int) or not 0 <= index < len(frames):
                raise ValueError('出現注釈のフレームが不正です')
            onset[key] = {'preview_frame': index, **frames[index]}
    if len(onset) == 2 and onset['last_hidden']['rgb_time_s'] >= onset['first_visible']['rgb_time_s']:
        raise ValueError('最後に見えないRGB時刻 < 最初に見えるRGB時刻にしてください')
    rois = value.get('rois', [])
    if not isinstance(rois, list) or len(rois) > 20:
        raise ValueError('ROI一覧が不正です')
    spatial = None
    if rois and any(not isinstance(item, dict) for item in rois):
        raise ValueError("ROIの形式が不正です")
    if rois:
        if summary.get('view_frame') != 'evs':
            raise ValueError('ROIはEVS共通視野の動画で指定してください')
        import numpy as np
        mask_path = contained(folder / 'common_valid_mask.png', folder)
        try:
            import cv2
            mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        except ImportError:
            from PIL import Image
            mask = np.asarray(Image.open(mask_path).convert("L"))
        if mask is None:
            raise ValueError('共通視野マスクがありません')
        height, width = mask.shape
        cleaned = []
        for item in rois:
            values = [number(item[k]) for k in ('x', 'y', 'width', 'height')]
            if any(v != int(v) for v in values):
                raise ValueError('ROI座標は整数で指定してください')
            x, y, w, h = map(int, values)
            if not (0 <= x < x+w <= width and 32 <= y < y+h <= height):
                raise ValueError('ROIが画像外または動画の上部ラベルに重なっています')
            if not np.all(mask[y:y+h, x:x+w] == 255):
                raise ValueError('ROIに共通視野外の画素が含まれます')
            cleaned.append(dict(name=str(item.get('name', 'ROI'))[:80], x=x, y=y, width=w, height=h))
        rois = cleaned
        spatial = {key: summary.get(key) for key in (
            'view_frame', 'output_size', 'projection', 'depth_m', 'camchain', 'camchain_sha256',
            'rgb_to_view_homography', 'event_to_view_homography')}
        spatial['common_valid_mask'] = str(mask_path)
    if spatial is None and summary.get('view_frame') == 'evs':
        spatial = {key: summary.get(key) for key in ('view_frame', 'output_size', 'projection', 'depth_m', 'camchain', 'camchain_sha256', 'rgb_to_view_homography', 'event_to_view_homography')}
        spatial['common_valid_mask'] = str(contained(folder / 'common_valid_mask.png', folder))
    return dict(schema_version=1, session=dataset.parent.name, preview_id=folder.name,
                time_basis='relative_to_first_rgb_timestamp',
                reference_origin_s=frames[0]['reference_time_s'] - low,
                rgb_timestamp_source=summary.get('rgb_timestamp_source', 'bag'),
                time_sync=summary.get('time_sync'), time_sync_sha256=summary.get('time_sync_sha256'),
                intervals=intervals, onset=onset, rois=rois, spatial=spatial,
                updated_at=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()))


def save(dataset, payload):
    value = validate(dataset, payload.get('annotation'))
    path = annotation_file(dataset)
    with _LOCK:
        if payload.get('revision') != revision(path):
            raise ValueError('別の画面で注釈が更新されました。再読込してください')
        if path.is_file():
            backup = contained(path.parent / f'sequence_annotations.{uuid.uuid4().hex}.bak.json', path.parent)
            backup.write_bytes(path.read_bytes())
        temporary = path.parent / f'.annotations-{uuid.uuid4().hex}.tmp'
        temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
        temporary.replace(path)
    return dict(annotation=value, revision=revision(path), path=str(path))
