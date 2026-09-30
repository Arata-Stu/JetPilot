#!/usr/bin/env python3
"""Review a fixed ROI without modifying saved annotations or recordings."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys

from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools/led_sync_viewer'))
import annotations as ann
from serve import annotation_time_sync

GEOMETRY = ('view_frame', 'output_size', 'projection', 'depth_m',
            'camchain_sha256', 'rgb_to_view_homography', 'event_to_view_homography')


def extract_frame(video, seconds):
    result = subprocess.run([
        'ffmpeg', '-v', 'error', '-i', str(video), '-ss', str(seconds),
        '-frames:v', '1', '-f', 'image2pipe', '-vcodec', 'png', 'pipe:1',
    ], check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return Image.open(io.BytesIO(result.stdout)).convert('RGB')


def inspect_scene(dataset, candidate, output):
    saved = json.loads(ann.annotation_file(dataset).read_text())
    if saved['session'] != dataset.parent.name:
        raise ValueError('annotation session mismatch')
    folder, summary, frames = ann.preview(dataset, saved['preview_id'])
    if summary.get('view_frame') != 'evs':
        raise ValueError('EVS-coordinate preview required')
    if any(summary.get(k) is None for k in GEOMETRY if k != 'depth_m'):
        raise ValueError('missing calibration/projection metadata')
    digest = hashlib.sha256(annotation_time_sync(dataset).read_bytes()).hexdigest()
    if saved.get('time_sync_sha256') != digest or summary.get('time_sync_sha256') != digest:
        raise ValueError('annotation/video does not match current saved synchronization')
    width, height = summary['output_size']
    x, y, w, h = (candidate[k] for k in ('x', 'y', 'width', 'height'))
    if not (0 <= x < x+w <= width and 32 <= y < y+h <= height):
        raise ValueError('candidate outside image or overlaps timestamp label')
    mask = Image.open(folder / 'common_valid_mask.png').convert('L')
    if mask.size != (width, height):
        raise ValueError('common mask dimensions mismatch')
    valid = mask.crop((x, y, x+w, y+h)).histogram()[255]
    if not valid:
        raise ValueError('candidate has no common-view pixels')
    origin = frames[0]['reference_time_s'] - frames[0]['relative_time_s']
    if abs(saved['reference_origin_s'] - origin) > 1e-6:
        raise ValueError('annotation/video time origin mismatch')
    evaluations = [v for v in saved['intervals'] if v['label'] == 'evaluation']
    if not evaluations:
        raise ValueError('evaluation interval missing')
    samples = []
    for i, interval in enumerate(evaluations, 1):
        start, end = interval['start_s'], interval['end_s']
        if not frames[0]['relative_time_s'] <= start < end <= frames[-1]['relative_time_s']:
            raise ValueError('evaluation interval outside preview')
        samples += [(f'eval{i} start', start), (f'eval{i} middle', (start+end)/2),
                    (f'eval{i} end', end)]
    visible = saved.get('onset', {}).get('first_visible')
    if visible:
        t = visible['rgb_time_s'] - origin
        if not any(v['start_s'] <= t <= v['end_s'] for v in evaluations):
            raise ValueError('first-visible time outside evaluation')
        samples.append(('first visible', t))
    if len(samples) > 40:
        raise ValueError('too many evaluation intervals for contact sheet')
    tiles = []
    for label, t in samples:
        row = min(frames, key=lambda r: abs(r['relative_time_s'] - t))
        image = extract_frame(folder / 'rgb_vs_overlay.mp4', row['video_time_s'])
        if image.size != (2*width, height):
            raise ValueError('unexpected comparison video dimensions')
        image = image.crop((0, 0, width, height))
        draw = ImageDraw.Draw(image)
        for old in saved.get('rois', []):
            ox, oy, ow, oh = (old[k] for k in ('x', 'y', 'width', 'height'))
            draw.rectangle((ox, oy, ox+ow-1, oy+oh-1), outline='orange', width=2)
        draw.rectangle((x, y, x+w-1, y+h-1), outline='lime', width=3)
        tile = Image.new('RGB', (width, height+28), 'black')
        tile.paste(image, (0, 28))
        ImageDraw.Draw(tile).text((5, 5), f'{label}: {row["relative_time_s"]:.3f}s', fill='white')
        tiles.append(tile)
    sheet = Image.new('RGB', (width*2, (height+28)*((len(tiles)+1)//2)), 'black')
    for i, tile in enumerate(tiles):
        sheet.paste(tile, ((i % 2)*width, (i//2)*(height+28)))
    sheet.save(output)
    return dict(session=saved['session'], preview_id=saved['preview_id'],
                geometry={k: summary.get(k) for k in GEOMETRY},
                candidate=candidate, valid_pixel_count=valid,
                valid_fraction=valid/(w*h), previous_rois=saved.get('rois', []),
                review_image=str(output), visual_review_required=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record-root', action='append', required=True, type=Path)
    parser.add_argument('--sessions', nargs='+', help='Limit to these session folder names')
    parser.add_argument('--output', required=True, type=Path, help='New output directory')
    args = parser.parse_args()
    for root in args.record_root:
        if not root.is_dir():
            parser.error(f'record root does not exist: {root}')
        if not any((root / 'analysis/led_sync').glob('*/sequence_annotations.json')):
            parser.error(f'no saved annotations under record root: {root}')
    datasets = sorted(set(p.resolve() for root in args.record_root
                          for p in (root / 'analysis/led_sync').glob('*/led_sync_data.json')
                          if ann.annotation_file(p).is_file()))
    if args.sessions:
        missing = set(args.sessions) - {p.parent.name for p in datasets}
        if missing:
            parser.error(f'missing annotated sessions: {sorted(missing)}')
        datasets = [p for p in datasets if p.parent.name in args.sessions]
    if not datasets:
        parser.error('no saved annotations found')
    if args.output.exists():
        parser.error('output already exists; choose a new directory')
    args.output.mkdir(parents=True)
    candidate = dict(name='band', x=0, y=70, width=640, height=272,
                     mask_policy='intersect_common')
    results, reference = [], None
    for i, dataset in enumerate(datasets, 1):
        try:
            result = inspect_scene(dataset, candidate,
                                   args.output / f'{i:02d}_{dataset.parent.name}.png')
            if reference is None:
                reference = result['geometry']
            result['status'] = 'OK' if result['geometry'] == reference else 'GEOMETRY_MISMATCH'
            print(f'{result["status"]}: {dataset.parent.name} '
                  f'valid={result["valid_pixel_count"]} ({result["valid_fraction"]:.1%})')
        except (OSError, ValueError, KeyError, TypeError, subprocess.CalledProcessError) as error:
            result = dict(session=dataset.parent.name, status='ERROR', error=str(error))
            print(f'ERROR: {dataset.parent.name}: {error}')
        result['dataset'] = str(dataset)
        results.append(result)
    (args.output / 'report.json').write_text(json.dumps(results, ensure_ascii=False, indent=2)+'\n')
    print(f'Report: {args.output}/report.json\nGreen=candidate; orange=previous ROI. '
          'Annotations unchanged. Visual review still required.')
    return int(any(r['status'] != 'OK' for r in results))


if __name__ == '__main__':
    raise SystemExit(main())
