"""Audit received static review artifacts, without running or changing the detector.

Usage: python audit_received.py REVIEW_FOLDER [--output NEW_FOLDER]
Requires NumPy, OpenCV and Tesseract. Visual decisions are separate manual input.
"""
import argparse
import bisect
import csv
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tempfile

import cv2
import numpy as np


def read(path):
    return json.loads(path.read_text())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    here = Path(__file__).resolve().parent
    repo = here.parents[4]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('review_folder', type=Path)
    parser.add_argument('--output', type=Path, default=here)
    args = parser.parse_args()
    source = args.review_folder.resolve()
    output = args.output.resolve()
    assert source != output and source not in output.parents
    manifest = read(source / 'review_manifest.json')
    reports = read(source / 'summary.json')
    run = read(source / 'run_config.json')
    assert read(source / 'review_errors.json') == []
    assert len(reports) == len(manifest['scenes']) == 24
    assert len({s['output_relative'] for s in manifest['scenes']}) == 24
    frozen = repo / 'docs/evidence/rc_popout_20260930/grid_background_frozen_20261006/freeze.json'
    assert digest(frozen) == manifest['frozen_manifest_sha256']
    for name, sha in run['review_code_sha256'].items():
        assert Path(name).name == name
        assert digest(repo / 'tools' / name) == sha
    for name, records in [('candidates_user_paste.csv', [s['source_candidate'] for s in manifest['scenes']]),
                          ('summary_user_paste.csv', manifest['summaries'])]:
        with (here.parent / name).open() as f:
            pasted = list(csv.DictReader(f))
        normalized = [{k: '' if v is None else str(v) for k, v in r.items()} for r in records]
        assert pasted == normalized, name
    assert len(manifest['summaries']) == 36
    videos, labels = [], []
    with tempfile.TemporaryDirectory(prefix='static-review-audit-') as tmp:
        crops = []
        for scene, report in zip(manifest['scenes'], reports):
            folder = source / scene['output_relative']
            assert report['status'] == 'complete'
            assert report['source_decode_strategy'] == 'sequential_from_start'
            assert report['candidate'] == scene
            assert dict(read(folder / 'summary.json'), output_relative=scene['output_relative']) == report
            for key in ('annotation_sha256', 'time_sync_sha256'):
                assert scene[key] == report[key]
            with (folder / 'frames.csv').open() as f:
                rows = list(csv.DictReader(f))
            assert len(rows) == report['frames']
            times = [float(r['recording_relative_s']) for r in rows]
            candidate = scene['candidate_recording_s']
            onset = scene['rgb_onset_recording_s']
            cross = bisect.bisect_left(times, candidate)
            assert 0 < cross < len(rows)
            assert abs((times[cross] - candidate) * 1000 - report['first_display_after_candidate_ms']) < 1e-7
            for i, row in enumerate(rows):
                assert int(row['output_frame']) == i
                assert int(row['source_frame']) == int(rows[0]['source_frame']) + i
                assert i == 0 or times[i] > times[i - 1]
                assert abs(float(row['candidate_delta_ms']) - (times[i] - candidate) * 1000) < 1e-7
                assert abs(float(row['onset_delta_ms']) - (times[i] - onset) * 1000) < 1e-7
                assert abs(float(row['reference_time_s']) - times[i] - scene['reference_origin_s']) < 1e-6
            snapshots = {'before_onset': max(0, bisect.bisect_right(times, onset - .1) - 1),
                         'onset_after': bisect.bisect_left(times, onset),
                         'candidate_before': cross - 1, 'candidate_after': cross}
            cap = cv2.VideoCapture(str(folder / 'candidate_review.mp4'))
            assert cap.isOpened()
            maes, i = [], 0
            while True:
                ok, frame = cap.read()
                if not ok:
                    break
                assert frame.shape == (608, 1280, 3)
                for name, index in snapshots.items():
                    if i != index:
                        continue
                    still = cv2.imread(str(folder / f'{name}.png'))
                    assert still.shape == frame.shape
                    maes.append(float(np.mean(np.abs(frame.astype(float) - still))))
                    crop = Path(tmp) / f'{len(crops):03d}.png'
                    assert cv2.imwrite(str(crop), still[:32, :280])
                    crops.append(crop)
                    labels.append(dict(candidate=scene['output_relative'], snapshot=name, mapped_time_s=times[i]))
                i += 1
            cap.release()
            assert i == report['frames'] and len(maes) == 4
            assert max(maes) < 10
            videos.append(dict(candidate=scene['output_relative'], decoded_frames=i,
                               png_vs_video_max_mae=max(maes), first_display_after_candidate_ms=report['first_display_after_candidate_ms']))
        listing = Path(tmp) / 'images.txt'
        listing.write_text('\n'.join(map(str, crops)))
        result = subprocess.run(['tesseract', str(listing), 'stdout', '--psm', '7'],
                                text=True, capture_output=True, check=True)
        texts = result.stdout.split('\f')
        if texts and not texts[-1].strip():
            texts.pop()
        assert len(texts) == len(labels) == 96
        for row, text in zip(labels, texts):
            match = re.search(r't\s*=\s*([0-9]+\.[0-9]+)\s*s', text)
            row['ocr_text'] = text.strip()
            row['burned_rgb_time_s'] = float(match[1]) if match else None
            row['burned_minus_mapped_ms'] = None if not match else (row['burned_rgb_time_s'] - row['mapped_time_s']) * 1000
    files = [dict(path=str(p.relative_to(source)), bytes=p.stat().st_size, sha256=digest(p))
             for p in sorted(source.rglob('*')) if p.is_file() and p.name != '.DS_Store']
    unreadable = sum(r['burned_rgb_time_s'] is None for r in labels)
    mismatches = sum(r['burned_minus_mapped_ms'] is not None and abs(r['burned_minus_mapped_ms']) > .75 for r in labels)
    result = dict(source_directory=str(source), audit_script_sha256=digest(Path(__file__)),
                  review_code_hashes_match_working_copy=True, frozen_manifest_hash_matches=True,
                  candidates_match_pasted_csv=True, summaries_match_pasted_csv=True,
                  received_files=files, decoded_videos=videos,
                  total_decoded_frames=sum(v['decoded_frames'] for v in videos),
                  timestamp_labels=dict(checked=len(labels), unreadable=unreadable, mismatches=mismatches,
                                        tolerance_ms=.75, max_abs_error_ms=max(abs(r['burned_minus_mapped_ms']) for r in labels if r['burned_minus_mapped_ms'] is not None)),
                  limitations='Contact sheets are visually reviewed separately. Full videos were decoded, not continuously watched. Original full previews, RAW/bag and detector arrays are unavailable locally; this audit does not establish physical synchronization or causality.')
    output.mkdir(parents=True, exist_ok=True)
    (output / 'verification.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    with (output / 'timestamp_label_audit.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(labels[0]))
        writer.writeheader()
        writer.writerows(labels)
    print(json.dumps({k: v for k, v in result.items() if k not in ('received_files', 'decoded_videos')}, ensure_ascii=False, indent=2))
    assert unreadable == mismatches == 0


if __name__ == '__main__':
    main()
