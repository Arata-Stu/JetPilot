"""Audit transferred review artifacts; no detector execution or source modification.

Usage: python audit_received.py REVIEW_FOLDER
Requires numpy, OpenCV, tesseract. Outputs beside this script.
"""
import bisect
import csv
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import tempfile
from collections import Counter

import cv2
import numpy as np


REVIEW_CODE_COMMIT = '9e0371c94f89a8092403dd5c5f4fc4312035c0de'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def write(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def main():
    source = Path(sys.argv[1]).resolve()
    output = Path(__file__).resolve().parent
    repo = output.parents[4]
    manifest = read(source/'review_manifest.json')
    run = read(source/'run_config.json')
    reports = read(source/'summary.json')
    assert read(source/'review_errors.json') == []
    assert len(reports) == len(manifest['scenes']) == 30
    assert all(r['status'] == 'complete' for r in reports)
    frozen = repo/'docs/evidence/rc_popout_20260930/grid_background_frozen_20261006/freeze.json'
    assert manifest['frozen_manifest_sha256'] == digest(frozen)
    # Preserve the received renderer version even after the local seek fix.
    for name, value in run['review_code_sha256'].items():
        assert Path(name).name == name
        blob = subprocess.run(['git', 'show', f'{REVIEW_CODE_COMMIT}:tools/{name}'],
                              cwd=repo, capture_output=True, check=True).stdout
        assert hashlib.sha256(blob).hexdigest() == value
    with (output.parent/'candidates_user_paste.csv').open() as stream:
        candidate_csv = list(csv.DictReader(stream))
    assert candidate_csv == [{k: '' if v is None else str(v) for k, v in s['source_candidate'].items()}
                             for s in manifest['scenes']]
    summaries = manifest['summaries']
    assert len(summaries) == 20
    with (output.parent/'summary_user_paste.csv').open() as stream:
        pasted = list(csv.DictReader(stream))
    assert pasted == [{k: '' if v is None else str(v) for k, v in s.items()} for s in summaries]
    inventory = [{'path':str(p.relative_to(source)), 'bytes':p.stat().st_size, 'sha256':digest(p)}
                 for p in sorted(source.rglob('*')) if p.is_file()]
    video_checks, timestamp_checks = [], []
    with tempfile.TemporaryDirectory(prefix='popout-review-ocr-') as tmp:
        label_paths = []
        for scene, report in zip(manifest['scenes'], reports):
            folder = source/scene['output_relative']
            assert dict(read(folder/'summary.json'),output_relative=scene['output_relative']) == report
            assert report['candidate'] == scene
            with (folder/'frames.csv').open() as stream:
                rows = list(csv.DictReader(stream))
            assert len(rows) == report['frames']
            times = [float(r['recording_relative_s']) for r in rows]
            candidate = scene['candidate_recording_s']
            onset = scene['rgb_onset_recording_s']
            cross = bisect.bisect_left(times, candidate)
            assert 0 < cross < len(rows)
            assert abs((times[cross]-candidate)*1000-report['first_display_after_candidate_ms']) < 1e-7
            for i, r in enumerate(rows):
                assert int(r['output_frame']) == i
                assert int(r['source_frame']) == int(rows[0]['source_frame'])+i
                assert not i or times[i] > times[i-1]
                assert abs(float(r['candidate_delta_ms'])-(times[i]-candidate)*1000) < 1e-7
                assert abs(float(r['reference_time_s'])-times[i]-scene['reference_origin_s']) < 1e-6
                if onset is None:
                    assert r['onset_delta_ms'] == ''
                else:
                    assert abs(float(r['onset_delta_ms'])-(times[i]-onset)*1000) < 1e-7
            snapshots = {
                'candidate_minus_100ms': max(0, bisect.bisect_right(times, candidate-.1)-1),
                'candidate_before': cross-1, 'candidate_after':cross,
                'candidate_plus_100ms': min(len(rows)-1, bisect.bisect_left(times,candidate+.1)),
            }
            cap = cv2.VideoCapture(str(folder/'candidate_review.mp4'))
            assert cap.isOpened()
            assert (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))) == (1280,608)
            maes=[]; i=0
            while True:
                ok,frame=cap.read()
                if not ok:
                    break
                for name,index in snapshots.items():
                    if index != i:
                        continue
                    still=cv2.imread(str(folder/f'{name}.png'))
                    assert still.shape == frame.shape
                    maes.append(float(np.mean(np.abs(frame.astype(float)-still))))
                    label=Path(tmp)/f'{len(label_paths):03d}.png'
                    cv2.imwrite(str(label),still[:32,:280])
                    label_paths.append(label)
                    timestamp_checks.append(dict(candidate=scene['output_relative'],snapshot=name,
                        mapped_time_s=times[i], next_mapped_time_s=times[i+1] if i+1 < len(times) else None))
                i+=1
            cap.release()
            assert i == report['frames'] and len(maes) == 4
            assert max(maes) < 10, 'PNG / decoded output MP4 mismatch'
            video_checks.append(dict(candidate=scene['output_relative'],decoded_frames=i,
                png_vs_decoded_frame_max_mae=max(maes),frame_mapping_arithmetic='passed'))
        listing=Path(tmp)/'images.txt'; listing.write_text('\n'.join(map(str,label_paths)))
        ocr=subprocess.run(['tesseract',str(listing),'stdout','--psm','7'],
                           text=True,capture_output=True,check=True)
        texts=ocr.stdout.split('\f')
        if texts and not texts[-1].strip():
            texts.pop()
        assert len(texts) == len(timestamp_checks)
        for r,text in zip(timestamp_checks,texts):
            found=re.search(r't\s*=\s*([0-9]+\.[0-9]+)\s*s',text)
            r['ocr_text']=text.strip()
            r['burned_rgb_time_s']=float(found[1]) if found else None
            r['burned_minus_mapped_ms']=None if not found else (r['burned_rgb_time_s']-r['mapped_time_s'])*1000
            r['mismatch_gt_0_75ms']=None if not found else abs(r['burned_minus_mapped_ms']) > .75
            r['matches_next_frame']=False if not found or r['next_mapped_time_s'] is None else abs(r['burned_rgb_time_s']-r['next_mapped_time_s']) <= .00075
    with (output/'timestamp_label_audit.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(timestamp_checks[0]))
        writer.writeheader(); writer.writerows(timestamp_checks)
    mismatches=[r for r in timestamp_checks if r['mismatch_gt_0_75ms']]
    # Two clips derived from the same source overlap in time. Compare only the
    # original timestamp header, unaffected by candidate-specific boxes/footer.
    reference_clip = 'test_16/evs_c001'
    suspect_clip = 'test_16/rgb_c001'
    reference_report = read(source/reference_clip/'summary.json')
    suspect_report = read(source/suspect_clip/'summary.json')
    assert reference_report['source_video_sha256'] == suspect_report['source_video_sha256']
    with (source/reference_clip/'frames.csv').open() as stream:
        reference_rows = list(csv.DictReader(stream))
    cap = cv2.VideoCapture(str(source/reference_clip/'candidate_review.mp4'))
    headers = {}
    for row in reference_rows:
        ok, frame = cap.read()
        assert ok
        headers[int(row['source_frame'])] = frame[:32,:280].astype(float)
    cap.release()
    with (source/suspect_clip/'frames.csv').open() as stream:
        suspect_rows = list(csv.DictReader(stream))
    overlap_checks = []
    for row in timestamp_checks:
        if row['candidate'] != suspect_clip:
            continue
        source_frame = next(int(r['source_frame']) for r in suspect_rows
                            if float(r['recording_relative_s']) == row['mapped_time_s'])
        still = cv2.imread(str(source/suspect_clip/f"{row['snapshot']}.png"))[:32,:280].astype(float)
        maes = {str(offset): float(np.mean(np.abs(still-headers[source_frame+offset])))
                for offset in (-1, 0, 1)}
        overlap_checks.append(dict(snapshot=row['snapshot'], source_frame=source_frame,
            header_mae_against_reference_at_frame_offset=maes, best_offset=min(maes,key=maes.get)))
    write(output/'overlap_audit.json', dict(same_source_video_sha256=reference_report['source_video_sha256'],
        reference_clip=reference_clip, suspect_clip=suspect_clip,
        method='Compare timestamp-header pixels (x=0:280, y=0:32) of saved PNGs to sequentially decoded reference clip frames; the reference clip passed its timestamp-label audit. MAE includes MP4 compression.',
        checks=overlap_checks))
    with (output/'visual_decisions.csv').open() as stream:
        decisions = list(csv.DictReader(stream))
    assert len(decisions) == 30
    assert {(r['session'],r['method'],int(r['candidate'])) for r in decisions} == {
        (r['session'],r['method'],int(r['candidate'])) for r in candidate_csv}
    visual_counts = {}
    for method in ('rgb', 'evs'):
        selected = [r for r in decisions if r['method'] == method]
        all_counts = Counter(r['classification'] for r in selected)
        near_counts = Counter(r['classification'] for r in selected if r['time_class'] == 'onset_to_250ms')
        classes = ('vehicle_supported', 'background', 'uncertain')
        visual_counts[method] = dict({key:all_counts[key] for key in classes},
            onset_to_250ms={key:near_counts[key] for key in classes},
            positive_sessions_with_background=sorted({r['session'] for r in selected
                if r['classification'] == 'background' and r['time_class'] != 'negative_recording'}))
    write(output/'visual_aggregate.json', visual_counts)
    result=dict(source_directory=str(source),audit_script_sha256=digest(Path(__file__)),
        frozen_manifest_hash_matches=True,review_code_reference_commit=REVIEW_CODE_COMMIT,
        received_review_code_hashes=run['review_code_sha256'],review_code_matches_reference_commit=True,
        current_working_review_code_hashes={name:digest(repo/'tools'/name) for name in run['review_code_sha256']},
        candidates_match_pasted_csv=True,summaries_match_pasted_csv=True,
        received_files=inventory,decoded_videos=video_checks,total_decoded_frames=sum(r['decoded_frames'] for r in video_checks),
        timestamp_labels=dict(checked=len(timestamp_checks),ocr_unreadable=sum(r['burned_rgb_time_s'] is None for r in timestamp_checks),
            mismatches=len(mismatches),candidates=sorted({r['candidate'] for r in mismatches}),
            all_mismatches_match_next_frame=all(r['matches_next_frame'] for r in mismatches)),
        limitations='Full source previews, bag/RAW and evaluation NPZ were not received. Decoding every frame is not visual inspection of every frame. OCR compares burned labels with mapping; it does not repair or establish physical synchronization.')
    write(output/'verification.json',result)
    print(json.dumps({k:v for k,v in result.items() if k not in ('received_files','decoded_videos')},ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
