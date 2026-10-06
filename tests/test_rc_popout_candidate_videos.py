import copy
import csv
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
import rc_popout_candidate_videos as video


def fixture(root, make_video=False):
    manifest = json.loads(video.DEFAULT_MANIFEST.read_text())
    chain = root/'chain.yaml'; chain.write_text('synthetic calibration')
    spatial = manifest['spatial']
    spatial.update(camchain=str(chain), camchain_sha256=video.digest(chain))
    reference = root/'source.mp4'
    if make_video:
        import cv2
        import numpy as np
        writer = cv2.VideoWriter(str(reference), cv2.VideoWriter_fourcc(*'mp4v'), 20., (1280,480))
        if not writer.isOpened():
            raise RuntimeError('fixture encoder unavailable')
        for i in range(50):
            image = np.full((480,1280,3), 65, np.uint8)
            for shift in (0,640):
                for x in range(0,640,32):
                    cv2.line(image,(shift+x,0),(shift+x,479),(95,95,95),1)
                for y in range(0,480,32):
                    cv2.line(image,(shift,y),(shift+639,y),(95,95,95),1)
                cv2.rectangle(image,(shift+10+i*4,270),(shift+45+i*4,310),(50,70,180),-1)
                cv2.putText(image,f'SYNTHETIC / frame {i}',(shift+50,60),cv2.FONT_HERSHEY_SIMPLEX,.65,(250,250,250),1)
            writer.write(image)
        writer.release()
    for scene in manifest['scenes']:
        session=scene['session']
        ann_dir=root/'analysis/led_sync'/session; ann_dir.mkdir(parents=True)
        preview=root/'analysis/scenario_overlay'/session/'matching'; preview.mkdir(parents=True)
        annotation=ann_dir/'sequence_annotations.json'
        video.write_json(annotation, dict(session=session, preview_id='matching'))
        sync=ann_dir/'time_sync_led.yaml'; sync.write_text('synthetic sync')
        scene.update(candidate_recording_s=10.237, candidate_from_drive_s=.237, drive_start_s=10.,
            rgb_onset_recording_s=10.1, reference_origin_s=100., rgb_timestamp_source='bag',
            annotation_sha256=video.digest(annotation), time_sync_sha256=video.digest(sync))
        summary=dict(spatial, bag=str(root/session),time_sync_sha256=scene['time_sync_sha256'],
            reference_origin_s=100.,rgb_timestamp_source='bag',fps=20.,rendered_frames=50,
            timeline='rgb',truncated=False,event_window_ms=10.,event_window_position='center')
        video.write_json(preview/'summary.json',summary)
        with (preview/'frames.csv').open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=['frame','video_time_s','reference_time_s','relative_time_s','rgb_time_s'])
            writer.writeheader()
            for i in range(50):
                relative=9.5+i/20
                writer.writerow(dict(frame=i,video_time_s=i/20,reference_time_s=100+relative,
                                     relative_time_s=relative,rgb_time_s=100+relative))
        if make_video:
            shutil.copyfile(reference,preview/'rgb_vs_overlay.mp4')
        else:
            (preview/'rgb_vs_overlay.mp4').write_bytes(b'not decoded in metadata tests')
    path=root/'manifest.json'; video.write_json(path,manifest)
    return path,manifest


class CandidateVideoTests(unittest.TestCase):
    def test_review_manifest_matches_measured_candidate_table(self):
        manifest=video.load_manifest(video.DEFAULT_MANIFEST)
        table=video.DEFAULT_MANIFEST.parent/'threshold_probe.csv'
        self.assertEqual(video.digest(table),manifest['candidate_table_sha256'])
        with table.open() as stream:
            rows={r['session']:r for r in csv.DictReader(stream) if r['sensor']=='evs' and r['candidates']=='1'}
        for scene in manifest['scenes']:
            r=rows[scene['session']]
            self.assertEqual(scene['candidate_recording_s'],float(r['first_recording_relative_s']))
            self.assertEqual([t['tile_id'] for t in scene['tiles']],[int(r['tile_a']),int(r['tile_b'])])

    def test_cropped_video_uses_frame_mapping_and_ceil_at_candidate(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);path,manifest=fixture(root)
            video.load_manifest(path)
            plan=video.find_preview(root,manifest['scenes'][0],manifest['spatial'],.3,.3)
            lo,hi,crossing=plan['span']
            self.assertEqual((lo,hi,crossing),(9,21,15))
            self.assertAlmostEqual(plan['frames'][crossing]['video_time_s'],.75)
            self.assertAlmostEqual(plan['frames'][crossing]['relative_time_s'],10.25)
            self.assertLess(plan['frames'][crossing-1]['relative_time_s'],10.237)
            self.assertEqual(video.main(['--record-root',str(root),'--output',str(root/'out'),
                                         '--manifest',str(path),'--dry-run']),0)
            self.assertFalse((root/'out').exists())

    def test_stale_sync_and_changed_annotations_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);_,manifest=fixture(root)
            scene=manifest['scenes'][0]; ann=root/'analysis/led_sync'/scene['session']
            sync=ann/'time_sync_led.yaml';sync.write_text('updated sync')
            with self.assertRaisesRegex(ValueError,'同期が候補解析時'):
                video.find_preview(root,scene,manifest['spatial'],.3,.3)
            sync.write_text('synthetic sync')
            (ann/'sequence_annotations.json').write_text('{}')
            with self.assertRaisesRegex(ValueError,'注釈が候補解析時'):
                video.find_preview(root,scene,manifest['spatial'],.3,.3)

    def test_mismatched_coordinates_and_broken_mapping_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);_,manifest=fixture(root);scene=manifest['scenes'][0]
            folder=root/'analysis/scenario_overlay'/scene['session']/'matching'
            summary=json.loads((folder/'summary.json').read_text())
            broken=copy.deepcopy(summary);broken['rgb_to_view_homography'][0][2]+=10
            video.write_json(folder/'summary.json',broken)
            with self.assertRaisesRegex(ValueError,'coordinate/calibration mismatch'):
                video.find_preview(root,scene,manifest['spatial'],.3,.3)
            video.write_json(folder/'summary.json',summary)
            data=(folder/'frames.csv').read_text().replace('0,0.0,','1,0.0,',1)
            (folder/'frames.csv').write_text(data)
            with self.assertRaisesRegex(ValueError,'frame/time mapping mismatch'):
                video.find_preview(root,scene,manifest['spatial'],.3,.3)

    def test_fallback_to_matching_preview_not_newest_bad_sync(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);_,manifest=fixture(root);scene=manifest['scenes'][0]
            folder=root/'analysis/scenario_overlay'/scene['session']/'matching'
            wrong=folder.parent/'newer_wrong';wrong.mkdir()
            summary=json.loads((folder/'summary.json').read_text());summary['time_sync_sha256']='0'*64
            video.write_json(wrong/'summary.json',summary)
            self.assertEqual(video.find_preview(root,scene,manifest['spatial'],.3,.3)['folder'],folder)

    def test_missing_recording_tail_allowed_when_clip_is_covered(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);_,manifest=fixture(root);scene=manifest['scenes'][0]
            folder=root/'analysis/scenario_overlay'/scene['session']/'matching'
            summary=json.loads((folder/'summary.json').read_text())
            summary.update(truncated=True,requested_frames=80)
            video.write_json(folder/'summary.json',summary)
            plan=video.find_preview(root,scene,manifest['spatial'],.3,.3)
            self.assertEqual(plan['span'],(9,21,15))
            self.assertTrue(plan['summary']['truncated'])

    def test_missing_tail_inside_required_clip_still_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);_,manifest=fixture(root);scene=manifest['scenes'][0]
            folder=root/'analysis/scenario_overlay'/scene['session']/'matching'
            summary=json.loads((folder/'summary.json').read_text())
            summary.update(truncated=True,requested_frames=80,rendered_frames=20)
            video.write_json(folder/'summary.json',summary)
            lines=(folder/'frames.csv').read_text().splitlines(keepends=True)
            (folder/'frames.csv').write_text(''.join(lines[:21]))
            with self.assertRaisesRegex(ValueError,'does not cover the full requested clip'):
                video.find_preview(root,scene,manifest['spatial'],.3,.3)

    def test_event_timeline_error_distinguished_from_truncation(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);_,manifest=fixture(root);scene=manifest['scenes'][0]
            folder=root/'analysis/scenario_overlay'/scene['session']/'matching'
            summary=json.loads((folder/'summary.json').read_text())
            summary.update(timeline='event',truncated=False)
            video.write_json(folder/'summary.json',summary)
            with self.assertRaisesRegex(ValueError,"timeline='event', truncated=False"):
                video.find_preview(root,scene,manifest['spatial'],.3,.3)

    def test_four_synthetic_videos_encode_and_keep_frame_count(self):
        try:
            import cv2
        except ImportError:
            self.skipTest('OpenCV not installed')
        if not shutil.which('ffmpeg'):
            self.skipTest('ffmpeg not installed')
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);path,manifest=fixture(root,True)
            # Exercise a real, decodable prefix of a longer requested recording.
            partial=root/'analysis/scenario_overlay'/manifest['scenes'][0]['session']/'matching/summary.json'
            summary=json.loads(partial.read_text());summary.update(truncated=True,requested_frames=80)
            video.write_json(partial,summary)
            before=video.digest(root/'source.mp4')
            out=root/'out'
            self.assertEqual(video.main(['--record-root',str(root),'--output',str(out),'--manifest',str(path)]),0)
            results=json.loads((out/'summary.json').read_text())
            self.assertEqual(len(results),4)
            self.assertTrue(results[0]['source_truncated'])
            self.assertFalse(results[1]['source_truncated'])
            for result in results:
                self.assertEqual(result['status'],'complete')
                self.assertEqual(result['frames'],12)
                self.assertAlmostEqual(result['fps'],5.)
                self.assertAlmostEqual(result['first_display_after_candidate_ms'],13.)
                folder=out/result['session']
                self.assertTrue((folder/'contact_sheet.png').is_file())
                with (folder/'frames.csv').open() as stream:
                    rows=list(csv.DictReader(stream))
                self.assertEqual(len(rows),12)
                self.assertEqual(rows[0]['source_frame'],'9')
                # Verify both source panels have the same reference-box pixels.
                marked=cv2.imread(str(folder/'candidate_after.png'))
                tile=next(s for s in manifest['scenes'] if s['session']==result['session'])['tiles'][0]
                x,y=tile['x'],tile['y']
                self.assertEqual(marked[y,x].tolist(),[0,150,255])
                self.assertEqual(marked[y,x+640].tolist(),[0,150,255])
            self.assertEqual(before,video.digest(root/'source.mp4'))
            with self.assertRaises(SystemExit):
                video.main(['--record-root',str(root),'--output',str(out),'--manifest',str(path)])


if __name__ == '__main__':
    unittest.main()
