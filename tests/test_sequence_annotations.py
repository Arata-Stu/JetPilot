import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('sequence_annotations', ROOT / 'tools/led_sync_viewer/annotations.py')
a = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(a)


class SequenceAnnotationTests(unittest.TestCase):
    def test_preview_switch_preserves_saved_timing(self):
        import shutil
        a.save(self.dataset, dict(annotation=self.value, revision=None))
        shutil.copytree(self.folder, self.folder.parent / 'other')
        result = a.context(self.dataset, 'other')
        self.assertTrue(result['migrated'])
        self.assertEqual(result['annotation']['intervals'], json.loads(a.annotation_file(self.dataset).read_text())['intervals'])
        self.assertEqual(result['annotation']['preview_id'], 'other')
        self.assertEqual(json.loads(a.annotation_file(self.dataset).read_text())['preview_id'], 'common')

    def test_activity_candidates_use_source_time_and_group_bursts(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            counts = [1] * 30
            counts[10:13] = [100, 200, 100]
            counts[23] = 150
            (folder / 'frames.csv').write_text('event_count\n' + '\n'.join(map(str, counts)))
            frames = [dict(relative_time_s=5+i/10, video_time_s=i) for i in range(30)]
            candidates = a.motion_candidates(folder, frames)
            self.assertEqual(len(candidates), 2)
            self.assertAlmostEqual(candidates[0]['time_s'], 6.1)
            self.assertAlmostEqual(candidates[0]['start_s'], 5.5)
            self.assertAlmostEqual(candidates[0]['end_s'], 6.7)
            (folder / 'frames.csv').write_text('event_count\n' + '\n'.join(['1']*30))
            self.assertEqual(a.motion_candidates(folder, frames), [])

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.dataset = self.root / 'analysis/led_sync/scene/led_sync_data.json'
        self.dataset.parent.mkdir(parents=True)
        self.dataset.write_text('{}')
        self.folder = self.root / 'analysis/scenario_overlay/scene/common'
        self.folder.mkdir(parents=True)
        summary = dict(bag=str(self.root/'scene'), fps=60, view_frame='evs', output_size=[64, 64],
                       projection='fixed-depth', depth_m=2.2, rgb_timestamp_source='bag')
        (self.folder/'summary.yaml').write_text(json.dumps(summary))
        (self.folder/'summary.json').write_text(json.dumps(summary))
        (self.folder/'rgb_vs_overlay.mp4').touch()
        with (self.folder/'frames.csv').open('w') as stream:
            writer = csv.DictWriter(stream, fieldnames=['frame','video_time_s','reference_time_s','relative_time_s','rgb_time_s'])
            writer.writeheader()
            for i in range(4):
                writer.writerow(dict(frame=i, video_time_s=i/60, reference_time_s=100+5+i/1000,
                                     relative_time_s=5+i/1000, rgb_time_s=105 if i<3 else 105.003))
        self.value = dict(session='scene', preview_id='common', intervals=[dict(start_s=5,end_s=5.002,label='evaluation')],
                          rois=[], onset={})

    def test_save_and_reload_uses_source_time_not_video_time(self):
        result = a.save(self.dataset, dict(annotation=self.value, revision=None))
        self.assertEqual(result['annotation']['intervals'][0]['start_s'],5)
        self.assertEqual(result['annotation']['reference_origin_s'],100)
        self.assertEqual(result['annotation']['spatial']['depth_m'],2.2)
        self.assertEqual(a.context(self.dataset)['annotation'], result['annotation'])
        self.assertTrue((self.folder/'rgb_vs_overlay.mp4').is_file())

    def test_onset_repeated_held_rgb_is_not_distinct_observation(self):
        self.value['onset']={'last_hidden':0, 'first_visible':2}
        with self.assertRaises(ValueError): a.validate(self.dataset,self.value)
        self.value['onset']['first_visible']=3
        result=a.validate(self.dataset,self.value)
        self.assertEqual(result['onset']['last_hidden']['rgb_time_s'],105)

    def test_invalid_intervals_rejected(self):
        for start,end in [(5.002,5.001),(0,5.001),(5,99),(float('nan'),5.002)]:
            self.value['intervals'][0].update(start_s=start,end_s=end)
            with self.subTest(start=start,end=end), self.assertRaises(ValueError):
                a.validate(self.dataset,self.value)

    def test_conflict_and_backups(self):
        first=a.save(self.dataset,dict(annotation=self.value))
        with self.assertRaises(ValueError): a.save(self.dataset,dict(annotation=self.value))
        a.save(self.dataset,dict(annotation=self.value,revision=first['revision']))
        self.assertEqual(len(list(self.dataset.parent.glob('*.bak.json'))),1)

    def test_cannot_escape_scene_root(self):
        with self.assertRaises(ValueError): a.preview(self.dataset,'../outside')
        (self.folder.parent/'evil').symlink_to(self.root, target_is_directory=True)
        with self.assertRaises(ValueError): a.preview(self.dataset,'evil')

    def test_wrong_session_rejected(self):
        self.value['session']='other'
        with self.assertRaises(ValueError): a.validate(self.dataset,self.value)

    def test_roi_must_be_inside_common_mask(self):
        import numpy as np
        from types import SimpleNamespace
        mask=np.full((64,64),255,dtype=np.uint8);mask[:,0:3]=0
        cv=SimpleNamespace(imread=lambda *a:mask,IMREAD_GRAYSCALE=0)
        with patch.dict(sys.modules,{'cv2':cv}):
            self.value['rois']=[dict(name='left',x=4,y=35,width=10,height=10)]
            result=a.validate(self.dataset,self.value)
            self.assertEqual(result['rois'][0]['x'],4)
            self.value['rois'][0]['x']=0
            with self.assertRaises(ValueError): a.validate(self.dataset,self.value)
            self.value['rois'][0].update(x=0,width=64,mask_policy='intersect_common')
            result=a.validate(self.dataset,self.value)
            self.assertEqual(result['rois'][0]['valid_pixel_count'],61*10)
            self.assertEqual(result['rois'][0]['mask_policy'],'intersect_common')
            self.value['rois'][0].update(width=2)
            with self.assertRaises(ValueError): a.validate(self.dataset,self.value)
            self.value['rois'][0].update(x=4,y=0)
            with self.assertRaises(ValueError): a.validate(self.dataset,self.value)

if __name__=='__main__': unittest.main()
