import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('share_roi', ROOT / 'scripts/experiments/share_rc_popout_roi.py')
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)


class ShareRoiTests(unittest.TestCase):
    def setUp(self):
        from PIL import Image
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.names = [f'scene{i}' for i in range(6)]
        for i, name in enumerate(self.names):
            data = self.root / 'analysis/led_sync' / name / 'led_sync_data.json'
            data.parent.mkdir(parents=True)
            data.write_text('{}')
            sync = data.parent / 'time_sync_led.yaml'
            sync.write_text(f'sync{i}')
            folder = self.root / 'analysis/scenario_overlay' / name / 'common'
            folder.mkdir(parents=True)
            matrix = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
            summary = dict(bag=str(self.root/name), fps=1, view_frame='evs', output_size=[64,64],
                           projection='rotation-only', depth_m=None, camchain='chain', camchain_sha256='same',
                           rgb_to_view_homography=matrix, event_to_view_homography=matrix,
                           time_sync=str(sync), time_sync_sha256=hashlib.sha256(sync.read_bytes()).hexdigest())
            for filename in ('summary.json','summary.yaml'):
                (folder/filename).write_text(json.dumps(summary))
            (folder/'rgb_vs_overlay.mp4').touch()
            (folder/'frames.csv').write_text('frame,video_time_s,reference_time_s,relative_time_s,rgb_time_s\n0,0,100,0,100\n1,1,101,1,101\n2,2,102,2,102\n')
            Image.new('L', (64,64),255).save(folder/'common_valid_mask.png')
            rois = [dict(name='band',x=0,y=35,width=64,height=20,mask_policy='intersect_common')] if i==0 else []
            m.ann.save(data,dict(revision=None,annotation=dict(session=name,preview_id='common',
                intervals=[dict(start_s=i/10,end_s=2,label='evaluation')],onset={},rois=rois)))

    def test_share_six_preserves_timing_sync_and_backups(self):
        jobs, rois = m.prepare(self.root, self.names[0], self.names)
        self.assertEqual(len(jobs),5)
        for data,value,revision in jobs:
            path=m.ann.annotation_file(data)
            before=json.loads(path.read_text())
            m.ann.save(data,dict(annotation=value,revision=revision))
            after=json.loads(path.read_text())
            for key in ('intervals','onset','time_sync','time_sync_sha256','reference_origin_s','preview_id'):
                self.assertEqual(before[key],after[key])
            self.assertEqual(after['rois'],rois)
            self.assertEqual(len(list(path.parent.glob('sequence_annotations.*.bak.json'))),1)

    def test_mismatched_geometry_rejects_before_writes(self):
        p=self.root/'analysis/scenario_overlay/scene5/common/summary.json'
        summary=json.loads(p.read_text());summary['projection']='fixed-depth'
        p.write_text(json.dumps(summary))
        before={p:p.read_bytes() for p in self.root.rglob('sequence_annotations.json')}
        with self.assertRaises(ValueError):m.prepare(self.root,self.names[0],self.names)
        self.assertEqual(before,{p:p.read_bytes() for p in before})


if __name__ == '__main__': unittest.main()
