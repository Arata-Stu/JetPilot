import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('roi_candidate', ROOT / 'scripts/experiments/review_rc_popout_roi_candidate.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class CandidateTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.dataset = self.root / 'analysis/led_sync/scene/led_sync_data.json'
        self.dataset.parent.mkdir(parents=True)
        self.dataset.write_text('{}')
        sync = self.dataset.parent / 'time_sync_led.yaml'
        sync.write_text('saved sync')
        digest = MODULE.hashlib.sha256(sync.read_bytes()).hexdigest()
        self.folder = self.root / 'analysis/scenario_overlay/scene/view'
        self.folder.mkdir(parents=True)
        summary = dict(bag=str(self.root/'scene'), fps=1, view_frame='evs', output_size=[640,480],
                       projection='rotation-only', depth_m=None, camchain_sha256='calibration',
                       rgb_to_view_homography=[[1,0,0],[0,1,0],[0,0,1]],
                       event_to_view_homography=[[1,0,0],[0,1,0],[0,0,1]], time_sync_sha256=digest)
        (self.folder/'summary.json').write_text(json.dumps(summary))
        (self.folder/'summary.yaml').write_text('unused')
        (self.folder/'rgb_vs_overlay.mp4').write_bytes(b'placeholder')
        (self.folder/'frames.csv').write_text(
            'frame,video_time_s,reference_time_s,relative_time_s,rgb_time_s\n'
            '0,0,100,0,100\n1,1,101,1,101\n2,2,102,2,102\n')
        mask = Image.new('L', (640,480), 255)
        mask.paste(0, (0,0,10,480))
        mask.save(self.folder/'common_valid_mask.png')
        saved = dict(session='scene', preview_id='view', reference_origin_s=100,
                     time_sync_sha256=digest, intervals=[dict(label='evaluation',start_s=0,end_s=2)],
                     onset={'first_visible':dict(rgb_time_s=101,preview_frame=1)},
                     rois=[dict(name='left',x=10,y=100,width=620,height=200)])
        self.annotation = self.dataset.parent/'sequence_annotations.json'
        self.annotation.write_text(json.dumps(saved))
        self.candidate = dict(name='band',x=0,y=70,width=640,height=272,mask_policy='intersect_common')

    def test_candidate_clips_mask_and_preserves_inputs(self):
        before = {p:p.read_bytes() for p in self.root.rglob('*') if p.is_file()}
        with patch.object(MODULE, 'extract_frame', return_value=Image.new('RGB',(1280,480))):
            result = MODULE.inspect_scene(self.dataset, self.candidate, self.root/'review.png')
        self.assertEqual(result['valid_pixel_count'], 630*272)
        self.assertTrue(result['visual_review_required'])
        with Image.open(self.root/'review.png') as sheet:
            self.assertEqual(sheet.size, (1280,1016))
        for path, content in before.items():
            self.assertEqual(path.read_bytes(), content)

    def test_rejects_old_sync(self):
        (self.dataset.parent/'time_sync_led.yaml').write_text('changed sync')
        with self.assertRaisesRegex(ValueError, 'synchronization'):
            MODULE.inspect_scene(self.dataset,self.candidate,self.root/'review.png')

    def test_rejects_no_common_pixels(self):
        Image.new('L',(640,480),0).save(self.folder/'common_valid_mask.png')
        with self.assertRaisesRegex(ValueError, 'no common-view'):
            MODULE.inspect_scene(self.dataset,self.candidate,self.root/'review.png')

    @unittest.skipUnless(shutil.which('ffmpeg'), 'ffmpeg required')
    def test_real_video_frame_extraction(self):
        video = self.root/'synthetic.mkv'
        subprocess.run(['ffmpeg','-v','error','-f','lavfi','-i',
                        'color=c=red:s=1280x480:r=1:d=3','-c:v','ffv1',str(video)],
                       check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        image = MODULE.extract_frame(video, 1)
        self.assertEqual(image.size, (1280,480))
        r, g, b = image.getpixel((100,100))
        self.assertGreater(r, 200)
        self.assertLess(max(g,b), 10)


if __name__ == '__main__':
    unittest.main()
