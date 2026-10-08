import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import rc_popout_detection_figure as figure

ROOT=Path(__file__).resolve().parents[1]
BUNDLE=ROOT/'record/09-30/analysis/development_debug_bundle01.zip'


class TimingAndSupportTests(unittest.TestCase):
    def test_latest_native_rgb_frame_never_uses_future(self):
        times=[1790000000.,1790000000.0167,1790000000.0338]
        self.assertEqual(figure.held_rgb_index(times,times[1]),1)
        self.assertEqual(figure.held_rgb_index(times,times[2]-.002),1)
        with self.assertRaisesRegex(ValueError,'past RGB'):
            figure.held_rgb_index(times,times[0]-.001)
        with self.assertRaisesRegex(ValueError,'past RGB'):
            figure.held_rgb_index(times,times[-1]+.101)
        with self.assertRaisesRegex(ValueError,'timestamp'):
            figure.held_rgb_index([1.,1.],1.)

    def test_partial_grid_tiles_intersect_roi_not_whole_cell(self):
        common=np.ones((480,640),bool)
        meta=dict(roi=dict(x=0,y=70,width=640,height=272),tiles=[
            dict(x=0,y=64,width=32,height=32,valid_pixels=832),
            dict(x=0,y=320,width=32,height=32,valid_pixels=704)])
        figure.validate_support(common,meta)
        common[70,0]=False
        with self.assertRaisesRegex(ValueError,'support differs'):
            figure.validate_support(common,meta)


@unittest.skipUnless(BUNDLE.exists(),'local received real-data archive not available')
class RealArchiveTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.evidence=figure.load_evidence(BUNDLE,figure.FROZEN,'test_05')
        cls.result,cls.index=figure.replay(cls.evidence)

    def test_replay_preserves_saved_candidate_and_values(self):
        self.assertAlmostEqual(self.result['time_s'][self.index],12.877008237838744,places=8)
        self.assertEqual(list(self.result['tile_id'][self.result['winner_pair'][self.index]]),[177,178])
        p=ROOT/'record/09-30/analysis/grid_background_dev_20261006/test_05/evs_background_maps.npz'
        if p.exists():
            with np.load(p,allow_pickle=False) as saved:
                for key in ('predicted_density','residual_z','state_s'):
                    np.testing.assert_allclose(self.result[key][self.index],saved[key][self.index],rtol=1e-6,atol=1e-7)
        bad=copy.deepcopy(self.evidence)
        bad['candidate']['candidate_recording_s']+=.001
        with self.assertRaisesRegex(ValueError,'differs from the saved'):
            figure.replay(bad)

    def test_source_sync_and_annotation_identity_are_required(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);ann_dir=root/'analysis/led_sync/test_05';ann_dir.mkdir(parents=True)
            ann=ann_dir/'sequence_annotations.json';ann.write_text('{"session":"test_05"}')
            sync=ann_dir/'time_sync_led_auto.yaml';sync.write_text('pinned sync')
            chain=root/'chain.yaml';chain.write_text('pinned camera calibration')
            bag=root/'test_05';bag.mkdir();(bag/'metadata.yaml').write_text('fixture');(bag/'data.mcap').write_bytes(b'fixture')
            e=copy.deepcopy(self.evidence)
            e['candidate']['annotation_sha256']=figure.digest(ann)
            e['candidate']['time_sync_sha256']=figure.digest(sync)
            e['parameters']['input_definition']['spatial']['camchain_sha256']=figure.digest(chain)
            # A newer manual sync cannot silently substitute for the pinned auto.
            (ann_dir/'time_sync_led.yaml').write_text('newer different sync')
            sources=figure.validate_sources(root,chain,e)
            self.assertEqual(sources['time_sync'],sync)
            sync.write_text('changed')
            with self.assertRaisesRegex(ValueError,'no time-sync YAML matches'):
                figure.validate_sources(root,chain,e)
            sync.write_text('pinned sync');ann.write_text('{}')
            with self.assertRaisesRegex(ValueError,'annotation changed'):
                figure.validate_sources(root,chain,e)

    def test_native_extraction_checks_timing_and_real_projection_support(self):
        try:
            import cv2
            sys.path.insert(0,str(figure.CALIBRATION))
            from multi_sensor_calibration import scenario_overlay as renderer
        except ImportError:
            self.skipTest('OpenCV/calibration modules unavailable')
        e=self.evidence
        origin=e['candidate']['reference_origin_s']
        times=[origin]+list(origin+e['scene']['data']['rgb']['time_s'])
        selected=figure.held_rgb_index(times,origin+self.result['time_s'][self.index])
        pixels=np.full((480,848,3),(10,20,30),np.uint8)
        sources=dict(bag=Path('/unused'),camchain=figure.CALIBRATION/'config/calibrations/rc_popout_default/kalibr-camchain.yaml')
        with mock.patch.object(renderer,'_selected_rgb_times',return_value=(origin,list(range(len(times))),times)), \
             mock.patch.object(renderer,'_rgb_frames',return_value=[(times[selected],pixels)]) as frames:
            rgb,common,timing=figure.extract_rgb(sources,e,self.result,self.index)
            self.assertEqual(rgb.shape,(480,640,3))
            self.assertGreaterEqual(timing['rgb_age_ms'],0)
            self.assertLess(timing['rgb_age_ms'],100)
            self.assertEqual(frames.call_args.args[-1],[selected])
            np.testing.assert_array_equal(rgb[common][0],[30,20,10])
        with mock.patch.object(renderer,'_selected_rgb_times',return_value=(origin+.01,[],times)):
            with self.assertRaisesRegex(ValueError,'origin differs'):
                figure.extract_rgb(sources,e,self.result,self.index)

    def test_preflight_never_decodes_or_writes(self):
        with tempfile.TemporaryDirectory() as tmp:
            output=Path(tmp)/'not-created'
            with mock.patch.object(figure,'validate_sources',return_value={}), \
                 mock.patch.object(figure,'extract_rgb',side_effect=AssertionError('decode called')), \
                 mock.patch.object(figure,'replay',side_effect=AssertionError('inference called')):
                status=figure.main(['--record-root',tmp,'--output',str(output),
                                   '--bundle',str(BUNDLE),'--preflight'])
            self.assertEqual(status,0)
            self.assertFalse(output.exists())

    def test_renderer_has_no_text_and_retains_numerical_arrays(self):
        try:
            import matplotlib.pyplot as plt
            from PIL import Image
        except ImportError:
            self.skipTest('Matplotlib/Pillow unavailable')
        evidence=self.evidence; result=self.result; index=self.index
        with tempfile.TemporaryDirectory() as tmp:
            output=Path(tmp)
            rgb=np.full((480,640,3),100,np.uint8)
            with mock.patch('matplotlib.figure.Figure.savefig',autospec=True) as save:
                figure.draw_figure(output,rgb,np.ones((480,640),bool),evidence,result,index)
                for call in save.call_args_list:
                    fig=call.args[0]
                    self.assertFalse(fig.texts)
                    self.assertTrue(all(not any(t.get_text() for t in ax.texts) for ax in fig.axes))
            values=figure.map_values(evidence,result,index)
            with np.load(output/'source_values.npz',allow_pickle=False) as data:
                for key in values:
                    np.testing.assert_array_equal(data[key],values[key])
            with Image.open(output/'rgb_roi.png') as image:
                self.assertEqual(image.size,(640,272))


if __name__=='__main__':
    unittest.main()
