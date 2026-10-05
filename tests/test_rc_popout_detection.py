import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from contextlib import ExitStack
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
sys.path.insert(0,str(ROOT/'ros2_ws/src/tool/multi_sensor_calibration'))
spec=importlib.util.spec_from_file_location('detection',ROOT/'tools/rc_popout_detection.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

class DetectionTests(unittest.TestCase):
    def test_common_mask_policies_preserve_saved_semantics(self):
        common = np.array([[True, True], [False, True]])
        inside = np.array([[True, True], [False, False]])
        np.testing.assert_array_equal(m.apply_common_mask(inside, common, 'inside_common'), inside)
        full = np.ones((2, 2), dtype=bool)
        np.testing.assert_array_equal(m.apply_common_mask(full, common, 'intersect_common'), common)
        with self.assertRaisesRegex(ValueError, 'outside common view'):
            m.apply_common_mask(full, common, 'inside_common')
        with self.assertRaisesRegex(ValueError, 'empty'):
            m.apply_common_mask(~common, common, 'intersect_common')
        with self.assertRaisesRegex(ValueError, 'unsupported'):
            m.apply_common_mask(full, common, 'unknown')

    def test_spatial_rejects_single_pixel_and_diffuse_activity(self):
        hot=np.array([[0,0]]*22)
        score,peak,pixels=m.spatial_scores([hot],3,2,64,64,32,3,np)
        self.assertEqual(score,[0,0]);self.assertEqual(peak[0],22);self.assertEqual(pixels[0],1)
        # Enough total events, but fewer than 20 in each separate tile.
        diffuse=np.array([[0,p] for p in [0,1,2,32,33,34,2048,2049,2050,2080,2081,2082]]*2)
        score,_,_=m.spatial_scores([diffuse],3,2,64,64,32,3,np)
        self.assertEqual(score[0],6)
        cluster=np.array([[1,p] for p in [0,1,2]]*8)
        score,_,_=m.spatial_scores([cluster],4,2,64,64,32,3,np)
        self.assertEqual(score,[24,24,0])
        reverse,_,_=m.spatial_scores([cluster[::-1][:7],cluster[::-1][7:]],4,2,64,64,32,3,np)
        self.assertEqual(score,reverse)

    def test_spatial_counts_distinct_pixels_across_window_once(self):
        parts=[np.array([[0,0],[0,1],[1,0],[1,1]])]
        score,peak,pixels=m.spatial_scores(parts,2,2,64,64,32,3,np)
        self.assertEqual(score,[0]);self.assertEqual(peak,[4]);self.assertEqual(pixels,[2])

    def test_intervals_exclude_and_reject_overlap(self):
        ann={'intervals':[dict(label='evaluation',start_s=1,end_s=5),dict(label='exclude',start_s=2,end_s=3)]}
        self.assertEqual(m.evaluation_intervals(ann),[(1,2),(3,5)])
        ann['intervals'].append(dict(label='evaluation',start_s=4,end_s=6))
        with self.assertRaises(ValueError):m.evaluation_intervals(ann)

    def test_causal_windows_and_episode_reset(self):
        np.testing.assert_array_equal(m.trailing_counts([0,2,3,0,0],2,np),[2,5,3,0])
        self.assertEqual(m.crossings([(0,1,5),(0,2,6),(0,3,0),(0,4,5),(1,5,5)],5),[1,4,5])

    def test_source_analysis_preserves_event_multiplicity_and_rejects_changed_sync(self):
        from multi_sensor_calibration.models import ClockEstimate
        with tempfile.TemporaryDirectory() as temp, ExitStack() as stack:
            root=Path(temp); folder=root/'analysis/led_sync/scene';folder.mkdir(parents=True)
            session=root/'scene';session.mkdir();(session/'camera.raw').touch()
            sync=folder/'sync.yaml';sync.write_text('sync')
            chain=root/'chain.yaml';chain.write_text('calibration')
            ann=dict(session='scene',intervals=[dict(label='evaluation',start_s=.1,end_s=.9)],
                     time_sync=str(sync),time_sync_sha256=m.digest(sync),reference_origin_s=100,
                     onset={'first_visible':{'rgb_time_s':100.4}})
            ann_path=folder/'sequence_annotations.json';ann_path.write_text(json.dumps(ann))
            config=dict(roi=dict(x=0,y=0,width=4,height=4,mask_policy='intersect_common'),spatial=dict(
                view_frame='evs',output_size=[4,4],camchain=str(chain),camchain_sha256=m.digest(chain),
                projection='fixed-depth',depth_m=2.2,rgb_to_view_homography=np.eye(3).tolist()))
            camera=dict(matrix=np.eye(3),distortion=np.zeros(5),size=(4,4))
            cv=SimpleNamespace(CV_32FC1=0,INTER_LINEAR=0,BORDER_CONSTANT=0,COLOR_BGR2GRAY=0,
                initUndistortRectifyMap=lambda *a:(None,None),remap=lambda im,*a,**k:im.copy(),
                warpPerspective=lambda im,*a,**k:im.copy(),cvtColor=lambda im,*a:im[:,:,0],
                undistortPoints=lambda pts,*a,**k:pts)
            stack.enter_context(patch.dict(sys.modules,{'cv2':cv}))
            stack.enter_context(patch('multi_sensor_calibration.calibration_overlay._camera',return_value=camera))
            stack.enter_context(patch('multi_sensor_calibration.io.load_yaml',side_effect=lambda p:
                {'models':{'evs':ClockEstimate.identity(100,.001).to_dict()}} if Path(p)==sync else {'cam1':{'T_cn_cnm1':np.eye(4).tolist()}}))
            times=[100+i/10 for i in range(11)]
            stack.enter_context(patch('multi_sensor_calibration.scenario_overlay._selected_rgb_times',return_value=(100,list(range(11)),times)))
            def frames(*a):
                return iter((t,np.full((4,4,3),100 if i>=4 else 0,np.uint8)) for i,t in enumerate(times))
            stack.enter_context(patch('multi_sensor_calibration.scenario_overlay._rgb_frames',side_effect=frames))
            events=np.zeros(5,dtype=[('x','u2'),('y','u2'),('t','i8')]);events['t']=[0,350100,350100,350100,1000000]
            source=SimpleNamespace(width=4,height=4,anchor=SimpleNamespace(reference_time_s=100.,scale=1.,source_time_us=0.),batches=lambda:iter([SimpleNamespace(events=events)]))
            stack.enter_context(patch('multi_sensor_calibration.evs_sources.MetavisionFileSource',return_value=source))
            args=SimpleNamespace(rgb_topic='rgb',rgb_pixel_delta=15,rgb_threshold=.1,evs_threshold=3,step_ms=1,window_bins=2,spatial=True,tile_px=32,min_active_pixels=3,spatial_threshold=3)
            entry=dict(session='scene',annotation=str(ann_path))
            result,rgb,evs=m.analyze(config,entry,args)
            self.assertEqual(max(x[2] for x in evs),3)
            self.assertAlmostEqual(result['rgb']['first_trigger_s'],.4)
            self.assertGreater(result['evs']['first_trigger_s'],.3501)
            self.assertLess(result['evs']['first_trigger_s'],.354)
            self.assertEqual(result['valid_pixels'],16)
            self.assertEqual(result['evs_spatial']['episodes'],0)
            self.assertIn('evs_spatial',result['score_distribution'])
            m.write_plot(root/'scores.svg',result,rgb,evs)
            m.write_report(root,[result])
            self.assertTrue((root/'index.html').is_file())
            # Same event multiset: scrambled in-batch, then late after an EOF-like time.
            for partitions in ([[4,2,0,1,3]], [[0,4],[2],[1,3]], [[4],[3,2],[1,0]]):
                with self.subTest(partitions=partitions):
                    source.batches=lambda:iter(SimpleNamespace(events=events[index]) for index in partitions)
                    from rc_popout_tile_activity import TileSink
                    sink=TileSink(2)
                    reordered,rr,ee=m.analyze(config,entry,args,tile_sink=sink)
                    arrays=sink.finish()
                    self.assertEqual(ee,evs)
                    self.assertEqual(rr,rgb)
                    np.testing.assert_array_equal(arrays['rgb']['counts'].sum(axis=1)/16, [r[2] for r in rgb])
                    np.testing.assert_array_equal(arrays['evs']['counts'].sum(axis=1), [r[2] for r in evs])
                    np.testing.assert_array_equal(arrays['evs']['time_s'], [r[1] for r in evs])
                    self.assertEqual(arrays['rgb']['counts'].max(),4)
                    self.assertEqual(arrays['evs']['counts'][:,0].max(),3)
                    self.assertEqual(arrays['evs']['counts'][:,1:].max(),0)
                    self.assertEqual(reordered['evs'],result['evs'])
                    self.assertGreater(reordered['event_ordering']['backward_steps'],0)
                    self.assertGreater(reordered['event_ordering']['max_backward_step_us'],0)
            # Reordering does not disable the recording-extent check.
            source.batches=lambda:iter([SimpleNamespace(events=events[1:4])])
            with self.assertRaisesRegex(ValueError,'does not cover'):
                m.analyze(config,entry,args)
            sync.write_text('changed')
            with self.assertRaisesRegex(ValueError,'sync changed'):m.analyze(config,entry,args)

if __name__=='__main__':unittest.main()
