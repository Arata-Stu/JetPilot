import contextlib
import csv
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
import rc_popout_flow as flow
import analyze_rc_popout_flow as runner


class FlowTests(unittest.TestCase):
    def setUp(self):
        cv2.setNumThreads(1)
        self.p = dict(flow.DEFAULTS)
        self.mask = np.ones((256, 384), bool)
        self.geo = flow.make_geometry(self.mask, self.p)

    def correspondences(self):
        y, x = np.mgrid[8:120:8, 8:184:8]
        return np.column_stack((x.ravel(), y.ravel())).astype(np.float32)

    def texture(self):
        rng = np.random.default_rng(42)
        return cv2.GaussianBlur(rng.integers(0, 256, (128,192), dtype=np.uint8), (3,3), .7)

    def test_translation_and_expansion_explained_by_background(self):
        a = self.correspondences()
        matrix = np.array([[1.015, .008, 1.2], [-.005, 1.01, -.7]])
        b = (a @ matrix[:, :2].T+matrix[:,2]).astype(np.float32)
        result = flow.motion_residual(a, b, .03, self.geo, self.p)
        self.assertTrue(result['ready'], result)
        self.assertEqual(result['tile_outliers'].sum(), 0)
        self.assertEqual(result['pair_score'].max(), 0)
        np.testing.assert_allclose(result['matrix'], matrix, atol=1e-4)

    def test_local_independent_motion_survives_dominant_background(self):
        a = self.correspondences()
        b = a + [1., .5]
        moving = (a[:,0]>=64) & (a[:,0]<112) & (a[:,1]>=48) & (a[:,1]<80)
        b[moving] += [4., 0.]
        result = flow.motion_residual(a, b.astype(np.float32), .03, self.geo, self.p)
        self.assertTrue(result['ready'], result)
        self.assertEqual(int(result['outlier'].sum()), int(moving.sum()))
        self.assertGreaterEqual(result['pair_score'].max(), 1.)
        np.testing.assert_allclose(result['matrix'], [[1,0,1],[0,1,.5]], atol=1e-4)

    def test_sparse_or_collinear_background_is_unknown_not_zero(self):
        a = self.correspondences()
        for points in (a[:3], a[a[:,1] == 8]):
            result = flow.motion_residual(points, points+[1.,0], .03, self.geo, self.p)
            self.assertFalse(result['ready'])
            self.assertNotIn('pair_score', result)
        detector = flow.FlowDetector(self.geo, self.p)
        for t in np.arange(0, .101, .01):
            row, _ = detector.update(0, float(t), np.zeros((128,192), np.uint8), float(t)-.01)
        self.assertEqual(row['reason'], 'insufficient_tracks')
        self.assertIsNone(row['score'])
        self.assertFalse(row['active'])
        self.assertEqual(row['ready_observed_step_s'], 0)

    def test_real_lk_tracks_textured_background_and_moving_patch(self):
        image = self.texture()
        transform = np.float32([[1,0,1],[0,1,0]])
        moved = cv2.warpAffine(image, transform, (192,128))
        points = flow.feature_points(image, self.geo, self.p)
        a, b = flow.tracked_points(image, moved, points, self.geo, self.p)
        result = flow.motion_residual(a, b, .03, self.geo, self.p)
        self.assertTrue(result['ready'], result)
        self.assertLess(result['pair_score'].max(), 1.)
        # Piecewise translation: same background, patch travels faster.
        moving = moved.copy()
        moving[40:88, 64:128] = cv2.warpAffine(image, np.float32([[1,0,5],[0,1,0]]), (192,128))[40:88,64:128]
        a, b = flow.tracked_points(image, moving, points, self.geo, self.p)
        result = flow.motion_residual(a, b, .03, self.geo, self.p)
        self.assertTrue(result['ready'], result)
        self.assertGreaterEqual(result['pair_score'].max(), 1.)

    def test_persistence_is_per_pair_and_resets_on_unknown(self):
        state = flow.PairPersistence(2, self.p)
        self.assertFalse(state.update(0, np.array([2.,0]))[0].any())
        self.assertFalse(state.update(.01, np.array([0.,2]))[0].any())
        self.assertFalse(state.update(.02, np.array([2.,0]))[0].any())
        self.assertTrue(state.update(.04, np.array([2.,0]))[1][0])
        self.assertFalse(state.update(.05, None)[0].any())
        self.assertFalse(state.update(.06, np.array([2.,0]))[0].any())

    def test_reference_causal_nonoverlapping_window_and_gap_reset(self):
        detector = flow.FlowDetector(self.geo, self.p)
        image = self.texture()
        rows = []
        for t in np.arange(0, .101, .01):
            row, _ = detector.update(0, float(t), image, float(t)-.01)
            rows.append(row.copy())
        for r in rows:
            if r['reference_time_s'] is not None:
                self.assertLessEqual(r['reference_time_s'], r['support_start_s']+1e-9)
                self.assertGreaterEqual(r['reference_dt_s'], .03-1e-9)
        frozen = json.dumps(rows, sort_keys=True)
        detector.update(0, .11, 255-image, .10)
        self.assertEqual(json.dumps(rows, sort_keys=True), frozen)
        row, _ = detector.update(0, .30, image, .29)
        self.assertTrue(row['state_reset'])
        self.assertIsNone(row['reference_time_s'])
        row, _ = detector.update(1, .31, image, .30)
        self.assertTrue(row['state_reset'])
        self.assertIsNone(row['reference_time_s'])
        with self.assertRaisesRegex(ValueError, 'must increase'):
            detector.update(1, .30, image, .29)

    def test_prefix_results_unchanged_by_different_future_frames(self):
        image = self.texture()
        def run(future):
            detector = flow.FlowDetector(self.geo, self.p)
            result = []
            for i in range(12):
                current = image if i < 8 else future
                row, _ = detector.update(0, i*.01, current, i*.01-.01)
                result.append(row)
            return result
        self.assertEqual(run(image)[:8], run(255-image)[:8])

    def test_geometry_does_not_join_row_wrap_or_invalid_tiles(self):
        ids = self.geo['ids']
        for a, b in ids[self.geo['pairs']]:
            if b-a == 1:
                self.assertEqual(a//12, b//12)
        mask = self.mask.copy(); mask[0:2, 0] = False
        geo = flow.make_geometry(mask, self.p)
        self.assertFalse(geo['mask'][0,0])
        self.assertEqual(geo['lookup'][0,0], -1)
        self.assertFalse(geo['tracking_mask'][0,0])
        self.assertTrue(geo['tracking_mask'][30,30])

    def test_phase_coverage_reports_unknown_separately(self):
        rows=[dict(time_s=t,ready_observed_step_s=dt,active=active,alarm=alarm)
              for t,dt,active,alarm in [(0.,0.,False,False),(.01,.01,False,False),
                                       (.02,0.,False,False),(.03,0.,False,False),(.04,.01,True,True)]]
        phases=[dict(phase='drive',intervals=[(.005,.035)])]
        result=runner.phase_summaries(rows,phases)[0]
        self.assertAlmostEqual(result['evaluation_seconds'],.03)
        self.assertAlmostEqual(result['ready_observed_seconds'],.01)
        self.assertAlmostEqual(result['unobservable_seconds'],.02)
        self.assertEqual(result['candidates'],0)

    def test_event_histogram_preserves_late_duplicates_and_excludes_future(self):
        p = dict(self.p, event_window_ms=20.)
        with tempfile.TemporaryDirectory() as tmp:
            cache = runner.EventImages(tmp, self.mask, [(0., .06),(.1,.16)], p, 100)
            try:
                # Arbitrary order, duplicated event, and a later interval.
                cache.add(0, np.array([.035,.005,.005,.015]), np.array([80]*4), np.array([80]*4))
                cache.add(1, np.array([.135]), np.array([80]), np.array([80]))
                frames = list(cache.frames())
                early = [(t,im.copy()) for i,t,s,im in frames if i==0 and t<=.03]
                cache.add(0, np.array([.045]*20), np.array([100]*20), np.array([100]*20))
                again = [(t,im) for i,t,s,im in cache.frames() if i==0 and t<=.03]
                for (t, im), (u, other) in zip(early,again):
                    self.assertEqual(t,u); np.testing.assert_array_equal(im,other)
                self.assertEqual(int(cache.maps[0][0,40,40]),2)
                self.assertEqual(int(cache.maps[0][1,40,40]),1)
                self.assertEqual(int(cache.maps[0][3,40,40]),1)
                self.assertTrue(all(not im.any() for i,t,s,im in frames if i==1 and t<=.13))
                for i,t,s,im in frames:
                    self.assertAlmostEqual(t-s,.02)
            finally:
                cache.close()
            with self.assertRaisesRegex(ValueError, 'limit=1'):
                runner.EventImages(tmp, self.mask, [(0.,10.)], p, 1)

    def test_setting_validation(self):
        for edit in ({'downsample':3}, {'min_inlier_ratio':1.1}, {'event_window_ms':15.},
                     {'reference_lag_ms':5.}, {'min_tracks':True}, {'residual_px':float('nan')}):
            with self.subTest(edit=edit), self.assertRaises(ValueError):
                flow.validate(dict(self.p,**edit))

    def test_no_background_coverage_not_reported_as_success(self):
        with tempfile.TemporaryDirectory() as tmp:
            output=Path(tmp)
            sink=runner.FlowSink(output, tmp, self.p, 100)
            try:
                sink.initialize(self.mask,[(0.,.10)])
                for t in np.arange(0,.10,.01):
                    sink.rgb_frame(0,float(t),np.zeros_like(self.mask,np.uint8))
                result=runner.save_sensor(sink,'rgb','scene',dict(drive_start_s=0.,rgb_first_visible_from_drive_s=.08))
                self.assertEqual(result['status'],'unobservable')
                self.assertEqual(result['ready_fraction'],0)
                self.assertEqual(result['unobservable_seconds'],.1)
                scores=list(csv.DictReader(io.StringIO((output/'rgb_flow_scores.csv').read_text())))
                self.assertTrue(all(r['score']=='' for r in scores))
            finally:
                sink.close()


class CliTests(unittest.TestCase):
    def test_development_only_pipeline_and_existing_output_protection(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); chain=root/'chain.yaml';chain.write_text('calibration')
            sessions=['none','pos']
            entries=[]
            for s in sessions:
                folder=root/'analysis/led_sync'/s; folder.mkdir(parents=True)
                ann=folder/'sequence_annotations.json'
                ann.write_text(json.dumps(dict(session=s,intervals=[dict(label='evaluation',start_s=0.,end_s=.2)])))
                (root/s).mkdir(); (root/s/'events.raw').touch();(root/s/'events.raw.metadata.yaml').write_text('anchor')
                entries.append(dict(session=s,annotation=str(ann)))
            config=dict(common=dict(spatial=dict(camchain=str(chain),camchain_sha256=runner.digest(chain),output_size=[384,256]),
                                    sessions=entries+[dict(session='held_out',annotation='/MUST_NOT_READ')]),parameters={})
            source=root/'source.json';source.write_text(json.dumps(config))
            split=root/'split.json';split.write_text(json.dumps(dict(groups=[
                dict(condition='none',development=['none'],evaluation=['held_out']),
                dict(condition='popout',development=['pos'],evaluation=['held_out_2'])])))
            def check(folder, source_dir):
                self.assertIn(folder.name,sessions)
                entry=next(e for e in entries if e['session']==folder.name)
                return dict(drive_start_s=0.,rgb_first_visible_from_drive_s=None),dict(annotation_sha256=runner.digest(entry['annotation']),time_sync_sha256='sync')
            def extract(config,entry,args,*,frame_sink):
                self.assertIn(entry['session'],sessions)
                frame_sink.initialize(np.ones((256,384),bool),[(0.,.2)])
                rng=np.random.default_rng(7)
                image=rng.integers(0,256,(256,384),dtype=np.uint8)
                y,x=np.mgrid[20:230:6,20:360:6]
                for i in range(12):
                    frame_sink.rgb_frame(0,i/60,image)
                for k in range(20):
                    frame_sink.events(0,np.full(x.size,k*.01+.005),x.ravel(),y.ravel())
                return dict(annotation_sha256=runner.digest(entry['annotation']),time_sync_sha256='sync',rgb_first_visible_s=None),[],[]
            argv=['--motion-dir',str(root),'--score-config',str(source),'--split',str(split),'--output',str(root/'out')]
            with patch.object(runner,'check_scene',side_effect=check),patch.object(runner,'analyze',side_effect=extract),contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(runner.main(argv+['--dry-run']),0)
                self.assertFalse((root/'out').exists())
                self.assertEqual(runner.main(argv),0)
                run=json.loads((root/'out/run_config.json').read_text())
                self.assertEqual(run['sessions'],sessions)
                self.assertFalse((root/'out/held_out').exists())
                self.assertFalse(list((root/'out').glob('*/.event_frames_*')))
                results=list(csv.DictReader(io.StringIO((root/'out/summary.csv').read_text())))
                self.assertEqual(len(results),4)
                self.assertTrue(all(r['status']=='complete' for r in results),results)
                with self.assertRaises(SystemExit),contextlib.redirect_stderr(io.StringIO()):runner.main(argv)
                with self.assertRaises(SystemExit),contextlib.redirect_stderr(io.StringIO()):runner.main(argv+['--subset','evaluation'])


if __name__=='__main__':
    unittest.main()
