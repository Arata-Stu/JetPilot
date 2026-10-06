import contextlib
import copy
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
import rc_popout_grid_background as m
import analyze_rc_popout_grid_background as runner
from rc_popout_candidate_videos import load_manifest
from test_rc_popout_local_detection import fixture


def background_fixture(sensor):
    data,meta=fixture(sensor,duration=2.)
    t=data['time_s']; pattern=np.linspace(.5,1.,12)
    amplitude=np.where((t>=.5)&(t<1.5),.15+.1*np.sin(t*8),0.)
    data['counts'][:]=np.round(1024*(amplitude[:,None]*pattern)**2).astype(int)
    phases=[dict(phase=name,start_from_drive_s=a-.5,end_from_drive_s=b-.5,
                 intervals=[[a,b]],complete=True,command_coverage=True)
            for name,a,b in [('pre_drive',0.,.5),('drive',.5,1.5)]]
    return data,meta,phases


class GridBackgroundTests(unittest.TestCase):
    def test_background_reconstruction_and_local_pair_for_both_sensors(self):
        for sensor in ('rgb','evs'):
            with self.subTest(sensor=sensor):
                data,meta,phases=background_fixture(sensor)
                model=m.fit_background(data,meta,phases,sensor,m.DEFAULTS)
                negative=m.score_maps(data,meta,sensor,m.DEFAULTS,model,32)
                threshold=m.calibrate_threshold(negative,m.DEFAULTS)['threshold_s']
                self.assertEqual(m.alarm_episodes(negative,threshold,.5)[0],[])
                # Same background plus a new adjacent local component.
                positive=copy.deepcopy(data)
                positive['counts'][positive['time_s']>=1.,:2]+=300
                result=m.score_maps(positive,meta,sensor,m.DEFAULTS,model,32)
                episodes,_,_=m.alarm_episodes(result,threshold,.5)
                self.assertTrue(episodes)
                self.assertGreaterEqual(episodes[0]['start_time_s'],1.)
                self.assertLess(episodes[0]['start_time_s'],1.15)
                self.assertEqual(episodes[0]['pair_tile_ids_at_start'],[0,1])
                # Model can be saved and loaded without pickle or refitting.
                loaded=json.loads(json.dumps(model))
                replay=m.score_maps(positive,meta,sensor,m.DEFAULTS,loaded,32)
                np.testing.assert_array_equal(result['score'],replay['score'])

    def test_fitting_ignores_waiting_outside_observed_phases(self):
        data,meta,phases=background_fixture('evs')
        model=m.fit_background(data,meta,phases,'evs',m.DEFAULTS)
        data['counts'][data['time_s']>=1.6]=999999
        self.assertEqual(model,m.fit_background(data,meta,phases,'evs',m.DEFAULTS))
        for model_phase in model['models']:
            indexes=model_phase['fit_indexes']
            self.assertLessEqual(len(indexes),51)
        phases[0]['command_coverage']=False
        with self.assertRaisesRegex(ValueError,'complete observed'):
            m.fit_background(data,meta,phases,'evs',m.DEFAULTS)

    def test_causal_prefix_and_unseen_future(self):
        data,meta,phases=background_fixture('evs')
        model=m.fit_background(data,meta,phases,'evs',m.DEFAULTS)
        data['counts'][data['time_s']>=.8,:2]+=400
        original=m.score_maps(data,meta,'evs',m.DEFAULTS,model,32)
        mask=data['time_s']<1.1
        short={k:v if k in ('tile_id','valid_pixels') else v[mask] for k,v in data.items()}
        prefix=m.score_maps(short,meta,'evs',m.DEFAULTS,model,32)
        np.testing.assert_allclose(prefix['score'],original['score'][mask],rtol=0,atol=1e-12)
        data['counts'][~mask]=99999
        future=m.score_maps(data,meta,'evs',m.DEFAULTS,model,32)
        np.testing.assert_allclose(future['score'][mask],prefix['score'],rtol=0,atol=1e-12)

    def test_gaps_intervals_and_rgb_bad_support_reset(self):
        for sensor in ('rgb','evs'):
            for kind in ('gap','interval'):
                data,meta,phases=background_fixture(sensor)
                model=m.fit_background(data,meta,phases,sensor,m.DEFAULTS)
                data['counts'][data['time_s']>=.8,:2]+=400
                cut=int(np.searchsorted(data['time_s'],.9))
                if kind=='gap':
                    data['time_s'][cut:]+=.3
                    data['support_start_s'][cut if sensor=='evs' else cut+1:]+=.3
                else: data['interval'][cut:]=1
                result=m.score_maps(data,meta,sensor,m.DEFAULTS,model,32)
                self.assertTrue(result['reset'][cut]);self.assertEqual(result['score'][cut],0.)
                if kind=='gap' and sensor=='rgb': self.assertFalse(result['ready'][cut])

    def test_isolated_tiles_are_not_adjacent_candidates(self):
        data,meta,phases=background_fixture('evs')
        data['counts'][:]=0
        model=m.fit_background(data,meta,phases,'evs',m.DEFAULTS)
        for columns in ([0],[0,5],[5,6]):
            positive=copy.deepcopy(data)
            positive['counts'][np.ix_(positive['time_s']>=.8,columns)]=400
            result=m.score_maps(positive,meta,'evs',m.DEFAULTS,model,32)
            self.assertEqual(m.alarm_episodes(result,.01,.5)[0],[])

    def test_calibration_entire_negative_not_positive_onset(self):
        negative=dict(score=np.array([0.,.1,.2]),ready=np.ones(3,bool))
        self.assertAlmostEqual(m.calibrate_threshold(negative,m.DEFAULTS)['threshold_s'],.22)
        negative['score'][-1]=.8
        self.assertAlmostEqual(m.calibrate_threshold(negative,m.DEFAULTS)['threshold_s'],.88)

    def test_invalid_settings_geometry_and_fitted_arrays(self):
        for key,value in [('rank',2.5),('residual_quantile',1.),('threshold_margin',1.),('decay_tau_s',float('nan'))]:
            with self.assertRaises(ValueError): m.validate_settings(dict(m.DEFAULTS,**{key:value}))
        data,meta,phases=background_fixture('evs')
        model=m.fit_background(data,meta,phases,'evs',m.DEFAULTS)
        for change in ('tile_id','scale'):
            broken=copy.deepcopy(model)
            if change=='tile_id': broken['tile_id'][0]=99
            else: broken['models'][0]['scale'][0]=0.
            with self.assertRaises(ValueError): m.score_maps(data,meta,'evs',m.DEFAULTS,broken,32)

    def test_cli_artifacts_holdout_guard_and_positive_independence(self):
        split=json.loads(runner.DEFAULT_SPLIT.read_text())
        scenes={}
        for name in runner.select_sessions(split,'development'):
            arrays={}
            for sensor in ('rgb','evs'):
                data,meta,phases=background_fixture(sensor)
                if name!='t_0.2-none': data['counts'][data['time_s']>=1.,:2]+=300
                arrays[sensor]=data
            scenes[name]=dict(meta=meta,data=arrays,alignment=dict(session=name,drive_start_s=.5,
                phases=phases,reference_origin_s=1000.,timestamp_source='bag',
                rgb_first_visible_from_drive_s=None if name=='t_0.2-none' else .5),
                source_hashes=dict(annotation_sha256='a'*64,time_sync_sha256='b'*64))
        definition=dict(tile_px=32,step_ms=1.,window_bins=2,
                        spatial=dict(view_frame='evs',output_size=meta['output_size']))
        with tempfile.TemporaryDirectory() as temp:
            out=Path(temp)/'run'
            with patch.object(runner,'load_input',return_value=(scenes,definition,{})),contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(runner.main(['--bundle','unused','--output',str(out),'--probe-margins','1.5','2']),0)
                with self.assertRaises(SystemExit): runner.main(['--bundle','unused','--output',str(out)])
            summary=json.loads((out/'summary.json').read_text())
            self.assertEqual(len(summary),10)
            self.assertTrue((out/'background_models.json').exists())
            self.assertTrue((out/'test_01/evs_background_maps.npz').exists())
            self.assertTrue((out/'threshold_sensitivity.csv').exists())
            manifest=load_manifest(out/'candidate_review_manifest.json')
            self.assertEqual(len(manifest['scenes']),4)
            self.assertEqual(manifest['detector_window_ms'],2.)
            self.assertFalse((out/'test_02').exists())
            params=json.loads((out/'detector_parameters.json').read_text())
            changed=copy.deepcopy(scenes)
            for name,s in changed.items():
                if name!='t_0.2-none':
                    for sensor,d in s['data'].items(): d['counts'][:]=0
                    s['alignment']['rgb_first_visible_from_drive_s']=100.
            second=Path(temp)/'changed'
            with patch.object(runner,'load_input',return_value=(changed,definition,{})),contextlib.redirect_stdout(io.StringIO()):
                runner.main(['--bundle','unused','--output',str(second)])
            self.assertEqual((out/'background_models.json').read_bytes(),(second/'background_models.json').read_bytes())
            self.assertEqual(params['calibration'],json.loads((second/'detector_parameters.json').read_text())['calibration'])
            self.assertFalse((second/'candidate_review_manifest.json').exists())
            self.assertFalse(json.loads((second/'candidate_review_status.json').read_text())['available'])
            changed['test_02']=changed['test_01']
            with self.assertRaisesRegex(ValueError,'unexpected development'):
                runner.validate_scenes(changed)


if __name__=='__main__':
    unittest.main()
