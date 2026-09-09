"""Standard-library tests: Optuna is intentionally replaced, not imported."""
import unittest
from types import SimpleNamespace as NS
from jetpilot_console.simulation_optuna import SimulationOptimizer, metric_score, SPACES


class Trial:
    def suggest_float(self, name, low, high): return (low+high)/2
    def suggest_int(self, name, low, high): return (low+high)//2


class Study:
    def __init__(self): self.told=[]
    def ask(self): return Trial()
    def tell(self, trial, score): self.told.append(score)


def metrics(rms=.2, **kw):
    return dict(rmsError=rms,maxError=.3,steeringRate=.1,distance=10.,time=20.,status='時間終了',**kw)


class OptimizerTest(unittest.TestCase):
    def setUp(self):
        self.studies=[]
        def study(**kw):
            result=Study();self.studies.append(result);return result
        self.now=0.
        self.optimizer=SimulationOptimizer(loader=lambda:NS(create_study=study,samplers=NS(TPESampler=lambda **kw:kw)),clock=lambda:self.now)
        self.body=dict(controller='pure_pursuit',settings={**{k:.5 for k in SPACES['pure_pursuit']},'targetSpeedMps':2.,'wheelbaseM':.26},trials=5,duration=20.)

    def test_baseline_then_bounded_candidates_keep_best_and_fix_vehicle_settings(self):
        r=self.optimizer.start(self.body)
        self.assertTrue(r['is_baseline']);self.assertEqual(r['settings'],self.body['settings'])
        r=self.optimizer.tell(dict(session_id=r['session_id'],sequence=0,metrics=metrics()))
        self.assertEqual(r['baseline']['score'],.2)
        for i,score in enumerate([.15,.3,.1,.12,.14],1):
            self.assertEqual(r['settings']['targetSpeedMps'],2.)
            self.assertEqual(r['settings']['wheelbaseM'],.26)
            r=self.optimizer.tell(dict(session_id=r['session_id'],sequence=i,metrics=metrics(score)))
        self.assertTrue(r['done']);self.assertEqual(r['best']['score'],.1);self.assertEqual(r['completed'],5)
        self.assertEqual(len(self.studies[0].told),5)

    def test_invalid_candidate_cannot_win_and_duplicate_tell_is_rejected(self):
        r=self.optimizer.start(self.body);key=r['session_id']
        self.optimizer.tell(dict(session_id=key,sequence=0,metrics=metrics()))
        bad=metrics(.0);bad['status']='追従不能'
        r=self.optimizer.tell(dict(session_id=key,sequence=1,metrics=bad))
        self.assertEqual(r['best']['score'],.2);self.assertEqual(self.studies[0].told,[1e12])
        with self.assertRaises(ValueError):self.optimizer.tell(dict(session_id=key,sequence=1,metrics=metrics()))

    def test_failed_baseline_allows_search_but_missing_samples_never_score_zero(self):
        r=self.optimizer.start(self.body);bad=metrics();bad['rmsError']=None
        r=self.optimizer.tell(dict(session_id=r['session_id'],sequence=0,metrics=bad))
        self.assertIsNone(r['baseline']);self.assertIsNone(r['best'])
        r=self.optimizer.tell(dict(session_id=r['session_id'],sequence=1,metrics=metrics()))
        self.assertEqual(r['best']['score'],.2)

    def test_bad_request_expiry_and_cancel(self):
        for changes in [dict(trials=0),dict(trials=True),dict(duration=float('nan')),dict(controller='all'),dict(settings={})]:
            with self.assertRaises(ValueError):self.optimizer.start({**self.body,**changes})
        r=self.optimizer.start(self.body);self.optimizer.stop(dict(session_id=r['session_id']))
        with self.assertRaises(ValueError):self.optimizer.tell(dict(session_id=r['session_id'],sequence=0,metrics=metrics()))
        r=self.optimizer.start(self.body);self.now=1900
        with self.assertRaises(ValueError):self.optimizer.tell(dict(session_id=r['session_id'],sequence=0,metrics=metrics()))

    def test_missing_optional_dependency_is_actionable(self):
        def missing():raise ImportError('absent')
        service=SimulationOptimizer(loader=missing)
        with self.assertRaisesRegex(RuntimeError,'requirements-simulation-optuna'):
            service.start(self.body)

    def test_scoring_rejects_nonfinite_shortened_and_failed_runs(self):
        for value in [float('nan'),float('inf'),-1,True]:
            with self.assertRaises(ValueError):metric_score(metrics(value),20)
        for change in [dict(time=1.),dict(distance=0.),dict(status='実行中')]:
            m=metrics();m.update(change);self.assertIsNone(metric_score(m,20))
        m=metrics();m['distance']=1
        self.assertIsNone(metric_score(m,20,metrics()))

if __name__=='__main__':unittest.main()
