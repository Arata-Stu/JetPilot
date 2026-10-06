import contextlib
import csv
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from zipfile import ZipFile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
import rc_popout_flow_review as m


def rows():
    result=[]
    for i, (ready,active) in enumerate([(False,False),(True,False),(True,True),(True,True),(False,False),(True,True),(True,True)]):
        result.append(dict(time_s=i*.1,from_drive_s=i*.1-.2,interval=0,
            ready=ready,active=active,alarm=i in (2,5),state_reset=i==0,
            reason='ok' if ready else 'insufficient_tracks',
            score=1.5 if ready else None,tracks=40 if ready else 2,
            inliers=30 if ready else 0,inlier_tiles=10 if ready else 0,
            inlier_ratio=.75 if ready else None,span_x=.8 if ready else None,
            span_y=.6 if ready else None,ready_observed_step_s=.1 if i in (2,3,6) else 0))
    return result


def fixture(root):
    split=root/'split.json'
    split.write_text(json.dumps(dict(groups=[dict(condition='none',development=['none'],evaluation=['held_out']),
                                           dict(condition='popout',development=['pos'],evaluation=['held_out_2'])])))
    source=root/'source';source.mkdir()
    params=dict(algorithm=m.ALGORITHM,settings=dict(min_tracks=24),split_sha256=m.digest(split))
    (source/'detector_parameters.json').write_text(json.dumps(params))
    (source/'run_config.json').write_text(json.dumps(dict(params,subset='development',sessions=['none','pos'])))
    for name in ('summary.json','summary.csv'):
        (source/name).write_text('fixture')
    (source/'held_out').mkdir();(source/'held_out/SECRET').write_text('must not be bundled')
    for session in ('none','pos'):
        folder=source/session;folder.mkdir()
        (folder/'geometry.json').write_text(json.dumps(dict(tiles=[dict(tile_id=0,x=0,y=0),dict(tile_id=1,x=32,y=0)])))
        values=rows()
        snapshot={s:dict(first_frame=dict(file=f'{s}_first_frame.png')) for s in ('rgb','evs')}
        result=dict(params,alignment=dict(session=session,drive_start_s=.2,
            rgb_first_visible_from_drive_s=.1 if session=='pos' else None,
            phases=[dict(phase='drive',intervals=[(.2,.6)])]),
            extraction=dict(intervals=[(0.,.7)]),snapshots=snapshot,
            summaries=[dict(method=s,samples=7,ready_samples=5,candidates=2) for s in ('rgb','evs')],
            status_counts={s:dict(ok=5,insufficient_tracks=2) for s in ('rgb','evs')})
        (folder/'result.json').write_text(json.dumps(result))
        for sensor in ('rgb','evs'):
            m.write_csv(folder/f'{sensor}_flow_scores.csv',values)
            (folder/f'{sensor}_candidates.json').write_text(json.dumps([dict(time_s=t,pairs=[[0,1]]) for t in (.2,.5)]))
            for name in ('flow_tiles.npz','pair_starts.json','phases.json','first_frame.png'):
                (folder/f'{sensor}_{name}').write_bytes(b'fixture')
    return source,split


class ReviewTests(unittest.TestCase):
    def test_coverage_does_not_backdate_alarm_or_bridge_unknown(self):
        r=rows()
        ready,active,longest=m.coverage(r,[(.15,.65)])
        self.assertAlmostEqual(ready,.25)
        self.assertAlmostEqual(active,.2)
        self.assertAlmostEqual(longest,.15)
        r[3]['state_reset']=True
        ready,active,longest=m.coverage(r,[(.15,.65)])
        self.assertAlmostEqual(ready,.15)
        self.assertAlmostEqual(active,.1)
        self.assertAlmostEqual(longest,.1)

    def test_scopes_exclude_waiting_and_missing_fits_from_medians(self):
        r=rows()
        summary,reasons=m.summarize(r,[(.15,.35)],'pos','evs','drive')
        self.assertEqual(summary['samples'],2)
        self.assertEqual(summary['most_common_rejection'],'')
        self.assertEqual(summary['tracks_p50'],40.)
        self.assertEqual(summary['inlier_ratio_p50'],.75)
        self.assertEqual(reasons[0]['reason'],'ok')
        empty,_=m.summarize(r,[], 'pos','rgb','empty')
        self.assertIsNone(empty['ready_fraction'])
        self.assertIsNone(empty['tracks_p50'])

    def test_csv_rejects_nonfinite_time_and_unknown_with_score(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'scores.csv'
            data=rows();data[0]['score']=0.
            m.write_csv(path,data)
            with self.assertRaisesRegex(ValueError,'quality/reason/score'):
                m.load_rows(path)
            data=rows();data[2]['time_s']=float('nan')
            m.write_csv(path,data)
            with self.assertRaisesRegex(ValueError,'nonfinite'):
                m.load_rows(path)

    def test_bundle_preserves_input_and_excludes_evaluation(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            root=Path(tmp);source,split=fixture(root)
            before={str(p):m.digest(p) for p in source.rglob('*') if p.is_file()}
            argv=['--flow-dir',str(source),'--split',str(split),'--output',str(root/'review'),'--bundle']
            self.assertEqual(m.main(argv),0)
            self.assertEqual(before,{str(p):m.digest(p) for p in source.rglob('*') if p.is_file()})
            with ZipFile(root/'review/flow_debug_bundle.zip') as archive:
                self.assertFalse(any('held_out' in n for n in archive.namelist()))
                self.assertIn('source/pos/evs_flow_scores.csv',archive.namelist())
                self.assertIn('review/rejection_reasons.csv',archive.namelist())
            summaries=list(csv.DictReader(io.StringIO((root/'review/summary.csv').read_text())))
            self.assertEqual(len(summaries),14)  # 3 scopes for none; 4 for pos, each sensor.
            pairs=list(csv.DictReader(io.StringIO((root/'review/candidate_locations.csv').read_text())))
            self.assertEqual(len(pairs),8)
            with self.assertRaises(SystemExit),contextlib.redirect_stderr(io.StringIO()):m.main(argv)

    def test_rejects_changed_split_and_truncated_csv(self):
        with tempfile.TemporaryDirectory() as tmp:
            source,split=fixture(Path(tmp))
            run=json.loads((source/'run_config.json').read_text());run['sessions'].append('held_out')
            (source/'run_config.json').write_text(json.dumps(run))
            with self.assertRaisesRegex(ValueError,'split/session'):
                m.inspect_run(source,split)
            run['sessions'].pop();(source/'run_config.json').write_text(json.dumps(run))
            path=source/'pos/rgb_flow_scores.csv'
            path.write_text('\n'.join(path.read_text().splitlines()[:-1])+'\n')
            with self.assertRaisesRegex(ValueError,'counts differ'):
                m.inspect_run(source,split)


if __name__=='__main__':unittest.main()
