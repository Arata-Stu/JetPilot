import importlib.util
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
import rc_popout_motion_analysis as m


def commands(shift=0):
    return [(round(i*.02+shift,6), .1 if 100<=i<200 else 0,
             .2 if 200<=i<225 else 0, 0) for i in range(301)]


class MotionTests(unittest.TestCase):
    def test_command_phases_ignore_led_wait_offset(self):
        zero,p=m.find_phases(commands(),[(0,6)],1,.1)
        later,q=m.find_phases(commands(10),[(10,16)],1,.1)
        self.assertEqual(zero,2);self.assertEqual(later,12)
        for a,b in zip(p,q):
            for key in ('phase','start_from_drive_s','end_from_drive_s','evaluation_seconds','complete'):
                self.assertEqual(a[key],b[key])
        self.assertEqual([v['evaluation_seconds'] for v in p],[1,2,.5])

    def test_incomplete_and_excluded_preperiod_is_marked(self):
        _,p=m.find_phases(commands(),[(1.5,2.5),(3,6)],1,.1)
        self.assertFalse(p[0]['complete'])
        self.assertFalse(p[1]['complete'])
        self.assertEqual(p[1]['evaluation_seconds'],1.5)

    def test_missing_gaps_multiple_and_unobserved_end_rejected(self):
        bad_cases=[[],commands()[110:],commands()[:150],
                   [row for row in commands() if not 3<row[0]<3.5],
                   [(t,.1 if 1<=t<1.5 else thr,brk,rev) for t,thr,brk,rev in commands()]]
        for rows in bad_cases:
            with self.subTest(length=len(rows)),self.assertRaises(ValueError):
                m.find_phases(rows,[(0,6)],1,.1)

    def test_neutral_end_has_no_invented_brake_phase(self):
        rows=[(t,thr,0,rev) for t,thr,brk,rev in commands()]
        _,p=m.find_phases(rows,[(0,6)],1,.1)
        self.assertEqual([v['phase'] for v in p],['pre_drive','drive'])

    def test_phase_boundary_windows_and_time_weighted_occupancy(self):
        phase=dict(intervals=[(1,2)])
        rows=[(0,.99,0),(0,1.01,50),(0,1.05,50),(0,1.15,0),(0,1.25,0)]
        selected=m.phase_scores(rows,phase,'rgb',.002,.11)
        self.assertNotIn(1.01,[r[1] for r in selected])
        stats=m.statistics(selected,20,1,np)
        self.assertAlmostEqual(stats['above_threshold_seconds'],.1)
        self.assertAlmostEqual(stats['observed_score_seconds'],.2)
        self.assertAlmostEqual(stats['above_threshold_fraction'],.5)
        self.assertEqual(stats['episodes'],1)
        evs=m.phase_scores([(0,1.001,3),(0,1.003,4)],phase,'evs',.002,.0015)
        self.assertEqual([r[1] for r in evs],[1.003])

    def test_existing_scores_and_recorded_messages_end_to_end(self):
        from multi_sensor_calibration.rosbag import BagMessage
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);folder=root/'scores/scene';folder.mkdir(parents=True)
            ann_path=root/'record/analysis/led_sync/scene/sequence_annotations.json'
            ann_path.parent.mkdir(parents=True)
            sync=root/'sync.yaml';sync.write_text('sync')
            ann=dict(session='scene',reference_origin_s=100,rgb_timestamp_source='bag',
                     time_sync=str(sync),intervals=[dict(label='evaluation',start_s=0,end_s=6)])
            ann_path.write_text(json.dumps(ann))
            result=dict(annotation=str(ann_path),annotation_sha256=m.digest(ann_path),time_sync_sha256=m.digest(sync),
                        intervals=[[0,6]],rgb_first_visible_s=3,rgb=dict(threshold=.1),evs=dict(threshold=20))
            (folder/'result.json').write_text(json.dumps(result))
            (folder.parent/'run_config.json').write_text(json.dumps(dict(parameters=dict(step_ms=1,window_bins=2))))
            for sensor in ('rgb','evs'):
                (folder/f'{sensor}_scores.csv').write_text('interval,relative_time_s,score\n'+''.join(f'0,{i*.02},30\n' for i in range(301)))
            messages=[BagMessage('/vehicle/control_cmd','ControlCommand',round((100+t)*1e9),
                                 SimpleNamespace(throttle=thr,brake=brk,reverse=rev)) for t,thr,brk,rev in commands()]
            with patch('multi_sensor_calibration.rosbag.iter_messages',return_value=iter(messages)):
                _,alignment,stats,_,_=m.analyze_scene(folder,1,.1,np)
            self.assertAlmostEqual(alignment['drive_start_s'],2)
            self.assertEqual(alignment['rgb_first_visible_from_drive_s'],1)
            self.assertEqual(len(stats),6)
            self.assertEqual(json.loads(ann_path.read_text()),ann)
            out=root/'motion_output'
            with patch('multi_sensor_calibration.rosbag.iter_messages',return_value=iter(messages)), \
                    patch.object(sys,'argv',['motion','--scores-dir',str(folder.parent),'--output',str(out)]), \
                    contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(m.main(),0)
            self.assertIn('Time from drive command', (out/'scene/aligned_scores.svg').read_text())
            self.assertIn('brake', (out/'scene/aligned_scores.svg').read_text())
            self.assertEqual(len((out/'phase_summary.csv').read_text().splitlines()),7)
            self.assertTrue((out/'index.html').is_file())
            ann_path.write_text('{}')
            with self.assertRaisesRegex(ValueError,'annotations changed'):
                m.analyze_scene(folder,1,.1,np)


if __name__=='__main__':unittest.main()
