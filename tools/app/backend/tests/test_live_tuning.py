import hashlib
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from jetpilot_console.live_tuning import encoded, map_identity, prepare_snapshot, validate_snapshot, remote_request
from jetpilot_console.tuning_state import TuningState
from jetpilot_console.map_detail import create_custom_line


def seal(snapshot):
    snapshot.pop('revision', None)
    snapshot['revision'] = hashlib.sha256(encoded(snapshot)).hexdigest()
    return snapshot


def sample(line='centerline'):
    return seal({'format':1,'map_id':'map','frame_id':'map','closed':False,'line':line,
                 'points':[[0,0,0,0,0,1,0],[1,1,0,0,0,1,0]]})


class StateTests(unittest.TestCase):
    def setUp(self):
        self.now=10.
        self.state=TuningState('map', lambda:self.now)

    def stopped(self):
        self.state.mode=3
        self.state.mode_at=self.now
        self.state.update_speed(0.)
        self.state.stopped_since=self.now-2

    def test_apply_only_after_fresh_confirmed_stop(self):
        request={'snapshot':sample(),'expected_revision':''}
        with self.assertRaises(ValueError): self.state.apply(request)
        self.stopped()
        self.state.apply(request)
        self.assertEqual(self.state.active,request['snapshot'])
        self.state.mode=1
        with self.assertRaises(ValueError): self.state.apply({'snapshot':sample('raceline'),'expected_revision':self.state.active['revision']})
        self.assertEqual(self.state.active['line'],'centerline')

    def test_rollback_and_compare_and_swap(self):
        self.stopped()
        one,two=sample(),sample('raceline')
        self.state.apply({'snapshot':one,'expected_revision':''})
        with self.assertRaises(ValueError): self.state.apply({'snapshot':two,'expected_revision':''})
        self.state.apply({'snapshot':two,'expected_revision':one['revision']})
        self.state.apply({'expected_revision':two['revision']},rollback=True)
        self.assertEqual(self.state.active,one)

    def test_stale_speed_mode_and_conflicts_reject_apply(self):
        for field,value in [('speed_at',0),('mode_at',0),('conflict',True),('stopped_since',None)]:
            self.setUp();self.stopped();setattr(self.state,field,value)
            with self.assertRaises(ValueError): self.state.apply({'snapshot':sample(),'expected_revision':''})

    def test_disconnect_cannot_rearm_in_auto(self):
        self.stopped();self.state.touch_lease()
        self.state.active=sample();self.state.localization='localized';self.state.localization_at=self.now
        self.state.pose={'x':0,'y':0,'yaw':0}
        self.assertTrue(self.state.ready())
        self.now+=5
        self.state.mode=1;self.state.mode_at=self.now
        self.state.update_speed(0);self.state.localization_at=self.now
        self.state.touch_lease()
        self.assertFalse(self.state.ready())
        self.stopped();self.state.touch_lease()
        self.assertTrue(self.state.ready())

    def test_odometry_gap_resets_stop_confirmation(self):
        self.stopped()
        self.now += 1
        self.state.update_speed(0.)
        self.assertEqual(self.state.stopped_since, self.now)
        self.assertTrue(self.state.apply_issue())

    def test_bad_hash_map_and_numeric_data(self):
        for field,value in [('map_id','other'),('points',[[0,0,0,0,0,-1,0],[1,1,0,0,0,1,0]])]:
            candidate=sample();candidate[field]=value;seal(candidate)
            with self.assertRaises(ValueError):validate_snapshot(candidate,'map')
        candidate=sample();candidate['line']='tampered'
        with self.assertRaises(ValueError):validate_snapshot(candidate,'map')


class SnapshotTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)/'course';self.root.mkdir()
        self.config=SimpleNamespace(map_root=self.root.parent)
        (self.root/'vslam_reference_snapshot.json').write_text('{"reference":"test"}')
        (self.root/'course_hd_map.yaml').write_text('''frame_id: map
primary_lane_id: primary
lanes:
  - id: primary
    closed_loop: false
    left_bound: [[0, 1], [6, 1]]
    right_bound: [[0, -1], [6, -1]]
    centerline: [[0, 0], [3, 0], [6, 0]]
section_gates:
  - id: start
    lane_id: primary
    s_m: 1
    line: [[1,-1],[1,1]]
  - id: end
    lane_id: primary
    s_m: 5
    line: [[5,-1],[5,1]]
sections:
  - id: slow
    lane_id: primary
    start_gate_id: start
    end_gate_id: end
    start_s_m: 1
    end_s_m: 5
''')
        (self.root/'course_hd_map_centerline.csv').write_text('0,0\n3,0\n6,0\n')
        (self.root/'course_raceline.csv').write_text('0;0;0;0;0;1;0\n3;3;0;0;0;1;0\n6;6;0;0;0;1;0\n')

    def prepare(self,line='centerline',**extra):
        return prepare_snapshot(self.config,{'map_dir':str(self.root),'line':line,'speed_mps':1.,'section_speeds_mps':{'slow':0.5},**extra})

    def test_all_line_types_compile_with_section_speed(self):
        detail=create_custom_line(self.config,{'map_dir':str(self.root),'name':'調整ライン','source_type':'centerline'})
        custom=detail['custom_lines'][0]['id']
        for line in ('centerline','raceline','custom:'+custom):
            result=self.prepare(line)
            self.assertEqual(result['line'],line)
            self.assertTrue(all(row[5]<=0.5+1e-9 for row in result['points'] if 1<=row[1]<=5))
            self.assertEqual(validate_snapshot(result,result['map_id']),result)

    def test_identity_survives_mtime_but_detects_content(self):
        before=map_identity(self.root)
        path=self.root/'vslam_reference_snapshot.json'
        os.utime(path,(1,1));self.assertEqual(before,map_identity(self.root))
        path.write_text('other');self.assertNotEqual(before,map_identity(self.root))

    def test_map_identity_reports_missing_directory_and_empty_assets(self):
        missing=self.root/'missing'
        with self.assertRaisesRegex(ValueError, 'Map directory does not exist'):
            map_identity(missing)
        missing.mkdir()
        (missing/'cuvslam_map').mkdir()
        (missing/'cuvgl_map').symlink_to(missing/'not-mounted')
        with self.assertRaisesRegex(ValueError, 'No localization map files found') as caught:
            map_identity(missing)
        self.assertIn(str(missing), str(caught.exception))

    def test_unknown_section_and_foreign_frame_rejected(self):
        with self.assertRaises(ValueError):self.prepare(section_speeds_mps={'deleted':1})
        path=self.root/'course_hd_map.yaml'
        path.write_text(path.read_text().replace('frame_id: map','frame_id: odom'))
        with self.assertRaises(ValueError):self.prepare()

    def test_source_changes_produce_new_revision(self):
        one=self.prepare()
        two=self.prepare(speed_mps=0.8)
        self.assertNotEqual(one['revision'],two['revision'])
        self.assertEqual(one['map_id'],two['map_id'])

    def test_http_prepare_persists_exact_snapshot_and_apply_uses_it(self):
        from contextlib import nullcontext
        from jetpilot_console.main import Handler
        self.config.state_dir=self.root/'state'
        responses=[]
        handler=SimpleNamespace(
            server=SimpleNamespace(state=SimpleNamespace(config=self.config,
                tasks=SimpleNamespace(guard_resources=lambda keys:nullcontext()))),
            _json=lambda result, *status:responses.append(result))
        Handler._tuning_action(handler,'prepare',{'map_dir':str(self.root),'line':'centerline'})
        snapshot=responses[-1]['snapshot']
        self.assertEqual((self.config.state_dir/'tuning'/(snapshot['revision']+'.json')).read_bytes(), encoded(snapshot))
        with patch('jetpilot_console.main.remote_request', return_value={'revision':snapshot['revision']}) as remote:
            Handler._tuning_action(handler,'apply',{'revision':snapshot['revision'],'expected_revision':'old','snapshot':{'ignored':True}})
            self.assertEqual(remote.call_args.args[-1],{'snapshot':snapshot,'expected_revision':'old'})
            Handler._tuning_action(handler,'apply',{'revision':'../../outside'})
            self.assertEqual(remote.call_count,1)
            self.assertIn('error',responses[-1])

    def test_remote_payload_is_stdin_not_shell(self):
        config=SimpleNamespace(state_dir=self.root/'state',jetson_user='pilot',jetson_ips=['127.0.0.1'])
        with patch('jetpilot_console.live_tuning.subprocess.run') as run:
            run.return_value=SimpleNamespace(returncode=0,stdout=b'{"ok":true}')
            payload={'literal':'$(touch /tmp/never-execute)'}
            self.assertEqual(remote_request(config,{},'apply',payload),{'ok':True})
            args,kwargs=run.call_args
            self.assertNotIn(payload['literal'],' '.join(args[0]))
            self.assertEqual(kwargs['input'],encoded(payload))

    def test_remote_connection_errors_identify_the_failed_stage(self):
        config=SimpleNamespace(state_dir=self.root/'state',jetson_user='pilot',jetson_ips=['127.0.0.1'])
        cases = [
            (255, b'', b'Permission denied (publickey)', 'SSH接続に失敗'),
            (127, b'', b'python3: command not found', 'python3'),
            (0, b'{"connection_error":"Connection refused"}', b'', 'SSH接続は成功'),
            (0, b'login banner\n{}', b'', 'JSON応答'),
            (0, b'[]', b'', '応答形式'),
            (0, '{"error":"STOPにしてください"}'.encode(), b'', 'STOPにしてください'),
        ]
        for code, stdout, stderr, expected in cases:
            with self.subTest(expected=expected), patch('jetpilot_console.live_tuning.subprocess.run') as run:
                run.return_value=SimpleNamespace(returncode=code,stdout=stdout,stderr=stderr)
                with self.assertRaisesRegex(ValueError, expected):
                    remote_request(config, {}, 'status')

    def test_remote_timeout_is_not_retried(self):
        import subprocess
        config=SimpleNamespace(state_dir=self.root/'state',jetson_user='pilot',jetson_ips=['127.0.0.1'])
        with patch('jetpilot_console.live_tuning.subprocess.run', side_effect=subprocess.TimeoutExpired('ssh', 12)) as run:
            with self.assertRaisesRegex(ValueError, '自動再送していません'):
                remote_request(config, {}, 'apply')
            self.assertEqual(run.call_count, 1)

if __name__=='__main__':unittest.main()
