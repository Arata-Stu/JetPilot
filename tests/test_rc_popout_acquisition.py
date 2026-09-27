"""収録監査と出現なしのドライラン。外部パッケージ・ROS不要。"""
import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'audit', ROOT / 'scripts/experiments/audit_rc_popout_recordings.py')
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


class AcquisitionTests(unittest.TestCase):
    def test_missing_sidecar_and_empty_raw(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp) / 'session'
            folder.mkdir()
            (folder / 'capture.raw').touch()
            self.assertEqual(AUDIT.discover(tmp), [folder])
            issues = AUDIT.audit_session(folder)['issues']
            self.assertIn('missing_mcap', issues)
            self.assertIn('empty:capture.raw', issues)
            self.assertIn('missing:capture.raw.metadata.yaml', issues)

    def test_presence_does_not_certify_quality_or_scene(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp) / 'rcp_r001_dir-left_thr-20pct'
            folder.mkdir()
            for name in ('metadata.yaml', 'data.mcap', 'data.raw', 'data.raw.metadata.yaml'):
                (folder / name).write_text('placeholder')
            report = AUDIT.audit_session(folder)
            self.assertEqual(report['issues'], [])
            self.assertEqual(report['scene_label'], 'unknown')
            self.assertEqual(report['planned_direction_from_filename'], 'left')
            self.assertEqual(report['file_status'], 'present_not_validated')
            self.assertEqual(report['sync_quality'], 'unchecked')

    def test_none_dry_run_does_not_publish_or_mutate_progress(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            (folder / 'plan.tsv').write_text(
                'run_id\tcamera_wall_distance\tobstacle_wall_distance\tgap_width\tlighting\tdirection\tthrottle\trepetition\n'
                'r001\t200cm\t100cm\t90cm\tnormal\tnone\tna\t1\n')
            for name in ('completed.txt', 'skipped.txt'):
                (folder / name).touch()
            result = subprocess.run(
                ['bash', str(ROOT / 'scripts/experiments/rc_popout_experiment.sh'),
                 '--dry-run', '--experiment-dir', tmp],
                input='n\n\n\n\n', text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('相手車は動かさず', result.stdout)
            self.assertIn('[dry-run] BagRequest', result.stdout)
            self.assertNotIn('フルトリガで飛び出し', result.stdout)
            self.assertFalse((folder / 'attempts.jsonl').exists())
            self.assertEqual((folder / 'completed.txt').read_text(), '')


if __name__ == '__main__':
    unittest.main()
