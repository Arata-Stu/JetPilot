import importlib.util
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts/experiments'))
import apply_rc_popout_roi_candidate as apply

SPEC = importlib.util.spec_from_file_location('fixtures', ROOT/'tests/test_rc_popout_roi_candidate.py')
fixtures = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(fixtures)


class ApplyTests(unittest.TestCase):
    def setUp(self):
        fixtures.CandidateTests.setUp(self)
        chain = self.root/'chain.yaml'
        chain.write_text('calibration')
        summary = json.loads((self.folder/'summary.json').read_text())
        summary.update(camchain=str(chain), camchain_sha256=apply.digest(chain))
        (self.folder/'summary.json').write_text(json.dumps(summary))
        saved = json.loads(self.annotation.read_text())
        saved['intervals'][0]['note'] = ''
        self.annotation.write_text(json.dumps(saved))
        self.report = [dict(status='OK', dataset=str(self.dataset), session='scene',
                            candidate=self.candidate, valid_pixel_count=630*272,
                            geometry={k:summary[k] for k in apply.GEOMETRY})]

    def test_apply_preserves_timing_and_backs_up_original(self):
        original = self.annotation.read_bytes()
        jobs, config = apply.prepare(self.report)
        self.assertEqual(self.annotation.read_bytes(), original)
        self.assertEqual(len(config['sessions']), 1)
        for dataset, value, revision in jobs:
            apply.ann.save(dataset, dict(annotation=value, revision=revision))
        saved = json.loads(self.annotation.read_text())
        self.assertEqual(saved['rois'][0]['name'], 'band')
        self.assertEqual(saved['intervals'], json.loads(original)['intervals'])
        self.assertEqual(saved['onset']['first_visible']['rgb_time_s'], 101)
        backups = list(self.annotation.parent.glob('sequence_annotations.*.bak.json'))
        self.assertEqual(len(backups), 1)
        self.assertEqual(backups[0].read_bytes(), original)

    def test_stale_sync_rejected_without_write(self):
        original = self.annotation.read_bytes()
        (self.dataset.parent/'time_sync_led.yaml').write_text('new sync')
        with self.assertRaisesRegex(ValueError, 'stale synchronization'):
            apply.prepare(self.report)
        self.assertEqual(self.annotation.read_bytes(), original)


if __name__ == '__main__':
    unittest.main()
