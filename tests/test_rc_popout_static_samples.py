import contextlib
import copy
import csv
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
import rc_popout_static_samples as samples


class StaticSamplesTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.static = self.root/'static'
        self.static.mkdir()
        chain = self.root/'chain.yaml'
        chain.write_text('saved calibration')
        self.spatial = dict(view_frame='evs', output_size=[640, 480], projection='rotation-only',
            camchain=str(chain), camchain_sha256=samples.digest(chain),
            rgb_to_view_homography=[[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
            event_to_view_homography=[[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
        scenes = []
        for i, session in enumerate(samples.SESSIONS):
            bag = self.root/session
            bag.mkdir()
            raw, mcap = bag/'camera.raw', bag/'bag.mcap'
            raw.write_bytes(b'raw'); mcap.write_bytes(b'bag')
            sync = bag/'time_sync_led_auto.yaml'
            sync.write_text('saved sync')
            annotation = bag/'sequence_annotations.json'
            samples.write_json(annotation, dict(session=session, time_sync=str(sync), time_sync_sha256=samples.digest(sync),
                reference_origin_s=100., onset={'first_visible': {'rgb_time_s': 101.}}, rgb_timestamp_source='bag'))
            identity = dict(annotation_sha256=samples.digest(annotation), time_sync_sha256=samples.digest(sync),
                recording_directory=str(bag), small_files=[dict(path=str(p), sha256=samples.digest(p)) for p in (annotation, sync)],
                recording_files=[dict(path=str(p), bytes=p.stat().st_size) for p in (raw, mcap)])
            scenes.append(dict(session=session, condition='left' if i < 3 else 'right', transmitter_limit='100pct',
                source_identity=identity, annotation=str(annotation), reference_origin_s=100., rgb_first_visible_s=1., intervals=[[0., 2.]]))
        samples.write_json(self.static/'preflight.json', dict(input_definition={'spatial': self.spatial}, scenes=scenes))
        samples.write_json(self.static/'run_config.json', dict(status='complete', preflight_sha256=samples.digest(self.static/'preflight.json')))

    def test_preflight_all_six_pins_auto_even_if_manual_exists_and_writes_nothing(self):
        (self.root/samples.SESSIONS[0]/'time_sync_led.yaml').write_text('different manual sync')
        output = self.root/'samples'
        before = {p: samples.digest(p) for p in self.root.rglob('*') if p.is_file()}
        with contextlib.redirect_stdout(io.StringIO()) as log, patch.object(samples.subprocess, 'run') as run:
            code = samples.main(['--static-dir', str(self.static), '--output', str(output), '--preflight'])
        self.assertEqual(code, 0)
        self.assertIn('6 sessions', log.getvalue())
        self.assertIn('--timeline event', log.getvalue())
        self.assertIn('--view-frame evs', log.getvalue())
        self.assertIn('--event-window-position before', log.getvalue())
        self.assertIn('time_sync_led_auto.yaml', log.getvalue())
        self.assertNotIn('--time-sync '+str(self.root/samples.SESSIONS[0]/'time_sync_led.yaml'), log.getvalue())
        run.assert_not_called()
        self.assertFalse(output.exists())
        self.assertEqual(before, {p: samples.digest(p) for p in before})

    def test_source_mutations_and_clip_outside_annotation_rejected(self):
        session = samples.SESSIONS[0]
        with self.assertRaisesRegex(ValueError, 'exceeds'):
            samples.prepare(self.static, [session], 1.1, .35)
        for name, pattern in (('time_sync_led_auto.yaml', 'sync changed'),
                              ('sequence_annotations.json', 'annotation changed'),
                              ('bag.mcap', 'byte size')):
            p = self.root/session/name
            saved = p.read_bytes(); p.write_bytes(saved+b' ')
            with self.assertRaisesRegex(ValueError, pattern):
                samples.prepare(self.static, [session], .15, .35)
            p.write_bytes(saved)

    def render_fixture(self):
        _, scenes = samples.prepare(self.static, [samples.SESSIONS[0]], .002, .004)
        scene = scenes[0]
        rows = []
        for i in range(6):
            t = 100.+scene['start_s']+i*.001
            rgb = 100.99 if t < 101. else 101.
            rows.append(dict(frame=i, video_time_s=i/60, reference_time_s=t, relative_time_s=t-100.,
                rgb_time_s=rgb, rgb_age_ms=(t-rgb)*1000, event_start_us=i*1000, event_end_us=i*1000+2000, event_count=1))
        summary = dict(self.spatial, timeline='event', step_ms=1., fps=60., event_window_ms=2., event_window_position='before',
            rgb_timestamp_source='bag', rgb_display_policy='latest_at_or_before', time_sync_sha256=scene['time_sync_sha256'],
            reference_origin_s=100., truncated=False, rendered_frames=len(rows), alpha=.85)
        folder = self.root/'render'
        folder.mkdir()
        samples.write_json(folder/'summary.json', summary)
        with (folder/'frames.csv').open('w') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
        return scene, summary, rows, folder

    def test_event_manifest_and_stills_reject_future_rgb_and_wrong_geometry(self):
        scene, summary, rows, folder = self.render_fixture()
        checked, frames = samples.validate_render(folder, scene, self.spatial, 1., 60.)
        selected = samples.select_stills(frames, scene, [-1., 0., 2.], 1.)
        self.assertEqual([s['label'] for s in selected], ['before', 'onset', 'after'])
        self.assertEqual(selected[1]['frame']['rgb_time_s'], 101.)
        changed = copy.deepcopy(summary)
        changed['rgb_to_view_homography'][0][2] = 10.
        samples.write_json(folder/'summary.json', changed)
        with self.assertRaisesRegex(ValueError, 'geometry'):
            samples.validate_render(folder, scene, self.spatial, 1., 60.)
        samples.write_json(folder/'summary.json', summary)
        rows[0]['rgb_time_s'] = rows[0]['reference_time_s']+.01
        with (folder/'frames.csv').open('w') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
        with self.assertRaisesRegex(ValueError, 'causal'):
            samples.validate_render(folder, scene, self.spatial, 1., 60.)

    def test_old_held_image_is_not_labeled_as_onset(self):
        scene, _, rows, _ = self.render_fixture()
        for row in rows:
            row['rgb_time_s'] = 100.99
        with self.assertRaisesRegex(ValueError, 'first-visible RGB frame is absent'):
            samples.select_stills(rows, scene, [-1., 0., 2.], 1.)

    def test_main_records_success_and_failed_render_in_gallery(self):
        scene, summary, rows, _ = self.render_fixture()
        output = self.root/'presentation'

        def render(command, **kwargs):
            path = Path(command[command.index('--output-dir')+1])
            if samples.SESSIONS[1] in str(path):
                raise samples.subprocess.CalledProcessError(2, command)
            path.mkdir()
            samples.write_json(path/'summary.json', summary)
            with (path/'frames.csv').open('w') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)

        with patch.object(samples.subprocess, 'run', side_effect=render), \
             patch.object(samples, 'write_stills') as stills, \
             contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            code = samples.main(['--static-dir', str(self.static), '--output', str(output),
                '--before-s', '.002', '--after-s', '.004', '--sample-offset-ms', '-1', '0', '2',
                '--sessions', *samples.SESSIONS[:2]])
        self.assertEqual(code, 1)
        self.assertEqual([r['status'] for r in samples.read_json(output/'summary.json')], ['complete', 'failed'])
        stills.assert_called_once()
        self.assertTrue((output/'run_config.json').is_file())
        self.assertIn('polarity_only.mp4', (output/'index.html').read_text())
        self.assertIn(samples.SESSIONS[1], (output/'index.html').read_text())

    def test_truncated_or_short_clip_is_rejected(self):
        scene, summary, rows, folder = self.render_fixture()
        summary['truncated'] = True
        samples.write_json(folder/'summary.json', summary)
        with self.assertRaisesRegex(ValueError, 'ended before'):
            samples.validate_render(folder, scene, self.spatial, 1., 60.)
        summary['truncated'] = False
        summary['rendered_frames'] -= 2
        samples.write_json(folder/'summary.json', summary)
        with (folder/'frames.csv').open('w') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows[:-2])
        with self.assertRaisesRegex(ValueError, 'shorter'):
            samples.validate_render(folder, scene, self.spatial, 1., 60.)

    def test_lossless_stills_use_raw_and_held_rgb_with_event_window_checks(self):
        import cv2
        import numpy as np
        sys.path.insert(0, str(samples.CALIBRATION))
        from multi_sensor_calibration import scenario_overlay as renderer
        scene, summary, rows, folder = self.render_fixture()
        selected = samples.select_stills(rows, scene, [-1., 0., 2.], 1.)
        summary['output_size'] = [64, 48]
        (folder/'video').mkdir()
        mask = np.full((48, 64), 255, dtype=np.uint8)
        mask[:, -1] = 0
        cv2.imwrite(str(folder/'video/common_valid_mask.png'), mask)
        camera = dict(camera_model='pinhole', distortion_model='radtan', resolution=[64, 48],
            intrinsics=[50., 50., 32., 24.], distortion_coeffs=[0., 0., 0., 0.])
        rgb = [(100.99, np.full((48, 64, 3), 40, dtype=np.uint8)),
               (101., np.full((48, 64, 3), 80, dtype=np.uint8))]
        polarity = np.full((48, 64), 127, dtype=np.uint8)
        polarity[20, 20] = 255
        events = [SimpleNamespace(image=polarity, window=SimpleNamespace(
            start_us=s['frame']['event_start_us'], end_us=s['frame']['event_end_us'], event_count=1)) for s in selected]
        with patch('multi_sensor_calibration.io.load_yaml', return_value={'cam0': camera, 'cam1': camera}), \
             patch.object(renderer, '_selected_rgb_times', return_value=(100., [0, 1], [100.99, 101.])), \
             patch.object(renderer, '_rgb_frames', return_value=iter(rgb)), \
             patch.object(renderer, '_event_frames', return_value=(iter(events), None)):
            samples.write_stills(folder, scene, self.spatial, summary, selected)
        self.assertEqual(len(list((folder/'stills').glob('*/*.png'))), 9)
        before = cv2.imread(str(folder/'stills/before/rgb.png'))
        onset = cv2.imread(str(folder/'stills/onset/rgb.png'))
        evs = cv2.imread(str(folder/'stills/onset/evs.png'))
        self.assertEqual(before[20, 20].tolist(), [40]*3)
        self.assertEqual(onset[20, 20].tolist(), [80]*3)
        self.assertEqual(onset[20, -1].tolist(), [35]*3)
        self.assertEqual(evs[20, 20].tolist(), [255, 0, 0])
        self.assertTrue((folder/'contact_sheet.png').is_file())


if __name__ == '__main__':
    unittest.main()
