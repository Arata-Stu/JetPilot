import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import create_map_with_vgl as mapping


class MappingTest(unittest.TestCase):
    def test_unknown_cli_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'config_folder_path'):
            mapping.require_options('--model_dir MODEL', ['--model_dir', '--config_folder_path'])

    def test_feature_extraction_gets_model_and_copied_config(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            base = root / 'maps'
            model = root / 'models with spaces'
            share = root / 'share'
            launch = root / 'launch'
            source = Path(__file__).resolve().parents[1] / 'ros2_ws/src/launch/jetpilot_system_launch/config/localization/vgl_config/keypoint_creation_config.pb.txt'
            original = source.read_text()
            config_dir = share / 'configs/isaac'
            config_dir.mkdir(parents=True)
            (config_dir / source.name).symlink_to(source)
            runtime_dir = launch / 'config/localization/vgl_config'
            runtime_dir.mkdir(parents=True)
            (runtime_dir / source.name).symlink_to(source)
            generated = base / 'new_map'
            commands = []
            def output(command, **kwargs):
                if '--help' in command:
                    return '--map_folder --raw_image_folder --config_folder_path --model_dir --binary_folder_path --extract_feature --build_bow_index --feature_type'
                if command[-1] == 'jetpilot_system_launch':
                    return str(launch)
                return str(share)
            def run(command, **kwargs):
                commands.append(command)
                if 'create_map_offline.py' in command:
                    self.assertNotIn('cuvgl', command)
                    frames = generated / 'map_frames/rectified'
                    frames.mkdir(parents=True)
                    (frames / 'frames_meta.json').write_text('{}')
                else:
                    self.assertIn(f'--model_dir={model.resolve()}', command)
                    self.assertIn('--extract_feature', command)
                    self.assertIn('--build_bow_index', command)
                    config = generated / 'vgl_mapping_config' / source.name
                    self.assertFalse(config.is_symlink())
                    self.assertIn('opt: 240\n      opt: 424', config.read_text())
                    self.assertNotIn('--no-extract_feature', command)
                    out = generated / 'cuvgl_map'
                    (out / 'keyframes').mkdir()
                    (out / 'keyframes/frames_meta.json').write_text('{}')
                    (out / 'bow_index.pb').touch()
                    (out / 'vocabulary').mkdir()
            args = SimpleNamespace(steps_to_run=['edex', 'compute_poses', 'cuvgl'],
                                   width=424, height=240, base_output_folder=base,
                                   model_dir=model, sensor_data_bag=root / 'bag',
                                   camera_topic_config=root / 'camera.yaml', fs_model_res='low_res')
            with patch.object(mapping, 'check_engine', return_value={'input_shape': [1,3,240,424]}), \
                 patch.object(mapping.subprocess, 'check_output', side_effect=output), \
                 patch.object(mapping.subprocess, 'run', side_effect=run):
                mapping.build(args)
            self.assertEqual(len(commands), 2)
            self.assertEqual(source.read_text(), original)
            self.assertIn('"status": "complete"', (generated / 'vgl_profile.json').read_text())

    def test_engine_shape_mismatch_stops_before_map_creation(self):
        with tempfile.TemporaryDirectory() as tmp:
            args = SimpleNamespace(steps_to_run=['edex', 'compute_poses', 'cuvgl'],
                width=424, height=240, base_output_folder=Path(tmp) / 'out', model_dir=Path(tmp))
            with patch.object(mapping, 'require_options'), \
                 patch.object(mapping.subprocess, 'check_output', return_value=''), \
                 patch.object(mapping, 'check_engine', side_effect=ValueError('input mismatch')), \
                 patch.object(mapping.subprocess, 'run') as run:
                with self.assertRaisesRegex(ValueError, 'input mismatch'):
                    mapping.build(args)
                run.assert_not_called()
                self.assertFalse(args.base_output_folder.exists())

if __name__ == '__main__':
    unittest.main()
