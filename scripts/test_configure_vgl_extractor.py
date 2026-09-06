import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from configure_vgl_extractor import block, configure


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'ros2_ws/src/launch/jetpilot_system_launch/config/localization/vgl_config/keypoint_creation_config.pb.txt'
SCRIPT = ROOT / 'scripts/export_vgl_tensorrt_engines.sh'


class ExtractorConfigTest(unittest.TestCase):
    def test_changes_only_aliked_dimensions(self):
        original = SOURCE.read_text()
        result = configure(original, 424, 240)
        start, end = block(original, 'aliked_detector')
        new_start, new_end = block(result, 'aliked_detector')
        self.assertEqual(original[:start], result[:new_start])
        self.assertEqual(original[end:], result[new_end:])
        for field in ('min', 'opt', 'max'):
            self.assertIn(f'{field}: 1\n      {field}: 3\n      {field}: 240\n      {field}: 424', result)
        self.assertIn('use_fp16: true', result)

    def test_rejects_unexpected_schema(self):
        with self.assertRaises(ValueError):
            configure(SOURCE.read_text().replace('aliked_detector', 'other_detector'), 424, 240)
        with self.assertRaises(ValueError):
            configure(SOURCE.read_text().replace('min: 3', 'min: 1'), 424, 240)

    def test_exporter_receives_custom_config_and_native_is_unchanged(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            bin_dir = root / 'bin'
            bin_dir.mkdir()
            prefix = root / 'package'
            exporters = prefix / 'bin/visual_mapping'
            exporters.mkdir(parents=True)
            config_dir = prefix / 'configs/isaac'
            config_dir.mkdir(parents=True)
            source = config_dir / 'keypoint_creation_config.pb.txt'
            source.write_text(SOURCE.read_text())
            ros2 = bin_dir / 'ros2'
            ros2.write_text('#!/bin/bash\necho "$TEST_PACKAGE"\n')
            ros2.chmod(0o755)
            for name in ('export_lightglue_engine', 'export_extractor_engine'):
                executable = exporters / name
                executable.write_text('#!/bin/bash\nprintf "%s\\n" "$@" >> "$TEST_LOG"\n')
                executable.chmod(0o755)
            log = root / 'calls'
            env = dict(os.environ, PATH=f'{bin_dir}:{os.environ["PATH"]}',
                       TEST_PACKAGE=str(prefix), TEST_LOG=str(log), ROS2_WS=str(root))
            env.pop('OUTPUT_MODEL_DIR', None)
            for options, size in (([], '424x240'), (['--width', '640', '--height', '480'], '640x480')):
                subprocess.run(['bash', str(SCRIPT), '--yes', *options], env=env, check=True, capture_output=True)
                copied = root / f'isaac_ros_assets/models/visual_global_localization_{size}/keypoint_creation_config.pb.txt'
                width, height = map(int, size.split('x'))
                self.assertEqual(copied.read_text(), configure(source.read_text(), width, height))
                self.assertIn(str(copied), log.read_text())
            subprocess.run(['bash', str(SCRIPT), '--yes', '--native-profile'], env=env, check=True, capture_output=True)
            self.assertIn(str(source), log.read_text())
            self.assertEqual(source.read_text(), SOURCE.read_text())
            for options in (['--width', '0'], ['--height'], ['--native-profile', '--width', '424']):
                result = subprocess.run(['bash', str(SCRIPT), *options], env=env, capture_output=True)
                self.assertNotEqual(result.returncode, 0)
            engine = root / 'isaac_ros_assets/models/visual_global_localization_424x240/aliked_lightglue/aliked.engine'
            engine.parent.mkdir()
            engine.touch()
            result = subprocess.run(['bash', str(SCRIPT), '--yes'], env=env, capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn(b'engines already exist', result.stderr)


if __name__ == '__main__':
    unittest.main()
