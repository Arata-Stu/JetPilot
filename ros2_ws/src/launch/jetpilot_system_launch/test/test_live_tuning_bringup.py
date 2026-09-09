"""Standard-library-only tests; commands are dry runs, never launch hardware."""
import ast
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = next(p for p in Path(__file__).resolve().parents if (p / 'scripts/bringup.sh').is_file())


class TuningBringupTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.map = Path(self.tmp.name) / 'group' / 'timestamp-map'
        (self.map / 'cuvslam_map').mkdir(parents=True)
        (self.map / 'cuvslam_map' / 'test.mdb').write_text('dry run fixture')

    def run_launcher(self, *args):
        env = dict(os.environ, BRINGUP_DEFAULT_VEHICLE='jpbb', BRINGUP_MAP_DIR='',
                   BRINGUP_RACELINE_CSV='', BRINGUP_CUSTOM_LINE_CSV='')
        return subprocess.run(['bash', str(ROOT/'scripts/bringup.sh'), *args, '--dry-run'],
                              capture_output=True, text=True, env=env)

    def test_tuning_preset_starts_complete_localization_and_exclusive_controller(self):
        result = self.run_launcher('tuning', '--map', str(self.map))
        self.assertEqual(result.returncode, 0, result.stderr)
        for token in ('vehicle      : jpbb', 'enable_live_tuning:=true', 'enable_localization:=true',
                      'enable_sensor_kit:=true', 'enable_operation:=true', 'enable_control:=false',
                      'enable_e2e_inference:=false', 'enable_planning:=false',
                      'enable_hd_map_publisher:=true', 'enable_section_localizer:=true',
                      'enable_foxglove:=true'):
            self.assertIn(token, result.stdout)
        self.assertIn(str(ROOT/'tools/app/runtime/tuning.launch.py'), result.stdout)

    def test_parent_map_is_rejected(self):
        result = self.run_launcher('tuning', '--map', str(self.map.parent))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('timestamped child', result.stderr)

    def test_conflicting_controller_is_rejected(self):
        for name in ('enable_control', 'enable_e2e_inference', 'enable_competition_planning'):
            result = self.run_launcher('tuning', '--map', str(self.map), '--set', name+':=true')
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('cannot be combined', result.stderr)

    def test_default_vehicle_and_explicit_override(self):
        self.assertIn('vehicle      : jpbb', self.run_launcher('drive').stdout)
        self.assertIn('vehicle      : vesc', self.run_launcher('drive','--vehicle','vesc').stdout)
        self.assertIn('vehicle      : none', self.run_launcher('sensor').stdout)
        self.assertNotEqual(self.run_launcher('drive','--vehicle','none').returncode, 0)

    def test_launch_include_forwards_map_and_calibration_and_rejects_conflicts(self):
        source = ROOT/'ros2_ws/src/launch/jetpilot_system_launch/launch/bringup.launch.py'
        function = next(n for n in ast.parse(source.read_text()).body if isinstance(n, ast.FunctionDef) and n.name=='_include_live_tuning')
        class Config:
            def __init__(self, name): self.name=name
            def perform(self, context): return context[self.name]
        namespace = {'os':os,'LaunchConfiguration':Config,
                     '_launch_bool':lambda c,n:c.get(n,False),
                     'PythonLaunchDescriptionSource':lambda p:p,
                     'IncludeLaunchDescription':lambda source,launch_arguments:(source,dict(launch_arguments))}
        exec(compile(ast.Module(body=[function],type_ignores=[]), str(source), 'exec'), namespace)
        context={'enable_live_tuning':True,'enable_localization':True,'enable_vslam':True,
                 'enable_localization_manager':True,'enable_operation':True,'map_dir':str(self.map),
                 'live_tuning_launch_file':str(ROOT/'tools/app/runtime/tuning.launch.py'),
                 'control_param':'/controller.yaml','control_throttle_calibration_file':'/calibration.yaml'}
        result=namespace['_include_live_tuning'](context)
        self.assertEqual(result[0][1], {'map_dir':str(self.map),'controller_config':'/controller.yaml','throttle_calibration_file':'/calibration.yaml'})
        for flag in ('enable_control','enable_rosbag_replay','use_sim_time'):
            with self.assertRaises(RuntimeError): namespace['_include_live_tuning']({**context,flag:True})

if __name__ == '__main__': unittest.main()
