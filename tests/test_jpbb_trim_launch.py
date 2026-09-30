"""Exercise launch decisions without ROS installed."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1] / 'ros2_ws/src/launch/jetpilot_system_launch/launch'

def truth(value):
    return str(value).lower() in ('true', '1')

class Args(SimpleNamespace):
    def __getattr__(self, key): return False

class TrimLaunchTests(unittest.TestCase):
    def test_tool_overrides_profile_only_when_vehicle_owns_trim(self):
        tree = ast.parse((ROOT/'tool.launch.py').read_text())
        function = next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='add_nodes')
        env = dict(lu=SimpleNamespace(ArgumentContainer=object, is_true=truth, Node=lambda **kwargs: kwargs),
                   lut=SimpleNamespace(ParameterValue=lambda value,**kwargs:value))
        exec(compile(ast.Module(body=[function],type_ignores=[]),'tool.launch.py','exec'),env)
        for owner in (True,False):
            actions=env['add_nodes'](Args(enable_teleop=True, vehicle_steering_trim=owner))
            node=next(n for n in actions if n['name']=='teleop_cmd_node')
            self.assertEqual(node['parameters'][-1]['steering_offset_enabled'],not owner)

    def test_bringup_selects_jpbb_active_vehicle_only(self):
        tree=ast.parse((ROOT/'bringup.launch.py').read_text())
        # Top-level ArgumentContainer values are unresolved substitutions, not strings.
        class Deferred:
            def __init__(self, name): self.name = name
            def __str__(self): return '<unresolved launch substitution>'

        env = dict(lut=SimpleNamespace(Substitution=object), _as_bool=truth,
                   lu=SimpleNamespace(perform_context=lambda ctx, value: ctx[value.name]))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                   and n.name == '_VehicleSteeringTrim')
        exec(compile(ast.Module(body=[cls], type_ignores=[]), 'bringup.launch.py', 'exec'), env)
        expressions=[v for node in ast.walk(tree) if isinstance(node,ast.Dict)
                     for k,v in zip(node.keys,node.values)
                     if isinstance(k,ast.Constant) and k.value=='vehicle_steering_trim']
        self.assertEqual(len(expressions),1)
        code=compile(ast.Expression(expressions[0]),'bringup.launch.py','eval')
        env['args'] = Args(enable_vehicle=Deferred('enabled'),
                           vehicle_interface_pkg=Deferred('pkg'))
        substitution = eval(code, env)
        for enabled,pkg,expected in [('true','jetpilot_bridge_interface','true'),
                                      ('false','jetpilot_bridge_interface','false'),
                                      ('true','pca9685_rc_driver','false'),
                                      ('true','jetpilot_vesc_interface','false')]:
            self.assertEqual(substitution.perform(dict(enabled=enabled, pkg=pkg)), expected)

if __name__=='__main__':unittest.main()
