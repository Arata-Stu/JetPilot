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
        expressions=[v for node in ast.walk(tree) if isinstance(node,ast.Dict)
                     for k,v in zip(node.keys,node.values)
                     if isinstance(k,ast.Constant) and k.value=='vehicle_steering_trim']
        self.assertEqual(len(expressions),1)
        code=compile(ast.Expression(expressions[0]),'bringup.launch.py','eval')
        for enabled,pkg,expected in [(True,'jetpilot_bridge_interface',True),
                                      (False,'jetpilot_bridge_interface',False),
                                      (True,'pca9685_rc_driver',False),
                                      (True,'jetpilot_vesc_interface',False)]:
            self.assertEqual(eval(code,dict(lu=SimpleNamespace(is_true=truth),
                args=Args(enable_vehicle=enabled,vehicle_interface_pkg=pkg))),expected)

if __name__=='__main__':unittest.main()
