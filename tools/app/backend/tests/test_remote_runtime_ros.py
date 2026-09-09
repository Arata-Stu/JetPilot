import sys
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
from jetpilot_console.remote_runtime_ros import execute


class RosRequestTests(unittest.TestCase):
    def run_request(self, request, parameters=None, recording=True, reject=None):
        parameters = dict(parameters or {})
        writes = []
        class Request: pass
        class Param:
            def __init__(self, name, value): self.name, self.value = name, value
            def to_parameter_msg(self): return self
        def encoded(value):
            if isinstance(value, bool): return NS(type=1,bool_value=value)
            if isinstance(value, str): return NS(type=4,string_value=value)
            if isinstance(value, list): return NS(type=9,string_array_value=value)
            if value is None: return NS(type=0)
            return NS(type=3,double_value=value)
        def client(kind, endpoint):
            def call(msg):
                if endpoint.endswith('/get_parameters'):
                    result = NS(values=[encoded(parameters.get(name)) for name in msg.names])
                else:
                    change = {p.name:p.value for p in msg.parameters}
                    writes.append(change)
                    success = not (reject and reject(change))
                    if success: parameters.update(change)
                    result = NS(result=NS(successful=success,reason='rejected'))
                return NS(done=lambda:True,result=lambda:result)
            return NS(wait_for_service=lambda **kw:True,call_async=call)
        def subscription(_type, topic, callback, depth):
            callback(NS(recording=recording,current_uri='/bags/run',message='recording' if recording else 'idle',last_event='event'))
            return object()
        node = NS(create_client=client,create_subscription=subscription,destroy_node=lambda:None)
        modules = {
            'rclpy':NS(init=lambda **kw:None,create_node=lambda _:node,shutdown=lambda:None,
                       spin_once=lambda *a,**kw:None,spin_until_future_complete=lambda *a,**kw:None),
            'rclpy.parameter':NS(Parameter=Param),
            'rcl_interfaces.srv':NS(GetParameters=NS(Request=Request),SetParametersAtomically=NS(Request=Request)),
            'jetpilot_msgs.msg':NS(BagStatus=object),
        }
        with patch.dict(sys.modules, modules):
            result = execute(request)
        return result, writes, parameters

    def test_actual_bag_status_and_destination(self):
        for recording, state in ((True,'recording'),(False,'idle')):
            result,_,_ = self.run_request({'action':'bag-status'},recording=recording)
            self.assertEqual(result['bag']['state'],state)
            self.assertEqual(result['bag']['current_uri'],'/bags/run')

    def test_parameter_requires_node_capability(self):
        request={'action':'param-set','node':'/e2e_control_decoder','parameter':'fixed_throttle','value':0.3}
        with self.assertRaisesRegex(RuntimeError,'未対応'):
            self.run_request(request)
        result, writes, _ = self.run_request(request, {'dynamic_tuning_parameters':['fixed_throttle'],'fixed_throttle':0.2})
        self.assertEqual(result['value'],0.3)
        self.assertEqual(writes,[{'fixed_throttle':0.3}])

    def test_controller_string_parameter_round_trip(self):
        result, writes, _ = self.run_request(
            {'action':'param-set','node':'/path_tracking_controller_node','parameter':'algorithm','value':'kinematic_mpc'},
            {'dynamic_tuning_parameters':['algorithm'],'algorithm':'pure_pursuit'})
        self.assertEqual(result['value'],'kinematic_mpc')
        self.assertEqual(writes,[{'algorithm':'kinematic_mpc'}])

    def test_camera_preserves_resolution_and_enable_state(self):
        result,writes,params=self.run_request({'action':'camera-set','node':'/realsense','stream':'infra','fps':30},
          {'depth_module.infra_profile':'424x240x60','enable_infra1':True,'enable_infra2':False})
        self.assertEqual(writes,[{'enable_infra1':False,'enable_infra2':False},
                                {'depth_module.infra_profile':'424x240x30'},
                                {'enable_infra1':True,'enable_infra2':False}])
        self.assertEqual(result['camera']['profile'],'424x240x30')

    def test_camera_rejected_profile_reports_failure(self):
        with self.assertRaisesRegex(RuntimeError,'復元しました'):
            self.run_request({'action':'camera-set','node':'/realsense','stream':'rgb','fps':90},
              {'rgb_camera.color_profile':'424x240x30','enable_color':True},
              reject=lambda change:change.get('rgb_camera.color_profile')=='424x240x90')


if __name__=='__main__':unittest.main()
