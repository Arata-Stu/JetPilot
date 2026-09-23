"""Build the new launch graph with stdlib doubles, without importing ROS."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

from test_sections import ROOT, document, map_digest


class LaunchContractTests(unittest.TestCase):
    def test_metadata_drives_tensor_binding_and_head_order(self):
        class Action:
            def __init__(self,*args,**kwargs): self.args=args;self.kwargs=kwargs
        class Configuration:
            def __init__(self,name):self.name=name
            def perform(self,context):return context[self.name]
        modules={
            'launch':types.SimpleNamespace(LaunchDescription=Action),
            'launch.actions':types.SimpleNamespace(DeclareLaunchArgument=Action,OpaqueFunction=Action,RegisterEventHandler=Action),
            'launch.event_handlers':types.SimpleNamespace(OnShutdown=Action),
            'launch.substitutions':types.SimpleNamespace(LaunchConfiguration=Configuration),
            'launch_ros':types.SimpleNamespace(),
            'launch_ros.actions':types.SimpleNamespace(ComposableNodeContainer=Action,LoadComposableNodes=Action,Node=Action),
            'launch_ros.descriptions':types.SimpleNamespace(ComposableNode=Action),
            'yaml':types.SimpleNamespace(safe_load=json.loads),
        }
        with tempfile.TemporaryDirectory() as tmp, patch.dict(sys.modules,modules):
            root=Path(tmp);local=root/'track';local.mkdir();(local/'track_hd_map.yaml').write_text(json.dumps(document()))
            metadata={'model_kind':'section_multihead','onnx_verified':True,'map_document':document(),
                'map_sha256':map_digest(document()),'heads':[{'name':'straight','throttle':.3},{'name':'curve','throttle':.2},{'name':'generic','throttle':.15}],
                'generic_head':'generic','output':{'head_order':['straight','curve','generic']},
                'input':{'mean':[.485,.456,.406],'std':[.229,.224,.225]},
                'section_policy':{'straight':{'head':'straight','throttle':.3},'curve':{'head':'curve','throttle':.2}}}
            (root/'metadata.json').write_text(json.dumps(metadata))
            spec=importlib.util.spec_from_file_location('section_launch',ROOT/'ros2_ws/src/perception/jetpilot_e2e_inference/launch/section_multihead_tensor_rt.launch.py')
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            context={'model_root':str(root),'localization_map':str(local),'use_sim_time':'false','input_image_width':'424',
                'input_image_height':'240','image_topic':'/image','camera_info_topic':'/info','control_cmd_topic':'/auto/control_cmd',
                'container_name':'container','run_standalone':'true'}
            actions=module.setup(context)
            container=actions[-1]
            encoder,trt,decoder=container.kwargs['composable_node_descriptions']
            self.assertEqual(trt.kwargs['parameters'][0]['output_binding_names'],['control'])
            parameters=decoder.kwargs['parameters'][0]
            self.assertEqual(parameters['section_head_indices'],[0,1])
            self.assertEqual(parameters['generic_head_index'],2)
            self.assertEqual(parameters['generic_throttle'],.15)
            self.assertEqual(actions[0].kwargs['executable'],'hd_map_section_localizer_node.py')
            # Call the shutdown cleanup action; temporary map must not leak.
            handler=actions[2].args[0]
            handler.kwargs['on_shutdown'][0].kwargs['function'](context)
            changed=document();changed['frame_id']='different';(local/'track_hd_map.yaml').write_text(json.dumps(changed))
            with self.assertRaisesRegex(ValueError,'differ'):module.setup(context)


if __name__ == '__main__':unittest.main()
