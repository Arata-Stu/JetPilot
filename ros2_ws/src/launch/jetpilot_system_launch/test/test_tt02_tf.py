"""TT-02 geometry and launch wiring checks; only the Python standard library."""
import ast
import copy
import importlib.util
import json
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('tt02_tf', ROOT/'launch/vehicle_tt02.launch.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def quaternion(rpy):
    r, p, y = (v/2 for v in rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]


class TT02TFTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((ROOT/'config/vehicle/tt02_cad.json').read_text())

    def records(self, **kwargs):
        return {child:(parent, xyz, rpy) for parent,child,xyz,rpy in MODULE.build_transforms(self.config, **kwargs)}

    def test_d455_composition_uses_left_ir_origin(self):
        records = self.records()
        position = [0., 0., 0.]
        child = 'realsense_camera_link'
        while child != 'base_link':
            child, xyz, rpy = records[child]
            self.assertEqual(rpy, [0, 0, 0])
            position = [a+b for a,b in zip(position, xyz)]
        for actual, expected in zip(position, [0.07194, 0.04750, 0.06450]):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(records['d455_mount_datum'][1], [0.005, 0, 0.060])
        self.assertFalse(any('optical' in child or 'imu' in child for child in records))

    def test_silky_mount_and_optical_axes_and_driver_alias(self):
        records = self.records(publish_evs=True)
        self.assertEqual(records['event_camera'][0], 'silky_optical_frame')
        self.assertEqual(records['evs_link'][0], 'silky_mount_link')
        q = quaternion(records['silky_optical_frame'][2])
        for actual, expected in zip(q, [-0.5, 0.5, -0.5, 0.5]):
            self.assertAlmostEqual(actual, expected)
        x,y,z,w=q
        # Optical +Z points forward, +X right, +Y down in the mount axes.
        self.assertAlmostEqual(2*(x*z+y*w), 1)
        self.assertAlmostEqual(2*(x*y+z*w), -1)
        self.assertAlmostEqual(2*(y*z+x*w), -1)
        position=[sum(values) for values in zip(self.config['plate_to_mount']['xyz'],
                  self.config['mount_to_silky']['xyz'], self.config['silky_to_optical']['xyz'])]
        for actual,expected in zip(position,[0.0295, 0, 0.10538]):
            self.assertAlmostEqual(actual,expected)

    def test_measurement_only_changes_base_to_plate(self):
        old=self.records(publish_evs=True)
        self.config['base_to_plate']={'xyz':[0.12,0,0.03],'rpy':[0,0,0.1], 'measured':True}
        new=self.records(publish_evs=True)
        self.assertNotEqual(old.pop('tt02_plate_link'),new.pop('tt02_plate_link'))
        self.assertEqual(old,new)

    def test_footprint_needs_height_and_does_not_duplicate_dynamic_base_parent(self):
        self.assertNotIn('base_link',self.records())
        with self.assertRaisesRegex(ValueError,'Measure'):
            self.records(localization_frame='base_footprint')
        self.config['base_height_m']=0.033
        records=self.records(localization_frame='base_footprint')
        self.assertEqual(records['base_link'],('base_footprint',[0,0,0.033],[0,0,0]))
        self.assertNotIn('base_footprint',records)

    def test_frame_aliases_do_not_create_duplicate_publishers(self):
        records=self.records(camera_frame='d455_link', evs_frame='silky_mount_link',
                             evs_optical_frame='silky_optical_frame',publish_evs=True)
        self.assertEqual(len(records),7)
        with self.assertRaises(ValueError): self.records(camera_frame='base_link')
        with self.assertRaises(ValueError): self.records(evs_frame='camera_mount_link',publish_evs=True)
        with self.assertRaises(ValueError): self.records(camera_frame='/bad')

    def test_estimated_ground_keeps_confirmed_support_and_assumed_clearance_separate(self):
        records=self.records()
        self.assertEqual(records['tt02_ground_estimate'],('tt02_plate_link',[0,0,-0.08],[0,0,0]))
        self.config['ground_estimate']['lower_deck_mount_height_m']=0.03
        self.assertAlmostEqual(self.records()['tt02_ground_estimate'][1][2],-0.085)
        self.config['ground_estimate']['enabled']=False
        self.assertNotIn('tt02_ground_estimate',self.records())

    def test_invalid_numbers_rejected(self):
        for value in ([0,0], [0,float('nan'),0], [0,'0',0]):
            config=copy.deepcopy(self.config);config['mount_to_d455']['xyz']=value
            with self.assertRaises(ValueError): MODULE.build_transforms(config)

    def test_vehicle_launch_selects_one_layout_and_forwards_arguments(self):
        lu=ModuleType('isaac_ros_launch_utils'); lut=ModuleType('isaac_ros_launch_utils.all_types')
        lu.ArgumentContainer=object;lut.LaunchDescription=object
        lu.is_true=lambda v: str(v).lower()=='true'
        lu.include=lambda *a,**kw: ('include',a,kw)
        lu.Node=lambda **kw: ('node',kw)
        lu.log_info=lambda *a: ('log',a)
        lu.all_types=lut
        with patch.dict(sys.modules,{'isaac_ros_launch_utils':lu,'isaac_ros_launch_utils.all_types':lut}):
            spec=importlib.util.spec_from_file_location('vehicle_tf_test', ROOT/'launch/vehicle.launch.py')
            vehicle=importlib.util.module_from_spec(spec);spec.loader.exec_module(vehicle)
            # Read literal argument defaults, without invoking ROS launch.
            defaults={}
            for node in ast.walk(ast.parse((ROOT/'launch/vehicle.launch.py').read_text())):
                if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute) and node.func.attr=='add_arg':
                    try: defaults[ast.literal_eval(node.args[0])]=ast.literal_eval(node.args[1])
                    except (ValueError,TypeError): pass
            defaults.update(enable_vehicle_interface=False,vehicle_description_tf_config='/tmp/custom.json',
                            publish_vehicle_evs_description=True)
            args=SimpleNamespace(**defaults)
            actions=vehicle.add_vehicle(args)
            includes=[a for a in actions if a[0]=='include']
            self.assertEqual(len(includes),1)
            self.assertEqual(includes[0][1][1],'launch/vehicle_tt02.launch.py')
            self.assertEqual(includes[0][2]['launch_arguments']['config_file'],'/tmp/custom.json')
            self.assertEqual(includes[0][2]['launch_arguments']['evs_optical_frame'],'event_camera')
            self.assertFalse(any(a[0]=='node' for a in actions))
            args.vehicle_description_layout='legacy'
            actions=vehicle.add_vehicle(args)
            self.assertEqual(len([a for a in actions if a[0]=='node']),2)
            self.assertFalse(any(a[0]=='include' for a in actions))
            args.publish_vehicle_description=False
            self.assertEqual(vehicle.add_vehicle(args),[])

    def test_bringup_uses_same_localization_and_event_header_frames(self):
        source=(ROOT/'launch/bringup.launch.py').read_text()
        self.assertIn("'vehicle_description_localization_frame': args.localization_base_frame",source)
        self.assertIn("'silky_evcam_frame_id': args.vehicle_description_evs_optical_frame",source)
        source=(ROOT/'launch/sensor_kit.launch.py').read_text()
        self.assertIn("'silky_evcam_frame_id': args.silky_evcam_frame_id",source)


if __name__=='__main__': unittest.main()
