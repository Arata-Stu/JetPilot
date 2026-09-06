import json
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import patch

from jetpilot_console.camera_projection import (ProjectionCollector, TransformBuffer, IDENTITY,
                                              inverse, matrix, multiply, localization_fingerprint)
from jetpilot_console.analysis_worker import AnalysisOptions, extract_analysis


def header(frame, ns):
    return N(frame_id=frame, stamp=N(sec=ns//10**9, nanosec=ns%10**9))


def info(frame='optical', ns=10**9, fx=500):
    return N(header=header(frame,ns), width=640,height=480,
             k=[fx,0,320,0,fx,240,0,0,1],d=[0.,0.,0.,0.,0.],r=[1,0,0,0,1,0,0,0,1],
             p=[fx,0,320,0,0,fx,240,0,0,0,1,0],distortion_model='plumb_bob',
             binning_x=0,binning_y=0,roi=N(x_offset=0,y_offset=0,width=0,height=0,do_rectify=False))


def tf(parent,child,ns,position=(0,0,0),rotation=(0,0,0,1)):
    return N(header=header(parent,ns),child_frame_id=child,
             transform=N(translation=N(**dict(zip('xyz',position))),rotation=N(**dict(zip('xyzw',rotation)))))


class CameraProjectionTest(unittest.TestCase):
    def test_rigid_chain_and_inverse_preserve_roll_pitch_and_optical_axes(self):
        buffer=TransformBuffer()
        buffer.add('map','base',10**9,[2,0,0],[0,0,0,1])
        buffer.add('base','optical',0,[0,0,1],[0.5,-0.5,0.5,-0.5],True)
        buffer.prepare()
        camera_from_map=buffer.resolve('optical','map',10**9)
        point=[sum(camera_from_map[r*4+k]*[12,0,0,1][k] for k in range(4)) for r in range(3)]
        self.assertEqual(point,[0,1,10])
        for actual,expected in zip(multiply(camera_from_map,inverse(camera_from_map)),IDENTITY):
            self.assertAlmostEqual(actual,expected)

    def test_interpolation_and_stale_or_future_tf(self):
        buffer=TransformBuffer()
        buffer.add('map','camera',10**9,[0,0,0],[0,0,0,1])
        buffer.add('map','camera',1100_000_000,[2,0,0],[0,0,math.sin(math.pi/4),math.cos(math.pi/4)])
        buffer.prepare()
        transform=buffer.resolve('map','camera',1050_000_000)
        self.assertAlmostEqual(transform[3],1)
        self.assertAlmostEqual(transform[0],math.sqrt(0.5))
        self.assertIsNone(buffer.resolve('map','camera',999_999_999))
        self.assertIsNone(buffer.resolve('map','camera',1300_000_000))

    def test_each_camera_uses_its_own_acquisition_timestamp(self):
        collector=ProjectionCollector()
        for camera in ('left','right'):
            collector.add_info(f'/{camera}/camera_info',info(camera,900_000_000),900_000_000)
            collector.transforms.add('base',camera,0,[0,0,0],[0,0,0,1],True)
        collector.transforms.add('map','base',900_000_000,[0,0,0],[0,0,0,1])
        collector.transforms.add('map','base',1000_000_000,[1,0,0],[0,0,0,1])
        frames=[{'channels':{f'/{camera}/image_raw':{'frame_id':camera,'header_timestamp_ns':str(ns)}
                              for camera,ns in [('left',1000_000_000),('right',950_000_000)]}}]
        meta=collector.compile(frames,'/left/image_raw')
        self.assertEqual(meta['ready_frames'],2)
        self.assertAlmostEqual(frames[0]['channels']['/left/image_raw']['projection']['camera_from_map'][3],-1)
        self.assertAlmostEqual(frames[0]['channels']['/right/image_raw']['projection']['camera_from_map'][3],-0.5)

    def test_missing_and_ambiguous_calibration_never_produce_a_transform(self):
        collector=ProjectionCollector()
        frames=[{'channels':{'/image_raw':{'frame_id':'optical','header_timestamp_ns':str(10**9)}}}]
        collector.compile(frames,'/image_raw')
        self.assertIn('CameraInfo',frames[0]['channels']['/image_raw']['projection']['issue'])
        collector.add_info('/a/camera_info',info(),10**9)
        collector.add_info('/b/camera_info',info(),10**9)
        collector.compile(frames,'/image_raw')
        self.assertIn('曖昧',frames[0]['channels']['/image_raw']['projection']['issue'])
        self.assertNotIn('camera_from_map',frames[0]['channels']['/image_raw']['projection'])

    def test_offline_pose_overrides_recorded_map_tf_without_losing_3d_orientation(self):
        collector=ProjectionCollector(offline=True)
        collector.add_tf(N(transforms=[tf('map','base',10**9,position=(100,0,0))]),10**9)
        collector.add_pose({'frame_id':'map','child_frame_id':'base','_header_timestamp_ns':10**9,
                            'x':1,'y':2,'z':3,'orientation':[0,math.sin(0.2),0,math.cos(0.2)]})
        collector.transforms.prepare()
        actual=collector.transforms.resolve('map','base',10**9)
        self.assertEqual([actual[i] for i in (3,7,11)],[1,2,3])
        self.assertAlmostEqual(actual[2],math.sin(0.4))

    def test_hd_edits_keep_pose_provenance_but_localization_map_changes_invalidate_it(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'cuvslam_map').mkdir()
            (root/'cuvslam_map'/'map.mdb').write_bytes(b'first')
            original=localization_fingerprint(root)
            (root/'course_hd_map.yaml').write_text('new lane')
            self.assertEqual(original,localization_fingerprint(root))
            (root/'cuvslam_map'/'map.mdb').write_bytes(b'changed map')
            self.assertNotEqual(original,localization_fingerprint(root))

    def test_worker_discovers_calibration_and_tf_and_packages_frame_projection(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory); bag=root/'bag';bag.mkdir()
            optical_q=(0.5,-0.5,0.5,-0.5)
            messages=[('/camera/camera_info',info(),10**9),
                      ('/tf_static',N(transforms=[tf('base','optical',0,(0,0,1),optical_q)]),10**9),
                      ('/tf',N(transforms=[tf('map','base',10**9)]),10**9),
                      ('/camera/image_raw',N(header=header('optical',10**9)),10**9)]
            topics={'/camera/camera_info':'sensor_msgs/msg/CameraInfo','/tf':'tf2_msgs/msg/TFMessage',
                    '/tf_static':'tf2_msgs/msg/TFMessage','/camera/image_raw':'sensor_msgs/msg/Image'}
            class Reader:
                def has_next(self): return bool(messages)
                def read_next(self): return messages.pop(0)
            with patch('jetpilot_console.analysis_worker._open_reader',return_value=(Reader(),topics)), \
                 patch('jetpilot_console.analysis_worker._deserializers',return_value=(lambda msg,typ:msg,lambda typ:object)), \
                 patch('jetpilot_console.analysis_worker._decode_image',return_value=object()), \
                 patch('jetpilot_console.analysis_worker._write_jpeg',return_value=(640,480)):
                extract_analysis(AnalysisOptions(rosbag=bag,analysis_dir=root/'analysis',image_topic='/camera/image_raw'))
            timeline=json.loads((root/'analysis'/'timeline.json').read_text())
            self.assertEqual(timeline['camera_projection']['ready_frames'],1)
            projection=timeline['frames'][0]['channels']['/camera/image_raw']['projection']
            self.assertEqual(projection['timestamp_ns'],str(10**9))
            self.assertEqual(len(projection['camera_from_map']),16)
