#!/usr/bin/env python3
"""Visualize ID-bearing detections on a TF ground plane; no actuation output."""
import json
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray
from jetpilot_object_detection.opponent_geometry import project_contact, Trails, rotate


def seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


def transform(value):
    t=value.transform.translation;q=value.transform.rotation
    return ((t.x,t.y,t.z),(q.x,q.y,q.z,q.w))


class OpponentProjectionNode(Node):
    def __init__(self):
        super().__init__('opponent_projection')
        defaults={'detections_topic':'/perception/detections', 'camera_info_topic':'/realsense/color/camera_info',
                  'map_frame':'map', 'odom_frame':'odom', 'image_geometry':'raw',
                  'source_width':424, 'source_height':240, 'max_range_m':8.,
                  'max_message_age_s':0.75, 'history_seconds':15., 'gap_seconds':0.5,
                  'max_tracks':8, 'max_points':200, 'max_speed_mps':8.}
        self.p={key:self.declare_parameter(key,value).value for key,value in defaults.items()}
        if self.p['image_geometry'] not in ('raw','rectified'):
            raise ValueError('image_geometry must be raw or rectified')
        for key in ('max_range_m','max_message_age_s','history_seconds','gap_seconds','max_speed_mps'):
            if not math.isfinite(self.p[key]) or self.p[key]<=0: raise ValueError(f'Invalid {key}')
        if not 1<=self.p['max_tracks']<=32 or not 2<=self.p['max_points']<=1000:
            raise ValueError('max_tracks must be 1..32 and max_points 2..1000')
        self.trails=Trails(self.p['max_tracks'],self.p['max_points'],self.p['history_seconds'],
                           self.p['gap_seconds'],self.p['max_speed_mps'])
        self.tf=Buffer(cache_time=Duration(seconds=10.),node=self);self.listener=TransformListener(self.tf,self)
        self.infos=deque(maxlen=200);self.pending=deque(maxlen=30)
        self.last_stamp=None;self.last_frame=None;self.map_alignment=None
        self.reason='Waiting for tracked detections, CameraInfo and ground TF'
        self.markers=self.create_publisher(MarkerArray,'/perception/opponents/markers',10)
        self.status=self.create_publisher(String,'/perception/opponents/status',10)
        # Bounded stable topic slots; the status message maps slots to full track IDs.
        self.paths=[self.create_publisher(Path,f'/perception/opponents/track_{i}/path',10)
                    for i in range(self.p['max_tracks'])]
        self.create_subscription(CameraInfo,self.p['camera_info_topic'],self.infos.append,qos_profile_sensor_data)
        self.create_subscription(Detection2DArray,self.p['detections_topic'],self.pending.append,qos_profile_sensor_data)
        self.create_timer(0.1,self.tick)

    def lookup(self, child, stamp):
        return transform(self.tf.lookup_transform(self.p['map_frame'],child,Time.from_msg(stamp)))

    def process(self, message):
        header=message.header;stamp=seconds(header.stamp)
        if stamp<=0: raise ValueError('Missing acquisition timestamp')
        if self.last_stamp is not None and stamp<=self.last_stamp:
            return
        candidates=[info for info in self.infos if info.header.frame_id==header.frame_id and
                    0<seconds(info.header.stamp)<=stamp]
        if not candidates: raise ValueError('No matching CameraInfo at acquisition time')
        info=max(candidates,key=lambda value:seconds(value.header.stamp))
        roi=info.roi;bx=max(1,info.binning_x);by=max(1,info.binning_y)
        width=(roi.width or info.width)/bx;height=(roi.height or info.height)/by
        if (width,height)!=(self.p['source_width'],self.p['source_height']):
            raise ValueError('CameraInfo dimensions do not match detector source dimensions')
        model={'k':list(info.k),'d':list(info.d),'r':list(info.r),'p':list(info.p),
               'roi':[roi.x_offset,roi.y_offset,roi.width,roi.height,roi.do_rectify],
               'binning':(bx,by),'image_size':(width,height),'distortion_model':info.distortion_model}
        camera=self.lookup(header.frame_id,header.stamp)
        try:
            ground=self.lookup('base_footprint',header.stamp);estimated=False
        except TransformException:
            ground=self.lookup('tt02_ground_estimate',header.stamp);estimated=True
        if self.last_frame is not None and self.last_frame!=header.frame_id:
            self.trails.tracks.clear()
        # Localization corrections must not be rendered as opponent movement.
        try:
            alignment=self.lookup(self.p['odom_frame'],header.stamp)
            if self.map_alignment is not None:
                translation_jump=math.dist(alignment[0],self.map_alignment[0])
                old=rotate(self.map_alignment[1],(1,0,0));new=rotate(alignment[1],(1,0,0))
                if translation_jump>0.3 or sum(a*b for a,b in zip(old,new))<math.cos(0.2):
                    self.trails.tracks.clear()
            self.map_alignment=alignment
        except TransformException:
            self.map_alignment=None
        self.last_stamp=stamp;self.last_frame=header.frame_id
        accepted=0;rejected=0;last_rejection=''
        for detection in message.detections:
            if not detection.id or not detection.results: continue
            best=max(detection.results,key=lambda value:value.hypothesis.score)
            if best.hypothesis.class_id!='vehicle': continue
            box=detection.bbox
            try:
                point=project_contact((box.center.position.x,box.center.position.y,box.size_x,box.size_y),
                                      model,camera,ground,self.p['image_geometry'],self.p['max_range_m'])
                self.trails.observe(detection.id,stamp,point,estimated);accepted+=1
            except (ValueError,OverflowError,ZeroDivisionError) as error:
                last_rejection=str(error);rejected+=1
        self.reason=f'Ground projection: {accepted} accepted, {rejected} rejected; ' + (
            'TT-02 estimated plane' if estimated else 'base_footprint plane')
        if last_rejection: self.reason+='; '+last_rejection

    def tick(self):
        now=self.get_clock().now();now_s=now.nanoseconds/1e9
        if self.trails.last_time is not None and now_s<self.trails.last_time:
            self.pending.clear();self.infos.clear();self.last_stamp=None;self.last_frame=None
            self.map_alignment=None
        self.trails.expire(now_s)
        while self.pending:
            message=self.pending[0];age=now_s-seconds(message.header.stamp)
            if age>self.p['max_message_age_s']:
                self.pending.popleft();self.reason='Dropped stale detections / unavailable calibration or TF';continue
            if age < -0.01: break
            try:
                self.process(message);self.pending.popleft()
            except (TransformException,ValueError,KeyError,OverflowError,ZeroDivisionError) as error:
                self.reason=str(error)
                break  # Retry until TF/CameraInfo arrive, bounded by message age.
        self.publish_views(now,now_s)

    def publish_views(self, now, now_s):
        array=MarkerArray()
        clear=Marker();clear.header.frame_id=self.p['map_frame'];clear.header.stamp=now.to_msg()
        clear.action=Marker.DELETEALL;array.markers.append(clear)
        slots={track['slot']:(key,track) for key,track in self.trails.tracks.items()}
        for slot,publisher in enumerate(self.paths):
            path=Path();path.header.frame_id=self.p['map_frame'];path.header.stamp=now.to_msg()
            if slot in slots:
                key,track=slots[slot];points=track['points'];age=now_s-points[-1][0]
                color=(1.,0.65,0.15) if track['estimated'] else (0.2,0.85,1.)
                for stamp,point in points:
                    pose=PoseStamped();pose.header.frame_id=self.p['map_frame']
                    pose.header.stamp=Time(nanoseconds=round(stamp*1e9)).to_msg()
                    pose.pose.position=Point(x=point[0],y=point[1],z=point[2]);pose.pose.orientation.w=1.
                    path.poses.append(pose)
                for kind in ('trail','label','position'):
                    if kind=='position' and age>self.p['gap_seconds']: continue
                    marker=Marker();marker.header=path.header;marker.ns=kind;marker.id=slot
                    marker.action=Marker.ADD;marker.pose.orientation.w=1.
                    marker.color.r,marker.color.g,marker.color.b=color
                    marker.color.a=1. if age<=self.p['gap_seconds'] else 0.35
                    marker.lifetime=Duration(seconds=0.4).to_msg()
                    if kind=='trail':
                        marker.type=Marker.LINE_STRIP;marker.scale.x=0.025
                        marker.points=[pose.pose.position for pose in path.poses]
                        if len(marker.points)<2: continue
                    elif kind=='label':
                        marker.type=Marker.TEXT_VIEW_FACING;marker.scale.z=0.12
                        marker.pose.position=Point(x=points[-1][1][0],y=points[-1][1][1],z=points[-1][1][2]+0.2)
                        marker.text=f'track_{slot} #{key.rsplit(":",1)[-1]} ' + (
                            'projected' if age<=self.p['gap_seconds'] else f'lost {age:.1f}s')
                        if track['estimated']: marker.text+=' [ground estimate]'
                    else:
                        marker.type=Marker.SPHERE;marker.scale.x=marker.scale.y=marker.scale.z=0.08
                        marker.pose.position=path.poses[-1].pose.position
                    array.markers.append(marker)
            publisher.publish(path)  # Empty paths clear expired/reassigned slots.
        self.markers.publish(array)
        self.status.publish(String(data=json.dumps({'status':self.reason,'frame':self.p['map_frame'],
            'slots':{str(slot):{'track_id':key,'age_s':round(now_s-track['points'][-1][0],3),
                              'estimated_ground':track['estimated']} for slot,(key,track) in slots.items()}})))


def main(args=None):
    rclpy.init(args=args);node=OpponentProjectionNode()
    try: rclpy.spin(node)
    finally: node.destroy_node();rclpy.shutdown()


if __name__=='__main__': main()
