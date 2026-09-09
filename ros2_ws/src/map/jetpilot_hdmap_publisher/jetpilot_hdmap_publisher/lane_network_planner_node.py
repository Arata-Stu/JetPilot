#!/usr/bin/env python3
"""Optional graph planner. Sole publisher of planning outputs in network mode."""
import json
import math
from pathlib import Path

import yaml
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage
from std_msgs.msg import String, Bool, Float32
from jetpilot_msgs.msg import Trajectory, TrajectoryPoint
from jetpilot_hdmap_publisher.lane_network import Tracker, source_hash, stations


class LaneNetworkPlanner(Node):
    def __init__(self):
        super().__init__('lane_network_planner')
        self.path = Path(self.declare_parameter('hd_map_yaml_path', '').value).expanduser()
        self.initial = self.declare_parameter('initial_lane_id', '').value
        self.mode = self.declare_parameter('line_mode', 'centerline').value
        self.speed = float(self.declare_parameter('target_speed_mps', .5).value)
        self.deceleration = float(self.declare_parameter('decel_mps2', 1.).value)
        self.lateral_accel = float(self.declare_parameter('lateral_accel_mps2', 1.).value)
        self.endpoint_margin = float(self.declare_parameter('endpoint_margin_m', .4).value)
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        if self.mode not in ('centerline','raceline') or not all(math.isfinite(v) and v>0 for v in (self.speed,self.deceleration,self.lateral_accel,self.endpoint_margin)):
            raise ValueError('invalid network planner parameters')
        self.tracker = None
        self.signature = None
        self.map_error = 'waiting for network map'
        self.choice_error = ''
        self.frame = 'map'
        self.fingerprint = ''
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(PathMessage,'/planning/trajectory',qos)
        self.profile_pub = self.create_publisher(Trajectory,'/planning/trajectory_profile',qos)
        self.speed_pub = self.create_publisher(Float32,'/planning/target_speed',qos)
        self.ready_pub = self.create_publisher(Bool,'/planning/ready',qos)
        self.lane_pub = self.create_publisher(String,'/planning/current_lane',qos)
        self.selected_pub = self.create_publisher(String,'/planning/selected_lane',qos)
        self.status_pub = self.create_publisher(String,'/planning/network_status',qos)
        self.create_subscription(String,'/planning/network_branch_choices',self.on_choices,10)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf,self)
        self.clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.05,self.tick,clock=self.clock)
        self.create_timer(1.,self.reload,clock=self.clock)
        self.reload()

    def reload(self):
        try:
            raw = self.path.read_bytes()
            if raw == self.signature:
                self.map_error = ''
                return
            data = yaml.safe_load(raw)
            lanes = data['lanes']
            if not any(l.get('successor_ids') for l in lanes):
                raise ValueError('map has no lane connections')
            fingerprint = source_hash(lanes,data.get('obstacles',[]))
            if self.mode == 'raceline' and any(not l.get('network_raceline') or l.get('network_source_hash') != fingerprint for l in lanes):
                raise ValueError('network racelines missing or stale; regenerate them')
            initial = self.initial or data.get('primary_lane_id',lanes[0]['id'])
            if self.tracker is not None:
                # Never relocate to a different lane after an on-disk revision.
                raise ValueError('network map changed; stop and restart planner with explicit initial lane')
            self.tracker = Tracker(lanes,initial)
            self.frame = data.get('frame_id','map')
            self.fingerprint = fingerprint
            self.signature = raw
            self.map_error = ''
        except Exception as exc:
            self.map_error = str(exc)

    def on_choices(self, message):
        try:
            if self.tracker is None:
                raise ValueError('network map is not ready')
            self.tracker.set_choices(json.loads(message.data))
            self.choice_error = ''
        except (ValueError,TypeError) as exc:
            self.choice_error = str(exc)

    def tick(self):
        path, profile = PathMessage(), Trajectory()
        now = self.get_clock().now()
        path.header.stamp = profile.header.stamp = now.to_msg()
        path.header.frame_id = profile.header.frame_id = self.frame
        reason, ready, target = '', False, 0.
        try:
            if self.map_error or self.choice_error or self.tracker is None:
                raise ValueError(self.map_error or self.choice_error or 'map unavailable')
            tf = self.tf.lookup_transform(self.frame,self.base_frame,Time())
            age = (now-Time.from_msg(tf.header.stamp)).nanoseconds/1e9
            if not -.1 <= age <= .3:
                raise ValueError('localization transform stale')
            t = tf.transform.translation
            self.tracker.update((t.x,t.y),max_step=.5)
            points = self.tracker.path(self.mode)
            # Reserve body clearance at an unresolved fork / terminal route end.
            ss = stations(points)
            stop_s = ss[-1]-self.endpoint_margin if ss else 0.
            points = [p for p,s in zip(points,ss) if s <= stop_s]
            if len(points) < 2:
                raise ValueError('route ended or waiting for branch choice')
            ss = stations(points)
            profile.line_id = 'network:' + self.tracker.current_lane_id
            profile.display_name = self.tracker.current_lane_id
            profile.source_hash = self.fingerprint
            profile.closed = False
            speeds = []
            for i,p in enumerate(points):
                pose = PoseStamped(); pose.header = path.header
                pose.pose.position.x,pose.pose.position.y = p
                a,b = points[max(0,i-1)],points[min(len(points)-1,i+1)]
                yaw = math.atan2(b[1]-a[1],b[0]-a[0])
                pose.pose.orientation.z = math.sin(yaw/2); pose.pose.orientation.w = math.cos(yaw/2)
                path.poses.append(pose)
                point = TrajectoryPoint(); point.pose = pose.pose; point.s_m = ss[i]
                curvature = 0.
                if 0 < i < len(points)-1:
                    u = (p[0]-a[0],p[1]-a[1]); v = (b[0]-p[0],b[1]-p[1])
                    denominator = math.dist(a,p)*math.dist(p,b)*math.dist(a,b)
                    if denominator > 1e-9:
                        curvature = 2*(u[0]*v[1]-u[1]*v[0])/denominator
                point.curvature_radpm = curvature
                speeds.append(min(self.speed,math.sqrt(self.lateral_accel/max(abs(curvature),1e-6))))
                profile.points.append(point)
            # An unresolved fork, dead end, or truncated horizon is a stopping endpoint.
            speeds[-1] = 0.
            for i in range(len(speeds)-2,-1,-1):
                speeds[i] = min(speeds[i],math.sqrt(speeds[i+1]**2+2*self.deceleration*(ss[i+1]-ss[i])))
            for point,speed in zip(profile.points,speeds):
                point.longitudinal_velocity_mps = speed
            target = speeds[min(1,len(speeds)-1)]
            ready = True
            reason = 'following ' + self.tracker.current_lane_id
        except Exception as exc:
            path.poses = []; profile.points = []
            reason = str(exc)
        self.path_pub.publish(path); self.profile_pub.publish(profile)
        self.speed_pub.publish(Float32(data=target)); self.ready_pub.publish(Bool(data=ready))
        current = self.tracker.current_lane_id if self.tracker else ''
        self.lane_pub.publish(String(data=current if ready else ''))
        self.selected_pub.publish(String(data=current if ready else ''))
        self.status_pub.publish(String(data=reason))


def main():
    rclpy.init()
    node = LaneNetworkPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
