#!/usr/bin/env python3
"""Independent final-planning guard; never triggers collision recovery."""
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32
from jetpilot_msgs.msg import DrivableArea, PlanningSafetyStatus, Trajectory
from tf2_ros import Buffer, TransformListener

from jetpilot_hdmap_publisher.drivable_guard import Environment, Settings, motion_issue, route_issue


class DrivableGuard(Node):
    def __init__(self):
        super().__init__('drivable_guard')
        self.cfg = Settings(**{name: float(self.declare_parameter(name,default).value)
                               for name,default in vars(Settings()).items()})
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.input_timeout = float(self.declare_parameter('input_timeout_s', .5).value)
        self.map_timeout = float(self.declare_parameter('map_timeout_s', 3.).value)
        if not all(math.isfinite(v) and v > 0 for v in (self.input_timeout,self.map_timeout)):
            raise ValueError('timeouts must be finite and positive')
        self.environment = None
        self.environment_key = None
        self.map_frame = ''
        self.map_received = None
        self.map_error = 'waiting for physical HD map'
        self.profile = None
        self.profile_received = None
        self.odom = self.path = self.speed = None
        self.odom_received = self.path_received = self.speed_received = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf,self)
        self.pub = self.create_publisher(PlanningSafetyStatus,'/planning/safety_status',QoSProfile(depth=1))
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(DrivableArea,'/hd_map/drivable_area',self.on_map,latched)
        self.create_subscription(Odometry,'/visual_slam/tracking/odometry',
                                 lambda m:self.receive('odom',m),
                                 QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Path,'/planning/trajectory',lambda m:self.receive('path',m),latched)
        self.create_subscription(Trajectory,'/planning/trajectory_profile',lambda m:self.receive('profile',m),latched)
        self.create_subscription(Float32,'/planning/target_speed',lambda m:self.receive('speed',m),latched)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.05,self.tick,clock=self.steady_clock)

    def receive(self,name,message):
        setattr(self,name,message)
        setattr(self,name+'_received',time.monotonic())

    def on_map(self,message):
        self.map_received = time.monotonic()
        self.map_frame = message.header.frame_id
        if not message.valid or not self.map_frame:
            self.environment = None
            self.environment_key = None
            self.map_error = message.reason or 'invalid HD map'
            return
        try:
            lanes = [(tuple((p.x,p.y) for p in lane.left_bound),
                      tuple((p.x,p.y) for p in lane.right_bound),lane.closed) for lane in message.lanes]
            obstacles = [(o.id,tuple((p.x,p.y) for p in o.polygon),o.margin_m) for o in message.obstacles]
            key = (self.map_frame,lanes,obstacles)
            if key != self.environment_key:
                self.environment = Environment(lanes,obstacles)
                self.environment_key = key
            self.map_error = ''
        except (ValueError,TypeError,OverflowError) as exc:
            self.environment = None
            self.environment_key = None
            self.map_error = 'invalid physical HD map: ' + str(exc)

    def fresh(self,name,timeout):
        received = getattr(self,name+'_received')
        return received is not None and time.monotonic()-received <= timeout

    def stamp_fresh(self,stamp):
        age = (self.get_clock().now()-Time.from_msg(stamp)).nanoseconds/1e9
        return -.1 <= age <= self.input_timeout

    def evaluate(self):
        if self.environment is None or not self.fresh('map',self.map_timeout):
            return False,False,self.map_error or 'physical HD map heartbeat stale'
        if not self.fresh('odom',self.input_timeout) or not self.stamp_fresh(self.odom.header.stamp):
            return False,False,'odometry missing or stale'
        if self.odom.child_frame_id != self.base_frame:
            return False,False,'odometry twist frame does not match base_frame'
        transform = self.tf.lookup_transform(self.map_frame,self.base_frame,Time())
        if not self.stamp_fresh(transform.header.stamp):
            return False,False,'map-to-base transform stale'
        t,q = transform.transform.translation,transform.transform.rotation
        values=(t.x,t.y,t.z,q.x,q.y,q.z,q.w)
        if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in (q.x,q.y,q.z,q.w))-1)> .01:
            return False,False,'invalid map-to-base transform'
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        twist=self.odom.twist.twist
        velocity=(twist.linear.x,twist.linear.y,twist.angular.z)
        issue=motion_issue(self.environment,(t.x,t.y,yaw),velocity,self.cfg)
        if not issue:
            # The stop command centres steering. Cover that possible braking path too.
            issue=motion_issue(self.environment,(t.x,t.y,yaw),(*velocity[:2],0.),self.cfg)
        if issue:
            return False,True,issue
        if not self.fresh('speed',self.input_timeout) or not math.isfinite(self.speed.data):
            return False,False,'target speed missing, stale or invalid'
        speed=max(math.hypot(*velocity[:2]),abs(self.speed.data))
        closed=False
        if self.profile is not None and self.profile.points:
            if not self.fresh('profile',self.input_timeout) or not self.stamp_fresh(self.profile.header.stamp):
                return False,False,'selected trajectory profile stale'
            if self.profile.header.frame_id != self.map_frame:
                return False,False,'selected trajectory profile frame does not match HD map'
            speeds=[p.longitudinal_velocity_mps for p in self.profile.points]
            if not all(math.isfinite(v) for v in speeds):
                return False,False,'non-finite trajectory speed'
            speed=max(speed,max(abs(v) for v in speeds))
            points=[(p.pose.position.x,p.pose.position.y) for p in self.profile.points]
            closed=self.profile.closed
        else:
            if not self.fresh('path',self.input_timeout) or not self.stamp_fresh(self.path.header.stamp):
                return False,False,'selected trajectory missing or stale'
            if self.path.header.frame_id != self.map_frame:
                return False,False,'selected trajectory frame does not match HD map'
            points=[(p.pose.position.x,p.pose.position.y) for p in self.path.poses]
        issue=route_issue(self.environment,(t.x,t.y),points,speed,self.cfg,closed)
        if issue:
            return False,False,'selected trajectory unsafe: '+issue
        return True,False,'physical stopping envelope and selected trajectory clear'

    def tick(self):
        try:
            ready,emergency,reason=self.evaluate()
        except Exception as exc:
            # TF unavailable or malformed inputs remove permission; they are not a collision.
            ready,emergency,reason=False,False,'safety check unavailable: '+str(exc)
        output=PlanningSafetyStatus()
        output.header.stamp=self.get_clock().now().to_msg()
        output.header.frame_id=self.map_frame
        output.ready,output.emergency,output.reason=ready,emergency,reason
        self.pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node=DrivableGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
