#!/usr/bin/env python3
"""Independent final-planning guard; never triggers collision recovery."""
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
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
        self.publish_debug_markers = bool(
            self.declare_parameter('publish_debug_markers', True).value)
        self.debug_marker_topic = str(self.declare_parameter(
            'debug_marker_topic', '/planning/drivable_guard/markers').value)
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
        self.debug_pub = (self.create_publisher(MarkerArray,self.debug_marker_topic,QoSProfile(depth=1))
                          if self.publish_debug_markers else None)
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
        self.debug_shapes = {name: [] for name in ('turning_stop','straight_stop','route')}
        self.debug_info = {}
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
        self.debug_info.update(position=(t.x,t.y), measured_speed=math.hypot(*velocity[:2]),
                               target_speed=None, profile_speed=None, check_speed=None,
                               stopping_distance=self.cfg.stopping_distance(math.hypot(*velocity[:2])))
        turning_trace = lambda shape, hit: self.debug_shapes['turning_stop'].append((shape,bool(hit)))
        straight_trace = lambda shape, hit: self.debug_shapes['straight_stop'].append((shape,bool(hit)))
        issue=motion_issue(self.environment,(t.x,t.y,yaw),velocity,self.cfg,trace=turning_trace)
        if not issue:
            # The stop command centres steering. Cover that possible braking path too.
            issue=motion_issue(self.environment,(t.x,t.y,yaw),(*velocity[:2],0.),self.cfg,
                               trace=straight_trace)
        if issue:
            return False,True,issue
        if not self.fresh('speed',self.input_timeout) or not math.isfinite(self.speed.data):
            return False,False,'target speed missing, stale or invalid'
        speed=max(math.hypot(*velocity[:2]),abs(self.speed.data))
        self.debug_info['target_speed'] = abs(self.speed.data)
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
            self.debug_info['profile_speed'] = max(abs(v) for v in speeds)
            points=[(p.pose.position.x,p.pose.position.y) for p in self.profile.points]
            closed=self.profile.closed
        else:
            if not self.fresh('path',self.input_timeout) or not self.stamp_fresh(self.path.header.stamp):
                return False,False,'selected trajectory missing or stale'
            if self.path.header.frame_id != self.map_frame:
                return False,False,'selected trajectory frame does not match HD map'
            points=[(p.pose.position.x,p.pose.position.y) for p in self.path.poses]
        self.debug_info['check_speed'] = speed
        self.debug_info['stopping_distance'] = self.cfg.stopping_distance(speed)
        route_trace = lambda shape, hit: self.debug_shapes['route'].append((shape,bool(hit)))
        issue=route_issue(self.environment,(t.x,t.y),points,speed,self.cfg,closed,trace=route_trace)
        if issue:
            return False,False,'selected trajectory unsafe: '+issue
        return True,False,'physical stopping envelope and selected trajectory clear'

    @staticmethod
    def _point(x, y, z=.03):
        point = Point()
        point.x, point.y, point.z = x, y, z
        return point

    def _shape_marker(self, stamp, marker_id, namespace, marker_type, shapes, color):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.
        marker.scale.x = .015 if marker_type == Marker.LINE_LIST else 1.
        marker.scale.y = marker.scale.z = 1.
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        for polygon, _ in shapes:
            if marker_type == Marker.TRIANGLE_LIST:
                for index in range(1,len(polygon)-1):
                    for vertex in (polygon[0],polygon[index],polygon[index+1]):
                        marker.points.append(self._point(*vertex))
            else:
                for first,second in zip(polygon,polygon[1:]+polygon[:1]):
                    marker.points.extend((self._point(*first,.04),self._point(*second,.04)))
        return marker

    def build_debug_markers(self, stamp, ready, emergency, reason):
        output = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        output.markers.append(clear)
        colors = {
            'turning_stop': (1.,.65,0.,.10),
            'straight_stop': (0.,.75,1.,.10),
            'route': (0.,1.,.3,.08),
        }
        marker_id = 0
        for name in ('turning_stop','straight_stop','route'):
            shapes = getattr(self,'debug_shapes',{}).get(name,[])
            safe = [entry for entry in shapes if not entry[1]]
            hit = [entry for entry in shapes if entry[1]]
            for entries,suffix,color in ((safe,'safe',colors[name]),
                                         (hit,'collision',(1.,0.,0.,.45))):
                if not entries:
                    continue
                for marker_type in (Marker.TRIANGLE_LIST,Marker.LINE_LIST):
                    marker_color = color if marker_type == Marker.TRIANGLE_LIST else (*color[:3],.8)
                    output.markers.append(self._shape_marker(
                        stamp,marker_id,'drivable_guard/'+name+'/'+suffix,
                        marker_type,entries,marker_color))
                    marker_id += 1
        info = getattr(self,'debug_info',{})
        if 'position' in info:
            def value(name):
                raw = info.get(name)
                return '-' if raw is None else f'{raw:.2f}'
            label = Marker()
            label.header.frame_id, label.header.stamp = self.map_frame, stamp
            label.ns, label.id = 'drivable_guard/status', marker_id
            label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            label.pose.position = self._point(*info['position'],.35)
            label.pose.orientation.w = 1.
            label.scale.z = .12
            if emergency or not ready:
                label.color.r, label.color.g, label.color.b = 1.,.2,.2
            else:
                label.color.r, label.color.g, label.color.b = .2,1.,.2
            label.color.a = 1.
            label.text = ('drivable_guard: ' + ('EMERGENCY' if emergency else 'READY' if ready else 'BLOCKED') +
                          f'\nv(measured/target/profile/check)={value("measured_speed")}/'
                          f'{value("target_speed")}/{value("profile_speed")}/{value("check_speed")} m/s'
                          f'\nstopping_distance={value("stopping_distance")} m\n{reason}')
            output.markers.append(label)
        return output

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
        if getattr(self,'debug_pub',None) is not None:
            self.debug_pub.publish(self.build_debug_markers(output.header.stamp,ready,emergency,reason))


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
