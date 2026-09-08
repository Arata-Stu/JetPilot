#!/usr/bin/env python3
"""Jetson-only ROS adapter. Run via tuning.launch.py with existing localization/operation."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'backend'))
import argparse
import json
import math
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

from jetpilot_console.live_tuning import MAX_BYTES, encoded, map_identity, validate_snapshot
from jetpilot_console.tuning_state import TuningState


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
    from std_msgs.msg import Bool, Float32, String
    from nav_msgs.msg import Odometry, Path as RosPath
    from geometry_msgs.msg import PoseStamped, Point
    from jetpilot_msgs.msg import Trajectory, TrajectoryPoint, OperationModeState, DrivableArea, DrivableLane, StaticObstacle

    parser = argparse.ArgumentParser()
    parser.add_argument('--map-dir', required=True)
    args, ros_args = parser.parse_known_args()
    identity = map_identity(Path(args.map_dir).resolve())
    rclpy.init(args=ros_args)
    requests = queue.Queue(maxsize=8)

    class Bridge(Node):
        def __init__(self):
            super().__init__('live_tuning_bridge')
            self.state = TuningState(identity)
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.profile = self.create_publisher(Trajectory, '/tuning/trajectory_profile', qos)
            self.path = self.create_publisher(RosPath, '/tuning/trajectory', qos)
            self.speed = self.create_publisher(Float32, '/tuning/target_speed', qos)
            self.ready_pub = self.create_publisher(Bool, '/tuning/ready', qos)
            self.environment_pub = self.create_publisher(DrivableArea, '/tuning/drivable_area', qos)
            self.create_subscription(OperationModeState, '/operation_mode/state', self.mode, qos)
            self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.odometry, qos_profile_sensor_data)
            self.create_subscription(String, '/localization/pose_hint_state', self.localized, qos)
            self.messages = None
            self.create_timer(0.1, self.tick)

        def mode(self, msg):
            self.state.mode, self.state.mode_at = msg.mode, self.state.clock()

        def localized(self, msg):
            self.state.localization, self.state.localization_at = msg.data, self.state.clock()

        def odometry(self, msg):
            age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
            if not 0 <= age <= 0.5:
                self.state.update_speed(None)
                return
            v = msg.twist.twist.linear
            value = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
            self.state.update_speed(value if math.isfinite(value) else None)

        def build_environment(self, snapshot):
            environment = DrivableArea()
            environment.header.frame_id = snapshot['frame_id']
            hd_map = snapshot['hd_map']
            def points(rows):
                output = []
                for row in rows:
                    p = Point()
                    p.x, p.y = float(row[0]), float(row[1])
                    p.z = float(row[2]) if len(row) > 2 else 0.0
                    output.append(p)
                return output
            for lane in hd_map.get('lanes', []):
                if ('drivable_left_bound' in lane) != ('drivable_right_bound' in lane):
                    raise ValueError('Both physical bounds must be supplied together')
                left = lane.get('drivable_left_bound', lane.get('left_bound', []))
                right = lane.get('drivable_right_bound', lane.get('right_bound', []))
                if not left and not right:
                    continue
                item = DrivableLane()
                item.id, item.closed = lane['id'], lane['closed_loop']
                item.left_bound, item.right_bound = points(left), points(right)
                environment.lanes.append(item)
            for obstacle in hd_map.get('obstacles', []):
                item = StaticObstacle()
                item.id = obstacle['id']
                item.polygon = points(obstacle['polygon'])
                item.margin_m = float(obstacle.get('margin_m', 0.0))
                environment.obstacles.append(item)
            environment.valid = bool(environment.lanes)
            environment.reason = '' if environment.valid else 'snapshot has no physical bounds'
            return environment

        def build_messages(self, snapshot):
            msg = Trajectory()
            msg.header.frame_id = 'map'
            msg.line_id = snapshot['line']
            msg.display_name = snapshot['display_name']
            msg.source_hash = snapshot['revision']
            msg.closed = snapshot['closed']
            path = RosPath()
            path.header.frame_id = 'map'
            for s, x, y, yaw, curvature, speed, acceleration in snapshot['points']:
                p = TrajectoryPoint()
                p.s_m = s
                p.pose.position.x, p.pose.position.y = float(x), float(y)
                p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
                p.curvature_radpm = curvature
                p.longitudinal_velocity_mps = speed
                p.longitudinal_acceleration_mps2 = acceleration
                msg.points.append(p)
                stamped = PoseStamped()
                stamped.header.frame_id = 'map'
                stamped.pose = p.pose
                path.poses.append(stamped)
            return msg, path, self.build_environment(snapshot)

        def tick(self):
            state = self.state
            state.conflict = (self.count_publishers('/auto/control_cmd') > 1 or
                              self.count_publishers('/tuning/trajectory_profile') > 1)
            state.pose = None
            try:
                tf = self.buffer.lookup_transform('map', 'base_link', Time())
                stamp = Time.from_msg(tf.header.stamp)
                age = (self.get_clock().now() - stamp).nanoseconds / 1e9
                t, q = tf.transform.translation, tf.transform.rotation
                if 0 <= age <= 0.5 and all(math.isfinite(v) for v in (t.x, t.y, q.x, q.y, q.z, q.w)):
                    state.pose = {'x': t.x, 'y': t.y,
                                  'yaw': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))}
            except Exception:
                pass
            try:
                action, body, reply, deadline = requests.get_nowait()
                try:
                    if time.monotonic() > deadline:
                        raise ValueError('request expired')
                    if action == 'status':
                        state.touch_lease()
                        result = state.status()
                    elif action == 'active':
                        result = {'snapshot': state.active}
                    else:
                        candidate = state.previous if action == 'rollback' else body.get('snapshot')
                        if candidate is None:
                            raise ValueError('適用する版がありません。')
                        validate_snapshot(candidate, identity)
                        messages = self.build_messages(candidate)
                        result = state.apply(body, rollback=action == 'rollback')
                        self.messages = messages
                    reply.put(result)
                except Exception as exc:
                    reply.put({'error': str(exc)})
            except queue.Empty:
                pass
            ready = state.ready()
            self.ready_pub.publish(Bool(data=ready))
            self.speed.publish(Float32(data=max(p[5] for p in state.active['points']) if ready else 0.0))
            if self.messages:
                msg, path, environment = self.messages
                msg.header.stamp = path.header.stamp = environment.header.stamp = self.get_clock().now().to_msg()
                self.environment_pub.publish(environment)
                self.profile.publish(msg)
                self.path.publish(path)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_POST(self):
            try:
                if self.headers.get('Host') != '127.0.0.1:8781' or self.headers.get('Origin'):
                    raise ValueError('only local SSH transport is accepted')
                action = self.path.lstrip('/')
                if action not in ('status', 'apply', 'rollback', 'active'):
                    raise ValueError('unknown action')
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= MAX_BYTES:
                    raise ValueError('invalid request size')
                self.connection.settimeout(3)
                body = json.loads(self.rfile.read(size))
                if not isinstance(body, dict):
                    raise ValueError('request must be an object')
                reply = queue.Queue(maxsize=1)
                requests.put_nowait((action, body, reply, time.monotonic() + 4))
                result = reply.get(timeout=5)
            except Exception as exc:
                result = {'error': str(exc)}
            data = encoded(result)
            self.send_response(400 if 'error' in result else 200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

    server = HTTPServer(('127.0.0.1', 8781), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node = Bridge()
    try:
        rclpy.spin(node)
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
