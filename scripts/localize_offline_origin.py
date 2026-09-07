#!/usr/bin/env python3
"""Localize a paused offline replay after tracking starts; validate its snapshot."""
import argparse
import json
import math
from pathlib import Path
import time


def validate_snapshot(path):
    data = json.loads(Path(path).read_text())
    localization = data.get('localization') or {}
    if not (localization.get('required') and localization.get('confirmed')):
        raise ValueError('snapshot does not confirm saved-map localization')
    samples = data.get('odometry_samples') or []
    if len(samples) < 2 or any(s.get('frame_id') != 'map' for s in samples):
        raise ValueError('snapshot needs at least two odometry samples in map frame')
    cloud = data.get('landmarks') or {}
    if cloud.get('width', 0) * cloud.get('height', 0) <= 0 or not cloud.get('data'):
        raise ValueError('snapshot has no landmark points')


def localize(timeout, replay_rate):
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Path as PathMsg
    from std_msgs.msg import String
    from rosbag2_interfaces.srv import Resume, SetRate
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

    rclpy.init()
    node = rclpy.create_node('offline_origin_initializer')
    state = {'name': None, 'reason': '', 'stamp': None}
    def on_state(msg):
        try:
            payload = json.loads(msg.data)
            state['name'] = payload['state']
            state['reason'] = payload.get('reason', '')
        except (ValueError, KeyError, TypeError):
            pass
    def on_path(msg):
        if msg.poses:
            state['stamp'] = msg.header.stamp
    def wait_until(predicate, description):
        deadline = time.monotonic() + timeout
        while not predicate():
            if not rclpy.ok() or time.monotonic() >= deadline:
                raise RuntimeError(f'{description}: timeout; state={state["name"]}, reason={state["reason"]}')
            rclpy.spin_once(node, timeout_sec=0.1)
    def call(client, request, description):
        wait_until(client.service_is_ready, description + ' service')
        future = client.call_async(request)
        wait_until(future.done, description)
        result = future.result()
        if result is None:
            raise RuntimeError(description + ': no response')
        return result
    try:
        node.create_subscription(String, '/localization/pose_hint_state', on_state,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(PathMsg, '/visual_slam/tracking/slam_path', on_path,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        # Use the manager so its confirmation state also gates snapshot recording.
        publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        resume = node.create_client(Resume, '/rosbag2_player/resume')
        set_rate = node.create_client(SetRate, '/rosbag2_player/set_rate')
        wait_until(lambda: state['name'] == 'waiting_for_manual' and publisher.get_subscription_count() > 0,
                   'waiting for localization manager')
        call(resume, Resume.Request(), 'resume slow replay')
        wait_until(lambda: state['stamp'] is not None, 'waiting for first SLAM path')
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = state['stamp']
        pose.pose.pose.orientation.w = 1.0
        publisher.publish(pose)
        print('[stage] SLAM tracking ready; sent origin hint automatically', flush=True)
        wait_until(lambda: state['name'] == 'localized', 'saved-map localization')
        print('[stage] saved-map localization confirmed', flush=True)
        request = SetRate.Request()
        request.rate = replay_rate
        if not call(set_rate, request, 'restore replay rate').success:
            raise RuntimeError('rosbag player rejected replay rate')
        print(f'[stage] replay rate restored to {replay_rate}', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check-snapshot')
    parser.add_argument('--timeout', type=float, default=60.0)
    parser.add_argument('--replay-rate', type=float, default=1.0)
    args = parser.parse_args()
    if not all(math.isfinite(v) and v > 0 for v in (args.timeout, args.replay_rate)):
        parser.error('timeout and replay rate must be positive and finite')
    try:
        if args.check_snapshot:
            validate_snapshot(args.check_snapshot)
        else:
            localize(args.timeout, args.replay_rate)
    except (RuntimeError, ValueError, OSError) as error:
        parser.exit(1, f'error: {error}\n')


if __name__ == '__main__':
    main()
