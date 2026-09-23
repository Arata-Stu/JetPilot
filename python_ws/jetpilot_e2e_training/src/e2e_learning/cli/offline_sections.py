"""ROS-only replay worker: saved-map localization -> image-time section labels.

Run inside the installed ROS workspace. No training dependencies are needed here.
"""
from __future__ import annotations
import argparse
from collections import deque
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from e2e_learning.data.sections import SectionMap


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--request", required=True)
    args = parser.parse_args()
    request = json.loads(Path(args.request).read_text())
    # Isolate replay from a live vehicle. Backend serializes tasks on this domain.
    os.environ["ROS_DOMAIN_ID"] = str(request.get("ros_domain_id", 121))
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.duration import Duration
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener
    geometry = SectionMap(request["map_document"], request.get("max_lane_distance_m", 1.0))
    rclpy.init()
    node = Node("section_dataset_replay", parameter_overrides=[Parameter("use_sim_time", value=True)])
    buffer = Buffer(cache_time=Duration(seconds=30))
    listener = TransformListener(buffer, node)
    status = {"localized":False, "received":0., "lane":"", "lane_received":0.}
    pending, labels = deque(), {}
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def localization(msg):
        try:
            status["localized"] = json.loads(msg.data).get("state") == "localized"
        except (ValueError, AttributeError):
            status["localized"] = False
        status["received"] = time.monotonic()

    def image(msg):
        stamp = int(msg.header.stamp.sec)*1_000_000_000+int(msg.header.stamp.nanosec)
        valid = status["localized"] and time.monotonic()-status["received"] < 2.
        if not valid or stamp <= 0:
            labels[str(stamp)] = "unknown"
        else:
            pending.append((stamp, time.monotonic()))

    def lane(msg):
        status["lane"], status["lane_received"] = msg.data, time.monotonic()

    subscriptions = [node.create_subscription(String, "/localization/pose_hint_state", localization, qos),
                     node.create_subscription(Image, request["image_topic"], image, qos_profile_sensor_data),
                     node.create_subscription(String, "/planning/current_lane", lane, qos)]

    def drain():
        for _ in range(len(pending)):
            stamp, received = pending.popleft()
            if not status["localized"] or time.monotonic()-status["received"] >= 2.:
                labels[str(stamp)] = "unknown"
                continue
            try:
                tf = buffer.lookup_transform(geometry.frame, request.get("base_frame", "base_link"), Time(nanoseconds=stamp))
                t = tf.transform.translation
                current_lane = status["lane"] if time.monotonic()-status["lane_received"] < .5 else ""
                labels[str(stamp)] = geometry.resolve(t.x, t.y, current_lane)
            except Exception:
                if time.monotonic()-received < 3.:
                    pending.append((stamp, received))
                else:
                    labels[str(stamp)] = "unknown"

    stack = player = None
    def stop(process):
        if process is not None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                return
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        command = ["ros2", "launch", "jetpilot_system_launch", "bringup.launch.py",
                   "use_sim_time:=true", "enable_rosbag_replay:=false",
                   "enable_sensor_kit:=false", "enable_operation:=false", "enable_control:=false",
                   "enable_vehicle:=false", "publish_vehicle_description:=true",
                   "enable_localization:=true", "enable_localization_manager:=true",
                   "vslam_enable_slam:=true", "enable_tool:=false", "enable_rviz:=false",
                   "enable_e2e_inference:=false", "enable_planning:=false",
                   f"map_dir:={request['localization_map']}",
                   f"vslam_localize_on_startup:={'false' if request.get('localization_mode') == 'vgl' else 'true'}",
                   f"enable_vgl:={'true' if request.get('localization_mode') == 'vgl' else 'false'}"]
        if request.get("camera_topic_config"):
            command.append(f"vgl_topic_config_file:={request['camera_topic_config']}")
        stack = subprocess.Popen(command, start_new_session=True)
        deadline = time.monotonic()+120
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if stack.poll() is not None:
                raise RuntimeError("VSLAM launch exited before replay")
            if any("visual_slam" in name for name in node.get_node_names()) and node.count_publishers("/localization/pose_hint_state"):
                break
        else:
            raise RuntimeError("VSLAM readiness timed out")
        topics = request["replay_topics"]
        if not topics:
            raise ValueError("VSLAM用のセンサーtopicがありません")
        player = subprocess.Popen(["ros2", "bag", "play", request["rosbag"], "--clock", "--delay", "3",
                                   "--rate", str(request.get("replay_rate", .5)), "--topics", *topics], start_new_session=True)
        replay_deadline = time.monotonic()+request.get("replay_duration_sec", 3600)/request.get("replay_rate", .5)+120
        while player.poll() is None:
            if time.monotonic() > replay_deadline:
                raise RuntimeError("rosbag replay timed out")
            if stack.poll() is not None:
                raise RuntimeError("VSLAM exited during replay")
            rclpy.spin_once(node, timeout_sec=.02)
            drain()
        if player.returncode != 0:
            raise RuntimeError(f"rosbag replay failed: {player.returncode}")
        deadline = time.monotonic()+4
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            drain()
        for stamp, _ in pending:
            labels[str(stamp)] = "unknown"
        if not any(value != "unknown" for value in labels.values()):
            raise RuntimeError("保存地図への定位を確認できませんでした。開始位置・VGL設定・センサーtopicを確認してください")
        Path(request["trace_path"]).write_text(json.dumps(labels))
    finally:
        stop(player)
        stop(stack)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
