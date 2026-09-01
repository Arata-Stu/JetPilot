#!/usr/bin/env python3
"""Publish current section from HD map section gates and map->base_link TF."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional, Sequence, Tuple

import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from jetpilot_hdmap_publisher.hd_map_publisher_node import (
    HdMap,
    HdMapFileSignature,
    Lane,
    Section,
    cumulative_s,
    hd_map_file_signature,
    load_stable_hd_map,
    section_points,
    to_geometry_point,
)


Point3 = Tuple[float, float, float]


def transient_local_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def project_point_to_lane_s(point: Point3, lane: Lane) -> Tuple[float, float]:
    points = lane.centerline
    if not points:
        return 0.0, float("inf")
    if len(points) == 1:
        return 0.0, math.hypot(point[0] - points[0][0], point[1] - points[0][1])

    s_values, _total = cumulative_s(points, lane.closed_loop)
    segment_count = len(points) if lane.closed_loop and len(points) >= 3 else len(points) - 1
    best_s = 0.0
    best_distance = float("inf")
    for index in range(segment_count):
        next_index = (index + 1) % len(points)
        ax, ay, _az = points[index]
        bx, by, _bz = points[next_index]
        vx = bx - ax
        vy = by - ay
        denom = vx * vx + vy * vy
        if denom <= 1.0e-12:
            continue
        ratio = max(0.0, min(1.0, ((point[0] - ax) * vx + (point[1] - ay) * vy) / denom))
        cx = ax + ratio * vx
        cy = ay + ratio * vy
        distance = math.hypot(point[0] - cx, point[1] - cy)
        if distance < best_distance:
            best_distance = distance
            best_s = s_values[index] + math.sqrt(denom) * ratio
    return best_s, best_distance


def section_contains_s(section: Section, lane: Lane, s_value: float) -> bool:
    _s_values, total_length = cumulative_s(lane.centerline, lane.closed_loop)
    if total_length <= 1.0e-9:
        return False
    if lane.closed_loop:
        s = s_value % total_length
        start = section.start_s_m % total_length
        end = section.end_s_m % total_length
        if start > end:
            return s >= start or s < end
        return start <= s < end
    return section.start_s_m <= s_value < section.end_s_m


class HdMapSectionLocalizerNode(Node):
    def __init__(self) -> None:
        super().__init__("hd_map_section_localizer")
        self.hd_map_yaml_path = self.declare_parameter("hd_map_yaml_path", "").value
        self.frame_id_override = self.declare_parameter("frame_id_override", "").value
        self.base_frame = str(self.declare_parameter("base_frame", "base_link").value)
        self.update_rate_hz = max(float(self.declare_parameter("update_rate_hz", 10.0).value), 0.1)
        self.retry_interval_sec = max(float(self.declare_parameter("retry_interval_sec", 2.0).value), 0.1)
        self.tf_timeout_sec = max(float(self.declare_parameter("tf_timeout_sec", 0.02).value), 0.001)
        self.max_lane_distance_m = max(float(self.declare_parameter("max_lane_distance_m", 1.0).value), 0.0)
        self.current_section_topic = str(
            self.declare_parameter("current_section_topic", "/localization/current_section").value
        )
        self.current_marker_topic = str(
            self.declare_parameter(
                "current_section_marker_topic",
                "/localization/current_section_marker",
            ).value
        )

        qos = transient_local_qos()
        self.section_pub = self.create_publisher(String, self.current_section_topic, qos)
        self.marker_pub = self.create_publisher(Marker, self.current_marker_topic, qos)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.hd_map: Optional[HdMap] = None
        self.loaded_map_signature: Optional[HdMapFileSignature] = None
        self.rejected_map_signature: Optional[HdMapFileSignature] = None
        self.last_load_issue_key: Optional[tuple[object, ...]] = None
        self.current_section = "unknown"
        self.timer = self.create_timer(1.0 / self.update_rate_hz, self.on_timer)
        self.reload_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.reload_timer = self.create_timer(
            self.retry_interval_sec,
            self.reload_hd_map,
            clock=self.reload_clock,
        )
        self.try_load_hd_map()

    def log_load_issue(self, key: tuple[object, ...], message: str, *, warning: bool = False) -> None:
        if key == self.last_load_issue_key:
            return
        self.last_load_issue_key = key
        if warning:
            self.get_logger().warn(message)
        else:
            self.get_logger().error(message)

    def try_load_hd_map(self) -> bool:
        if not self.hd_map_yaml_path:
            self.log_load_issue(
                ("empty_path",),
                "Parameter hd_map_yaml_path is empty. Waiting for a YAML path.",
                warning=True,
            )
            return False
        path = Path(str(self.hd_map_yaml_path)).expanduser().resolve()
        signature = hd_map_file_signature(path)
        if signature is None:
            suffix = "; keeping the previous valid map" if self.hd_map is not None else ""
            self.log_load_issue(
                ("missing", str(path)),
                f"HD map YAML not found or not a regular file: {path}{suffix}",
            )
            return False
        if self.hd_map is not None and signature == self.loaded_map_signature:
            self.rejected_map_signature = None
            self.last_load_issue_key = None
            return False
        if self.hd_map is not None and signature == self.rejected_map_signature:
            return False
        try:
            candidate, stable_signature = load_stable_hd_map(path, str(self.frame_id_override))
        except Exception as exc:  # noqa: BLE001
            self.rejected_map_signature = signature
            suffix = "; keeping the previous valid map" if self.hd_map is not None else ""
            self.log_load_issue(
                ("invalid", signature, type(exc).__name__, str(exc)),
                f"Failed to load HD map YAML {path}: {exc}{suffix}",
            )
            return False
        reloaded = self.hd_map is not None
        self.hd_map = candidate
        self.loaded_map_signature = stable_signature
        self.rejected_map_signature = None
        self.last_load_issue_key = None
        self.get_logger().info(
            f"{'Reloaded' if reloaded else 'Loaded'} HD map sections: {path} "
            f"(sections={len(candidate.sections)}, frame={candidate.frame_id}, base={self.base_frame})"
        )
        # Do not leave a transient-local section from a previous map visible
        # while TF for this revision is unavailable. Publish on initial load as
        # well so startup is deterministic while /clock is paused.
        self.publish_current_section("unknown")
        return True

    def reload_hd_map(self) -> None:
        self.try_load_hd_map()

    def lookup_pose(self) -> Optional[Point3]:
        if self.hd_map is None:
            return None
        try:
            tf_msg: TransformStamped = self.tf_buffer.lookup_transform(
                self.hd_map.frame_id,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"HD map section TF lookup failed ({self.hd_map.frame_id} <- {self.base_frame}): {exc}",
                throttle_duration_sec=2.0,
            )
            return None
        t = tf_msg.transform.translation
        return (float(t.x), float(t.y), float(t.z))

    def resolve_section(self, pose: Point3) -> str:
        if self.hd_map is None or not self.hd_map.sections:
            return "unknown"

        best_lane: Optional[Lane] = None
        best_s = 0.0
        best_distance = float("inf")
        section_lane_ids = {section.lane_id for section in self.hd_map.sections}
        for lane in self.hd_map.lanes:
            if lane.lane_id not in section_lane_ids or len(lane.centerline) < 2:
                continue
            s_value, distance = project_point_to_lane_s(pose, lane)
            if distance < best_distance:
                best_lane = lane
                best_s = s_value
                best_distance = distance

        if best_lane is None or best_distance > self.max_lane_distance_m:
            return "unknown"

        for section in self.hd_map.sections:
            if section.lane_id == best_lane.lane_id and section_contains_s(section, best_lane, best_s):
                return section.section_id
        return "unknown"

    def build_current_marker(self, section_id: str) -> Marker:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "hd_map/current_section"
        marker.id = 0
        if self.hd_map is None or section_id == "unknown":
            marker.action = Marker.DELETE
            return marker
        marker.header.frame_id = self.hd_map.frame_id
        section = next((candidate for candidate in self.hd_map.sections if candidate.section_id == section_id), None)
        if section is None:
            marker.action = Marker.DELETE
            return marker
        lane = self.hd_map.lane_by_id(section.lane_id)
        if lane is None:
            marker.action = Marker.DELETE
            return marker
        points = section_points(lane.centerline, lane.closed_loop, section.start_s_m, section.end_s_m)
        if len(points) < 2:
            marker.action = Marker.DELETE
            return marker
        marker.header.frame_id = self.hd_map.frame_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.12
        marker.color.r = 1.0
        marker.color.g = 0.15
        marker.color.b = 0.1
        marker.color.a = 0.95
        marker.points = [to_geometry_point(point, 0.12) for point in points]
        return marker

    def publish_current_section(self, section_id: str) -> None:
        msg = String()
        msg.data = section_id
        self.section_pub.publish(msg)
        self.marker_pub.publish(self.build_current_marker(section_id))
        if section_id != self.current_section:
            self.get_logger().info(f"HD map section changed: {self.current_section} -> {section_id}")
            self.current_section = section_id

    def on_timer(self) -> None:
        if self.hd_map is None:
            return
        pose = self.lookup_pose()
        if pose is None:
            self.publish_current_section("unknown")
            return
        self.publish_current_section(self.resolve_section(pose))


def main() -> None:
    rclpy.init()
    node = HdMapSectionLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
