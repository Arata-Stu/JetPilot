#!/usr/bin/env python3
from __future__ import annotations

import argparse
import base64
import json
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict

from vslam_map_tools.snapshot_serialization import (
    invert_transform,
    legacy_full_vslam_path,
    odometry_to_sample,
    pose_to_dict,
    transform_odometry_sample,
    transform_to_dict,
)


def parse_boolean(value: str) -> bool:
    normalized = str(value).strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"invalid boolean value: {value}")


def normalized_frame(value: str) -> str:
    return str(value or "").strip().lstrip("/")


class VslamReferenceSnapshotRecorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("vslam_reference_snapshot_recorder")
        self.args = args
        if args.max_odometry_samples < 2:
            raise ValueError("--max-odometry-samples must be at least 2")
        if args.min_localized_odometry_samples < 2:
            raise ValueError("--min-localized-odometry-samples must be at least 2")
        self.output_path = Path(args.output).expanduser().resolve()
        self.latest_path: PathMsg | None = None
        self.latest_odom: Odometry | None = None
        self.latest_landmarks: PointCloud2 | None = None
        self.latest_trajectory: MarkerArray | None = None
        self.vslam_odom_history: list[dict[str, object]] = []
        self.latest_accepted_odom: dict[str, object] | None = None
        self.require_localized_map = bool(args.require_localized_map)
        self.max_odometry_samples = min(args.max_odometry_samples, 50000) if self.require_localized_map else args.max_odometry_samples
        self.map_frame = normalized_frame(args.map_frame)
        self.localization_state = "disabled" if not self.require_localized_map else "waiting"
        self.localization_confirmed = False
        self.localization_confirmed_seen = False
        self.map_from_frame: dict[str, dict[str, object]] = {}
        self.raw_odom_count = 0
        self.rejected_unlocalized_count = 0
        self.rejected_missing_tf_count = 0
        self.accepted_odom_count = 0
        self.odom_history_stride = 1
        self.path_seen = False
        self.odom_seen = False
        self.landmarks_seen = False
        self.trajectory_seen = False
        self.dirty = False
        self.path_count = 0
        self.odom_count = 0
        self.landmarks_count = 0
        self.trajectory_count = 0
        self.snapshot_write_count = 0

        # VSLAM topics may be published as BEST_EFFORT, so request a permissive QoS.
        best_effort_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.create_subscription(PathMsg, args.path_topic, self.on_path, best_effort_qos)
        if getattr(args, 'odom_topic', None):
            self.create_subscription(Odometry, args.odom_topic, self.on_odom, best_effort_qos)
        if getattr(args, 'landmarks_topic', None):
            self.create_subscription(PointCloud2, args.landmarks_topic, self.on_landmarks, best_effort_qos)
        if getattr(args, 'trajectory_topic', None):
            self.create_subscription(MarkerArray, args.trajectory_topic, self.on_trajectory, best_effort_qos)
        if self.require_localized_map:
            status_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                String,
                args.localization_state_topic,
                self.on_localization_state,
                status_qos,
            )
            self.create_subscription(TFMessage, args.tf_topic, self.on_tf, best_effort_qos)
        if args.write_interval_sec > 0.0:
            self.create_timer(args.write_interval_sec, self.flush_if_dirty)
        if args.status_interval_sec > 0.0:
            self.create_timer(args.status_interval_sec, self.log_status)
        self.get_logger().info(
            f"Recording VSLAM reference snapshot: path={args.path_topic}, odom={args.odom_topic}, landmarks={getattr(args, 'landmarks_topic', None)}, trajectory={getattr(args, 'trajectory_topic', None)}, output={self.output_path}, require_localized_map={self.require_localized_map}"
        )

    @staticmethod
    def stamp_text(msg) -> str:
        stamp = getattr(getattr(msg, "header", None), "stamp", None)
        if stamp is None:
            return "stamp=<none>"
        return f"stamp={stamp.sec}.{stamp.nanosec:09d}"

    def on_path(self, msg: PathMsg) -> None:
        self.latest_path = msg
        self.path_count += 1
        if not self.path_seen:
            self.path_seen = True
            self.get_logger().info(
                f"Received first path on {self.args.path_topic}: frame={msg.header.frame_id}, {self.stamp_text(msg)}, poses={len(msg.poses)}."
            )
        self.dirty = True

    def on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.odom_count += 1
        self.raw_odom_count += 1
        sample = odometry_to_sample(
            msg,
            received_timestamp_ns=self.get_clock().now().nanoseconds,
        )
        if self.require_localized_map:
            if not self.localization_confirmed:
                self.rejected_unlocalized_count += 1
                return
            source_frame = normalized_frame(msg.header.frame_id)
            if source_frame != self.map_frame:
                map_from_source = self.map_from_frame.get(source_frame)
                if map_from_source is None:
                    self.rejected_missing_tf_count += 1
                    return
                sample = transform_odometry_sample(
                    sample,
                    map_from_source,
                    parent_frame=self.map_frame,
                )
            else:
                sample["frame_id"] = self.map_frame

        self.accepted_odom_count += 1
        self.latest_accepted_odom = sample
        if self.accepted_odom_count % self.odom_history_stride == 0:
            self.vslam_odom_history.append(sample)
        if len(self.vslam_odom_history) >= self.max_odometry_samples:
            self.vslam_odom_history = self.vslam_odom_history[::2]
            self.odom_history_stride *= 2

        if not self.odom_seen:
            self.odom_seen = True
            self.get_logger().info(
                f"Received first odometry on {self.args.odom_topic}: frame={msg.header.frame_id}, child={msg.child_frame_id}, {self.stamp_text(msg)}."
            )
        self.dirty = True

    def on_localization_state(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            state = str(payload.get("state") or "unknown") if isinstance(payload, dict) else "unknown"
        except (TypeError, ValueError, json.JSONDecodeError):
            state = "invalid"
        changed = state != self.localization_state
        self.localization_state = state
        self.localization_confirmed = state == "localized"
        self.localization_confirmed_seen = (
            self.localization_confirmed_seen or self.localization_confirmed
        )
        if changed:
            self.get_logger().info(
                f"Localization state is now {state}; accepting_map_odometry={self.localization_confirmed}."
            )

    def on_tf(self, msg: TFMessage) -> None:
        for stamped in msg.transforms:
            parent = normalized_frame(stamped.header.frame_id)
            child = normalized_frame(stamped.child_frame_id)
            if not parent or not child:
                continue
            transform = transform_to_dict(stamped.transform)
            if parent == self.map_frame:
                self.map_from_frame[child] = transform
            elif child == self.map_frame:
                self.map_from_frame[parent] = invert_transform(transform)

    def on_landmarks(self, msg: PointCloud2) -> None:
        self.latest_landmarks = msg
        self.landmarks_count += 1
        if not self.landmarks_seen:
            self.landmarks_seen = True
            self.get_logger().info(
                f"Received first landmarks on {self.args.landmarks_topic}: frame={msg.header.frame_id}, {self.stamp_text(msg)}, width={msg.width}, height={msg.height}, point_step={msg.point_step}."
            )
        self.dirty = True

    def on_trajectory(self, msg: MarkerArray) -> None:
        self.latest_trajectory = msg
        self.trajectory_count += 1
        if not self.trajectory_seen:
            self.trajectory_seen = True
            self.get_logger().info(
                f"Received first trajectory on {self.args.trajectory_topic}: markers={len(msg.markers)}."
            )
        self.dirty = True

    def log_status(self) -> None:
        path_publishers = self.count_publishers(self.args.path_topic)
        odom_publishers = self.count_publishers(self.args.odom_topic) if self.args.odom_topic else 0
        landmarks_publishers = (
            self.count_publishers(self.args.landmarks_topic)
            if self.args.landmarks_topic else 0
        )
        trajectory_publishers = (
            self.count_publishers(self.args.trajectory_topic)
            if self.args.trajectory_topic else 0
        )
        self.get_logger().info(
            "status: "
            f"messages path={self.path_count}, odom={self.odom_count}, landmarks={self.landmarks_count}, trajectory={self.trajectory_count}; "
            f"publishers path={path_publishers}, odom={odom_publishers}, landmarks={landmarks_publishers}, trajectory={trajectory_publishers}; "
            f"snapshot_writes={self.snapshot_write_count}"
        )

    def flush_if_dirty(self) -> None:
        if not self.dirty:
            return
        self.write_snapshot()
        self.dirty = False

    def snapshot_data(self) -> dict[str, object]:
        data: dict[str, object] = {
            "path": None,
            "odometry": None,
            "odometry_samples": [],
            "landmarks": None,
            "trajectory": None,
            "full_vslam_path": None,
            "localization": {
                "required": self.require_localized_map,
                "confirmed": self.localization_confirmed,
                "confirmed_once": self.localization_confirmed_seen,
                "last_state": self.localization_state,
                "map_frame": self.map_frame,
                "map_transform_seen": bool(self.map_from_frame),
                "raw_odometry_samples": self.raw_odom_count,
                "accepted_odometry_samples": self.accepted_odom_count,
                "minimum_required_odometry_samples":
                    self.args.min_localized_odometry_samples,
                "rejected_before_localized": self.rejected_unlocalized_count,
                "rejected_without_map_transform": self.rejected_missing_tf_count,
                "history_stride": self.odom_history_stride,
                "maximum_stored_odometry_samples": self.max_odometry_samples,
            },
        }

        if self.latest_path is not None and not self.require_localized_map:
            data["path"] = {
                "frame_id": self.latest_path.header.frame_id,
                "poses": [
                    pose_to_dict(pose_stamped.pose)
                    for pose_stamped in self.latest_path.poses
                ],
            }

        if self.latest_odom is not None:
            data["odometry"] = (
                self.latest_accepted_odom
                if self.latest_accepted_odom is not None
                else odometry_to_sample(self.latest_odom)
            )

        if self.latest_landmarks is not None:
            msg = self.latest_landmarks
            data["landmarks"] = {
                "header": {
                    "frame_id": msg.header.frame_id,
                },
                "height": msg.height,
                "width": msg.width,
                "fields": [
                    {"name": f.name, "offset": f.offset, "datatype": f.datatype, "count": f.count}
                    for f in msg.fields
                ],
                "is_bigendian": msg.is_bigendian,
                "point_step": msg.point_step,
                "row_step": msg.row_step,
                "data": base64.b64encode(msg.data).decode("ascii"),
                "is_dense": msg.is_dense
            }

        if self.vslam_odom_history:
            odometry_samples = list(self.vslam_odom_history)
            if (
                self.latest_accepted_odom is not None
                and odometry_samples[-1].get("received_timestamp_ns")
                != self.latest_accepted_odom.get("received_timestamp_ns")
            ):
                odometry_samples.append(self.latest_accepted_odom)
            data["odometry_samples"] = odometry_samples
            data["full_vslam_path"] = legacy_full_vslam_path(odometry_samples)

        if self.latest_trajectory is not None:
            data["trajectory"] = message_to_ordereddict(self.latest_trajectory)

        return data

    def write_snapshot(self) -> bool:
        if self.require_localized_map and (
            not self.localization_confirmed
            or self.accepted_odom_count < self.args.min_localized_odometry_samples
            or not self.vslam_odom_history
        ):
            return False
        if self.latest_path is None and self.latest_odom is None:
            return False

        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = self.output_path.with_suffix(self.output_path.suffix + ".tmp")
        json_options = (
            {"ensure_ascii": True, "separators": (",", ":")}
            if self.require_localized_map
            else {"ensure_ascii": True, "indent": 2}
        )
        tmp_path.write_text(
            json.dumps(self.snapshot_data(), **json_options) + "\n",
            encoding="utf-8",
        )
        tmp_path.replace(self.output_path)
        self.snapshot_write_count += 1
        return True

    def summary_text(self) -> str:
        return (
            f"messages path={self.path_count}, odom={self.odom_count}, landmarks={self.landmarks_count}, trajectory={self.trajectory_count}; "
            f"accepted_map_odom={self.accepted_odom_count}, rejected_unlocalized={self.rejected_unlocalized_count}, rejected_missing_tf={self.rejected_missing_tf_count}; "
            f"localization_state={self.localization_state}, snapshot_writes={self.snapshot_write_count}; output={self.output_path}"
        )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Record the latest VSLAM path/odometry snapshot to disk.")
    parser.add_argument("--path-topic", default="/visual_slam/tracking/slam_path")
    parser.add_argument("--odom-topic", default="/visual_slam/tracking/odometry")
    parser.add_argument("--landmarks-topic", default="")
    parser.add_argument("--trajectory-topic", default="")
    parser.add_argument("--localization-state-topic", default="/localization/pose_hint_state")
    parser.add_argument("--tf-topic", default="/tf")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument(
        "--require-localized-map",
        type=parse_boolean,
        default=False,
        help="Only retain odometry after localization is confirmed and transform it into map frame.",
    )
    parser.add_argument("--max-odometry-samples", type=int, default=200000)
    parser.add_argument("--min-localized-odometry-samples", type=int, default=10)
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--write-interval-sec",
        type=float,
        default=1.0,
        help="Periodic snapshot interval. Set to 0 to write only during shutdown.",
    )
    parser.add_argument("--status-interval-sec", type=float, default=5.0)
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args(args=rclpy.utilities.remove_ros_args()[1:])

    rclpy.init()
    node = VslamReferenceSnapshotRecorder(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        snapshot_written = node.write_snapshot()
        if snapshot_written:
            print(f"[vslam_reference_snapshot_recorder]: Saved VSLAM reference snapshot to {node.output_path}", flush=True)
        else:
            print("[vslam_reference_snapshot_recorder]: Warning: No VSLAM path/odometry messages were received; snapshot file was not written.", flush=True)
        print(f"[vslam_reference_snapshot_recorder]: Summary: {node.summary_text()}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
