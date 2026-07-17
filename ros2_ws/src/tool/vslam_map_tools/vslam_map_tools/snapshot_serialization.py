from __future__ import annotations

import copy
import math
from typing import Any


def pose_to_dict(pose: Any) -> dict[str, object]:
    return {
        "position": {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
        },
        "orientation": {
            "x": float(pose.orientation.x),
            "y": float(pose.orientation.y),
            "z": float(pose.orientation.z),
            "w": float(pose.orientation.w),
        },
    }


def twist_to_dict(twist: Any) -> dict[str, object]:
    return {
        "linear": {
            "x": float(twist.linear.x),
            "y": float(twist.linear.y),
            "z": float(twist.linear.z),
        },
        "angular": {
            "x": float(twist.angular.x),
            "y": float(twist.angular.y),
            "z": float(twist.angular.z),
        },
    }


def stamp_to_nanoseconds_string(stamp: Any) -> str:
    """Serialize a ROS timestamp without losing nanosecond precision in JSON clients."""

    seconds = int(getattr(stamp, "sec", 0))
    nanoseconds = int(getattr(stamp, "nanosec", 0))
    return str(seconds * 1_000_000_000 + nanoseconds)


def odometry_to_sample(
    msg: Any, *, received_timestamp_ns: int | None = None
) -> dict[str, object]:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    sample = {
        "timestamp_ns": stamp_to_nanoseconds_string(stamp),
        "frame_id": str(getattr(header, "frame_id", "")),
        "child_frame_id": str(getattr(msg, "child_frame_id", "")),
        "pose": pose_to_dict(msg.pose.pose),
        "twist": twist_to_dict(msg.twist.twist),
    }
    if received_timestamp_ns is not None:
        sample["received_timestamp_ns"] = str(int(received_timestamp_ns))
    return sample


def transform_to_dict(transform: Any) -> dict[str, object]:
    """Serialize a geometry_msgs/Transform-like value."""

    return {
        "translation": {
            "x": float(transform.translation.x),
            "y": float(transform.translation.y),
            "z": float(transform.translation.z),
        },
        "rotation": {
            "x": float(transform.rotation.x),
            "y": float(transform.rotation.y),
            "z": float(transform.rotation.z),
            "w": float(transform.rotation.w),
        },
    }


def _normalised_quaternion(value: dict[str, object]) -> tuple[float, float, float, float]:
    quaternion = (
        float(value.get("x", 0.0)),
        float(value.get("y", 0.0)),
        float(value.get("z", 0.0)),
        float(value.get("w", 1.0)),
    )
    norm = math.sqrt(sum(component * component for component in quaternion))
    if norm <= 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(component / norm for component in quaternion)


def _multiply_quaternions(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _rotate_vector(
    quaternion: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    qx, qy, qz, qw = quaternion
    vx, vy, vz = vector
    # Equivalent to q * [v, 0] * conjugate(q), expanded to avoid dependencies.
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def invert_transform(transform: dict[str, object]) -> dict[str, object]:
    """Invert a serialized rigid transform."""

    translation = transform.get("translation") or {}
    rotation = _normalised_quaternion(transform.get("rotation") or {})
    inverse_rotation = (-rotation[0], -rotation[1], -rotation[2], rotation[3])
    inverse_translation = _rotate_vector(
        inverse_rotation,
        (
            -float(translation.get("x", 0.0)),
            -float(translation.get("y", 0.0)),
            -float(translation.get("z", 0.0)),
        ),
    )
    return {
        "translation": dict(zip(("x", "y", "z"), inverse_translation)),
        "rotation": dict(zip(("x", "y", "z", "w"), inverse_rotation)),
    }


def transform_odometry_sample(
    sample: dict[str, object],
    transform: dict[str, object],
    *,
    parent_frame: str,
) -> dict[str, object]:
    """Apply parent<-odometry-frame to an odometry sample pose.

    Odometry twist remains unchanged: its linear magnitude is frame-invariant and is
    the value used by the analysis viewer. The source frame is retained for auditing.
    """

    result = copy.deepcopy(sample)
    pose = result.get("pose") or {}
    position = pose.get("position") or {}
    orientation = pose.get("orientation") or {}
    translation = transform.get("translation") or {}
    transform_rotation = _normalised_quaternion(transform.get("rotation") or {})
    rotated_position = _rotate_vector(
        transform_rotation,
        (
            float(position.get("x", 0.0)),
            float(position.get("y", 0.0)),
            float(position.get("z", 0.0)),
        ),
    )
    transformed_orientation = _multiply_quaternions(
        transform_rotation,
        _normalised_quaternion(orientation),
    )
    result["pose"] = {
        "position": {
            "x": rotated_position[0] + float(translation.get("x", 0.0)),
            "y": rotated_position[1] + float(translation.get("y", 0.0)),
            "z": rotated_position[2] + float(translation.get("z", 0.0)),
        },
        "orientation": dict(
            zip(("x", "y", "z", "w"), _normalised_quaternion(dict(zip(
                ("x", "y", "z", "w"), transformed_orientation
            ))))
        ),
    }
    result["source_frame_id"] = str(sample.get("frame_id") or "")
    result["frame_id"] = parent_frame
    return result


def legacy_full_vslam_path(
    samples: list[dict[str, object]],
) -> dict[str, object] | None:
    """Build the pre-existing pose-only path schema from timed samples."""

    if not samples:
        return None
    return {
        "frame_id": samples[-1]["frame_id"],
        "poses": [sample["pose"] for sample in samples],
    }
