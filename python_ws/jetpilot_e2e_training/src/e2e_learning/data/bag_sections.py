"""Assign existing bag section topics, map-frame poses, or TF to sample stamps."""
from __future__ import annotations

import bisect
import json
from collections import defaultdict
from pathlib import Path

from .sections import CausalLabels, SectionMap, map_digest


def stamp_ns(msg, fallback):
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    value = int(stamp.sec)*1_000_000_000+int(stamp.nanosec) if stamp is not None else 0
    return value or int(fallback)


def rotate(q, v):
    x, y, z, w = q
    tx, ty, tz = 2*(y*v[2]-z*v[1]), 2*(z*v[0]-x*v[2]), 2*(x*v[1]-y*v[0])
    return (v[0]+w*tx+y*tz-z*ty, v[1]+w*ty+z*tx-x*tz, v[2]+w*tz+x*ty-y*tx)


class TransformTimeline:
    def __init__(self, max_age_ns):
        self.edges = defaultdict(list)
        self.static = {}
        self.max_age_ns = max_age_ns
        self.times = {}

    def add(self, transform, stamp, static=False):
        key = (transform.header.frame_id.lstrip("/"), transform.child_frame_id.lstrip("/"))
        t, r = transform.transform.translation, transform.transform.rotation
        q = (float(r.x), float(r.y), float(r.z), float(r.w))
        norm = sum(v*v for v in q)**0.5
        if norm < 1e-9:
            return
        value = ((float(t.x), float(t.y), float(t.z)), tuple(v/norm for v in q))
        if static:
            self.static[key] = value
        else:
            self.edges[key].append((stamp, value))

    def finish(self):
        for key, entries in self.edges.items():
            entries.sort(key=lambda x: x[0])
            self.times[key] = [x[0] for x in entries]

    def position(self, stamp, target, source):
        graph = defaultdict(list)
        transforms = dict(self.static)
        for key, entries in self.edges.items():
            i = bisect.bisect_right(self.times[key], stamp)-1
            if i >= 0 and stamp-entries[i][0] <= self.max_age_ns:
                transforms[key] = entries[i][1]
        for (parent, child), (t, q) in transforms.items():
            graph[child].append((parent, t, q))
            inverse = (-q[0], -q[1], -q[2], q[3])
            graph[parent].append((child, rotate(inverse, tuple(-v for v in t)), inverse))
        queue, seen = [(source.lstrip("/"), (0., 0., 0.))], set()
        for frame, point in queue:
            if frame == target.lstrip("/"):
                return point
            if frame in seen:
                continue
            seen.add(frame)
            for next_frame, t, q in graph[frame]:
                moved = rotate(q, point)
                queue.append((next_frame, tuple(moved[i]+t[i] for i in range(3))))
        return None


def recorded_labels(bag, rows, request):
    from rosbags.highlevel import AnyReader
    geometry = SectionMap(request["map_document"], request.get("max_lane_distance_m", 1.0))
    section_topic = request.get("section_topic", "/localization/current_section")
    pose_topic = request.get("pose_topic", "/visual_slam/tracking/odometry")
    lane_topic = "/planning/current_lane"
    records, poses, lanes, health = [], [], [], []
    stamped_topic = "/localization/section_state"
    max_age = request.get("max_label_age_sec", 0.3)
    tf = TransformTimeline(int(max_age*1e9))
    source = request.get("label_source", "auto")
    clock = request.get("timestamp_source", "header")
    with AnyReader([Path(bag)]) as reader:
        topics = {section_topic, stamped_topic, pose_topic, lane_topic, "/tf", "/tf_static", "/localization/pose_hint_state"}
        connections = [c for c in reader.connections if c.topic in topics]
        use_stamped = section_topic in {"/localization/current_section", stamped_topic} and any(c.topic == stamped_topic for c in connections)
        if use_stamped:
            section_topic = stamped_topic
        has_sections = any(c.topic == section_topic for c in connections)
        use_sections = source == "section" or (source == "auto" and has_sections)
        for connection, recorded_at, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, connection.msgtype)
            stamp = stamp_ns(msg, recorded_at) if clock == "header" else int(recorded_at)
            if connection.topic == "/localization/pose_hint_state":
                try:
                    healthy = json.loads(msg.data).get("state") == "localized"
                except (ValueError, AttributeError):
                    healthy = False
                health.append((stamp, "localized" if healthy else "unknown"))
            elif connection.topic == section_topic and use_sections:
                if hasattr(msg, "section_id"):
                    if msg.map_sha256 != map_digest(request["map_document"]):
                        raise ValueError("bag内のsection地図と選択した地図が異なります。位置・TF方式で再割り当てしてください")
                    records.append((stamp, str(msg.section_id) if msg.valid else "unknown"))
                else:
                    records.append((stamp, str(msg.data)))
            elif connection.topic == lane_topic:
                lanes.append((stamp, str(msg.data)))
            elif connection.topic in {"/tf", "/tf_static"}:
                for transform in msg.transforms:
                    t = stamp_ns(transform, recorded_at) if clock == "header" else int(recorded_at)
                    tf.add(transform, t, connection.topic == "/tf_static")
            elif connection.topic == pose_topic:
                if str(msg.header.frame_id).lstrip("/") != geometry.frame.lstrip("/"):
                    continue  # Never treat odom-frame coordinates as map coordinates.
                if getattr(msg, "child_frame_id", request.get("base_frame", "base_link")) != request.get("base_frame", "base_link"):
                    continue
                pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
                poses.append((stamp, (float(pose.position.x), float(pose.position.y))))
    health_labels = CausalLabels(health, 2.0)
    def valid(stamp):
        return not health or health_labels.at(stamp) == "localized"
    if use_sections:
        if not records:
            raise ValueError("指定したsection topicにメッセージがありません")
        labels = CausalLabels(records, max_age)
        return {int(r["stamp"]): labels.at(int(r["stamp"])) if valid(int(r["stamp"])) else "unknown" for r in rows}
    tf.finish()
    pose_labels = CausalLabels(poses, max_age)
    lane_labels = CausalLabels(lanes, max_age)
    output = {}
    for row in rows:
        stamp = int(row["stamp"])
        if not valid(stamp):
            output[stamp] = "unknown"
            continue
        point = pose_labels.at(stamp)
        if point == "unknown":
            point = tf.position(stamp, geometry.frame, request.get("base_frame", "base_link"))
        lane = lane_labels.at(stamp)
        output[stamp] = geometry.resolve(point[0], point[1], "" if lane == "unknown" else lane) if point is not None and point != "unknown" else "unknown"
    return output
