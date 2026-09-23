"""Pure-Python section geometry, causal labels and portable multihead contract."""
from __future__ import annotations

import bisect
import hashlib
import json
import math
import re


def map_digest(document):
    return hashlib.sha256(json.dumps(document, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def validate_heads(heads, sections):
    if not isinstance(heads, list) or not heads:
        raise ValueError("少なくとも1つのheadが必要です")
    names, assigned = set(), set()
    generic = 0
    for head in heads:
        name = head.get("name", "")
        if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]{0,63}", name) or name in names:
            raise ValueError("head名は重複しない英数字・underscoreにしてください")
        names.add(name)
        throttle = float(head["throttle"])
        if not math.isfinite(throttle) or not 0 <= throttle <= 1:
            raise ValueError("throttleは0〜1の有限値が必要です")
        ids = head.get("sections", [])
        if not isinstance(ids, list) or len(ids) != len(set(ids)) or not set(ids) <= set(sections):
            raise ValueError("地図に存在しない、または重複したsectionです")
        if head.get("generic", False):
            generic += 1
        else:
            if not ids or assigned.intersection(ids):
                raise ValueError("専用headには重複しないsectionを割り当ててください")
            assigned.update(ids)
    if generic != 1:
        raise ValueError("汎用headは必ず1つ指定してください")
    if len(heads) < 2:
        raise ValueError("汎用headに加え、少なくとも1つの専用headが必要です")
    return heads


class SectionMap:
    def __init__(self, document, max_distance=1.0):
        self.document = document
        self.frame = str(document.get("frame_id", "map"))
        self.max_distance = max_distance
        self.sections = document.get("sections", [])
        self.ids = [str(s["id"]) for s in self.sections]
        if not self.ids or len(set(self.ids)) != len(self.ids):
            raise ValueError("地図に一意なsection定義が必要です")
        self.primary = document.get("primary_lane_id") or document.get("lanes", [{}])[0].get("id", "")
        self.lanes = []
        for lane in document.get("lanes", []):
            points = lane.get("centerline", [])
            points = [(float(p["x"]), float(p["y"])) if isinstance(p, dict) else (float(p[0]), float(p[1])) for p in points]
            if len(points) < 2:
                continue
            closed = lane.get("closed_loop", True)
            segments, station = [], 0.0
            for a, b in zip(points, points[1:] + ([points[0]] if closed else [])):
                length = math.hypot(b[0]-a[0], b[1]-a[1])
                if length > 1e-9:
                    segments.append((a, b, length, station))
                    station += length
            self.lanes.append((lane["id"], closed, segments, station, bool(lane.get("successor_ids"))))
        if not self.lanes:
            raise ValueError("地図にcenterlineが必要です")

    def resolve(self, x, y, lane_id=""):
        if not math.isfinite(x) or not math.isfinite(y):
            return "unknown"
        # Like the online localizer, lane networks require an explicit current lane.
        if any(lane[4] for lane in self.lanes) and not lane_id:
            return "unknown"
        best = None
        section_lanes = {s.get("lane_id", self.primary) for s in self.sections}
        for name, closed, segments, total, _ in self.lanes:
            if name not in section_lanes or (lane_id and name != lane_id):
                continue
            for a, b, length, start in segments:
                u = max(0., min(1., ((x-a[0])*(b[0]-a[0])+(y-a[1])*(b[1]-a[1])) / length**2))
                distance = math.hypot(x-a[0]-u*(b[0]-a[0]), y-a[1]-u*(b[1]-a[1]))
                if best is None or distance < best[0]:
                    best = (distance, name, start+u*length, total, closed)
        if best is None or best[0] > self.max_distance:
            return "unknown"
        _, name, station, total, closed = best
        matches = []
        for section in self.sections:
            if section.get("lane_id", self.primary) != name:
                continue
            start, end = float(section["start_s_m"]), float(section["end_s_m"])
            same = bool(section.get("start_gate_id")) and section.get("start_gate_id") == section.get("end_gate_id")
            if closed:
                full = end-start >= total-1e-9 or (same and abs(end-start) <= 1e-9)
                start, end, s = start % total, end % total, station % total
                inside = full or ((s >= start or s < end) if start > end else start <= s < end)
            else:
                inside = start <= station < end or (start < end and abs(end-total)<1e-9 and abs(station-total)<1e-9)
            if inside:
                matches.append(section["id"])
        return matches[0] if len(matches) == 1 else "unknown"


class CausalLabels:
    def __init__(self, records, max_age_sec):
        self.records = sorted(records, key=lambda item: item[0])
        self.times = [r[0] for r in self.records]
        self.max_age_ns = int(max_age_sec * 1e9)

    def at(self, stamp):
        index = bisect.bisect_right(self.times, stamp)-1
        if index < 0 or stamp-self.times[index] > self.max_age_ns:
            return "unknown"
        return self.records[index][1]


def split_sequences(rows, fraction=0.2):
    """Hold out complete bags, or a separated temporal tail for a single bag."""
    groups = sorted({r["sequence_id"] for r in rows})
    if len(groups) > 1:
        n = max(1, min(len(groups)-1, math.ceil(len(groups)*fraction)))
        held = set(groups[-n:])
        return ([i for i,r in enumerate(rows) if r["sequence_id"] not in held],
                [i for i,r in enumerate(rows) if r["sequence_id"] in held], "bag")
    ordered = sorted(range(len(rows)), key=lambda i: int(rows[i]["stamp"]))
    cut = int(len(rows)*(1-fraction))
    if not 0 < cut < len(rows):
        raise ValueError("学習・検証に必要なサンプルが不足しています")
    boundary = int(rows[ordered[cut]]["stamp"])
    train = [i for i in ordered[:cut] if int(rows[i]["stamp"]) < boundary-1_000_000_000]
    if not train:
        raise ValueError("時間分割には1秒の分離を含む十分なデータが必要です。別bagを追加してください")
    return train, ordered[cut:], "temporal_with_1s_gap"
