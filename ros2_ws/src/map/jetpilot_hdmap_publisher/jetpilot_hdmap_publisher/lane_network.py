"""Directed lane graph and stateful traversal, independent of ROS and third-party packages.

Geometry overlap never creates an edge. Only an explicit successor may be entered.
"""
import math


def xy(p):
    p = tuple(float(v) for v in p[:2])
    if len(p) != 2 or not all(math.isfinite(v) for v in p):
        raise ValueError("invalid network point")
    return p


def stations(points):
    result = [0.0]
    for a, b in zip(points, points[1:]):
        result.append(result[-1] + math.dist(a, b))
    return result


def project(points, position, low=0.0, high=math.inf):
    ss = stations(points)
    best = None
    for i, (a, b) in enumerate(zip(points, points[1:])):
        length = ss[i+1] - ss[i]
        if length < 1e-9 or ss[i+1] < low or ss[i] > high:
            continue
        t = sum((position[k]-a[k])*(b[k]-a[k]) for k in (0, 1))/length**2
        t = max(max(0., (low-ss[i])/length), min(min(1., (high-ss[i])/length), t))
        q = tuple(a[k]+t*(b[k]-a[k]) for k in (0, 1))
        hit = (math.dist(position[:2], q), ss[i]+t*length, i)
        if best is None or hit[0] < best[0]:
            best = hit
    if best is None:
        raise ValueError("no projectable lane segment")
    return best


def validate(lanes):
    by_id = {}
    for lane in lanes:
        key = lane.get("id")
        if not isinstance(key, str) or not key or key in by_id:
            raise ValueError("network lane IDs must be unique and nonempty")
        by_id[key] = lane
    for key, lane in by_id.items():
        successors = lane.get("successor_ids", [])
        if not isinstance(successors, list) or any(not isinstance(v, str) for v in successors):
            raise ValueError(f"{key}: successor_ids must be a list of lane IDs")
        if len(set(successors)) != len(successors):
            raise ValueError(f"{key}: duplicate successor")
        if lane.get("default_successor_id", "") not in ["", *successors]:
            raise ValueError(f"{key}: default successor must be a connected lane")
        if len(successors)>1 and lane.get("default_successor_id", "") not in successors:
            raise ValueError(f"{key}: デフォルト分岐を指定して保存してください")
        if not successors and not any(key in x.get("successor_ids", []) for x in lanes):
            continue
        if lane.get("closed_loop", True):
            raise ValueError(f"{key}: network lanes must be open; connect the last lane back to the first")
        points = [xy(p) for p in lane.get("centerline", [])]
        if len(points) < 2 or any(math.dist(a,b) < 1e-6 for a,b in zip(points,points[1:])):
            raise ValueError(f"{key}: centerline needs distinct consecutive points")
        for target in successors:
            if target == key or target not in by_id:
                raise ValueError(f"{key}: unknown or self successor {target}")
            other = [xy(p) for p in by_id[target].get("centerline", [])]
            if len(other) < 2 or math.dist(points[-1], other[0]) > 0.001:
                raise ValueError(f"{key} → {target}: endpoints must coincide (1 mm tolerance); create a connector")
            u = tuple(points[-1][k]-points[-2][k] for k in (0,1))
            v = tuple(other[1][k]-other[0][k] for k in (0,1))
            norm = math.hypot(*u)*math.hypot(*v)
            if norm < 1e-12 or sum(u[k]*v[k] for k in (0,1))/norm < math.cos(math.radians(30)):
                raise ValueError(f"{key} → {target}: heading changes over 30 degrees; adjust the connection curve")
    return by_id


def resample(points, step=.1):
    points = [xy(p) for p in points]
    ss = stations(points)
    if len(points) < 2 or ss[-1] <= 1e-6:
        raise ValueError("line is too short")
    count = max(3, math.ceil(ss[-1]/step))
    if count > 20000:
        raise ValueError("network lane is too long")
    result, i = [], 0
    for j in range(count+1):
        s = ss[-1]*j/count
        while i+1 < len(ss)-1 and ss[i+1] < s:
            i += 1
        t = (s-ss[i])/(ss[i+1]-ss[i])
        result.append([points[i][k]+t*(points[i+1][k]-points[i][k]) for k in (0,1)])
    return result


def smooth_candidate(lane, environment, clearance):
    """Bounded minimum-bending candidate; not a global minimum-time optimizer.

    The first/last two samples are fixed so alternatives share entry/exit tangents.
    Every accepted update preserves continuous segment clearance.
    """
    points = resample(lane["centerline"])
    for a, b in zip(points, points[1:]):
        issue = environment.issue(a, b, clearance)
        if issue:
            raise ValueError(f"{lane['id']}: centerline cannot seed raceline: {issue}")
    for _ in range(40):
        moved = 0.
        for i in range(2, len(points)-2):
            old = points[i]
            # Gradient descent of sum of squared second differences.
            q = [old[k]-.06*(6*old[k]-4*(points[i-1][k]+points[i+1][k])
                              +points[i-2][k]+points[i+2][k]) for k in (0,1)]
            if not environment.issue(points[i-1], q, clearance) and not environment.issue(q, points[i+1], clearance):
                points[i] = q
                moved = max(moved, math.dist(old,q))
        if moved < 1e-5:
            break
    return points


class Tracker:
    def __init__(self, lanes, initial_lane_id):
        self.lanes = validate(lanes)
        if initial_lane_id not in self.lanes:
            raise ValueError("initial lane is not in network")
        if self.lanes[initial_lane_id].get("closed_loop", True):
            raise ValueError("initial network lane must be open")
        self.current_lane_id = initial_lane_id
        self.station = None
        self.selected_next = ""
        self.previous_position = None
        self.choices = {}

    def set_choices(self, choices):
        if not isinstance(choices, dict):
            raise ValueError("branch choices must map source lane ID to successor lane ID")
        for source, target in choices.items():
            if source not in self.lanes or target not in self.lanes[source].get("successor_ids", []):
                raise ValueError(f"not a connected movement: {source} → {target}")
        # A committed movement stays fixed until the target is entered.
        self.choices = dict(choices)

    def successor(self, lane_id):
        lane = self.lanes[lane_id]
        options = lane.get("successor_ids", [])
        return self.choices.get(lane_id) or lane.get("default_successor_id") or (options[0] if len(options)==1 else "")

    def update(self, position, max_step=1.0, max_distance=1.0):
        position = xy(position)
        if self.previous_position is not None and math.dist(position,self.previous_position) > max_step:
            raise ValueError("localization jumped; current lane retained")
        lane = self.lanes[self.current_lane_id]
        points = [xy(p) for p in lane["centerline"]]
        length = stations(points)[-1]
        low = max(0., self.station-max_step) if self.station is not None else 0.
        high = self.station+max_step if self.station is not None else math.inf
        distance, s, _ = project(points,position,low,high)
        if distance > max_distance:
            raise ValueError("outside current lane tracking range; no nearest-lane fallback")
        self.station = s
        self.previous_position = position
        if not self.selected_next and length-s <= 1.0:
            self.selected_next = self.successor(self.current_lane_id)
        # Only the directed terminal gate permits a transition, never overlap.
        a,b = points[-2:]
        direction = tuple((b[k]-a[k])/math.dist(a,b) for k in (0,1))
        passed = sum((position[k]-b[k])*direction[k] for k in (0,1)) >= 0.
        if self.selected_next and s >= length-.05 and passed:
            target = self.lanes[self.selected_next]
            target_points = [xy(p) for p in target["centerline"]]
            distance, target_s, _ = project(target_points,position,0.,max_step)
            if distance > max_distance:
                raise ValueError("selected successor is outside tracking range")
            self.current_lane_id = self.selected_next
            self.station = target_s
            self.selected_next = ""
        return self.current_lane_id

    def path(self, mode="centerline", horizon=12.0):
        lane_id = self.current_lane_id
        result, visited, remaining = [], set(), horizon
        while lane_id and lane_id not in visited and remaining > 0:
            visited.add(lane_id)
            lane = self.lanes[lane_id]
            raw = lane.get("network_raceline", []) if mode == "raceline" else lane["centerline"]
            if len(raw) < 2:
                raise ValueError(f"{lane_id}: requested generated line is missing")
            points = [xy(p) for p in resample(raw)]
            if lane_id == self.current_lane_id and self.previous_position is not None:
                # Station identity comes from the current lane, not other overlapping lanes.
                center_length = stations(lane["centerline"])[-1]
                mapped = (self.station or 0.)/center_length*stations(points)[-1]
                _, _, i = project(points,self.previous_position,max(0.,mapped-1.),mapped+1.)
                points = points[max(0,i-1):]
            for p in points:
                if result and math.dist(result[-1],p) < 1e-6:
                    continue
                if result:
                    remaining -= math.dist(result[-1],p)
                result.append(p)
                if remaining <= 0:
                    break
            lane_id = (self.selected_next or self.successor(lane_id)) if lane_id == self.current_lane_id else self.successor(lane_id)
        if self.previous_position is not None and len(result)>2:
            start_distance = math.dist(self.previous_position,result[0])
            travelled = 0.
            for i in range(1,len(result)):
                travelled += math.dist(result[i-1],result[i])
                if travelled > 1. and math.dist(self.previous_position,result[i]) < max(.5,start_distance+.2):
                    result = result[:i]
                    break
        return result


def source_hash(lanes, obstacles):
    import hashlib
    import json
    number = lambda v: float(format(float(v), '.9g'))
    points = lambda rows: [[number(p[0]),number(p[1])] for p in rows]
    normalized = []
    for lane in lanes:
        item = {"id":lane["id"], "closed_loop":bool(lane.get("closed_loop", True)),
                "successor_ids":lane.get("successor_ids", []),
                "default_successor_id":lane.get("default_successor_id", "")}
        for key in ("left_bound","right_bound","centerline","drivable_left_bound","drivable_right_bound"):
            item[key] = points(lane.get(key,lane.get(key.replace("drivable_", ""),[])))
        normalized.append(item)
    obs = [{"id":o["id"],"polygon":points(o["polygon"]),"margin_m":number(o.get("margin_m",0.))} for o in obstacles]
    return hashlib.sha256(json.dumps([normalized,obs], sort_keys=True, separators=(",", ":")).encode()).hexdigest()



def generation_environment(lane, geometry):
    bounds = []
    for key in ("left_bound", "right_bound"):
        points = [xy(p) for p in lane[key]]
        if len(points) < 2:
            raise ValueError("generation bounds need at least two points")
        start = [points[0][k]+.01*(points[0][k]-points[1][k]) for k in (0,1)]
        end = [points[-1][k]+.01*(points[-1][k]-points[-2][k]) for k in (0,1)]
        bounds.append([start,*points,end])
    return geometry.Environment([(bounds[0],bounds[1],False)], [])
