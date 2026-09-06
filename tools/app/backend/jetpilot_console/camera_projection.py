"""CameraInfo/TF extraction for frame-accurate HD map overlays (stdlib only)."""
from __future__ import annotations

from bisect import bisect_right
from collections import deque
import hashlib
import json
import math
import statistics
from pathlib import Path

MAX_TF_AGE_NS = 150_000_000
IDENTITY = [1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.]


def localization_fingerprint(root: Path) -> str:
    """HD edits do not invalidate camera poses; a different localization map does."""
    digest = hashlib.sha256()
    found = False
    for name in ('cuvgl_map', 'cuvslam_map', 'vslam_reference_snapshot.json', 'vslam_landmarks.yaml'):
        path = root / name
        files = sorted(p for p in path.rglob('*') if p.is_file()) if path.is_dir() else [path] if path.is_file() else []
        for item in files:
            found = True
            stat = item.stat()
            digest.update(f'{item.relative_to(root)}:{stat.st_size}:{stat.st_mtime_ns}\n'.encode())
    return digest.hexdigest() if found else ''


def frame_name(value):
    return str(value or '').strip().lstrip('/')


def stamp_ns(message):
    stamp = getattr(getattr(message, 'header', None), 'stamp', None)
    value = int(getattr(stamp, 'sec', 0)) * 1_000_000_000 + int(getattr(stamp, 'nanosec', 0))
    return value if value > 0 else 0


def quaternion(values):
    q = [float(v) for v in values]
    length = math.sqrt(sum(v*v for v in q))
    if len(q) != 4 or not all(math.isfinite(v) for v in q) or length < 1e-9:
        raise ValueError('invalid orientation')
    return [v / length for v in q]


def matrix(position, rotation):
    x, y, z, w = quaternion(rotation)
    p = [float(v) for v in position]
    if len(p) != 3 or not all(math.isfinite(v) for v in p):
        raise ValueError('invalid position')
    return [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w), p[0],
            2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w), p[1],
            2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y), p[2],
            0., 0., 0., 1.]


def multiply(a, b):
    return [sum(a[row*4+k]*b[k*4+col] for k in range(4)) for row in range(4) for col in range(4)]


def inverse(a):
    out = IDENTITY[:]
    for row in range(3):
        for col in range(3):
            out[row*4+col] = a[col*4+row]
        out[row*4+3] = -sum(out[row*4+k]*a[k*4+3] for k in range(3))
    return out


class TransformBuffer:
    def __init__(self):
        self.static = {}
        self.dynamic = {}
        self.times = {}
        self.adjacent = {}

    def add(self, parent, child, timestamp, position, rotation, static=False):
        parent, child = frame_name(parent), frame_name(child)
        if not parent or not child or parent == child:
            return
        try:
            q = quaternion(rotation)
            p = [float(v) for v in position]
            value = matrix(p, q)
        except (TypeError, ValueError, OverflowError):
            return
        key = (parent, child)
        if static:
            self.static[key] = value
        elif timestamp > 0:
            self.dynamic.setdefault(key, []).append((int(timestamp), p, q))

    def prepare(self):
        for key, records in self.dynamic.items():
            records.sort(key=lambda record: record[0])
            self.times[key] = [record[0] for record in records]
        self.adjacent = {}
        for parent, child in self.static.keys() | self.dynamic.keys():
            self.adjacent.setdefault(child, []).append((parent, (parent, child), False))
            self.adjacent.setdefault(parent, []).append((child, (parent, child), True))

    def at(self, key, timestamp):
        records = self.dynamic.get(key)
        if not records:
            return self.static.get(key)
        index = bisect_right(self.times[key], timestamp) - 1
        # Never back-fill an unlocalized image with a future map transform.
        if index < 0 or timestamp - records[index][0] > MAX_TF_AGE_NS:
            return None
        before, p, q = records[index]
        if index + 1 < len(records):
            after, next_p, next_q = records[index + 1]
            if 0 < after - before <= MAX_TF_AGE_NS and before <= timestamp <= after:
                t = (timestamp-before)/(after-before)
                dot = sum(a*b for a,b in zip(q, next_q))
                if dot < 0:
                    next_q = [-v for v in next_q]
                    dot = -dot
                if dot > 0.9995:
                    q = quaternion([a+t*(b-a) for a,b in zip(q,next_q)])
                else:
                    theta = math.acos(min(1., dot))
                    q = [(math.sin((1-t)*theta)*a+math.sin(t*theta)*b)/math.sin(theta) for a,b in zip(q,next_q)]
                p = [a+t*(b-a) for a,b in zip(p,next_p)]
        return matrix(p,q)

    def resolve(self, target, source, timestamp):
        source, target = frame_name(source), frame_name(target)
        pending = deque([(source, IDENTITY[:])])
        seen = {source}
        while pending:
            current, accumulated = pending.popleft()
            if current == target:
                return accumulated
            for next_frame, key, invert in self.adjacent.get(current, []):
                if next_frame in seen:
                    continue
                transform = self.at(key, timestamp)
                if transform is None:
                    continue
                seen.add(next_frame)
                pending.append((next_frame, multiply(inverse(transform) if invert else transform, accumulated)))
        return None


class ProjectionCollector:
    def __init__(self, offline=False):
        self.transforms = TransformBuffer()
        self.infos = {}
        self.offline = offline

    def add_tf(self, message, bag_ns, static=False):
        for stamped in getattr(message, 'transforms', []):
            parent = frame_name(getattr(getattr(stamped,'header',None),'frame_id',''))
            child = frame_name(getattr(stamped,'child_frame_id',''))
            if self.offline and 'map' in (parent,child):
                continue
            transform = getattr(stamped,'transform',None)
            if transform is None:
                continue
            p, q = transform.translation, transform.rotation
            self.transforms.add(parent,child,stamp_ns(stamped) or bag_ns,
                                [p.x,p.y,p.z],[q.x,q.y,q.z,q.w],static)

    def add_pose(self, sample):
        rotation = sample.get('orientation')
        if not isinstance(rotation,list) or len(rotation) != 4:
            return
        self.transforms.add(sample.get('frame_id'),sample.get('child_frame_id'),
                            int(sample.get('_header_timestamp_ns') or sample.get('_timestamp_ns') or 0),
                            [sample.get('x'),sample.get('y'),sample.get('z')],rotation)

    def add_info(self, topic, message, bag_ns):
        try:
            roi = getattr(message,'roi',None)
            model = {
                'topic':topic, 'frame_id':frame_name(message.header.frame_id),
                'width':int(message.width), 'height':int(message.height),
                'k':list(message.k), 'd':list(message.d), 'r':list(message.r), 'p':list(message.p),
                'distortion_model':str(message.distortion_model),
                'binning_x':max(1,int(message.binning_x)), 'binning_y':max(1,int(message.binning_y)),
                'roi':{key:int(getattr(roi,key,0)) for key in ('x_offset','y_offset','width','height','do_rectify')},
            }
            if model['width'] <= 0 or model['height'] <= 0 or len(model['k']) != 9 or model['k'][0] <= 0 or model['k'][4] <= 0:
                return
            encoded = json.dumps(model,sort_keys=True,allow_nan=False)
        except (AttributeError,TypeError,ValueError):
            return
        history = self.infos.setdefault(topic,[])
        if not history or history[-1][2] != encoded:
            history.append((stamp_ns(message) or bag_ns,model,encoded))

    def compile(self, frames, primary_topic, map_dir=None):
        self.transforms.prepare()
        for history in self.infos.values():
            history.sort(key=lambda row:row[0])
        info_times = {topic:[row[0] for row in rows] for topic,rows in self.infos.items()}
        models = {}
        ready = 0
        issues = {}
        ground_frame = 'tt02_ground_estimate'
        for frame in frames:
            timestamp = int(frame.get('channels', {}).get(primary_topic, {}).get('header_timestamp_ns') or 0)
            if timestamp and self.transforms.resolve('map', 'base_footprint', timestamp) is not None:
                ground_frame = 'base_footprint'
                break
        ground_heights = []
        ground_tilted = False
        for frame in frames:
            primary = frame.get('channels', {}).get(primary_topic, {})
            timestamp = int(primary.get('header_timestamp_ns') or 0)
            ground = self.transforms.resolve('map', ground_frame, timestamp) if timestamp else None
            # A single UI Z represents a horizontal ground plane, not a ramp.
            if ground is not None and ground[10] >= math.cos(math.radians(2)):
                ground_heights.append(ground[11])
            elif ground is not None:
                ground_tilted = True
            for topic, channel in frame.get('channels',{}).items():
                timestamp = int(channel.get('header_timestamp_ns') or 0)
                optical = frame_name(channel.get('frame_id'))
                candidates = []
                if timestamp and optical:
                    for info_topic, history in self.infos.items():
                        index = bisect_right(info_times[info_topic],timestamp)-1
                        if index >= 0 and history[index][1]['frame_id'] == optical:
                            candidates.append(history[index][1])
                    image_topic = topic.removesuffix('/compressed').removesuffix('/compressedDepth')
                    namespace = image_topic.rsplit('/',1)[0]
                    preferred = [model for model in candidates if model['topic'] == namespace+'/camera_info']
                    if preferred:
                        candidates = preferred
                issue = '画像の撮影時刻またはoptical frameがありません' if not timestamp or not optical else ''
                if not issue and len(candidates) != 1:
                    issue = '対応するCameraInfoがありません' if not candidates else 'CameraInfoの対応が曖昧です'
                value = None if issue else self.transforms.resolve(optical,'map',timestamp)
                if not issue and value is None:
                    issue = '撮影時刻のmap→camera変換がありません（TF・自己位置の欠落または150ms超の遅延）'
                if issue:
                    channel['projection'] = {'issue':issue}
                    issues[issue] = issues.get(issue,0)+1
                    continue
                model = candidates[0]
                model_id = hashlib.sha256(json.dumps(model,sort_keys=True).encode()).hexdigest()[:16]
                models[model_id] = model
                channel['projection'] = {
                    'model_id':model_id, 'camera_from_map':[round(v,12) for v in value],
                    'image_geometry':'rectified' if '/image_rect' in topic else 'raw',
                    'timestamp_ns':str(timestamp),
                }
                ready += 1
        ground_z = None
        ground_issue = '路面基準のbase_footprint TFがありません。投影高さを確認してください。'
        if ground_tilted:
            ground_issue = '路面がMapのXY平面から傾いています。単一の投影高さを自動設定できません。'
        elif ground_heights:
            if max(ground_heights) - min(ground_heights) <= 0.05:
                ground_z = round(statistics.median(ground_heights), 6)
                ground_issue = ''
            else:
                ground_issue = '路面高さの変動が5cmを超えます。単一の投影高さを自動設定できません。'
        return {'schema_version':1, 'models':models, 'ready_frames':ready, 'issues':issues,
                'primary_topic':primary_topic, 'map_frame':'map', 'ground_z_m':ground_z,
                'ground_z_source':ground_frame + '_tf' if ground_z is not None else None,
                'ground_z_issue':ground_issue, 'max_tf_age_ms':MAX_TF_AGE_NS/1e6,
                'localization_fingerprint':localization_fingerprint(map_dir) if map_dir else ''}
