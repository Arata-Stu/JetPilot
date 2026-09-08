"""Static HD-map obstacle geometry, independent of ROS and third-party packages."""
from __future__ import annotations

import math


def segment_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    d = dx * dx + dy * dy
    t = max(0.0, min(1.0, ((p[0]-a[0])*dx + (p[1]-a[1])*dy)/d)) if d else 0.0
    return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)


def cross(a, b, c):
    return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])


def segments_distance(a, b, c, d):
    if cross(a, b, c)*cross(a, b, d) < 0 and cross(c, d, a)*cross(c, d, b) < 0:
        return 0.0
    return min(segment_distance(a,c,d), segment_distance(b,c,d), segment_distance(c,a,b), segment_distance(d,a,b))


def inside_polygon(p, polygon):
    inside = False
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        if segment_distance(p,a,b) <= 1e-9:
            return True
        if (a[1] > p[1]) != (b[1] > p[1]) and p[0] < (b[0]-a[0])*(p[1]-a[1])/(b[1]-a[1])+a[0]:
            inside = not inside
    return inside


def normalize_obstacles(value):
    if not isinstance(value, list) or len(value) > 200:
        raise ValueError('obstacles must be an array of at most 200 polygons')
    result, ids = [], set()
    for index, raw in enumerate(value):
        if not isinstance(raw, dict):
            raise ValueError(f'obstacles[{index}] must be an object')
        oid = str(raw.get('id') or f'obstacle_{index+1:03d}')
        if oid in ids or len(oid) > 64 or not all(c.isascii() and (c.isalnum() or c in '_.-') for c in oid):
            raise ValueError('obstacle IDs must be unique ASCII letters, digits, dot, underscore or hyphen')
        ids.add(oid)
        polygon = raw.get('polygon')
        if not isinstance(polygon, list) or not 3 <= len(polygon) <= 200:
            raise ValueError(f'{oid}: polygon needs 3–200 points')
        try:
            polygon = [[float(p[0]), float(p[1])] for p in polygon]
            height = float(raw.get('height_m', 0.3))
            margin = float(raw.get('margin_m', 0.0))
        except (ValueError, TypeError, IndexError) as exc:
            raise ValueError(f'{oid}: invalid coordinates, height or margin') from exc
        if any(not math.isfinite(v) or abs(v) > 1e6 for p in polygon for v in p):
            raise ValueError(f'{oid}: coordinates must be finite and within 1000 km')
        if not all(math.isfinite(v) and 0 <= v <= 100 for v in (height, margin)):
            raise ValueError(f'{oid}: height and margin must be 0–100 m')
        edges = list(zip(polygon, polygon[1:] + polygon[:1]))
        if any(math.dist(a,b) <= 1e-9 for a,b in edges):
            raise ValueError(f'{oid}: adjacent polygon points overlap')
        if abs(sum(a[0]*b[1]-b[0]*a[1] for a,b in edges)) <= 1e-9:
            raise ValueError(f'{oid}: polygon has no area')
        for i,(a,b) in enumerate(edges):
            for j,(c,d) in enumerate(edges):
                if j <= i+1 or (i == 0 and j == len(edges)-1):
                    continue
                if segments_distance(a,b,c,d) <= 1e-9:
                    raise ValueError(f'{oid}: polygon intersects itself')
        result.append({'id':oid, 'name':str(raw.get('name') or oid)[:120], 'polygon':polygon,
                       'height_m':height, 'margin_m':margin})
    return result


def obstacle_path_issue(points, closed_loop, obstacles, clearance_m=0.0):
    """Test full segments, including thin obstacles between sampled waypoints."""
    if not points:
        return ''
    edges = list(zip(points, points[1:] + (points[:1] if closed_loop else []))) or [(points[0],points[0])]
    for obstacle in obstacles:
        polygon = obstacle['polygon']
        margin = obstacle['margin_m'] + clearance_m
        for i,(a,b) in enumerate(edges):
            if inside_polygon(a,polygon) or inside_polygon(b,polygon) or any(
                segments_distance(a,b,c,d) <= margin + 1e-9
                for c,d in zip(polygon,polygon[1:] + polygon[:1])
            ):
                return f"segment[{i}] intersects obstacle {obstacle['name']} ({obstacle['id']}) or its clearance margin"
    return ''
