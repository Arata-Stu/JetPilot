"""Shared causal sparse-flow residual detector for RGB and event count images.

Only images, timestamps and the common ROI enter this module. No annotations,
command times, sensor labels or direction-specific rules enter detection.
Pixel thresholds use original EVS-coordinate pixels, even when downsampled.
"""
from collections import deque
import math

import cv2
import numpy as np


ALGORITHM = 'sparse_lk_affine_residual_adjacent_persistent_v1'
DEFAULTS = dict(
    downsample=2, tile_px=32, event_step_ms=10., event_window_ms=10.,
    event_count_clip=3., reference_lag_ms=30., max_reference_ms=70., max_gap_ms=100.,
    corners_per_tile=6, corner_quality=.01, corner_distance_px=6.,
    lk_window_px=42, lk_levels=2, fb_error_px=2., ransac_error_px=1.5,
    min_tracks=24, min_inlier_ratio=.55, min_inlier_tiles=8,
    min_span_x=.35, min_span_y=.20, max_affine_scale=1.25,
    residual_px=2., residual_speed_px_s=40., residual_mad_k=4.,
    min_outlier_points=2, min_outlier_fraction=.5, persistence_ms=20.)
INTEGERS = {k for k, v in DEFAULTS.items() if isinstance(v, int)}


def validate(p):
    if set(p) != set(DEFAULTS):
        raise ValueError('missing or unexpected flow settings')
    for k, v in p.items():
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            raise ValueError(f'{k} must be finite numeric')
        if v <= 0 and not (k == 'lk_levels' and v == 0):
            raise ValueError(f'{k} must be positive')
        if k in INTEGERS and not isinstance(v, int):
            raise ValueError(f'{k} must be an integer')
    for k in ('corner_quality', 'min_inlier_ratio', 'min_span_x', 'min_span_y', 'min_outlier_fraction'):
        if p[k] > 1:
            raise ValueError(f'{k} must be <= 1')
    if p['tile_px'] % p['downsample'] or p['tile_px'] / p['downsample'] < 8:
        raise ValueError('tile_px must divide by downsample into at least 8 pixels')
    ratio = p['event_window_ms'] / p['event_step_ms']
    if ratio < 1 or abs(ratio - round(ratio)) > 1e-9:
        raise ValueError('event window must be an integer multiple of event step')
    if not p['event_window_ms'] <= p['reference_lag_ms'] < p['max_reference_ms'] <= p['max_gap_ms']:
        raise ValueError('require event_window <= reference_lag < max_reference <= max_gap')
    if p['min_tracks'] < 6 or p['min_inlier_tiles'] < 3 or p['max_affine_scale'] <= 1:
        raise ValueError('insufficient affine constraints')


def make_geometry(mask, p):
    """Conservative downsampling: a pixel is valid only if all its inputs are."""
    h, w = mask.shape
    d, tile = p['downsample'], p['tile_px']
    if h % d or w % d or not mask.any():
        raise ValueError('nonempty mask and dimensions divisible by downsample required')
    small = mask.reshape(h//d, d, w//d, d).all(axis=(1, 3))
    yy, xx = np.indices(small.shape)
    cols = (w+tile-1)//tile
    grid = (yy*d//tile)*cols + xx*d//tile
    ids = np.unique(grid[small])
    if len(ids) < p['min_inlier_tiles']:
        raise ValueError('too few ROI tiles for background fit')
    lookup = np.full(small.shape, -1, np.int32)
    lookup[small] = np.searchsorted(ids, grid[small])
    locations = {int(i): n for n, i in enumerate(ids)}
    pairs = [(n, locations[j]) for i, n in locations.items()
             for j in (i+1, i+cols) if j in locations and (j != i+1 or i//cols == j//cols)]
    if not pairs:
        raise ValueError('no neighboring ROI tiles')
    tiles = [dict(tile_id=int(i), x=int(i%cols)*tile, y=int(i//cols)*tile,
                  width=min(tile, w-int(i%cols)*tile), height=min(tile, h-int(i//cols)*tile),
                  valid_pixels=int((lookup == n).sum())*d*d) for n, i in enumerate(ids)]
    # Do not select/accept points whose level-0 LK patch crosses zero-filled
    # ROI boundaries; those boundaries are an artifact, not scene structure.
    win = max(5, int(p['lk_window_px']/d) | 1)
    tracking = cv2.erode(small.astype(np.uint8), np.ones((win, win), np.uint8),
                         borderType=cv2.BORDER_CONSTANT, borderValue=0).astype(bool)
    return dict(mask=small, tracking_mask=tracking, lookup=lookup, ids=ids, tiles=tiles,
                pairs=np.asarray(pairs, dtype=int), original_size=[w, h])


def feature_points(gray, geo, p):
    points = []
    d = p['downsample']
    for tile in geo['tiles']:
        x, y, w, h = (tile[k]//d for k in ('x', 'y', 'width', 'height'))
        found = cv2.goodFeaturesToTrack(
            gray[y:y+h, x:x+w], maxCorners=p['corners_per_tile'],
            qualityLevel=p['corner_quality'], minDistance=p['corner_distance_px']/d,
            mask=geo['tracking_mask'][y:y+h, x:x+w].astype(np.uint8)*255, blockSize=3)
        if found is not None:
            points.append(found.reshape(-1, 2) + [x, y])
    return np.concatenate(points).astype(np.float32) if points else np.empty((0, 2), np.float32)


def point_tiles(points, geo):
    h, w = geo['mask'].shape
    finite = np.isfinite(points).all(axis=1)
    safe = np.where(finite[:, None], points, -1)
    ij = np.rint(safe).astype(int)
    inside = finite & (ij[:, 0] >= 0) & (ij[:, 0] < w) & (ij[:, 1] >= 0) & (ij[:, 1] < h)
    tiles = np.full(len(points), -1, int)
    tiles[inside] = geo['lookup'][ij[inside, 1], ij[inside, 0]]
    return tiles


def tracked_points(previous, current, points, geo, p):
    if len(points) == 0:
        return points, points
    d = p['downsample']
    win = max(5, int(p['lk_window_px']/d) | 1)
    kw = dict(winSize=(win, win), maxLevel=p['lk_levels'],
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, .01))
    q, status, _ = cv2.calcOpticalFlowPyrLK(previous, current, points, None, **kw)
    if q is None or status is None:
        return points[:0], points[:0]
    good = status.ravel().astype(bool) & (point_tiles(q, geo) >= 0)
    finite = np.flatnonzero(good)
    ij = np.rint(q[finite]).astype(int)
    good[finite] &= geo['tracking_mask'][ij[:, 1], ij[:, 0]]
    a, b = points[good], q[good]
    if len(a) == 0:
        return a, b
    back, status, _ = cv2.calcOpticalFlowPyrLK(current, previous, b, None, **kw)
    if back is None or status is None:
        return a[:0], b[:0]
    good = status.ravel().astype(bool) & np.isfinite(back).all(axis=1)
    good &= np.linalg.norm(back-a, axis=1)*d <= p['fb_error_px']
    return a[good], b[good]


def motion_residual(a, b, dt, geo, p):
    """RANSAC dominant affine motion; invalid estimates never become score=0."""
    result = dict(ready=False, reason='insufficient_tracks', tracks=len(a), inliers=0,
                  inlier_ratio=None, inlier_tiles=0, span_x=None, span_y=None,
                  residual_threshold_px=None, residual_p95_px=None, matrix=None)
    if len(a) < p['min_tracks']:
        return result
    cv2.setRNGSeed(0)
    matrix, flags = cv2.estimateAffine2D(a, b, method=cv2.RANSAC,
        ransacReprojThreshold=p['ransac_error_px']/p['downsample'],
        maxIters=2000, confidence=.99, refineIters=10)
    if matrix is None or flags is None or not np.isfinite(matrix).all():
        result['reason'] = 'affine_fit_failed'
        return result
    prediction = a @ matrix[:, :2].T + matrix[:, 2]
    residual = np.linalg.norm(b-prediction, axis=1)*p['downsample']
    # Recompute inliers after refinement instead of trusting the RANSAC mask.
    inlier = residual <= p['ransac_error_px']
    n = int(inlier.sum())
    tiles = point_tiles(b, geo)
    yy, xx = np.nonzero(geo['mask'])
    sx = float(np.ptp(a[inlier, 0]) / max(1, np.ptp(xx))) if n else 0.
    sy = float(np.ptp(a[inlier, 1]) / max(1, np.ptp(yy))) if n else 0.
    singular = np.linalg.svd(matrix[:, :2], compute_uv=False)
    result.update(inliers=n, inlier_ratio=n/len(a), inlier_tiles=len(np.unique(tiles[inlier])),
                  span_x=sx, span_y=sy, matrix=matrix.tolist())
    if n < p['min_tracks'] or n/len(a) < p['min_inlier_ratio']:
        result['reason'] = 'insufficient_inliers'
        return result
    if (result['inlier_tiles'] < p['min_inlier_tiles'] or sx < p['min_span_x'] or sy < p['min_span_y']):
        result['reason'] = 'insufficient_background_coverage'
        return result
    if (np.linalg.det(matrix[:, :2]) <= 0 or singular.min() < 1/p['max_affine_scale']
            or singular.max() > p['max_affine_scale']):
        result['reason'] = 'implausible_affine_scale'
        return result
    center = float(np.median(residual[inlier]))
    scale = max(.25, 1.4826*float(np.median(np.abs(residual[inlier]-center))))
    limit = max(p['residual_px'], p['residual_speed_px_s']*dt, center+p['residual_mad_k']*scale)
    outlier = residual > limit
    count = np.bincount(tiles, minlength=len(geo['ids']))
    bad = np.bincount(tiles[outlier], minlength=len(geo['ids']))
    fraction = bad / np.maximum(count, 1)
    strength = np.minimum(bad/p['min_outlier_points'], fraction/p['min_outlier_fraction'])
    pair_score = strength[geo['pairs']].min(axis=1)
    result.update(ready=True, reason='ok', residual_threshold_px=limit,
                  residual_p95_px=float(np.percentile(residual, 95)),
                  tile_tracks=count, tile_outliers=bad, pair_score=pair_score,
                  points=b, prediction=prediction, outlier=outlier, inlier=inlier)
    return result


class PairPersistence:
    def __init__(self, count, p):
        self.p = p
        self.since = np.full(count, np.nan)
        self.qualified = np.zeros(count, bool)

    def update(self, t, score):
        previous = self.qualified.copy()
        if score is None:
            self.since[:] = np.nan
            self.qualified[:] = False
        else:
            high = score >= 1.
            self.since[~high] = np.nan
            self.since[high & np.isnan(self.since)] = t
            self.qualified = high & (t-self.since >= self.p['persistence_ms']/1000-1e-9)
        return self.qualified.copy(), self.qualified & ~previous


class FlowDetector:
    """Only past image endpoints are used. Reference and state reset on gaps."""
    def __init__(self, geo, p):
        validate(p)
        self.geo, self.p = geo, p
        self.history = deque()
        self.previous = None
        self.persistence = PairPersistence(len(geo['pairs']), p)
        self.active = False
        self.previous_ready = False

    def update(self, interval, t, gray, support_start):
        if gray.dtype != np.uint8 or gray.shape != self.geo['mask'].shape:
            raise ValueError('flow image shape/dtype mismatch')
        if not math.isfinite(t) or not math.isfinite(support_start) or support_start > t:
            raise ValueError('invalid image timestamp/support')
        if self.previous is not None and (t <= self.previous[1] or interval < self.previous[0]):
            raise ValueError('image times must increase and intervals cannot go backwards')
        dt = 0. if self.previous is None else t-self.previous[1]
        reset = (self.previous is None or interval != self.previous[0] or dt > self.p['max_gap_ms']/1000+1e-9)
        if reset:
            self.history.clear()
            self.persistence.update(t, None)
            self.active = self.previous_ready = False
        while self.history and self.history[0][0] < t-self.p['max_reference_ms']/1000-1e-9:
            self.history.popleft()
        # Reference must end before the current event window starts as well.
        eligible = [f for f in self.history if f[0] <= min(support_start, t-self.p['reference_lag_ms']/1000)+1e-9]
        detail = dict(ready=False, reason='reference_warmup', tracks=0, inliers=0,
                      inlier_ratio=None, inlier_tiles=0, span_x=None, span_y=None,
                      residual_threshold_px=None, residual_p95_px=None, matrix=None)
        reference_t = None
        if eligible:
            reference_t, image, points = eligible[-1]
            a, b = tracked_points(image, gray, points, self.geo, self.p)
            detail = motion_residual(a, b, t-reference_t, self.geo, self.p)
        qualified, starts = self.persistence.update(t, detail.get('pair_score'))
        active = bool(qualified.any())
        score = detail.get('pair_score')
        peak = None if score is None else int(np.argmax(score))
        winner = None if peak is None else self.geo['ids'][self.geo['pairs'][peak]].tolist()
        row = {k: detail[k] for k in ('ready', 'reason', 'tracks', 'inliers', 'inlier_ratio',
               'inlier_tiles', 'span_x', 'span_y', 'residual_threshold_px', 'residual_p95_px')}
        row.update(interval=int(interval), time_s=float(t), support_start_s=float(support_start),
            reference_time_s=reference_t, reference_dt_s=None if reference_t is None else t-reference_t,
            state_reset=reset, observed_step_s=0. if reset else dt,
            ready_observed_step_s=dt if detail['ready'] and self.previous_ready and not reset else 0.,
            score=None if peak is None else float(score[peak]),
            tile_a=None if winner is None else winner[0], tile_b=None if winner is None else winner[1],
            active_pairs=int(qualified.sum()), pair_starts=int(starts.sum()),
            alarm=active and not self.active, active=active,
            outlier_points=None if score is None else int(detail['outlier'].sum()))
        for i, key in enumerate(('a00', 'a01', 'tx', 'a10', 'a11', 'ty')):
            row[key] = None if detail['matrix'] is None else float(np.asarray(detail['matrix']).ravel()[i])
        detail['started_pairs'] = np.flatnonzero(starts)
        detail['qualified_pairs'] = np.flatnonzero(qualified)
        self.history.append((t, gray.copy(), feature_points(gray, self.geo, self.p)))
        self.previous, self.previous_ready, self.active = (interval, t), detail['ready'], active
        return row, detail
