"""Dependency-free, conservative planar safety geometry.

A disk enclosing the complete body is swept continuously along each segment.
Only exposed edges of the physical lane union constrain the swept disk.
Internal seams and overlaps do not create fictitious walls; obstacles remain holes.
"""
from dataclasses import dataclass
import math

EPS = 1e-9


def point(p):
    p = (float(p[0]), float(p[1]))
    if not all(math.isfinite(v) for v in p):
        raise ValueError("non-finite geometry")
    return p


def edges(poly):
    return zip(poly, poly[1:] + poly[:1])


def distance(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    t = max(0., min(1., ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy))) if dx or dy else 0.
    return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)


def cross(a, b, c):
    return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])


def separation(a, b, c, d):
    if cross(a,b,c)*cross(a,b,d) < 0 and cross(c,d,a)*cross(c,d,b) < 0:
        return 0.
    return min(distance(a,c,d), distance(b,c,d), distance(c,a,b), distance(d,a,b))


def near(a, b, c, d, radius):
    if (min(a[0],b[0])-radius > max(c[0],d[0]) or
        max(a[0],b[0])+radius < min(c[0],d[0]) or
        min(a[1],b[1])-radius > max(c[1],d[1]) or
        max(a[1],b[1])+radius < min(c[1],d[1])):
        return False
    return separation(a,b,c,d) <= radius


def inside(p, poly):
    result = False
    for a,b in edges(poly):
        if near(p,p,a,b,EPS):
            return True
        if (a[1] > p[1]) != (b[1] > p[1]):
            if p[0] < a[0]+(p[1]-a[1])*(b[0]-a[0])/(b[1]-a[1]):
                result = not result
    return result


def polygon(raw):
    p = [point(v) for v in raw]
    if len(p)>1 and p[0] == p[-1]:
        p.pop()
    if not 3 <= len(p) <= 10000:
        raise ValueError("invalid polygon size")
    if abs(sum(a[0]*b[1]-b[0]*a[1] for a,b in edges(p))) <= EPS:
        raise ValueError("degenerate polygon")
    # Ignore adjacent shared vertices, reject intersections and zero-length edges.
    segments = list(edges(p))
    for i,(a,b) in enumerate(segments):
        if math.dist(a,b) <= EPS:
            raise ValueError("duplicate polygon vertex")
        for j in range(i+2, len(segments)):
            if i == 0 and j == len(segments)-1:
                continue
            if near(a,b,*segments[j],EPS):
                raise ValueError("self-intersecting polygon")
    return p


def split_parameters(a, b, c, d):
    """Split intersections and collinear overlap endpoints without geometry packages."""
    u = (b[0]-a[0], b[1]-a[1]); v = (d[0]-c[0], d[1]-c[1])
    denom = u[0]*v[1]-u[1]*v[0]
    w = (c[0]-a[0], c[1]-a[1])
    if abs(denom) > EPS:
        t = (w[0]*v[1]-w[1]*v[0])/denom
        q = (w[0]*u[1]-w[1]*u[0])/denom
        return [max(0., min(1.,t))] if -EPS <= t <= 1+EPS and -EPS <= q <= 1+EPS else []
    if abs(w[0]*u[1]-w[1]*u[0]) > EPS:
        return []
    length2 = u[0]**2+u[1]**2
    return [max(0.,min(1.,sum((p[k]-a[k])*u[k] for k in (0,1))/length2)) for p in (c,d)]


class Environment:
    def __init__(self, lanes, obstacles):
        self.lanes = []
        self.obstacles = []
        for left, right, closed in lanes:
            if closed:
                left, right = polygon(left), polygon(right)
                if any(near(a,b,c,d,EPS) for a,b in edges(left) for c,d in edges(right)):
                    raise ValueError("physical bounds intersect")
                if inside(left[0],right) == inside(right[0],left):
                    raise ValueError("closed physical bounds must be nested")
                self.lanes.append((left,right))
            else:
                self.lanes.append((polygon(list(left)+list(reversed(right))), None))
        if not self.lanes:
            raise ValueError("no physical drivable bounds")
        self.boundary = []
        all_edges = [edge for outer,other in self.lanes
                     for poly in (outer,other) if poly is not None for edge in edges(poly)]
        for a,b in all_edges:
            cuts = sorted(set([0.,1.] + [t for c,d in all_edges for t in split_parameters(a,b,c,d)]))
            length = math.dist(a,b)
            for lo,hi in zip(cuts,cuts[1:]):
                if (hi-lo)*length <= EPS:
                    continue
                start = tuple(a[k]+lo*(b[k]-a[k]) for k in (0,1))
                end = tuple(a[k]+hi*(b[k]-a[k]) for k in (0,1))
                mid = tuple((start[k]+end[k])/2 for k in (0,1))
                plus = minus = False
                for outer,other in self.lanes:
                    rings = [outer] if other is None else [outer,other]
                    touching = [(ring,c,d) for ring in rings for c,d in edges(ring)
                                if distance(mid,c,d) <= EPS]
                    if not touching:
                        member = inside(mid,outer) if other is None else inside(mid,outer) != inside(mid,other)
                        plus = plus or member
                        minus = minus or member
                    else:
                        for ring,c,d in touching:
                            # Signed orientation determines the material side exactly;
                            # no finite probe can accidentally bridge a narrow gap.
                            area = sum(u[0]*v[1]-v[0]*u[1] for u,v in edges(ring))
                            inner = other is not None and inside(ring[0], other if ring is outer else outer)
                            alignment = (b[0]-a[0])*(d[0]-c[0])+(b[1]-a[1])*(d[1]-c[1])
                            side = area*alignment*(-1 if inner else 1)
                            plus = plus or side > 0
                            minus = minus or side < 0
                if not (plus and minus):
                    self.boundary.append((start,end))
        for name, raw, margin in obstacles:
            if not math.isfinite(margin) or margin < 0:
                raise ValueError("invalid obstacle margin")
            self.obstacles.append((name,polygon(raw),margin))

    def issue(self, a, b, radius):
        for name, poly, margin in self.obstacles:
            if inside(a,poly) or inside(b,poly) or any(
                near(a,b,c,d,radius+margin+EPS) for c,d in edges(poly)
            ):
                return "static obstacle: " + name
        if self.contains(a) and self.contains(b) and all(
            not near(a,b,c,d,radius+EPS) for c,d in self.boundary
        ):
            return ""
        return "outside physical drivable bounds"

    def contains(self, p):
        return any(inside(p,outer) if other is None else inside(p,outer) != inside(p,other)
                   for outer,other in self.lanes)



@dataclass(frozen=True)
class Settings:
    # Extents relative to base_frame, including overhangs, not wheelbase.
    front_m: float = .25
    rear_m: float = .25
    width_m: float = .22
    margin_m: float = .05
    reaction_s: float = .3
    braking_mps2: float = 1.
    route_preview_m: float = .5
    step_m: float = .2

    def __post_init__(self):
        for name,value in vars(self).items():
            if not math.isfinite(value) or value < 0:
                raise ValueError("invalid safety setting: " + name)
        if min(self.front_m+self.rear_m,self.width_m,self.braking_mps2,self.step_m) <= 0:
            raise ValueError("body size, braking and step must be positive")

    @property
    def radius(self):
        # Circumscribed about base_frame, conservative even with asymmetric overhang.
        return math.hypot(max(self.front_m,self.rear_m), self.width_m/2)+self.margin_m

    def stopping_distance(self, speed):
        return speed*self.reaction_s + speed*speed/(2*self.braking_mps2)


def motion_issue(env, pose, velocity, cfg):
    """Constant body twist, then deceleration at constant curvature, forward/reverse.

    Exact arc endpoints; inflate each capsule by its arc/chord error so thin
    obstacles cannot hide between samples. Zero translation with rotation is
    covered by the base-centred circumcircle.
    """
    x,y,yaw = pose
    vx,vy,w = velocity
    if not all(math.isfinite(v) for v in (*pose,*velocity)):
        raise ValueError("non-finite pose or velocity")
    speed = math.hypot(vx,vy)
    travel = cfg.stopping_distance(speed)
    count = max(1,math.ceil(travel/cfg.step_m),math.ceil(abs(w)*(cfg.reaction_s+speed/cfg.braking_mps2)/.1))
    if count > 5000:
        raise ValueError("prediction exceeds computation limit")
    previous = (x,y)
    if speed < EPS:
        return env.issue(previous,previous,cfg.radius)
    # Equivalent constant-speed travel time gives braking arc with fixed curvature.
    duration = travel/speed
    for i in range(1,count+1):
        t = duration*i/count
        if abs(w) < EPS:
            bx,by = vx*t,vy*t
        else:
            bx = (vx*math.sin(w*t)+vy*(math.cos(w*t)-1))/w
            by = (vx*(1-math.cos(w*t))+vy*math.sin(w*t))/w
        current = (x+math.cos(yaw)*bx-math.sin(yaw)*by,
                   y+math.sin(yaw)*bx+math.cos(yaw)*by)
        sagitta = 0. if abs(w)<EPS else speed/abs(w)*(1-math.cos(abs(w)*duration/count/2))
        issue = env.issue(previous,current,cfg.radius+sagitta)
        if issue:
            return issue
        previous = current
    return ""


def route_issue(env, position, raw_points, speed, cfg, closed=False):
    """Check the selected route from the closest segment over the stopping horizon."""
    points = [point(p) for p in raw_points]
    if len(points)<2 or not math.isfinite(speed):
        raise ValueError("invalid selected trajectory")
    closed = closed or (len(points)>2 and math.dist(points[0],points[-1]) <= EPS)
    if closed and points[-1] != points[0]:
        points.append(points[0])
    best = None
    for i,(a,b) in enumerate(zip(points,points[1:])):
        dx,dy=b[0]-a[0],b[1]-a[1]
        length2=dx*dx+dy*dy
        if length2 <= EPS:
            continue
        t=max(0.,min(1.,((position[0]-a[0])*dx+(position[1]-a[1])*dy)/length2))
        p=(a[0]+t*dx,a[1]+t*dy)
        key=(math.dist(position,p),i,p)
        if best is None or key[0]<best[0]:
            best=key
    if best is None:
        raise ValueError("degenerate selected trajectory")
    _,index,start=best
    # Include the connector: a safe line must not grant permission across a wall.
    remaining=max(cfg.route_preview_m,cfg.stopping_distance(abs(speed)))
    chain=[position,start]+points[index+1:]
    if closed:
        loop_length=sum(math.dist(a,b) for a,b in zip(points,points[1:]))
        if loop_length <= EPS:
            raise ValueError("degenerate closed trajectory")
        loops=math.ceil(remaining/loop_length)+1
        if loops*len(points)>100000:
            raise ValueError("closed trajectory exceeds computation limit")
        chain += points[1:]*loops
    budget=5000
    for a,b in zip(chain,chain[1:]):
        length=math.dist(a,b)
        if length <= EPS:
            issue=env.issue(a,a,cfg.radius)
            if issue: return issue
            continue
        used=min(remaining,length)
        count=max(1,math.ceil(used/cfg.step_m))
        budget-=count
        if budget<0: raise ValueError("route exceeds computation limit")
        previous=a
        for j in range(1,count+1):
            t=used/length*j/count
            current=(a[0]+(b[0]-a[0])*t,a[1]+(b[1]-a[1])*t)
            issue=env.issue(previous,current,cfg.radius)
            if issue: return issue
            previous=current
        remaining-=used
        if remaining<=EPS: break
    return ""
