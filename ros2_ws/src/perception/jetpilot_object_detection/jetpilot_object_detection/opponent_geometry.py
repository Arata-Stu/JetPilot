"""Dependency-free ray/plane projection and bounded, discontinuity-aware trails."""
import math
from collections import deque


def rotate(q, p):
    norm = math.sqrt(sum(v*v for v in q))
    if len(q) != 4 or not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid TF quaternion')
    x,y,z,w = [v/norm for v in q]
    a,b,c = p
    return ((1-2*(y*y+z*z))*a+2*(x*y-z*w)*b+2*(x*z+y*w)*c,
            2*(x*y+z*w)*a+(1-2*(x*x+z*z))*b+2*(y*z-x*w)*c,
            2*(x*z-y*w)*a+2*(y*z+x*w)*b+(1-2*(x*x+y*y))*c)


def camera_ray(u, v, model, geometry='raw'):
    """Inverse CameraInfo distortion; unsupported calibration is rejected."""
    if geometry not in ('raw', 'rectified'):
        raise ValueError('image_geometry must be raw or rectified')
    if not all(math.isfinite(value) for value in [u,v]+model['k']+model['d']+model['r']+model['p']):
        raise ValueError('Nonfinite camera calibration')
    roi = model['roi']
    bx,by = model['binning']
    u = u*bx+roi[0]; v = v*by+roi[1]
    if geometry == 'rectified':
        if len(model['p']) != 12 or len(model['r']) != 9 or roi[4]:
            raise ValueError('Unsupported rectified ROI/R/P')
        p=model['p']; k=[p[0],p[1],p[2],p[4],p[5],p[6],0,0,1]
    else:
        k=model['k']
    if len(k)!=9 or k[0]<=0 or k[4]<=0 or abs(k[1])+abs(k[3])>1e-8:
        raise ValueError('Invalid/unsupported camera intrinsics')
    xd,yd=(u-k[2])/k[0],(v-k[5])/k[4]
    x,y=xd,yd
    if geometry == 'raw':
        d=model['d']; kind=model['distortion_model']
        if kind not in ('plumb_bob','rational_polynomial','') or (kind=='rational_polynomial' and len(d)!=8) or (kind in ('plumb_bob','') and len(d) not in (0,4,5)):
            raise ValueError('Unsupported raw distortion model/coefficients')
        d=d+[0.]*(8-len(d))
        def distort(a,b):
            r=a*a+b*b
            denominator=1+d[5]*r+d[6]*r*r+d[7]*r*r*r
            if abs(denominator)<1e-10: raise ValueError('Singular distortion')
            radial=(1+d[0]*r+d[1]*r*r+d[4]*r*r*r)/denominator
            return (a*radial+2*d[2]*a*b+d[3]*(r+2*a*a),
                    b*radial+d[2]*(r+2*b*b)+2*d[3]*a*b)
        for _ in range(15):
            a,b=distort(x,y);ex,ey=a-xd,b-yd
            if max(abs(ex),abs(ey))<1e-9: break
            h=1e-6; ax,ay=distort(x+h,y);bx_,by_=distort(x,y+h)
            j00,j10,j01,j11=(ax-a)/h,(ay-b)/h,(bx_-a)/h,(by_-b)/h
            det=j00*j11-j01*j10
            if abs(det)<1e-10: raise ValueError('Distortion inverse did not converge')
            x-=(j11*ex-j01*ey)/det;y-=(-j10*ex+j00*ey)/det
            if not math.isfinite(x+y) or abs(x)+abs(y)>100:
                raise ValueError('Invalid inverse distortion')
        a,b=distort(x,y)
        if max(abs(a-xd),abs(b-yd))>1e-6: raise ValueError('Distortion inverse did not converge')
        ray=(x,y,1.)
    else:
        # R maps original optical -> rectified; invert it for the optical TF.
        r=model['r']
        for i in range(3):
            for j in range(3):
                if abs(sum(r[k*3+i]*r[k*3+j] for k in range(3))-(1 if i==j else 0))>1e-4:
                    raise ValueError('Invalid rectification rotation')
        ray=tuple(sum(r[k*3+i]*[x,y,1.][k] for k in range(3)) for i in range(3))
    norm=math.sqrt(sum(v*v for v in ray))
    return tuple(v/norm for v in ray)


def project_contact(box, model, camera_tf, ground_tf, geometry='raw', max_range=8.):
    """bbox bottom centre estimates a ground contact, not vehicle axle centre."""
    x,y,w,h=box
    width,height=model['image_size']
    if not all(math.isfinite(v) for v in box) or w<=0 or h<=0:
        raise ValueError('Invalid box')
    bottom=y+h/2
    if x-w/2<1 or x+w/2>width-1 or bottom>=height-2 or bottom<0 or y-h/2<0:
        raise ValueError('Clipped box/contact point')
    ray=camera_ray(x,bottom,model,geometry)
    origin,rotation=camera_tf
    plane,plane_rotation=ground_tf
    direction=rotate(rotation,ray);normal=rotate(plane_rotation,(0,0,1))
    denominator=sum(a*b for a,b in zip(normal,direction))
    # Grazing rays amplify a one-pixel error into large range errors.
    if denominator>=-0.025: raise ValueError('Ray above or too close to horizon')
    distance=sum(n*(p-o) for n,p,o in zip(normal,plane,origin))/denominator
    if not math.isfinite(distance) or not 0.15<=distance<=max_range:
        raise ValueError('Ground contact outside projection range')
    return tuple(o+distance*d for o,d in zip(origin,direction))


class Trails:
    def __init__(self, max_tracks=8, max_points=200, retain_seconds=15., gap_seconds=0.5, max_speed=8.):
        self.max_tracks=max_tracks;self.max_points=max_points
        self.retain_seconds=retain_seconds;self.gap_seconds=gap_seconds;self.max_speed=max_speed
        self.tracks={};self.last_time=None

    def expire(self, now):
        if self.last_time is not None and now<self.last_time:
            self.tracks.clear()
        self.last_time=now
        self.tracks={key:value for key,value in self.tracks.items() if now-value['points'][-1][0]<=self.retain_seconds}

    def observe(self, track_id, stamp, point, estimated):
        if not track_id or not all(math.isfinite(v) for v in point): return
        if track_id not in self.tracks:
            if len(self.tracks)>=self.max_tracks:
                del self.tracks[min(self.tracks,key=lambda key:self.tracks[key]['points'][-1][0])]
            used={value['slot'] for value in self.tracks.values()}
            self.tracks[track_id]={'slot':next(i for i in range(self.max_tracks) if i not in used),
                                   'points':deque(maxlen=self.max_points), 'estimated':estimated}
        track=self.tracks[track_id];points=track['points']
        if points:
            dt=stamp-points[-1][0]
            if dt<=0: return
            distance=math.dist(point,points[-1][1])
            if dt>self.gap_seconds or distance>self.max_speed*dt+0.15 or track['estimated']!=estimated:
                points.clear()
        points.append((stamp,point));track['estimated']=estimated
        while points and stamp-points[0][0]>self.retain_seconds: points.popleft()
