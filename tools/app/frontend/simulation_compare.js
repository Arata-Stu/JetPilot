/* Browser-only comparison of the repository's lateral control equations.
 * Common bicycle/longitudinal model; no ROS actuator or safety-node emulation.
 * This file also runs in a Worker so MPC cannot block editing or cancellation.
 */
(function (root) {
  'use strict';
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const angle = v => Math.atan2(Math.sin(v), Math.cos(v));
  const defaults = Object.freeze({
    wheelbaseM: .26, minLookaheadM: .5, maxLookaheadM: 2, lookaheadGainS: .4,
    maxSteeringRad: .45, targetSpeedMps: 2, maxAccelMps2: 1.4,
    maxDecelMps2: 2.3, dragPerS: .04, dtS: .02,
    mapLateralGain: .4, mapScaleStart: 1.5, mapScaleEnd: 3, mapScaleFactor: .25,
    mpcSteps: 12, mpcDt: .05, mpcSamples: 15, mpcMinSpeed: .2,
    mpcPathWeight: 4, mpcHeadingWeight: .8, mpcSteeringWeight: .15, mpcTerminalWeight: 2,
  });
  const methods = [
    {id:'pure_pursuit', name:'Pure Pursuit', color:'#66c7ff'},
    {id:'map_pursuit', name:'Map Pursuit', color:'#ffad66'},
    {id:'kinematic_mpc', name:'Kinematic MPC', color:'#c59aff'},
  ];
  function project(path, x, y, closed) {
    let best = null;
    const n = closed ? path.length : path.length - 1;
    for (let i=0; i<n; i++) {
      const a=path[i], b=path[(i+1)%path.length], dx=b.x-a.x, dy=b.y-a.y;
      const l2=dx*dx+dy*dy;
      if (l2<=1e-12) continue;
      const t=clamp(((x-a.x)*dx+(y-a.y)*dy)/l2,0,1);
      const px=a.x+t*dx, py=a.y+t*dy, d2=(x-px)**2+(y-py)**2;
      if (!best || d2<best.d2) best={x:px,y:py,d2,heading:Math.atan2(dy,dx),index:i};
    }
    return best;
  }
  function control(method, path, car, s, closed) {
    if (method==='kinematic_mpc') {
      const speed=Math.max(Math.abs(car.speed),s.mpcMinSpeed);
      let bestCost=Infinity, steering=0, terminal=null;
      for (let sample=0; sample<s.mpcSamples; sample++) {
        const delta=-s.maxSteeringRad+2*s.maxSteeringRad*sample/(s.mpcSamples-1);
        let x=car.x, y=car.y, yaw=car.yaw, cost=s.mpcSteeringWeight*delta*delta, p;
        for (let step=0; step<s.mpcSteps; step++) {
          x+=speed*Math.cos(yaw)*s.mpcDt; y+=speed*Math.sin(yaw)*s.mpcDt;
          yaw=angle(yaw+speed/s.wheelbaseM*Math.tan(delta)*s.mpcDt);
          p=project(path,x,y,closed);
          if (!p) return null;
          cost+=s.mpcPathWeight*p.d2+s.mpcHeadingWeight*angle(yaw-p.heading)**2+s.mpcSteeringWeight*delta*delta;
        }
        cost+=s.mpcTerminalWeight*p.d2;
        if (cost<bestCost) { bestCost=cost; steering=delta; terminal=p; }
      }
      if (!terminal || (!closed && (terminal.x-car.x)*Math.cos(car.yaw)+(terminal.y-car.y)*Math.sin(car.yaw)<0)) return null;
      return steering;
    }
    let nearest=0, dist=Infinity;
    for (let i=0;i<path.length;i++) {
      const d=(path[i].x-car.x)**2+(path[i].y-car.y)**2;
      if(d<dist) {dist=d; nearest=i;}
    }
    const lookahead=clamp(s.minLookaheadM+Math.abs(car.speed)*s.lookaheadGainS,s.minLookaheadM,s.maxLookaheadM);
    let current=nearest, travelled=0;
    for(let i=0,n=closed?path.length:path.length-1-nearest;i<n;i++) {
      const next=(current+1)%path.length;
      travelled+=Math.hypot(path[next].x-path[current].x,path[next].y-path[current].y);
      current=next;
      if(travelled>=lookahead) break;
    }
    const dx=path[current].x-car.x, dy=path[current].y-car.y;
    const lx=Math.cos(car.yaw)*dx+Math.sin(car.yaw)*dy;
    let ly=-Math.sin(car.yaw)*dx+Math.cos(car.yaw)*dy;
    if(!closed && lx<0) return null;
    if(method==='map_pursuit') {
      ly+=s.mapLateralGain*(-Math.sin(car.yaw)*(path[nearest].x-car.x)+Math.cos(car.yaw)*(path[nearest].y-car.y));
    }
    const d2=lx*lx+ly*ly;
    if(d2<=Number.EPSILON) return null;
    let steering=Math.atan(s.wheelbaseM*2*ly/d2);
    if(method==='map_pursuit' && s.mapScaleEnd>s.mapScaleStart) {
      steering*=1-clamp((Math.abs(car.speed)-s.mapScaleStart)/(s.mapScaleEnd-s.mapScaleStart),0,1)*s.mapScaleFactor;
    }
    return clamp(steering,-s.maxSteeringRad,s.maxSteeringRad);
  }
  function run(input, progress=()=>{}) {
    const s={...defaults,...input.settings}, path=input.path;
    const duration=Number(input.duration ?? 20), offset=Number(input.offset ?? 0), yawOffset=Number(input.yawOffset ?? 0);
    if (!Array.isArray(path) || path.length<2 || path.length>20000 ||
        path.some(p=>!Number.isFinite(p.x)||!Number.isFinite(p.y)) ||
        Object.values(s).some(v=>!Number.isFinite(v)||v<0) ||
        ![duration,offset,yawOffset].every(Number.isFinite) || duration<=0 || duration>120 ||
        s.dtS<.01 || s.dtS>.1 || s.wheelbaseM<=0 || s.maxSteeringRad<=0 || s.maxSteeringRad>=Math.PI/2 ||
        s.minLookaheadM<=0 || s.maxLookaheadM<s.minLookaheadM || s.maxAccelMps2<=0 || s.maxDecelMps2<=0 ||
        !Number.isInteger(s.mpcSteps)||s.mpcSteps<1||s.mpcSteps>50||
        !Number.isInteger(s.mpcSamples)||s.mpcSamples<3||s.mpcSamples>51||s.mpcDt<=0||s.mapScaleFactor>1) {
      throw new Error('経路・比較時間・車両設定の範囲を確認してください。');
    }
    if(input.profile && path.some(p=>!Number.isFinite(p.speed_mps)||p.speed_mps<0)) throw new Error('速度profileが不正です。');
    const next=path.find(p=>Math.hypot(p.x-path[0].x,p.y-path[0].y)>1e-6);
    if(!next) throw new Error('長さのある経路が必要です。');
    const heading=Math.atan2(next.y-path[0].y,next.x-path[0].x);
    const start={x:path[0].x-Math.sin(heading)*offset,y:path[0].y+Math.cos(heading)*offset,yaw:heading+yawOffset,speed:0};
    const steps=Math.ceil(duration/s.dtS);
    return methods.map((method, methodIndex)=>{
      const car={...start}, trace=[{x:car.x,y:car.y}], initial=project(path,car.x,car.y,input.closed);
      let squaredError=0, maxError=Math.sqrt(initial.d2), time=0, distance=0, steeringChange=0, previous=0, samples=0;
      let status='時間終了';
      for(let i=0;i<steps;i++) {
        const dt=Math.min(s.dtS,duration-time);
        const p=project(path,car.x,car.y,input.closed);
        if(!input.closed && p.index===path.length-2 && Math.hypot(car.x-path.at(-1).x,car.y-path.at(-1).y)<=.15) {status='終点到達'; break;}
        const delta=control(method.id,path,car,s,input.closed);
        if(delta===null || !Number.isFinite(delta)) {status='追従不能';break;}
        // All methods use the same speed target, nearest station convention and vehicle model.
        let target=s.targetSpeedMps;
        if(input.profile) {
          let nearest=0,d=Infinity;
          for(let j=0;j<path.length;j++) {const q=(path[j].x-car.x)**2+(path[j].y-car.y)**2;if(q<d){d=q;nearest=j;}}
          target=path[nearest].speed_mps;
        }
        const error=target-car.speed;
        const accel=error>=0?Math.min(s.maxAccelMps2,error*1.8):Math.max(-s.maxDecelMps2,error*2.4);
        const previousSpeed=car.speed;
        car.speed=Math.max(0,car.speed+(accel-s.dragPerS*car.speed)*dt);
        const average=(previousSpeed+car.speed)/2;
        car.x+=average*Math.cos(car.yaw)*dt;car.y+=average*Math.sin(car.yaw)*dt;
        car.yaw=angle(car.yaw+average/s.wheelbaseM*Math.tan(delta)*dt);
        const e=Math.sqrt(project(path,car.x,car.y,input.closed).d2);
        squaredError+=e*e*dt;maxError=Math.max(maxError,e);
        steeringChange+=Math.abs(delta-previous);previous=delta;
        time+=dt;distance+=average*dt;samples++;
        trace.push({x:car.x,y:car.y});
        if(i%20===0) progress((methodIndex+i/steps)/methods.length);
      }
      return {...method,trace,time,distance,status,samples,maxError,rmsError:time?squaredError**.5/Math.sqrt(time):null,
        steeringRate:time?steeringChange/time:null};
    });
  }
  const api={defaults,methods,project,control,run};
  if(typeof module!=='undefined' && module.exports) module.exports=api;
  else root.SimulationCompare=api;
  if(typeof WorkerGlobalScope!=='undefined' && root instanceof WorkerGlobalScope) {
    root.onmessage=event=>{
      try {const results=run(event.data,progress=>root.postMessage({progress}));root.postMessage({results});}
      catch(error) {root.postMessage({error:error.message});}
    };
  }
})(typeof self!=='undefined'?self:globalThis);
