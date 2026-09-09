/* Stateful directed-lane traversal for the browser simulator. No position-based lane switching. */
(function(root) {
  'use strict';
  const distance=(a,b)=>Math.hypot(a.x-b.x,a.y-b.y);
  const xy=p=>({x:Number(p[0]),y:Number(p[1])});
  const stations=p=>{const s=[0];for(let i=1;i<p.length;i++)s.push(s.at(-1)+distance(p[i-1],p[i]));return s;};
  function sample(raw) {
    const p=raw.map(xy), s=stations(p), result=[];
    if(p.length<2 || p.some(q=>!Number.isFinite(q.x)||!Number.isFinite(q.y)) || s.some((v,i)=>i && v-s[i-1]<1e-6)) throw new Error('Laneの点列が不正です。');
    const count=Math.max(3,Math.ceil(s.at(-1)/.1));
    if(count>20000) throw new Error('Laneが長すぎます。');
    let i=0;
    for(let j=0;j<=count;j++) {const at=s.at(-1)*j/count;while(i<s.length-2 && s[i+1]<at)i++;const t=(at-s[i])/(s[i+1]-s[i]);result.push({x:p[i].x+t*(p[i+1].x-p[i].x),y:p[i].y+t*(p[i+1].y-p[i].y)});}
    return result;
  }
  function project(p,s,car,low=0,high=Infinity) {
    let best=null;
    for(let i=0;i<p.length-1;i++) {
      if(s[i+1]<low || s[i]>high)continue;
      const length=s[i+1]-s[i],a=p[i],b=p[i+1];
      let t=((car.x-a.x)*(b.x-a.x)+(car.y-a.y)*(b.y-a.y))/(length*length);
      t=Math.max(Math.max(0,(low-s[i])/length),Math.min(Math.min(1,(high-s[i])/length),t));
      const x=a.x+t*(b.x-a.x),y=a.y+t*(b.y-a.y),d2=(car.x-x)**2+(car.y-y)**2;
      if(!best || d2<best.d2)best={x,y,d2,s:s[i]+t*length,index:i};
    }
    return best;
  }
  function create(network) {
    const lanes=new Map();
    for(const raw of network.lanes || []) {
      if(!raw.id || lanes.has(raw.id) || raw.closed_loop)throw new Error('ネットワークには固有IDの開いたLaneが必要です。');
      if(network.mode==='raceline' && !(raw.network_raceline?.length>=2))throw new Error(`${raw.id}: Raceline候補を生成・保存してください。`);
      const center=sample(raw.centerline || []), path=network.mode==='raceline'?sample(raw.network_raceline || []):center;
      lanes.set(raw.id,{...raw,center,path,cs:stations(center),ps:stations(path),successor_ids:raw.successor_ids || []});
    }
    if(!lanes.has(network.initialLaneId))throw new Error('開始Laneを指定してください。');
    for(const lane of lanes.values()) {
      const options=lane.successor_ids;
      if(new Set(options).size!==options.length || options.some(id=>!lanes.has(id)||id===lane.id))throw new Error(`${lane.id}: 接続先が不正です。`);
      if((options.length>1 || lane.default_successor_id) && !options.includes(lane.default_successor_id))throw new Error(`${lane.id}: デフォルト分岐を指定して保存してください。`);
      for(const id of options) {
        const target=lanes.get(id);
        if(distance(lane.center.at(-1),target.center[0])>.001 || distance(lane.path.at(-1),target.path[0])>.001)throw new Error(`${lane.id} → ${id}: 接続端点が一致しません。`);
      }
    }
    function immediate(source,target) {
      if(!target)return '';
      const matches=lanes.get(source).successor_ids.filter(start=>{
        let id=start;const seen=new Set();
        while(id && !seen.has(id)) {if(id===target)return true;seen.add(id);const l=lanes.get(id);id=l?.successor_ids.length===1?l.successor_ids[0]:'';}
        return false;
      });
      return matches.length===1?matches[0]:'';
    }
    const junctions=new Map();
    for(const j of network.junctions || []) {
      for(const section of network.sections || []) {
        if(!j.activation_section_ids?.includes(section.id) || !lanes.has(section.lane_id))continue;
        const source=section.lane_id, previous=junctions.get(source);
        if(previous && previous.id!==j.id)throw new Error(`${source}: 同じLaneに複数の信号があります。信号ごとにLaneを分割してください。`);
        const mapping={};for(const dir of ['left','straight','right'])mapping[dir]=immediate(source,j.branches?.[dir]);
        junctions.set(source,{id:j.id,signal_id:j.signal_id,mapping,
          start:Math.min(previous?.start ?? Infinity,Number(section.start_s_m)),
          end:Math.max(previous?.end ?? 0,Number(section.end_s_m))});
      }
    }
    let signals={};
    const controls=[...lanes.values()].filter(l=>l.successor_ids.length>1 || junctions.has(l.id)).map(l=>{
      const j=junctions.get(l.id);
      return {laneId:l.id,defaultLaneId:l.default_successor_id || l.successor_ids[0] || '',signalId:j?.signal_id || '',
        options:j ? Object.entries(j.mapping).filter(([,id])=>id).map(([value,laneId])=>({value,laneId,label:{left:'左',straight:'直進',right:'右'}[value]}))
          : l.successor_ids.map(id=>({value:`lane:${id}`,laneId:id,label:id}))};
    });
    function setSignals(value) {
      if(!value || typeof value!=='object' || Array.isArray(value))throw new Error('Simulation信号の形式が不正です。');
      for(const [id,signal] of Object.entries(value)) {
        const control=controls.find(c=>c.laneId===id);
        if(!control || !['default','stop',...control.options.map(o=>o.value)].includes(signal))throw new Error(`${id}: 利用できない信号です。`);
      }
      signals={...value};
    }
    function selection(laneId,station=Infinity) {
      const l=lanes.get(laneId),j=junctions.get(laneId);
      const active=!j || station>=j.start;
      const signal=active?(signals[laneId] || 'default'):'default';
      const option=controls.find(c=>c.laneId===laneId)?.options.find(o=>o.value===signal);
      return {next:option?.laneId || l.default_successor_id || l.successor_ids[0] || '',
        active,stop:signal==='stop',stopStation:Math.min(l.cs.at(-1),j?.end ?? l.cs.at(-1)),signal};
    }
    function tracker() {
      let current=network.initialLaneId,station=null,committed='',projection=null,signalGatePassed=false;
      const history=[current];
      function update(car) {
        let lane=lanes.get(current);
        projection=project(lane.center,lane.cs,car,station===null?0:Math.max(0,station-.5),station===null?Infinity:station+.5);
        if(!projection || projection.d2>4)throw new Error('現在Laneから離れました。');
        station=projection.s;
        let choice=selection(current,station);
        const length=lane.cs.at(-1);
        if(signalGatePassed)choice={...choice,stop:false,next:committed || choice.next};
        if(!choice.stop && junctions.has(current) && station>=choice.stopStation) {
          signalGatePassed=true;
          if(!committed)committed=choice.next;
        }
        if(!committed && choice.active && !choice.stop && length-station<=1.0)committed=choice.next;
        const a=lane.center.at(-2),b=lane.center.at(-1);
        const passed=(car.x-b.x)*(b.x-a.x)+(car.y-b.y)*(b.y-a.y)>=0;
        if(choice.stop && station>=choice.stopStation-.001)throw new Error('停止線を通過しました。');
        if(committed && station>=length-.05 && passed) {
          const target=lanes.get(committed),hit=project(target.center,target.cs,car,0,.5);
          if(!hit || hit.d2>4)throw new Error('接続先Laneの入口から離れました。');
          current=committed;committed='';signalGatePassed=false;station=hit.s;projection=hit;history.push(current);lane=target;choice=selection(current,station);
        }
        const pathHit=project(lane.path,lane.ps,car,Math.max(0,station/lane.cs.at(-1)*lane.ps.at(-1)-.5),station/lane.cs.at(-1)*lane.ps.at(-1)+.5);
        const local=lane.path.slice(Math.max(0,pathHit.index-1));
        // Existing controller nearest searches must remain on the current Lane.
        local.nearestLimit=local.length;
        let remainingToStop=choice.stop?Math.max(0,choice.stopStation-station-.3):(!choice.next?Math.max(0,lane.cs.at(-1)-station-.15):Infinity);
        let distanceAhead=lane.cs.at(-1)-station;
        let id=committed || choice.next,remaining=12-stations(local).at(-1);const seen=new Set([current]);
        if(choice.stop)id='';
        while(id && !seen.has(id) && remaining>0) {
          seen.add(id);const nextLane=lanes.get(id);
          for(const p of nextLane.path.slice(1)) {remaining-=distance(local.at(-1),p);local.push(p);if(remaining<=0)break;}
          const nextChoice=selection(id);
          if(nextChoice.stop) {remainingToStop=Math.min(remainingToStop,Math.max(0,distanceAhead+nextChoice.stopStation-.3));break;}
          if(!nextChoice.next)remainingToStop=Math.min(remainingToStop,Math.max(0,distanceAhead+nextLane.cs.at(-1)-.15));
          distanceAhead+=nextLane.cs.at(-1);id=nextChoice.next;
        }
        return {path:local,laneId:current,nextLaneId:committed || choice.next,committed:Boolean(committed),signal:choice.signal,
          error:Math.sqrt(pathHit.d2),remainingToStop,waiting:choice.stop,ended:!choice.next && lane.cs.at(-1)-station<.2,history:[...history]};
      }
      return {update};
    }
    return {controls,setSignals,tracker};
  }
  const api={create};
  if(typeof module!=='undefined' && module.exports)module.exports=api;else root.NetworkSimulation=api;
})(typeof self!=='undefined'?self:globalThis);
