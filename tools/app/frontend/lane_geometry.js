/* Geometry helpers shared by the canvas editor and dependency-free tests. */
const LaneGeometry = (() => {
  const midpoint = (a, b) => [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
  const distance = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);
  function paired(lane) {
    return lane.boundary_mode === "paired" && lane.left_bound.length === lane.right_bound.length;
  }
  function centers(lane) {
    return lane.left_bound.map((p, i) => midpoint(p, lane.right_bound[i]));
  }
  function fromSpine(points, width) {
    if (!Number.isFinite(width) || width <= 0) throw new Error("幅は正の数で指定してください。");
    const left_bound = [], right_bound = [];
    points.forEach((p, i) => {
      const a = points[Math.max(0, i - 1)], b = points[Math.min(points.length - 1, i + 1)];
      const length = distance(a, b);
      const normal = length > 1e-9 ? [-(b[1] - a[1]) / length, (b[0] - a[0]) / length] : [0, 1];
      left_bound.push([p[0] + normal[0] * width / 2, p[1] + normal[1] * width / 2]);
      right_bound.push([p[0] - normal[0] * width / 2, p[1] - normal[1] * width / 2]);
    });
    return { left_bound, right_bound, centerline: points.map(p => [...p]) };
  }
  // Circular fillets keep the offset radius positive on the inside of a turn.
  function roundedSpine(points, width, margin = 0) {
    if (points.length < 3) return points.map(p => [...p]);
    const result = [[...points[0]]];
    for (let i = 1; i < points.length - 1; i++) {
      const a = points[i-1], p = points[i], b = points[i+1];
      const before = distance(a,p), after = distance(p,b);
      const u = p.map((v,k) => (v-a[k])/before), v = b.map((x,k) => (x-p[k])/after);
      const turn = Math.acos(Math.max(-1, Math.min(1, u[0]*v[0]+u[1]*v[1])));
      if (turn < .02) { result.push([...p]); continue; }
      const tangent = Math.tan(turn/2);
      const trim = Math.min(before*.45, after*.45, Math.max(width, .3)*tangent);
      const radius = trim/tangent;
      if (!Number.isFinite(radius) || radius <= width/2 + margin + .02) {
        const error = new Error(`描画点[${i}]の曲がりが幅に対して急すぎます。前後の点の間隔を広げるか、レーン幅・余裕を小さくしてください。`);
        error.pointIndex = i;
        throw error;
      }
      const sign = Math.sign(u[0]*v[1]-u[1]*v[0]);
      const start = p.map((x,k) => x-u[k]*trim);
      const center = [start[0]-u[1]*radius*sign, start[1]+u[0]*radius*sign];
      const angle = Math.atan2(start[1]-center[1], start[0]-center[0]);
      const count = Math.max(2, Math.ceil(turn/(Math.PI/18)), Math.ceil(radius*turn/.2));
      for (let j=0; j<=count; j++) {
        const t = angle + sign*turn*j/count;
        result.push([center[0]+radius*Math.cos(t), center[1]+radius*Math.sin(t)]);
      }
    }
    result.push([...points.at(-1)]);
    return result;
  }
  function drawnLane(points, width, margin = 0) {
    const spine = roundedSpine(points, width, margin);
    const result = fromSpine(spine, width);
    const physical = fromSpine(spine, width + 2*margin);
    result.drivable_left_bound = physical.left_bound;
    result.drivable_right_bound = physical.right_bound;
    return result;
  }
  function validationLocation(message, lanes) {
    // Only resolve an explicitly named Lane and field; points[] alone is ambiguous.
    for (const lane of lanes) {
      for (const field of ['drivable_left_bound', 'drivable_right_bound', 'left_bound', 'right_bound', 'centerline']) {
        const prefix = `${lane.id} ${field}:`;
        if (!message.startsWith(prefix)) continue;
        const match = message.slice(prefix.length).match(/\b(points?|segment)\[(\d+)\]/);
        if (!match) return null;
        const index = Number(match[2]), points = lane[field] || [];
        if (!points[index]) return null;
        const indices = [index];
        if (match[1] === 'segment') {
          const next = (index+1)%points.length;
          if (next === 0 && !lane.closed_loop) return null;
          indices.push(next);
        }
        return {laneId:lane.id, field, indices};
      }
    }
    return null;
  }
  function split(lane, index, id) {
    if (!paired(lane) || lane.centerline_mode === "manual") throw new Error("分割にはペア境界と自動centerlineが必要です。");
    const n = lane.left_bound.length;
    if (!Number.isInteger(index) || index < 1 || index >= n - 1) throw new Error("端点以外の境界点を選択してください。");
    const a = { ...lane, closed_loop: false }, b = { ...lane, id, primary: false, closed_loop: false };
    for (const field of ["left_bound", "right_bound", "drivable_left_bound", "drivable_right_bound"].filter(f => lane[f]?.length)) {
      if (lane[field].length !== n) throw new Error("分割には走行可能境界と経路生成境界の点数の一致が必要です。");
      a[field] = lane[field].slice(0, index + 1).map(p => [...p]);
      b[field] = lane[field].slice(index).map(p => [...p]);
      if (lane.closed_loop) b[field].push([...lane[field][0]]);
    }
    a.centerline = centers(a); b.centerline = centers(b);
    return [a, b];
  }
  function join(a, b) {
    if (a.id === b.id || a.closed_loop || b.closed_loop || !paired(a) || !paired(b)
        || a.centerline_mode === "manual" || b.centerline_mode === "manual") {
      throw new Error("結合には別々の開いたペアレーンと自動centerlineが必要です。");
    }
    if (a.left_bound.length < 2 || b.left_bound.length < 2) throw new Error("各レーンに2組以上の点が必要です。");
    const result = { ...a };
    const fields = ["left_bound", "right_bound", "drivable_left_bound", "drivable_right_bound"].filter(f => a[f]?.length || b[f]?.length);
    for (const field of fields) {
      if (!a[field]?.length || !b[field]?.length) throw new Error("結合する両Laneの境界を揃えてください。");
      if (distance(a[field].at(-1), b[field][0]) > 0.25) throw new Error("現在レーンの終点と結合先の始点を25cm以内に合わせてください。");
      result[field] = [...a[field].slice(0, -1), midpoint(a[field].at(-1), b[field][0]), ...b[field].slice(1)].map(p => [...p]);
    }
    result.closed_loop = ["left_bound", "right_bound"].every(f => distance(result[f][0], result[f].at(-1)) < 1e-6);
    if (result.closed_loop) for (const f of fields) result[f].pop();
    result.centerline = centers(result);
    return result;
  }
  function connector(a, b, id, mode = "curve") {
    if (a.closed_loop || b.closed_loop || a.id === b.id) throw new Error("接続元と接続先には異なる開いたLaneを選んでください。");
    const start = a.centerline.at(-1), end = b.centerline[0];
    if (!start || !end || a.centerline.length < 2 || b.centerline.length < 2) throw new Error("両LaneにCenterlineが必要です。");
    const gap = distance(start, end);
    if (gap < .001) throw new Error("端点が一致しています。「端点を直接接続」を使用してください。");
    const unit = (p, q) => { const d = distance(p,q); if (d < 1e-6) throw new Error("端点の向きが定まりません。"); return [(q[0]-p[0])/d,(q[1]-p[1])/d]; };
    const u = unit(a.centerline.at(-2), start), v = unit(end, b.centerline[1]);
    const cross = (p,q) => p[0]*q[1]-p[1]*q[0];
    const dot = (p,q) => p[0]*q[0]+p[1]*q[1];
    const delta = end.map((x,k)=>x-start[k]);
    const turn = Math.atan2(cross(u,v),dot(u,v));
    const samples = [];
    const append = (p,tangent) => {
      if (!samples.length || distance(samples.at(-1).p,p)>1e-8) samples.push({p,tangent});
      if (samples.length>2001) throw new Error("接続区間が長すぎます。");
    };
    const line = (p,q,tangent) => {
      const n = Math.max(1,Math.ceil(distance(p,q)/.1));
      if (n>2000) throw new Error("接続区間が長すぎます。");
      for (let i=0;i<=n;i++) append(p.map((x,k)=>x+(q[k]-x)*i/n),tangent);
    };
    const offsets = {};
    for (const key of ["left_bound","right_bound","drivable_left_bound","drivable_right_bound"]) {
      const fallback = key.replace("drivable_", "");
      const p = (a[key]?.length ? a[key] : a[fallback]).at(-1);
      const q = (b[key]?.length ? b[key] : b[fallback])[0];
      const local = (point,origin,tangent) => {
        const d=point.map((x,k)=>x-origin[k]);
        return [dot(d,tangent),cross(tangent,d)];
      };
      offsets[key]={p,q,from:local(p,start,u),to:local(q,end,v)};
    }
    const det=cross(u,v);
    const approach = Math.abs(det)>1e-6 ? cross(delta,v)/det : -1;
    const departure = Math.abs(det)>1e-6 ? cross(u,delta)/det : -1;
    if (mode !== "straight" && approach>1e-6 && departure>1e-6 && Math.abs(turn)<Math.PI-.05) {
      // One constant-radius arc plus any straight remainder, rather than an
      // ellipse-shaped cubic spanning unequal distances to the corner.
      const trim=Math.min(approach,departure);
      const radius=trim/Math.tan(Math.abs(turn)/2), sign=Math.sign(turn);
      const inner=Math.max(0,...Object.values(offsets).flatMap(o=>[sign*o.from[1],sign*o.to[1]]));
      if (radius<=inner+.02) throw new Error("カーブ半径がレーン幅に対して小さすぎます。入口・出口を交点から離してください。");
      const entry=start.map((x,k)=>x+u[k]*(approach-trim));
      const exit=end.map((x,k)=>x-v[k]*(departure-trim));
      const center=[entry[0]-u[1]*radius*sign,entry[1]+u[0]*radius*sign];
      line(start,entry,u);
      const angle=Math.atan2(entry[1]-center[1],entry[0]-center[0]);
      const n=Math.max(2,Math.ceil(radius*Math.abs(turn)/.1),Math.ceil(Math.abs(turn)/.05));
      if (n>2000) throw new Error("接続区間が長すぎます。");
      for (let i=1;i<=n;i++) {
        const theta=angle+turn*i/n;
        append([center[0]+radius*Math.cos(theta),center[1]+radius*Math.sin(theta)],[-sign*Math.sin(theta),sign*Math.cos(theta)]);
      }
      line(exit,end,v);
    } else {
      const count = Math.max(12, Math.ceil(gap/.1));
      if (count>2000) throw new Error("接続区間が長すぎます。");
      for (let i=0;i<=count;i++) {
        const t=i/count,h=1-t;
        const p=start.map((x,k)=>mode==="straight" ? x*h+end[k]*t
          : h*h*h*x+3*h*h*t*(x+u[k]*gap/3)+3*h*t*t*(end[k]-v[k]*gap/3)+t*t*t*end[k]);
        const d=start.map((x,k)=>mode==="straight" ? delta[k]
          : h*h*u[k]*gap+6*h*t*(end[k]-v[k]*gap/3-x-u[k]*gap/3)+t*t*v[k]*gap);
        const length=Math.hypot(...d);
        if (length<1e-8) throw new Error("滑らかな接続を作れません。端点の位置と向きを調整してください。");
        append(p,d.map(x=>x/length));
      }
    }
    samples[0]={p:[...start],tangent:u};
    samples[samples.length-1]={p:[...end],tangent:v};
    const stations=[0];
    for(let i=1;i<samples.length;i++) stations.push(stations.at(-1)+distance(samples[i-1].p,samples[i].p));
    const lane = {id, primary:false, closed_loop:false, boundary_mode:"paired", centerline_mode:"manual",
      successor_ids:[b.id], default_successor_id:b.id, centerline:samples.map(s=>s.p)};
    for (const [key,o] of Object.entries(offsets)) {
      if (mode === "straight") {
        lane[key]=samples.map((_,i)=>o.p.map((x,k)=>x+(o.q[k]-x)*i/(samples.length-1)));
        lane[key][0]=[...o.p]; lane[key][lane[key].length-1]=[...o.q];
        continue;
      }
      lane[key]=samples.map(({p,tangent},i)=>{
        const t=stations[i]/stations.at(-1), blend=t*t*(3-2*t);
        const along=o.from[0]*(1-blend)+o.to[0]*blend;
        const across=o.from[1]*(1-blend)+o.to[1]*blend;
        return [p[0]+along*tangent[0]-across*tangent[1],p[1]+along*tangent[1]+across*tangent[0]];
      });
      lane[key][0]=[...o.p]; lane[key][lane[key].length-1]=[...o.q];
    }
    return lane;
  }
  return { paired, centers, fromSpine, roundedSpine, drawnLane, validationLocation, split, join, connector };
})();
if (typeof module !== "undefined") module.exports = LaneGeometry;
