/* Camera optical coordinates: +X right, +Y down, +Z forward. */
const CameraProjection = (() => {
  function transform(point, m, z = 0) {
    const p = [Number(point[0]),Number(point[1]),Number(point[2] ?? z)];
    return [0,1,2].map(r => m[r*4+3]+p.reduce((sum,v,k) => sum+v*m[r*4+k],0));
  }
  function issue(model, geometry, width, height) {
    if (!model || !Array.isArray(model.k) || model.k.length !== 9 || !(model.k[0]>0 && model.k[4]>0)) return '有効なCameraInfoがありません';
    if (![model.d,model.r,model.p].every(Array.isArray) || ![...model.k,...model.d,...model.r,...model.p].every(Number.isFinite)) return 'カメラ校正値が不正です';
    if (geometry === 'rectified' && (model.r.length !== 9 || model.p.length !== 12 || model.p[0]<=0 || model.p[5]<=0 || !model.r.some(v=>v!==0))) return '補正画像用のR/P行列がありません';
    if (geometry !== 'rectified' && !['','plumb_bob','rational_polynomial','equidistant'].includes(model.distortion_model)) return `未対応の歪みモデル: ${model.distortion_model}`;
    if (geometry !== 'rectified' && ((model.distortion_model==='equidistant' && model.d.length!==4)
        || (model.distortion_model==='plumb_bob' && ![0,4,5].includes(model.d.length))
        || (model.distortion_model==='rational_polynomial' && model.d.length!==8))) return '歪み係数の数が不正です';
    const roi = model.roi || {};
    if (geometry === 'rectified' && roi.do_rectify && (roi.width || roi.height)) return '補正ROIの投影は未対応です';
    const expectedWidth = (roi.width || model.width)/(model.binning_x || 1);
    const expectedHeight = (roi.height || model.height)/(model.binning_y || 1);
    if (!(expectedWidth>0 && expectedHeight>0) || Math.abs((width/height)/(expectedWidth/expectedHeight)-1)>0.02) return '画像サイズとカメラ校正の縦横比が一致しません';
    return '';
  }
  function cameraPoint(point, model, geometry) {
    return geometry === 'rectified' ? [0,1,2].map(r => point.reduce((sum,v,k)=>sum+model.r[r*3+k]*v,0)) : point;
  }
  function pixel(p, model, geometry, width, height) {
    if (p[2] < 0.149999 || !p.every(Number.isFinite)) return null;
    let x=p[0]/p[2], y=p[1]/p[2];
    const d=model.d || [], radius=x*x+y*y;
    if (geometry !== 'rectified') {
      if (model.distortion_model === 'equidistant') {
        const r=Math.sqrt(radius), theta=Math.atan(r), t2=theta*theta;
        const distorted=theta*(1+d[0]*t2+d[1]*t2*t2+d[2]*t2**3+d[3]*t2**4);
        const scale=r>1e-12 ? distorted/r : 1;
        x*=scale; y*=scale;
      } else {
        const k=(i)=>Number(d[i] || 0);
        const denominator=model.distortion_model==='rational_polynomial' ? 1+k(5)*radius+k(6)*radius**2+k(7)*radius**3 : 1;
        if (Math.abs(denominator)<1e-10) return null;
        const radial=(1+k(0)*radius+k(1)*radius**2+k(4)*radius**3)/denominator;
        [x,y]=[x*radial+2*k(2)*x*y+k(3)*(radius+2*x*x), y*radial+k(2)*(radius+2*y*y)+2*k(3)*x*y];
      }
    }
    // TF already locates this camera's own optical centre. P's stereo baseline
    // translation must not be added again; R and P's left 3x3 act in this frame.
    const k=geometry==='rectified' ? [model.p[0],model.p[1],model.p[2],model.p[4],model.p[5],model.p[6],0,0,1] : model.k;
    const roi=model.roi || {}, bx=model.binning_x || 1, by=model.binning_y || 1;
    const u=(k[0]*x+k[1]*y+k[2]-(roi.x_offset || 0))/bx;
    const v=(k[3]*x+k[4]*y+k[5]-(roi.y_offset || 0))/by;
    const out=[u*width/((roi.width || model.width)/bx),v*height/((roi.height || model.height)/by)];
    return out.every(Number.isFinite) ? out : null;
  }
  function clip(a,b,width,height) {
    if (!a || !b) return null;
    let lo=0, hi=1;
    const dx=b[0]-a[0],dy=b[1]-a[1];
    for (const [p,q] of [[-dx,a[0]],[dx,width-a[0]],[-dy,a[1]],[dy,height-a[1]]]) {
      if (p===0) { if(q<0) return null; continue; }
      const t=q/p;
      if(p<0) lo=Math.max(lo,t); else hi=Math.min(hi,t);
      if(lo>hi) return null;
    }
    return [[a[0]+lo*dx,a[1]+lo*dy],[a[0]+hi*dx,a[1]+hi*dy]];
  }
  function segments(points,closed,model,matrix,geometry,width,height,z=0) {
    if (!Array.isArray(matrix) || matrix.length!==16 || !matrix.every(Number.isFinite) || issue(model,geometry,width,height)) return [];
    const transformed=points.map(p=>cameraPoint(transform(p,matrix,z),model,geometry));
    const output=[];
    const n=closed ? transformed.length : transformed.length-1;
    for(let i=0;i<n && output.length<20000;i++) {
      let a=transformed[i],b=transformed[(i+1)%transformed.length];
      if(!a.every(Number.isFinite)||!b.every(Number.isFinite)) continue;
      let lo=0,hi=1;
      const dz=b[2]-a[2];
      if(Math.abs(dz)<1e-12) { if(a[2]<0.15||a[2]>30) continue; }
      else {
        const t1=(0.15-a[2])/dz,t2=(30-a[2])/dz;
        lo=Math.max(0,Math.min(t1,t2)); hi=Math.min(1,Math.max(t1,t2));
        if(lo>hi) continue;
      }
      const diff=b.map((v,k)=>v-a[k]);
      b=a.map((v,k)=>v+diff[k]*hi); a=a.map((v,k)=>v+diff[k]*lo);
      const steps=Math.min(200,Math.max(1,Math.ceil(Math.hypot(...b.map((v,k)=>v-a[k]))/0.15)));
      let previous=pixel(a,model,geometry,width,height);
      for(let step=1;step<=steps;step++) {
        const next=pixel(a.map((v,k)=>v+(b[k]-v)*step/steps),model,geometry,width,height);
        const clipped=clip(previous,next,width,height);
        if(clipped) output.push(clipped);
        previous=next;
      }
    }
    return output;
  }
  return {transform,issue,pixel,cameraPoint,clip,segments};
})();
if (typeof module !== 'undefined') module.exports = CameraProjection;
