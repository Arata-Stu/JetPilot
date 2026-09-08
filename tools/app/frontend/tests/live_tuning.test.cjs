const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const path=require('node:path');
const source=fs.readFileSync(path.join(__dirname,'../live_tuning.js'),'utf8');
function context() {
  const detail={map:{path:'/map'},hd_map:{sections:[]}};
  const ctx={state:{tab:'maps',selectedMapPath:'/map',selectedMapDetail:detail},setInterval:()=>{},
    hasUnsavedMapEdits:()=>false,Date,drawPolyline:()=>{},canvasCssPixelScale:()=>1,
    mapPointProjector:()=>p=>p,drawCanvasArrow:()=>{},setTimeout:()=>{},clearTimeout:()=>{},
    $:()=>null,toast:()=>{},drawMapPreview:()=>{},esc:String,js:JSON.stringify};
  vm.createContext(ctx);vm.runInContext(source+'\nthis.t=tuning;',ctx);
  return ctx;
}
test('application requires a current preview, correct map, fresh status and no unsaved edits',()=>{
  const ctx=context();
  Object.assign(ctx.t,{connected:true,receivedAt:Date.now(),preview:{map_id:'A'},mapId:'A',
    previewDetail:ctx.state.selectedMapDetail,status:{apply_issue:''}});
  assert.equal(ctx.tuningApplyIssue(),'');
  ctx.t.previewDetail={};assert.match(ctx.tuningApplyIssue(),/プレビュー/);
  ctx.t.previewDetail=ctx.state.selectedMapDetail;
  ctx.t.preview.map_id='B';assert.match(ctx.tuningApplyIssue(),/一致/);
  ctx.t.preview.map_id='A';ctx.hasUnsavedMapEdits=()=>true;
  assert.match(ctx.tuningApplyIssue(),/プレビュー/);
  ctx.hasUnsavedMapEdits=()=>false;ctx.t.receivedAt=0;
  assert.match(ctx.tuningApplyIssue(),/接続/);
});
test('stale or unlocalized pose is not drawn',()=>{
  const ctx=context();let arrows=0;ctx.drawCanvasArrow=()=>arrows++;
  Object.assign(ctx.t,{enabled:true,mapPath:'/map',connected:true,receivedAt:Date.now(),
    status:{localization:'localized',pose:{x:1,y:2,yaw:0}}});
  ctx.drawTuningOverlay({},ctx.state.selectedMapDetail,100,100);assert.equal(arrows,1);
  ctx.t.receivedAt=0;ctx.drawTuningOverlay({},ctx.state.selectedMapDetail,100,100);assert.equal(arrows,1);
  ctx.t.receivedAt=Date.now();ctx.t.status.localization='stale';
  ctx.drawTuningOverlay({},ctx.state.selectedMapDetail,100,100);assert.equal(arrows,1);
});
test('editing speed during compilation invalidates pending preview',async()=>{
  const ctx=context();let done;
  ctx.api=()=>new Promise(resolve=>done=resolve);
  const pending=ctx.prepareTuning();
  ctx.t.speed=2;
  done({snapshot:{map_id:'A',revision:'test'}});
  await pending;
  assert.equal(ctx.t.preview,null);
  assert.match(ctx.t.error,/変わりました/);
});
test('disconnect discards pending pose responses',async()=>{
  const ctx=context();let done;
  ctx.api=()=>new Promise(resolve=>done=resolve);
  Object.assign(ctx.t,{enabled:true,mapPath:'/map',connected:true,mapId:'A'});
  const pending=ctx.pollTuning(0);
  ctx.disconnectTuning();
  done({map_id:'A',pose:{x:1,y:2,yaw:0},localization:'localized'});
  await pending;
  assert.equal(ctx.t.connected,false);assert.equal(ctx.t.status,null);assert.equal(ctx.t.trail.length,0);
});
test('live map redraw never loads raster images, video or point cloud and caps canvas size',()=>{
  const app=fs.readFileSync(path.join(__dirname,'../app.js'),'utf8');
  const begin=app.indexOf('function drawMapPreview()');
  const end=app.indexOf('\nfunction ',begin+1);
  const ctx=context();
  let imageLoads=0,camera=0,cloud=0;
  const canvas={getContext:()=>({clearRect(){},fillRect(){}})};
  ctx.state.selectedMapDetail.raster={image_url:'/large.png',width:10000,height:6000};
  Object.assign(ctx,{tuningVisible:()=>true,$:id=>id==='map-preview-canvas'?canvas:null,
    refreshTuningPanel(){},updateMapCameraView(){camera++;},drawPointCloud(){cloud++;},
    applyMapCanvasDisplay(){},drawGrid(){},drawMapLayers(){},drawTuningOverlay(){},
    Image:function(){imageLoads++;},console});
  vm.runInContext(app.slice(begin,end),ctx);
  ctx.drawMapPreview();
  assert.equal(imageLoads,0);assert.equal(camera,0);assert.equal(cloud,0);
  assert.ok(canvas.width<=1400 && canvas.height<=1000);
});
