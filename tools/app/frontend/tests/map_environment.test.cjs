const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
function context() {
  const ctx=vm.createContext({window:{},localStorage:{getItem:()=>null},console,document:{getElementById:id=>id==='map-preview-canvas'?{width:100,height:100}:null}});
  vm.runInContext(fs.readFileSync(path.join(__dirname,'../lane_geometry.js'),'utf8'),ctx);
  const source=fs.readFileSync(path.join(__dirname,'../app.js'),'utf8');
  vm.runInContext(source.slice(0,source.indexOf('\nwindow.')),ctx);
  vm.runInContext(`
    render=()=>{}; drawMapPreview=()=>{}; updateMapEditorChrome=()=>{}; toast=()=>{};
    mapEditorInteractionLocked=()=>false;
    state.selectedMapPath='/map';state.selectedMapDetail={map:{path:'/map'},hd_map:{lanes:[{id:'lane',primary:true,closed_loop:false,boundary_mode:'paired',centerline_mode:'auto',left_bound:[[0,1],[2,1],[4,1]],right_bound:[[0,-1],[2,-1],[4,-1]],centerline:[[0,0],[2,0],[4,0]]}]},raster:{width:100,height:100,resolution_m_per_px:1}};
    ensureMapEditor(state.selectedMapDetail);state.mapEditor.enabled=true;
    canvasEventInfo=e=>({canvas:{width:100,height:100,setPointerCapture:()=>{}},point:e.point,hitRadius:0.1});
    mapPointProjector=()=>p=>p;mapPixelToWorld=(d,w,h,p)=>p;
    globalThis.eventAt=(point)=>({point,button:0,pointerId:1,preventDefault:()=>{}});
  `,ctx);
  return ctx;
}
const read=(c,s)=>JSON.parse(vm.runInContext(`JSON.stringify(${s})`,c));
test('physical boundary drag is independent from paired generation and centerline, including undo',()=>{
  const c=context();
  const before=read(c,'activeEditorLane()');
  vm.runInContext(`state.mapEditor.activeField='drivable_left_bound';handleMapEditorPointerDown(eventAt([2,1]));handleMapEditorPointerMove(eventAt([2,2]));handleMapEditorPointerUp(eventAt([2,2]));`,c);
  assert.equal(read(c,'activeEditorLane().drivable_left_bound')[1][1],2);
  assert.deepEqual(read(c,'activeEditorLane().centerline'),before.centerline);
  assert.deepEqual(read(c,'activeEditorLane().left_bound'),before.left_bound);
  c.undoMapEditor();assert.deepEqual(read(c,'activeEditorLane()'),before);
  vm.runInContext(`state.mapEditor.activeField='left_bound';handleMapEditorPointerDown(eventAt([2,1]));handleMapEditorPointerMove(eventAt([2,.5]));`,c);
  assert.deepEqual(read(c,'activeEditorLane().drivable_left_bound'),before.drivable_left_bound);
  assert.notDeepEqual(read(c,'activeEditorLane().centerline'),before.centerline);
});
test('polygon create, translate, edit metadata, delete and undo preserve obstacle data',()=>{
  const c=context();c.startMapObstacle();
  assert.match(c.mapObstacleEditIssue(),/描画/);
  for(const p of [[1,0],[2,0],[2,1],[1,1]]) c.handleMapObstacleDown({point:p,button:0,pointerId:1,preventDefault(){}});
  c.finishMapObstacle();
  assert.equal(c.mapObstacleEditIssue(),'');
  c.updateMapObstacle('name','段ボール');c.updateMapObstacle('height_m','0.4');
  const before=read(c,'selectedMapObstacle()');
  vm.runInContext(`handleMapObstacleDown(eventAt([1.5,.5]));handleMapObstacleMove(eventAt([2.5,.5]));handleMapEditorPointerUp(eventAt([2.5,.5]));`,c);
  assert.deepEqual(read(c,'selectedMapObstacle().polygon'),[[2,0],[3,0],[3,1],[2,1]]);
  c.undoMapEditor();assert.deepEqual(read(c,'selectedMapObstacle()'),before);
  c.redoMapEditor();c.deleteMapObstacle();assert.deepEqual(read(c,'state.mapEditor.obstacles'),[]);
  c.undoMapEditor();assert.equal(read(c,'selectedMapObstacle().name'),'段ボール');
  vm.runInContext(`state.selectedMapDetail={map:{path:'/other'},hd_map:{lanes:[]}};ensureMapEditor(state.selectedMapDetail);`,c);
  assert.deepEqual(read(c,'state.mapEditor.obstacles'),[]);
});
test('combined save includes both corridor pairs and obstacle polygons',async()=>{
  const c=context();
  vm.runInContext(`
    state.mapEditor.obstacles=[{id:'box',name:'box',polygon:[[1,0],[2,0],[1,1]],height_m:.3,margin_m:.1}];
    state.mapEditor.dirty=true;mapEditorSaveState=()=>({issue:''});confirmAction=()=>true;
    ensureSectionEditor=()=>{};ensureJunctionEditor=()=>{};ensureCustomLineEditor=()=>{};invalidateMapPreflights=()=>{};
    api=async(url,options)=>{globalThis.sent=JSON.parse(options.body);return {map:{path:'/map'},hd_map:{lanes:sent.lanes,obstacles:sent.obstacles}};};
  `,c);
  await c.saveHdMapFromEditor();
  assert.equal(c.sent.obstacles[0].id,'box');
  assert.deepEqual(JSON.parse(JSON.stringify(c.sent.lanes[0].drivable_left_bound)),[[0,1],[2,1],[4,1]]);
  assert.equal(read(c,'state.mapEditor.obstacles')[0].id,'box');
});
