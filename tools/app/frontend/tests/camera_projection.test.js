const {test}=require('node:test');
const assert=require('node:assert/strict');
const P=require('../camera_projection.js');
const model={width:640,height:480,k:[500,0,320,0,500,240,0,0,1],d:[0,0,0,0,0],r:[1,0,0,0,1,0,0,0,1],p:[500,0,320,0,0,500,240,0,0,0,1,0],distortion_model:'plumb_bob',binning_x:1,binning_y:1,roi:{x_offset:0,y_offset:0,width:0,height:0}};
const identity=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1];
test('known optical point projects to a calibrated pixel and resized image',()=>{
  assert.deepEqual(P.pixel([0,1,10],model,'raw',640,480),[320,290]);
  assert.deepEqual(P.pixel([0,1,10],model,'raw',320,240),[160,145]);
});
test('raw distortion and rectified P/R are distinct, without double stereo baseline',()=>{
  const distorted={...model,d:[0.1,0,0,0,0]};
  assert.ok(P.pixel([1,0,2],distorted,'raw',640,480)[0]>570);
  const right={...model,p:[400,0,300,-80,0,400,200,0,0,0,1,0]};
  assert.deepEqual(P.pixel([0,0,2],right,'rectified',640,480),[300,200]);
  const rotated={...model,r:[0,-1,0,1,0,0,0,0,1]};
  assert.deepEqual(P.cameraPoint([1,0,2],rotated,'rectified'),[0,1,2]);
});
test('ROI and binning project in extracted image coordinates',()=>{
  const cropped={...model,binning_x:2,binning_y:2,roi:{x_offset:100,y_offset:40,width:400,height:320}};
  assert.equal(P.issue(cropped,'raw',200,160),'');
  assert.deepEqual(P.pixel([0,0,2],cropped,'raw',200,160),[110,100]);
  assert.notEqual(P.issue(cropped,'raw',640,480),'');
});
test('fisheye centre is finite and unsupported calibration is rejected',()=>{
  assert.notEqual(P.issue({...model,d:null},'raw',640,480),'');
  const fish={...model,distortion_model:'equidistant',d:[0,0,0,0]};
  assert.deepEqual(P.pixel([0,0,2],fish,'raw',640,480),[320,240]);
  assert.notEqual(P.issue({...model,distortion_model:'unsupported'},'raw',640,480),'');
  assert.notEqual(P.issue({...model,k:[0,0,320,0,0,240,0,0,1]},'raw',640,480),'');
});
test('near-plane crossing is clipped, rear and out-of-image segments disappear',()=>{
  assert.deepEqual(P.segments([[0,0,-1],[0,0,-2]],false,model,identity,'raw',640,480),[]);
  const segments=P.segments([[-1,0,-1],[1,0,5]],false,model,identity,'raw',640,480);
  assert.ok(segments.length>0);
  assert.ok(segments.flat().every(([x,y])=>x>=-1e-8&&x<=640+1e-8&&y>=-1e-8&&y<=480+1e-8));
  assert.equal(P.clip([-20,0],[-10,10],640,480),null);
});

const vm=require('node:vm'),fs=require('node:fs'),path=require('node:path');
function uiContext(){
  const context=vm.createContext({CameraProjection:P,state:{mapEditor:{mapPath:'/map',lanes:[]},customLineEditor:{}},console});
  vm.runInContext(fs.readFileSync(path.join(__dirname,'../camera_overlay_ui.js'),'utf8'),context);
  return context;
}
test('overlay permits HD drafts with same localization map but refuses mismatched map and old results',()=>{
  const c=uiContext();
  c.timeline={map:{path:'/map'},camera_projection:{localization_fingerprint:'same'}};
  c.detail={map:{path:'/map',fingerprint:'changed-hd-file'},localization_fingerprint:'same'};
  assert.equal(vm.runInContext('cameraProjectionMapIssue(timeline,detail)',c),'');
  c.detail.localization_fingerprint='changed-map';
  assert.notEqual(vm.runInContext('cameraProjectionMapIssue(timeline,detail)',c),'');
  c.timeline={map:{path:'/map'}};
  assert.match(vm.runInContext('cameraProjectionMapIssue(timeline,detail)',c),/再解析/);
});
test('draft lane overlay uses current unsaved points rather than saved geometry',()=>{
  const c=uiContext();
  c.state.mapEditor.lanes=[{left_bound:[[1,2],[3,4]],right_bound:[],centerline:[]}];
  c.detail={map:{path:'/map'},hd_map:{lanes:[{left_bound:[[9,9],[8,8]],right_bound:[],centerline:[]}]}};
  assert.equal(vm.runInContext('cameraOverlayLines(detail,true)[0].points[0][0]',c),1);
  assert.equal(vm.runInContext('cameraOverlayLines(detail,false)[0].points[0][0]',c),9);
});
test('ground TF initializes height including negative Z, while old zero metadata stays unconfirmed',()=>{
  const c=uiContext();c.state.analysis={};
  c.timeline={camera_projection:{ground_z_m:-0.12,ground_z_source:'base_footprint_tf'}};
  vm.runInContext("initializeAnalysisGroundHeight(timeline,'one')",c);
  assert.equal(c.state.analysis.projectionHeightM,-0.12);
  c.state.analysis.projectionHeightM=-0.2;
  vm.runInContext("initializeAnalysisGroundHeight(timeline,'one')",c);
  assert.equal(c.state.analysis.projectionHeightM,-0.2);
  c.timeline={camera_projection:{ground_z_m:0}};
  assert.equal(vm.runInContext('cameraGroundHeightDefault(timeline)',c),null);
  vm.runInContext("initializeAnalysisGroundHeight(timeline,'two')",c);
  assert.equal(c.state.analysis.projectionHeightM,0);
  assert.match(vm.runInContext('cameraGroundHeightHint(timeline)',c),/仮/);
});
test('estimated ground initializes Z with an explicit provisional label',()=>{
  const c=uiContext();
  c.timeline={camera_projection:{ground_z_m:-0.08,ground_z_source:'tt02_ground_estimate_tf'}};
  assert.equal(vm.runInContext('cameraGroundHeightDefault(timeline)',c),-0.08);
  assert.match(vm.runInContext('cameraGroundHeightHint(timeline)',c),/推定/);
});
