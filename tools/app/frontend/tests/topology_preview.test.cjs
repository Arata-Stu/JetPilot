const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');
function setup(){
 const canvas={width:100,height:100,setPointerCapture(){}};
 const c=vm.createContext({window:{},localStorage:{getItem:()=>null},console,document:{getElementById:()=>canvas}});
 const src=fs.readFileSync(path.join(__dirname,'../app.js'),'utf8');
 vm.runInContext(src.slice(0,src.indexOf('\nwindow.')),c);
 vm.runInContext(`
 render=()=>{};drawMapPreview=()=>{};updateSectionEditorChrome=()=>{};mapEditorInteractionLocked=()=>false;
 mapPointProjector=()=>p=>p;mapPixelToWorld=(d,w,h,p)=>p;
 canvasEventInfo=e=>({canvas:{width:100,height:100},point:e.point,hitRadius:.1});
 state.mapWorkspaceMode='topology';state.mapTopologyTool='sections';
 state.selectedMapDetail={map:{path:'/map'},hd_map:{primary_lane_id:'lane'}};
 Object.assign(state.sectionEditor,{enabled:true,mapPath:'/map',gates:[],dirty:false});
 sectionEditorLane=()=>({id:'lane',closed_loop:false,centerline:[[0,0],[10,0]],left_bound:[[0,1],[10,1]],right_bound:[[0,-1],[10,-1]]});
 sectionGatesForDetail=()=>state.sectionEditor.gates;markSectionEditorDirty=()=>state.sectionEditor.dirty=true;
 markJunctionEditorDirty=()=>state.junctionEditor.dirty=true;junctionsForDetail=()=>state.junctionEditor.junctions;
 `,c);
 return c;
}
const read=(c,code)=>JSON.parse(vm.runInContext(`JSON.stringify(${code})`,c));
const event=point=>({point,button:0,preventDefault(){}});
test('section hover shows interpolated placement and commits precisely the same gate without hover mutations',()=>{
 const c=setup();c.handleMapEditorPointerMove(event([4.2,.1]));
 const candidate=c.topologyPlacementCandidate(read(c,'state.selectedMapDetail'),{canvas:{width:100,height:100},point:[4.2,.1],hitRadius:.1});
 assert.equal(candidate.kind,'gate');assert.equal(candidate.projection.s_m,4.2);
 assert.deepEqual(read(c,'state.sectionEditor.gates'),[]);assert.equal(read(c,'state.sectionEditor.dirty'),false);
 c.handleSectionEditorPointerDown(event([4.2,.1]));
 const gate=read(c,'state.sectionEditor.gates[0]');
 assert.deepEqual(gate.line,JSON.parse(JSON.stringify(candidate.line)));assert.equal(gate.s_m,4.2);
 c.handleMapEditorPointerMove(event([4.2,0]));
 const selected=c.topologyPlacementCandidate(read(c,'state.selectedMapDetail'),{canvas:{width:100,height:100},point:[4.2,0],hitRadius:.1});
 assert.equal(selected.kind,'select-gate');c.handleSectionEditorPointerDown(event([4.2,0]));assert.equal(read(c,'state.sectionEditor.gates.length'),1);
});
test('out of range, pointer leave, wrong map and disabled editing do not retain a preview',()=>{
 const c=setup();c.handleMapEditorPointerMove(event([4,.1]));assert.ok(read(c,'topologyPlacementHover'));
 c.handleMapEditorPointerUp({type:'pointerleave'});assert.equal(read(c,'topologyPlacementHover'),null);
 c.handleMapEditorPointerMove(event([4,2]));assert.equal(read(c,'topologyPlacementHover'),null);
 vm.runInContext("state.sectionEditor.mapPath='/other'",c);
 c.handleMapEditorPointerMove(event([4,.1]));assert.equal(read(c,'topologyPlacementHover'),null);
});
test('junction placement is free position, previews the exact click, and existing markers select',()=>{
 const c=setup();vm.runInContext(`state.sectionEditor.enabled=false;state.mapTopologyTool='junctions';Object.assign(state.junctionEditor,{enabled:true,mapPath:'/map',placing:true,dragging:false,selectedJunctionId:'j',junctions:[{id:'j',position:[0,0]}],dirty:false});`,c);
 c.handleMapEditorPointerMove(event([4,3]));assert.equal(read(c,'state.junctionEditor.dirty'),false);
 const candidate=c.topologyPlacementCandidate(read(c,'state.selectedMapDetail'),{canvas:{width:100,height:100},point:[4,3],hitRadius:.1});assert.equal(candidate.kind,'junction');
 c.handleJunctionEditorPointerDown(event([4,3]));assert.deepEqual(read(c,'selectedJunction().position'),[4,3]);
 c.handleMapEditorPointerMove(event([4,3]));
 assert.equal(c.topologyPlacementCandidate(read(c,'state.selectedMapDetail'),{canvas:{width:100,height:100},point:[4,3],hitRadius:.1}).kind,'select-junction');
});
test('closed centerline preview includes last-to-first seam',()=>{
 const c=setup();vm.runInContext(`sectionEditorLane=()=>({id:'loop',closed_loop:true,centerline:[[0,0],[1,0],[1,1],[0,1]]});`,c);
 const result=c.topologyPlacementCandidate(read(c,'state.selectedMapDetail'),{canvas:{width:100,height:100},point:[-.05,.5],hitRadius:.1});
 assert.equal(result.kind,'gate');assert.equal(result.projection.s_m,3.5);assert.deepEqual(Array.from(result.point),[0,.5]);
});
