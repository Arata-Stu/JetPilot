const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const os=require('node:os');
const path=require('node:path');
const cp=require('node:child_process');
const vm=require('node:vm');
const api=require('../simulation_compare.js');
const straight=[{x:0,y:0},{x:1,y:0},{x:2,y:0},{x:10,y:0}];

test('all three methods use identical initial conditions and exact segment error',()=>{
 const results=api.run({path:straight,closed:false,offset:.2,yawOffset:.05,duration:2});
 assert.equal(results.length,3);
 for(const r of results){assert.deepEqual(r.trace[0],{x:0,y:.2});assert.equal(r.status,'時間終了');assert.ok(r.rmsError>=0);assert.ok(r.maxError>=.2);assert.ok(r.distance>0);}
 assert.equal(api.project([{x:0,y:0},{x:100,y:0}],50,1,false).d2,1);
 assert.notDeepEqual(results[0].trace,results[1].trace);
 assert.notDeepEqual(results[0].trace,results[2].trace);
});
test('deterministic run and custom zero speed profile',()=>{
 const input={path:straight.map(p=>({...p,speed_mps:0})),profile:true,duration:.4};
 const a=api.run(input),b=api.run(input);
 assert.deepEqual(a,b);
 for(const r of a) assert.equal(r.distance,0);
});
test('closed seam, open endpoint, and invalid inputs are handled',()=>{
 const p=[{x:0,y:0},{x:2,y:0},{x:2,y:2},{x:0,y:2}];
 assert.ok(Math.abs(api.project(p,-.1,1,true).d2-.01)<1e-12);
 assert.ok(api.project(p,-.1,1,false).d2>1);
});
test('goal and invalid settings',()=>{
 const r=api.run({path:[{x:0,y:0},{x:.1,y:0}],duration:1});
 assert.ok(r.every(x=>x.status==='終点到達' && x.rmsError===null));
 for(const input of [{path:straight,duration:NaN},{path:straight,settings:{mpcSteps:0}}, {path:straight,settings:{maxSteeringRad:2}}, {path:[{x:0,y:0},{x:0,y:0}]}]) assert.throws(()=>api.run(input));
});

test('browser equations match the actual C++ controllers',t=>{
 const probe=cp.spawnSync('clang++',['--version']);
 if(probe.error || probe.status!==0){t.skip('C++ compiler unavailable');return;}
 const root=path.resolve(__dirname,'../../../..');
 const dir=fs.mkdtempSync(path.join(os.tmpdir(),'jetpilot-controller-parity-'));
 try {
  const source=`#include <iostream>\n#include <iomanip>\n#include "jetpilot_controller/pure_pursuit.hpp"\n#include "jetpilot_controller/map_pursuit.hpp"\n#include "jetpilot_controller/kinematic_mpc.hpp"\nusing namespace jetpilot_controller;\nint main(){std::cout<<std::setprecision(17); for(double speed:{0.,1.,2.5}) {TrackingInput in;in.speed_mps=speed;in.path={{0.,.2},{.5,.2},{1.,.4},{2.,1.},{3.,1.}};in.path_closed_override=false;PurePursuit a{PurePursuitParams{}};MapPursuit b{MapPursuitParams{}};KinematicMpc c{KinematicMpcParams{}}; for(auto* ctrl: {static_cast<PathTrackingController*>(&a),static_cast<PathTrackingController*>(&b),static_cast<PathTrackingController*>(&c)}){auto result=ctrl->compute(in);std::cout<<result.valid<<" "<<result.steering_command*.45<<"\\n";}}}`;
  fs.writeFileSync(path.join(dir,'main.cpp'),source);
  const src=path.join(root,'ros2_ws/src/control/jetpilot_controller');
  cp.execFileSync('clang++',['-std=c++17','-O2','-I'+path.join(src,'include'),path.join(dir,'main.cpp'),...['pure_pursuit','map_pursuit','kinematic_mpc'].map(n=>path.join(src,'src',n+'.cpp')),'-o',path.join(dir,'parity')]);
  const rows=cp.execFileSync(path.join(dir,'parity'),{encoding:'utf8'}).trim().split('\n');
  let i=0;
  for(const speed of [0,1,2.5]) for(const m of api.methods){
   const [valid,expected]=rows[i++].split(' ').map(Number);
   const actual=api.control(m.id,[{x:0,y:.2},{x:.5,y:.2},{x:1,y:.4},{x:2,y:1},{x:3,y:1}],{x:0,y:0,yaw:0,speed},api.defaults,false);
   assert.equal(valid,1);assert.ok(Math.abs(actual-expected)<1e-12,`${m.id} ${speed}: ${actual} != ${expected}`);
  }
 } finally {fs.rmSync(dir,{recursive:true,force:true});}
});

function ui(){
 class Worker {constructor(){this.terminated=false;this.messages=[];Worker.instances.push(this);}postMessage(x){this.input=x;this.messages.push(x);}terminate(){this.terminated=true;}}
 Worker.instances=[];
 const ctx=vm.createContext({setTimeout,clearTimeout,Worker,window:{},localStorage:{getItem:()=>null},document:{querySelectorAll:()=>[],getElementById:()=>null},console,cancelAnimationFrame:()=>{}});
 const source=fs.readFileSync(path.join(__dirname,'../app.js'),'utf8');vm.runInContext(source.slice(0,source.indexOf('\nwindow.')),ctx);
 vm.runInContext(`drawSimulationPreview=()=>{};updateSimulationChrome=()=>{};updateSimulationComparisonChrome=()=>{};simulationPathPoints=()=>[{x:0,y:0},{x:10,y:0}];`,ctx);
 return {ctx,Worker};
}
test('comparison UI cancels workers and rejects obsolete results',()=>{
 const {ctx,Worker}=ui();ctx.startSimulationComparison();
 const a=Worker.instances[0];assert.equal(a.input.duration,20);
 ctx.cancelSimulationComparison();assert.ok(a.terminated);
 a.onmessage({data:{results:[{name:'obsolete'}]}});
 assert.equal(vm.runInContext('simulationComparison.results',ctx),null);
 ctx.startSimulationComparison();const b=Worker.instances[1];
 vm.runInContext('state.simulation.settings.targetSpeedMps=3',ctx);
 b.onmessage({data:{results:[{name:'wrong settings'}]}});
 assert.equal(vm.runInContext('simulationComparison.results',ctx),null);
 assert.ok(b.terminated);
});

test('existing decimal defaults do not block compare because of HTML step mismatch',()=>{
 const {ctx,Worker}=ui();
 ctx.document.querySelectorAll=()=>[
  {value:'0.45',min:'0.01',max:'',disabled:false,validity:{stepMismatch:true,badInput:false,customError:false}},
  {value:'Custom profile',disabled:true},
 ];
 ctx.startSimulationComparison();assert.equal(Worker.instances.length,1);
 ctx.cancelSimulationComparison();
 ctx.document.querySelectorAll=()=>[{value:'',min:'0.01',max:'',disabled:false,validity:{}}];
 ctx.startSimulationComparison();assert.equal(Worker.instances.length,1);
 assert.match(vm.runInContext('simulationComparison.error',ctx),/入力値/);
});

test('changing conditions clears completed comparison and reset cancels a running comparison',()=>{
 const {ctx,Worker}=ui();ctx.startSimulationComparison();
 Worker.instances[0].onmessage({data:{results:[{name:'completed'}]}});
 assert.notEqual(vm.runInContext('simulationComparison.results',ctx),null);
 ctx.updateSimulationComparisonOption('offset',{value:'.25',min:'-5',max:'5',setCustomValidity:()=>{}});
 assert.equal(vm.runInContext('simulationComparison.results',ctx),null);
 ctx.startSimulationComparison();ctx.resetSimulationStateFromPath({});
 assert.ok(Worker.instances.at(-1).terminated);
});

test('live sessions start at zero, advance all cars at the same clock and match batch results',()=>{
 const input={path:straight,closed:false,duration:.4,offset:.1,controller:'all'};
 const live=api.createSession(input);
 assert.ok(live.advance(0).frames.every(f=>f.time===0 && f.trace.length===1));
 const tick=live.advance(5);
 assert.ok(tick.frames.every(f=>Math.abs(f.time-.1)<1e-12 && f.trace.length===6));
 let result=tick;
 while(!result.done) result=live.advance();
 assert.deepEqual(result.frames,api.run(input));
 for(const controller of ['pure_pursuit','map_pursuit','kinematic_mpc']) {
  const one=api.createSession({...input,controller}).advance(3);
  assert.equal(one.frames.length,1);assert.equal(one.frames[0].id,controller);
  assert.ok(one.frames[0].time>0);
 }
});

test('Run uses the selected controller; Pause preserves the worker and Step resumes that state',()=>{
 const {ctx,Worker}=ui();ctx.requestAnimationFrame=()=>1;
 vm.runInContext("state.simulation.controller='kinematic_mpc'",ctx);
 ctx.toggleSimulationPlayback();
 const w=Worker.instances[0];assert.equal(w.input.type,'start');assert.equal(w.input.input.controller,'kinematic_mpc');
 const session=api.createSession(w.input.input);
 w.onmessage({data:session.advance(0)});
 ctx.simulationPlaybackTick(100);ctx.simulationPlaybackTick(200);
 assert.equal(w.input.type,'step');
 w.onmessage({data:session.advance(w.input.count)});
 const time=vm.runInContext('state.simulation.time',ctx);assert.ok(time>0);
 ctx.toggleSimulationPlayback();assert.equal(vm.runInContext('state.simulation.playing',ctx),false);assert.equal(w.terminated,false);
 const sent=w.messages.length;ctx.simulationPlaybackTick(300);assert.equal(w.messages.length,sent);
 ctx.stepSimulationOnce();assert.equal(w.input.count,5);
 w.onmessage({data:session.advance(5)});
 assert.ok(Math.abs(vm.runInContext('state.simulation.time',ctx)-time-.1)<1e-12);
 ctx.setSimulationController('map_pursuit');assert.ok(w.terminated);
 ctx.toggleSimulationPlayback();assert.equal(Worker.instances.at(-1).input.input.controller,'map_pursuit');
});

test('initial Step waits for initialization then advances without starting playback',()=>{
 const {ctx,Worker}=ui();ctx.stepSimulationOnce();const w=Worker.instances[0];
 const session=api.createSession(w.input.input);
 assert.equal(w.messages.length,1);
 w.onmessage({data:session.advance(0)});assert.equal(w.input.type,'step');assert.equal(w.input.count,5);
 w.onmessage({data:session.advance(5)});
 assert.equal(vm.runInContext('state.simulation.playing',ctx),false);
 assert.ok(Math.abs(vm.runInContext('state.simulation.time',ctx)-.1)<1e-12);
});

test('controller-specific controls follow selection and expose both in comparison',()=>{
 const {ctx}=ui();
 assert.equal(ctx.renderSimulationControllerSettings(),'');
 ctx.setSimulationController('map_pursuit');
 assert.match(ctx.renderSimulationControllerSettings(),/simulation-mapLateralGain/);
 assert.doesNotMatch(ctx.renderSimulationControllerSettings(),/simulation-mpcSteps/);
 ctx.setSimulationController('kinematic_mpc');
 assert.match(ctx.renderSimulationControllerSettings(),/simulation-mpcSteps/);
 assert.doesNotMatch(ctx.renderSimulationControllerSettings(),/simulation-mapLateralGain/);
 ctx.setSimulationController('all');
 const html=ctx.renderSimulationControllerSettings();
 assert.match(html,/simulation-mapScaleFactor/);assert.match(html,/simulation-mpcTerminalWeight/);
 assert.match(ctx.simulationMpcHorizonText(),/0.6秒/);
});

test('parameter edits reset live state, reach the worker, and reset only their own controller',()=>{
 const {ctx,Worker}=ui();ctx.requestAnimationFrame=()=>1;
 ctx.setSimulationController('all');ctx.toggleSimulationPlayback();const previous=Worker.instances[0];
 const input=value=>({value:String(value),min:'0',setCustomValidity:()=>{}});
 ctx.updateSimulationSetting('mapLateralGain',input(.9));assert.ok(previous.terminated);
 ctx.updateSimulationSetting('mpcSteps',input(20));
 ctx.updateSimulationSetting('mpcDt',input(.1));
 assert.match(ctx.simulationMpcHorizonText(),/2秒/);
 ctx.toggleSimulationPlayback();const settings=Worker.instances.at(-1).input.input.settings;
 assert.equal(settings.mapLateralGain,.9);assert.equal(settings.mpcSteps,20);assert.equal(settings.mpcDt,.1);
 ctx.resetSimulationControllerParameters('kinematic_mpc');
 const current=ctx.simulationComparisonInput().settings;
 assert.equal(current.mpcSteps,12);assert.equal(current.mpcDt,.05);assert.equal(current.mapLateralGain,.9);
 assert.equal(vm.runInContext('state.simulation.playing',ctx),false);
});

test('parameter inputs reject blank, fractional count, NaN and out-of-range values',()=>{
 const {ctx}=ui();
 for(const [key,value] of [['mpcSteps','1.5'],['mpcSteps','51'],['mpcSamples','2'],['mpcDt','0'],['mapScaleFactor','1.1'],['mapLateralGain',''],['mpcPathWeight','NaN']]){
  const before=ctx.simulationComparisonInput().settings[key];let error='';
  ctx.updateSimulationSetting(key,{value,min:'0',setCustomValidity:message=>error=message});
  assert.ok(error,key);assert.equal(ctx.simulationComparisonInput().settings[key],before);
 }
});

test('Map and MPC custom parameters actually change steering and defaults match the engine',()=>{
 const {ctx}=ui();const settings=ctx.simulationComparisonInput().settings;
 for(const [key] of vm.runInContext('Object.values(simulationControllerFields).flat()',ctx)) assert.equal(settings[key],api.defaults[key]);
 const path=[{x:0,y:.3},{x:1,y:.3},{x:2,y:1},{x:3,y:2}];const car={x:0,y:0,yaw:0,speed:2};
 assert.notEqual(api.control('map_pursuit',path,car,api.defaults,false),api.control('map_pursuit',path,car,{...api.defaults,mapLateralGain:2},false));
 assert.notEqual(api.control('kinematic_mpc',path,car,api.defaults,false),api.control('kinematic_mpc',path,car,{...api.defaults,mpcSteeringWeight:10000},false));
});

const turn=()=>new Promise(resolve=>setImmediate(resolve));
test('Optuna UI evaluates candidates without changing current settings and applies only on request',async()=>{
 const {ctx,Worker}=ui();
 const settings={...ctx.simulationComparisonInput().settings};let calls=0;
 ctx.api=async(url,options)=>{
  const body=JSON.parse(options.body);
  if(url.endsWith('/stop'))return {};
  if(url.endsWith('/start'))return {session_id:'study',sequence:0,settings,completed:0,trials:5,is_baseline:true};
  if(calls++===0)return {session_id:'study',sequence:1,settings:{...settings,minLookaheadM:.8},completed:0,trials:5,baseline:{score:.2},is_baseline:false};
  return {session_id:'study',sequence:1,completed:5,trials:5,done:true,baseline:{score:.2},best:{sequence:1,score:.1,settings:{...settings,minLookaheadM:.8}}};
 };
 const run=ctx.startSimulationOptimization();await turn();
 assert.equal(Worker.instances.length,1);
 const frame={rmsError:.2,maxError:.3,steeringRate:.1,distance:10,time:20,status:'時間終了'};
 Worker.instances[0].onmessage({data:{results:[frame]}});await turn();
 assert.equal(Worker.instances[1].input.settings.minLookaheadM,.8);
 assert.equal(ctx.simulationComparisonInput().settings.minLookaheadM,settings.minLookaheadM);
 Worker.instances[1].onmessage({data:{results:[{...frame,rmsError:.1}]}});await run;
 assert.match(ctx.renderSimulationOptimization(),/探索完了/);
 ctx.applySimulationOptimization();assert.equal(ctx.simulationComparisonInput().settings.minLookaheadM,.8);
});

test('late Optuna start responses are discarded and sessions released after cancellation',async()=>{
 const {ctx}=ui();let release;let stopped='';
 ctx.api=(url,options)=>url.endsWith('/start')?new Promise(resolve=>release=resolve):(stopped=JSON.parse(options.body).session_id,Promise.resolve({}));
 const pending=ctx.startSimulationOptimization();ctx.stopSimulationOptimization(true);
 release({session_id:'late'});await pending;
 assert.equal(stopped,'late');assert.equal(vm.runInContext('simulationOptimization.result',ctx),null);
});

test('missing Optuna dependency is shown without changing simulation parameters',async()=>{
 const {ctx}=ui();ctx.api=async()=>{throw new Error('Optunaがありません');};
 await ctx.startSimulationOptimization();
 assert.match(ctx.renderSimulationOptimization(),/Optunaがありません/);
 assert.equal(vm.runInContext('simulationOptimization.active',ctx),false);
});
