const test=require('node:test');
const assert=require('node:assert/strict');
const S=require('../simulation_compare.js');
const N=require('../network_simulation.js');
const lane=(id,p,next=[],def='')=>({id,closed_loop:false,centerline:p,network_raceline:p,successor_ids:next,default_successor_id:def});
function input() {
  const lanes=[lane('entry',[[0,0],[4,0]],['a','b'],'a'),
    lane('a',[[4,0],[5,0],[6,.3],[7,.3],[8,0],[9,0]],['merge']),
    lane('b',[[4,0],[5,0],[6,-.3],[7,-.3],[8,0],[9,0]],['merge']),
    lane('merge',[[9,0],[13,0]],['c','d'],'c'),
    lane('c',[[13,0],[14,0],[15,.3],[16,0],[17,0]],['exit']),
    lane('d',[[13,0],[14,0],[15,-.3],[16,0],[17,0]],['exit']),
    lane('exit',[[17,0],[21,0]])];
  return {path:[{x:0,y:0},{x:4,y:0}],network:{lanes,initialLaneId:'entry',mode:'centerline'},
    duration:35,settings:{targetSpeedMps:1},controller:'all'};
}
function finish(session){let out=session.advance(0);while(!out.done)out=session.advance(20);return out;}

test('all controllers follow defaults through two forks and merges, including overlapping lanes',()=>{
  const results=finish(S.createSession(input())).frames;
  for(const r of results){assert.equal(r.status,'終点到達');assert.deepEqual(r.laneHistory,['entry','a','merge','c','exit']);}
});

test('live signal stops and resumes every controller without resetting positions or clocks',()=>{
  const session=S.createSession({...input(),signals:{entry:'stop'}});
  let out=session.advance(500);
  for(const r of out.frames){assert.equal(r.status,'信号待ち');assert.equal(r.car.laneId,'entry');assert.ok(r.car.x<3.72);assert.ok(r.car.speed<.01);}
  session.setSignals({entry:'lane:b',merge:'lane:d'});
  out=finish(session);
  for(const r of out.frames){assert.equal(r.status,'終点到達');assert.deepEqual(r.laneHistory,['entry','b','merge','d','exit']);assert.ok(r.time>10);}
});

test('a direction change after commitment never teleports a car into another branch',()=>{
  const session=S.createSession({...input(),controller:'pure_pursuit'});
  let out=session.advance(1);
  while(!out.frames[0].car.branchCommitted)out=session.advance(1);
  assert.equal(out.frames[0].car.laneId,'entry');
  session.setSignals({entry:'lane:b'});
  assert.deepEqual(finish(session).frames[0].laneHistory,['entry','a','merge','c','exit']);
});

test('existing Junction directions map through connector lanes and respect activation sections',()=>{
  const data=input();
  data.network.lanes.find(l=>l.id==='a').successor_ids=['merge'];
  data.network.sections=[{id:'approach',lane_id:'entry',start_s_m:2,end_s_m:4}];
  data.network.junctions=[{id:'j',signal_id:'signal_1',activation_section_ids:['approach'],branches:{left:'a',straight:'b',right:'unknown'}}];
  const network=N.create(data.network);
  assert.deepEqual(network.controls[0].options.map(o=>o.value),['left','straight']);
  network.setSignals({entry:'stop'});
  const tracker=network.tracker();
  assert.equal(tracker.update({x:1,y:0}).waiting,false);
  for(let x=1.25;x<=2.25;x+=.25)tracker.update({x,y:0});
  assert.equal(tracker.update({x:2.25,y:0}).waiting,true);
  const session=S.createSession({...data,signals:{entry:'straight'},controller:'pure_pursuit'});
  assert.deepEqual(finish(session).frames[0].laneHistory,['entry','b','merge','c','exit']);
});

test('missing defaults and missing generated candidates are rejected; invalid signals are atomic',()=>{
  const data=input();data.network.lanes[0].default_successor_id='';
  assert.throws(()=>S.createSession(data),/デフォルト分岐/);
  const other=input();other.network.mode='raceline';other.network.lanes[1].network_raceline=[];
  assert.throws(()=>S.createSession(other),/Raceline候補/);
  const session=S.createSession({...input(),signals:{entry:'lane:b'},controller:'pure_pursuit'});
  assert.throws(()=>session.setSignals({entry:'lane:exit'}),/利用できない信号/);
  assert.ok(finish(session).frames[0].laneHistory.includes('b'));
});

test('raceline network uses the chosen branch and retains its identity where lanes overlap exactly',()=>{
  const data=input();data.network.mode='raceline';
  data.network.lanes[2].centerline=data.network.lanes[1].centerline;
  data.network.lanes[2].network_raceline=data.network.lanes[1].network_raceline;
  for(const r of finish(S.createSession({...data,signals:{entry:'lane:b'}})).frames)assert.ok(r.laneHistory.includes('b'));
});

test('a red signal after passing the Junction stop line applies on the next visit',()=>{
  const data=input();data.network.sections=[{id:'gate',lane_id:'entry',start_s_m:1,end_s_m:2}];
  data.network.junctions=[{id:'j',signal_id:'signal',activation_section_ids:['gate'],branches:{left:'a',straight:'b'}}];
  const network=N.create(data.network),tracker=network.tracker();
  for(let x=0;x<=2.25;x+=.25)tracker.update({x,y:0});
  network.setSignals({entry:'stop'});
  const state=tracker.update({x:2.5,y:0});
  assert.equal(state.waiting,false);assert.equal(state.nextLaneId,'a');assert.equal(state.committed,true);
});

test('live UI signal changes preserve worker, time and geometry signature; topology edits clear stale signals',()=>{
  const fs=require('node:fs'),vm=require('node:vm'),path=require('node:path');
  const data=input(),posts=[];
  const ctx=vm.createContext({window:{},localStorage:{getItem:()=>null},console,setInterval:()=>{},NetworkSimulation:N,
    document:{getElementById:()=>null},data,posts});
  const source=fs.readFileSync(path.join(__dirname,'../app.js'),'utf8');
  vm.runInContext(source.slice(0,source.indexOf('\nwindow.')),ctx);
  vm.runInContext(`
    state.selectedMapDetail={map:{path:'/course'},hd_map:{lanes:data.network.lanes.map((l,i)=>({...l,primary:i===0})),sections:[],junctions:[]}};
    state.simulation.source='network_centerline';state.simulation.time=4.2;state.simulation.playing=true;
    const before=JSON.stringify(simulationComparisonInput());
    const activeWorker={postMessage:msg=>posts.push(msg)};simulationLive.worker=activeWorker;
    setSimulationNetworkSignal('entry','lane:b');
    if(JSON.stringify(simulationComparisonInput())!==before)throw new Error('signature changed');
    if(simulationLive.worker!==activeWorker)throw new Error('worker replaced');
    if(state.simulation.time!==4.2 || !state.simulation.playing)throw new Error('simulation reset');
  `,ctx);
  assert.equal(posts.length,1);assert.equal(posts[0].type,'signals');assert.equal(posts[0].signals.entry,'lane:b');
  assert.match(vm.runInContext('renderSimulationNetworkControls(state.selectedMapDetail)',ctx),/通常 → a/);
  vm.runInContext("state.selectedMapDetail.hd_map.lanes[0].default_successor_id='b';simulationNetworkInput();",ctx);
  assert.equal(vm.runInContext('Object.keys(simulationNetwork.signals).length',ctx),0);
});

test('Junction directions resolve an outgoing lane through an explicit connector',()=>{
  const data=input();
  const entry=data.network.lanes[0];entry.successor_ids=['connector','b'];entry.default_successor_id='connector';
  data.network.lanes.find(l=>l.id==='a').centerline=[[5,0],[6,.3],[7,.3],[8,0],[9,0]];
  data.network.lanes.push(lane('connector',[[4,0],[4.5,0],[5,0]],['a']));
  data.network.sections=[{id:'s',lane_id:'entry',start_s_m:1,end_s_m:4}];
  data.network.junctions=[{id:'j',activation_section_ids:['s'],branches:{left:'a',straight:'b'}}];
  const left=N.create(data.network).controls[0].options.find(o=>o.value==='left');
  assert.equal(left.laneId,'connector');
});

test('raceline error is measured against that candidate while lane identity uses centerline',()=>{
  const data=input();data.network.mode='raceline';
  data.network.lanes[0].network_raceline=[[0,0],[1,.5],[3,.5],[4,0]];
  const tracker=N.create(data.network).tracker();
  const state=tracker.update({x:1.5,y:.5});
  assert.equal(state.laneId,'entry');assert.ok(state.error<1e-9);
});
