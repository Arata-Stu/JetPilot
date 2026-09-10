const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const source = fs.readFileSync(path.join(__dirname, '../runtime.js'), 'utf8');
const store = new Map();
let calls = [];
const context = vm.createContext({
  localStorage: {getItem:k=>store.get(k),setItem:(k,v)=>store.set(k,v)},
  esc: x=>String(x??'').replaceAll('<','&lt;').replaceAll('"','&quot;'),
  state: {tab:'runtime'}, render:()=>{}, Date, setInterval:()=>{}, document:{querySelector:()=>null},
  api: async (...args)=>{calls.push(args);return {state:'running',message:'sent'};},
});
vm.runInContext(source, context);
assert.match(vm.runInContext('renderRuntime()',context), /環境を準備/);
assert.match(vm.runInContext('renderRuntime()',context), /runtime-connect-card/);
assert.match(vm.runInContext('renderRuntime()',context), /runtime-launch-card/);
assert.match(vm.runInContext('renderRuntime()',context), /runtime-record-card/);
assert.match(vm.runInContext('renderRuntime()',context), /runtime-tune-card/);
assert.equal(calls.length,0,'render must not access the Jetson');
vm.runInContext("runtimeChange('preset','e2e',true)",context);
assert.match(vm.runInContext('renderRuntime()',context), /モデルディレクトリ/);
vm.runInContext("runtimeChange('sensor','event-camera',true)",context);
assert.match(vm.runInContext('renderRuntime()',context), /EVS蓄積窓/);
assert.match(vm.runInContext('renderRuntime()',context), /約100\.0 Hz/);
vm.runInContext("runtimeTune.sensor='evs'",context);
assert.match(vm.runInContext('renderRuntime()',context), /蓄積設定を適用/);
vm.runInContext("runtimeChange('preset','offline-vslam',true)",context);
const offline = vm.runInContext('renderRuntime()',context);
assert.match(offline,/bagのパス/);
assert.doesNotMatch(offline,/記録開始/);
assert.equal(JSON.parse(store.get('jetpilot-runtime-v1')).preset,'offline-vslam');
vm.runInContext("runtimeTuneTarget('/path_tracking_controller_node')",context);
assert.match(vm.runInContext('renderRuntime()',context),/kinematic_mpc/);
assert.match(vm.runInContext('renderRuntime()',context),/最小lookahead/);
vm.runInContext("runtimeTuneField('throttle_kp')",context);
assert.equal(vm.runInContext('runtimeTune.parameter',context),'throttle_kp');
(async()=>{
  await vm.runInContext("runtimeAction('status')",context);
  assert.equal(calls[0][0],'/api/runtime/status');
  assert.equal(JSON.parse(calls[0][1].body).preset,'offline-vslam');
  context.api = async()=>{throw new Error('SSH unavailable');};
  await vm.runInContext("runtimeAction('status')",context);
  assert.match(vm.runInContext('renderRuntime()',context),/SSH unavailable/);
  assert.equal(vm.runInContext('runtimeBusy',context),false);
  console.log('PASS: initial display, conditional fields, persistence, explicit API action, failure recovery');
})().catch(e=>{console.error(e);process.exitCode=1;});
