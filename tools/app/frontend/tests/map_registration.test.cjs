const {test} = require('node:test');
const assert = require('node:assert/strict');
const vm = require('node:vm');
const {readAppSource} = require('./console_source.cjs');

function context() {
  const ctx = vm.createContext({window:{}, localStorage:{getItem:()=>null}, console,
    document:{getElementById:()=>null}});
  const source = readAppSource();
  vm.runInContext(source.slice(0, source.indexOf('\nwindow.')), ctx);
  vm.runInContext(`render=()=>{}; toast=()=>{}; tuningVisible=()=>false;
    state.selectedMapPath='/maps/new';state.selectedMapDetail={map:{path:'/maps/new'}};`, ctx);
  return ctx;
}

test('matching a known pose accounts for rotation about the old map origin',()=>{
  const c=context();
  const tf=c.registrationFromPoses([2,3,20],[10,5,110]);
  assert.ok(Math.abs(tf.x_m-13)<1e-10);
  assert.ok(Math.abs(tf.y_m-3)<1e-10);
  assert.equal(tf.yaw_deg,90);
  assert.throws(()=>c.registrationFromPoses([NaN,0,0],[0,0,0]));
});

test('changing alignment invalidates preview, changing map resets the form',()=>{
  const c=context();
  vm.runInContext(`registrationFor(state.selectedMapDetail).preview={preview_token:'old'};`,c);
  c.updateMapRegistration('yaw_deg','45');
  assert.equal(vm.runInContext('state.mapRegistration.preview',c),null);
  assert.equal(vm.runInContext('state.mapRegistration.yaw_deg',c),'45');
  c.registrationFor({map:{path:'/maps/another'}});
  assert.equal(vm.runInContext('state.mapRegistration.source_map_dir',c),'');
  assert.equal(vm.runInContext('state.mapRegistration.yaw_deg',c),'0');
});

test('preview arriving after navigation cannot overwrite a different map',async()=>{
  const c=context();
  let complete;
  c.api=()=>new Promise(resolve=>{complete=resolve;});
  vm.runInContext(`hasUnsavedMapEdits=()=>false;
    registrationFor(state.selectedMapDetail).source_map_dir='/maps/source';`,c);
  const pending=c.previewMapRegistration();
  vm.runInContext(`state.selectedMapPath='/maps/another';state.selectedMapDetail={map:{path:'/maps/another'}};
    registrationFor(state.selectedMapDetail);`,c);
  complete({preview_token:'wrong map'});
  await pending;
  assert.equal(vm.runInContext('state.mapRegistration.preview',c),null);
});
