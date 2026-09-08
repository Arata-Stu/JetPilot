const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');

function context() {
  const inputs = {
    'custom-line-create-name': { value: '走行ライン' },
    'custom-line-create-source': { value: 'centerline' },
    'custom-line-create-speed': { value: '1.5' },
  };
  const ctx = vm.createContext({ window: {}, localStorage: { getItem: () => null }, console,
    document: { getElementById: id => inputs[id] || null } });
  const source = fs.readFileSync(path.join(__dirname, '../app.js'), 'utf8');
  vm.runInContext(source.slice(0, source.indexOf('\nwindow.')), ctx);
  vm.runInContext(`
    state.selectedMapPath = '/map/test';
    state.selectedMapDetail = { map: { path: '/map/test' }, centerline_csv: { points: [[0,0], [1,0], [1,1]] } };
    toast = () => { throw new Error('Transient toast must not carry the result'); };
    render = () => { customLineCreateForm(state.selectedMapDetail); };
  `, ctx);
  return ctx;
}

test('API failure remains visible after final redraw and retains clone inputs', async () => {
  const ctx = context();
  vm.runInContext(`api = async () => { throw new Error('point outside drivable corridor'); };`, ctx);
  await ctx.createCustomLine();
  ctx.render();
  const form = vm.runInContext('state.customLineCreate', ctx);
  assert.equal(form.error, true);
  assert.match(form.message, /point outside drivable corridor/);
  assert.equal(form.name, '走行ライン');
  assert.equal(form.source, 'centerline');
  assert.equal(form.speed, '1.5');
  assert.equal(ctx.actionBusy('custom-line:create'), false);
  const markup = ctx.renderCustomLineEditor(vm.runInContext('state.selectedMapDetail', ctx));
  assert.match(markup, /role="alert"/);
  assert.match(markup, /point outside drivable corridor/);
  assert.match(markup, /value="走行ライン"/);
});

test('successful clone selects returned line and keeps a persistent success message', async () => {
  const ctx = context();
  vm.runInContext(`
    api = async () => ({...state.selectedMapDetail, custom_lines: [{id:'new', name:'走行ライン', points:[[0,0],[1,0],[1,1]], default_speed_mps:1.5}]});
  `, ctx);
  await ctx.createCustomLine();
  assert.equal(vm.runInContext('state.customLineEditor.selectedId', ctx), 'new');
  assert.equal(vm.runInContext('state.customLineCreate.error', ctx), false);
  assert.match(vm.runInContext('state.customLineCreate.message', ctx), /作成しました/);
});

test('switching maps clears previous form and an in-flight error stays on the old form', async () => {
  const ctx = context();
  let reject;
  ctx.api = () => new Promise((resolve, fail) => { reject = fail; });
  const pending = ctx.createCustomLine();
  vm.runInContext(`state.selectedMapPath='/other'; state.selectedMapDetail={map:{path:'/other'}}; render();`, ctx);
  reject(new Error('old map failure'));
  await pending;
  assert.equal(vm.runInContext('state.customLineCreate.message', ctx), '');
  assert.equal(vm.runInContext('state.customLineCreate.name', ctx), '');
});
