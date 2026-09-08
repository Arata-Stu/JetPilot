const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');

function context() {
  const ctx = vm.createContext({ window: {}, localStorage: { getItem: () => null }, console, setInterval: () => {} });
  const source = fs.readFileSync(path.join(__dirname, '../app.js'), 'utf8');
  vm.runInContext(source.slice(0, source.indexOf('\nwindow.')), ctx);
  vm.runInContext(fs.readFileSync(path.join(__dirname, '../live_tuning.js'), 'utf8'), ctx);
  vm.runInContext('render = () => {}; toast = () => {};', ctx);
  return ctx;
}

test('every existing page belongs to a purpose and internal navigation selects its group', () => {
  const ctx = context();
  const pages = vm.runInContext('workspaces.flatMap(w => w.pages.map(p => p[0]))', ctx);
  assert.equal(new Set(pages).size, 9);
  for (const page of pages) {
    vm.runInContext(`state.tab = ${JSON.stringify(page)}`, ctx);
    const nav = ctx.renderWorkspaceNavigation();
    assert.equal((nav.match(/class="active"/g) || []).length, 1);
    assert.match(ctx.renderWorkspaceSubnav(), /aria-current="page"/);
  }
});

test('E2E evaluation excludes training forms and switching preserves selected model', () => {
  const ctx = context();
  vm.runInContext(`
    renderE2EPipeline = () => 'TRAINING_FORM';
    renderE2EAnalysisForm = () => 'EVALUATION_FORM';
    renderAnalysisList = () => ''; renderAnalysisViewer = () => '';
    pauseAnalysisPlayback = () => { state.analysis.playing = false; };
    state.analysis.e2eModelPath = '/model.onnx'; state.analysis.playing = true;
  `, ctx);
  assert.match(ctx.renderE2EAnalysis(), /EVALUATION_FORM/);
  assert.doesNotMatch(ctx.renderE2EAnalysis(), /TRAINING_FORM/);
  ctx.setE2EWorkspace('train');
  assert.match(ctx.renderE2EAnalysis(), /TRAINING_FORM/);
  assert.doesNotMatch(ctx.renderE2EAnalysis(), /EVALUATION_FORM/);
  assert.equal(vm.runInContext('state.analysis.playing', ctx), false);
  ctx.setE2EWorkspace('evaluate');
  assert.equal(vm.runInContext('state.analysis.e2eModelPath', ctx), '/model.onnx');
});

test('live adjustment blocks draft entry and editing, then restores editing on exit', () => {
  const ctx = context();
  vm.runInContext(`
    state.tab = 'maps'; state.selectedMapPath = '/map';
    state.mapEditor.dirty = true;
    stopSimulationLoop = () => {}; jetsonTarget = () => ({host:'localhost',user:'test'});
    disconnectTuning = () => { tuning.connected = false; };
  `, ctx);
  ctx.toggleLiveTuning();
  assert.equal(vm.runInContext('tuning.enabled', ctx), false);
  vm.runInContext('state.mapEditor.dirty = false', ctx);
  ctx.toggleLiveTuning();
  assert.equal(ctx.mapEditorInteractionLocked(), true);
  ctx.toggleLiveTuning();
  assert.equal(ctx.mapEditorInteractionLocked(), false);
});

test('live workspace shows map and adjustment controls without editing or generation panels', () => {
  const ctx = context();
  vm.runInContext(`
    state.tab = 'maps'; state.selectedMapPath = '/map';
    state.selectedMapDetail = {map:{path:'/map', name:'test'}};
    tuning.enabled = true; tuning.mapPath = '/map';
    renderLiveTuning = () => 'LIVE_CONTROLS';
    renderMapInspectorTabs = () => 'EDIT_CONTROLS';
    renderMapWorkspaceModes = () => 'EDIT_MODES';
    renderMapStageButton = () => 'GENERATE_BUTTON';
    renderSimulationDisclosure = () => 'SIMULATION';
  `, ctx);
  const live = ctx.renderMapWorkspace();
  assert.match(live, /LIVE_CONTROLS/);
  assert.match(live, /map-preview-canvas/);
  assert.doesNotMatch(live, /EDIT_CONTROLS|EDIT_MODES|GENERATE_BUTTON|SIMULATION/);
  vm.runInContext('tuning.enabled = false', ctx);
  const edit = ctx.renderMapWorkspace();
  assert.match(edit, /EDIT_CONTROLS/);
  assert.match(edit, /GENERATE_BUTTON/);
  assert.doesNotMatch(edit, /LIVE_CONTROLS/);
});
