const { test } = require('node:test');
const assert = require('node:assert/strict');
const G = require('../lane_geometry.js');
const lane = (points, closed = false) => ({ id: 'a', boundary_mode: 'paired', centerline_mode: 'auto', closed_loop: closed, ...G.fromSpine(points, 2) });
test('paired lane keeps requested width and center at every station', () => {
  const points = [[0, 0], [2, 0], [3, 2], [4, 3]];
  const a = lane(points);
  assert.deepEqual(G.centers(a), points);
  a.left_bound.forEach((p, i) => assert.ok(Math.abs(Math.hypot(p[0]-a.right_bound[i][0], p[1]-a.right_bound[i][1])-2) < 1e-10));
  assert.ok(a.left_bound[0][1] > a.right_bound[0][1]);
});
test('split and join round trip preserves open and closed geometry', () => {
  for (const closed of [false, true]) {
    const a = lane([[0, 0], [4, 0], [4, 4], [0, 4]], closed);
    const [first, second] = G.split(a, 2, 'b');
    assert.equal(first.closed_loop, false);
    assert.deepEqual(first.left_bound.at(-1), second.left_bound[0]);
    assert.deepEqual(G.join(first, second), a);
    assert.equal(a.left_bound.length, 4);
  }
});
test('rejects manual lines, unpaired boundaries, endpoints and distant joins', () => {
  const a = lane([[0, 0], [2, 0], [4, 0]]);
  assert.throws(() => G.split({...a, centerline_mode: 'manual'}, 1, 'b'));
  assert.throws(() => G.split({...a, boundary_mode: 'independent'}, 1, 'b'));
  assert.throws(() => G.split(a, 0, 'b'));
  assert.throws(() => G.split(a, 2, 'b'));
  assert.throws(() => G.join(a, {...lane([[10, 0], [12, 0]]), id: 'b'}));
  assert.throws(() => G.fromSpine([[0, 0]], NaN));
});

const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
function editorContext() {
  const context = vm.createContext({ window: {}, localStorage: {getItem: () => null}, console, LaneGeometry: G });
  const source = fs.readFileSync(path.join(__dirname, '../app.js'), 'utf8');
  vm.runInContext(source.slice(0, source.indexOf('\nwindow.')), context);
  vm.runInContext(`
    updateMapEditorChrome = () => {};
    drawMapPreview = () => {};
    render = () => {};
    state.mapEditor.enabled = true;
    state.mapEditor.lanes = [{id:'a', closed_loop:false, boundary_mode:'paired', centerline_mode:'auto', left_bound:[[0,1],[2,1],[4,1]], right_bound:[[0,-1],[2,-1],[4,-1]], centerline:[[0,0],[2,0],[4,0]]}];
    state.mapEditor.activeLaneId = 'a'; state.mapEditor.primaryLaneId = 'a';
    state.mapEditor.undoStack = []; state.mapEditor.redoStack = [];
  `, context);
  return context;
}
test('manual centerline survives boundary updates and undo/redo restores mode', () => {
  const c = editorContext();
  vm.runInContext(`setManualCenterline(true); activeEditorLane().centerline[1] = [2,0.5]; regenerateEditorCenterline(activeEditorLane());`, c);
  assert.equal(vm.runInContext('activeEditorLane().centerline[1][1]', c), 0.5);
  vm.runInContext('undoMapEditor()', c);
  assert.equal(vm.runInContext('activeEditorLane().centerline_mode', c), 'auto');
  vm.runInContext('redoMapEditor()', c);
  assert.equal(vm.runInContext('activeEditorLane().centerline[1][1]', c), 0.5);
  vm.runInContext('setManualCenterline(false)', c);
  assert.equal(vm.runInContext('activeEditorLane().centerline[1][1]', c), 0);
});
test('paired insertion and deletion keep opposite station correspondence', () => {
  const c = editorContext();
  vm.runInContext(`nearestEditorSegment = () => ({index:0}); insertEditorPoint(activeEditorLane(), 'left_bound', [1,1.5], {}, [0,0]); regenerateEditorCenterline(activeEditorLane());`, c);
  assert.equal(vm.runInContext('activeEditorLane().right_bound[1][0]', c), 1);
  assert.equal(vm.runInContext('activeEditorLane().centerline[1][1]', c), 0.25);
  vm.runInContext(`deleteEditorPoint({field:'right_bound', index:1})`, c);
  assert.equal(vm.runInContext('activeEditorLane().left_bound.length', c), 3);
  assert.equal(vm.runInContext('activeEditorLane().right_bound.length', c), 3);
});

function sectionEditorContext(closed = false) {
  const c = editorContext();
  vm.runInContext(`
    state.mapEditor.mapPath = '/maps/new'; state.mapEditor.dirty = true;
    state.selectedMapDetail = {map:{path:'/maps/new'}, hd_map:{exists:false, lanes:[], section_gates:[]}, raster:{width:200, height:200, resolution_m_per_px:0.05}};
    state.sectionEditor.mapPath = '/maps/new'; state.sectionEditor.gates = [];
    state.sectionEditor.enabled = true;
    globalThis.document = {getElementById: () => null};
    toast = () => {};
  `, c);
  if (closed) vm.runInContext(`Object.assign(activeEditorLane(), {closed_loop:true, ...LaneGeometry.fromSpine([[0,0],[4,0],[4,4],[0,4]], 1)});`, c);
  return c;
}
test('new unsaved map can explicitly define one section before its first YAML save', () => {
  for (const closed of [false, true]) {
    const c = sectionEditorContext(closed);
    assert.match(vm.runInContext('sectionDefinitionIssue(state.selectedMapDetail)', c), /Section未定義/);
    vm.runInContext('defineWholeCourseSection()', c);
    assert.equal(vm.runInContext('state.sectionEditor.gates.length', c), closed ? 1 : 2);
    assert.equal(vm.runInContext('sectionDefinitionIssue(state.selectedMapDetail)', c), '');
    assert.equal(vm.runInContext('sectionEditorLane(state.selectedMapDetail).id', c), 'a');
    vm.runInContext('state.mapEditor.enabled = false', c);
    assert.equal(vm.runInContext('editorLanesForDetail(state.selectedMapDetail).length', c), 1);
  }
});
test('missing sections block both UI save entry points before any request', async () => {
  const c = sectionEditorContext();
  vm.runInContext(`confirmAction = () => {throw new Error('unexpected confirmation')}; api = () => {throw new Error('unexpected request')}`, c);
  await vm.runInContext('saveHdMapFromEditor()', c);
  await vm.runInContext('saveSectionGatesFromEditor()', c);
});
test('section save delegates to combined save while geometry remains unsaved', async () => {
  const c = sectionEditorContext(true);
  vm.runInContext(`defineWholeCourseSection(); globalThis.combinedSaves = 0; saveHdMapFromEditor = async () => { globalThis.combinedSaves++; }`, c);
  await vm.runInContext('saveSectionGatesFromEditor()', c);
  assert.equal(c.combinedSaves, 1);
});
test('section UI rejects duplicate gates and an open lane with only one gate', () => {
  const c = sectionEditorContext();
  vm.runInContext('defineWholeCourseSection(); state.sectionEditor.gates.pop()', c);
  assert.match(vm.runInContext('sectionDefinitionIssue(state.selectedMapDetail)', c), /2つ以上/);
  vm.runInContext('state.sectionEditor.gates.push({...state.sectionEditor.gates[0], id:"duplicate"})', c);
  assert.match(vm.runInContext('sectionDefinitionIssue(state.selectedMapDetail)', c), /重複/);
});
test('rendered save buttons stay disabled until a section is explicitly defined', () => {
  const c = sectionEditorContext(true);
  assert.match(vm.runInContext('renderHdMapEditor(state.selectedMapDetail)', c), /id="map-editor-save"[^>]*disabled/);
  assert.match(vm.runInContext('renderSectionGateEditor(state.selectedMapDetail)', c), /id="section-editor-save"[^>]*disabled/);
  vm.runInContext('defineWholeCourseSection(); state.sectionEditor.enabled = false', c);
  assert.match(vm.runInContext('renderSectionEditorCounts(state.selectedMapDetail)', c), /G 1 \/ S 1/);
  assert.doesNotMatch(vm.runInContext('renderHdMapEditor(state.selectedMapDetail)', c), /id="map-editor-save"[^>]*disabled/);
});
test('removing gates in the draft allows geometry edits before defining replacement sections', () => {
  const c = sectionEditorContext(true);
  vm.runInContext(`
    state.selectedMapDetail.hd_map.sections = [{id:'old', lane_id:'a', start_gate_id:'gate', end_gate_id:'gate'}];
    state.sectionEditor.gates = [{id:'gate', lane_id:'a', s_m:0}];
  `, c);
  assert.notEqual(vm.runInContext('mapEditorDirectionReverseIssue(state.selectedMapDetail)', c), '');
  vm.runInContext('state.sectionEditor.gates = []; markSectionEditorDirty()', c);
  assert.equal(vm.runInContext('mapEditorDirectionReverseIssue(state.selectedMapDetail)', c), '');
  assert.match(vm.runInContext('sectionDefinitionIssue(state.selectedMapDetail)', c), /Section未定義/);
});

test('save feedback distinguishes generated centerline from missing sections', () => {
  const c = sectionEditorContext();
  assert.match(vm.runInContext('mapEditorSaveState(state.selectedMapDetail).centerline', c), /3点.*未保存/);
  assert.match(vm.runInContext('mapEditorSaveState(state.selectedMapDetail).issue', c), /Section未定義/);
  assert.equal(vm.runInContext('mapEditorSaveState(state.selectedMapDetail).canDefine', c), true);
  vm.runInContext('defineWholeCourseSection(); state.mapEditor.enabled = false;', c);
  assert.equal(vm.runInContext('mapEditorSaveState(state.selectedMapDetail).canSave', c), true);
  assert.match(vm.runInContext('renderHdMapEditor(state.selectedMapDetail)', c), /id="map-editor-save" class="primary/);
});

test('live save chrome reacts to sections, drawing completion and an in-flight save', () => {
  const c = sectionEditorContext();
  vm.runInContext(`
    globalThis.elements = Object.fromEntries(['map-editor-save', 'map-editor-save-reason', 'map-editor-status', 'map-editor-centerline-state', 'map-editor-define-section'].map(id => [id, {classList:{toggle(name, on){this[name] = on;}}}]));
    document.getElementById = id => elements[id] || null;
  `, c);
  // Restore the real incremental DOM updater (the shared fixture stubs it).
  const source = fs.readFileSync(path.join(__dirname, '../app.js'), 'utf8');
  vm.runInContext(source.slice(source.indexOf('function updateMapEditorChrome()'), source.indexOf('\nfunction mapCanvasFitScale')), c);
  vm.runInContext('updateMapEditorChrome()', c);
  assert.equal(c.elements['map-editor-save'].disabled, true);
  assert.match(c.elements['map-editor-save-reason'].textContent, /Section未定義/);
  vm.runInContext('defineWholeCourseSection()', c);
  assert.equal(c.elements['map-editor-save'].disabled, false);
  assert.equal(c.elements['map-editor-save'].classList.primary, true);
  vm.runInContext('state.mapEditor.drawingLane = true; updateMapEditorChrome()', c);
  assert.equal(c.elements['map-editor-save'].disabled, true);
  assert.match(c.elements['map-editor-save-reason'].textContent, /描画完了/);
  vm.runInContext('finishPairedLane()', c);
  assert.equal(c.elements['map-editor-save'].disabled, false);
  vm.runInContext('state.ui.pendingActions["hd-map:save"] = {}; markMapEditorDirty()', c);
  assert.equal(c.elements['map-editor-save'].disabled, true);
});

test('inspector tabs retain draft geometry and allow direct access to layers', () => {
  const c = sectionEditorContext();
  vm.runInContext('setMapInspectorTab("layers")', c);
  const html = vm.runInContext('renderMapInspectorTabs(state.selectedMapDetail)', c);
  assert.match(html, /Fine tune layers/);
  assert.doesNotMatch(html, /HD Map Edit/);
  assert.equal(vm.runInContext('state.mapEditor.dirty && state.mapEditor.enabled', c), true);
  vm.runInContext('setMapInspectorTab("edit")', c);
  assert.match(vm.runInContext('renderMapInspectorTabs(state.selectedMapDetail)', c), /HD Map Edit/);
  assert.equal(vm.runInContext('activeEditorLane().centerline.length', c), 3);
});

test('combined save sends generated centerline and sections, then clears the saved draft', async () => {
  const c = sectionEditorContext();
  vm.runInContext(`
    defineWholeCourseSection();
    state.selectedMapPath = state.selectedMapDetail.map.path;
    confirmAction = () => true;
    beginAction = () => true;
    endAction = () => {};
    invalidateMapPreflights = () => {};
    commitSelectedMapDetail = (context, saved) => { state.selectedMapDetail = saved; return true; };
    api = async (url, options) => {
      globalThis.savedPayload = JSON.parse(options.body);
      return {...state.selectedMapDetail,
        map:{...state.selectedMapDetail.map,artifacts:{centerline_csv:{exists:true}}},
        hd_map:{exists:true, lanes:savedPayload.lanes.map(l => ({...l,primary:l.id===savedPayload.primary_lane_id})), section_gates:savedPayload.section_gates}};
    };
  `, c);
  await vm.runInContext('saveHdMapFromEditor()', c);
  assert.equal(c.savedPayload.lanes[0].centerline.length, 3);
  assert.equal(c.savedPayload.section_gates.length, 2);
  assert.equal(vm.runInContext('state.mapEditor.dirty || state.sectionEditor.dirty', c), false);
  assert.equal(vm.runInContext('state.selectedMapDetail.map.artifacts.centerline_csv.exists', c), true);
});
