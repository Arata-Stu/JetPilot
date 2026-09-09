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
  const context = vm.createContext({ window: {}, localStorage: {getItem: () => null}, console, LaneGeometry: G, setInterval: () => {} });
  const source = fs.readFileSync(path.join(__dirname, '../app.js'), 'utf8');
  vm.runInContext(source.slice(0, source.indexOf('\nwindow.')), context);
  vm.runInContext(fs.readFileSync(path.join(__dirname, '../live_tuning.js'), 'utf8'), context);
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

function pointerEditorContext() {
  const c = sectionEditorContext();
  vm.runInContext(`
    state.sectionEditor.enabled = false;
    mapEditorInteractionLocked = () => false;
    mapEditorRasterReady = () => true;
    canvasEventInfo = event => ({canvas:{width:200, height:200}, point:event.point, hitRadius:0.1});
    mapPixelToWorld = (detail, width, height, point) => point;
    nearestEditorPoint = (detail, point) => {
      const field = state.mapEditor.activeField;
      const index = activeEditorLane()[field].findIndex(p => pointDistance(p, point) < 0.1);
      return index < 0 ? null : {field, index};
    };
    nearestEditorSegment = () => ({index:0});
    globalThis.down = point => handleMapEditorPointerDown({point, button:0, preventDefault(){}});
    globalThis.move = point => handleMapEditorPointerMove({point, preventDefault(){}});
    globalThis.up = () => handleMapEditorPointerUp({});
  `, c);
  return c;
}

test('lane placement hover previews the exact committed geometry without changing the draft or undo', () => {
  const c = pointerEditorContext();
  vm.runInContext(`
    state.mapEditor.drawingLane = true; state.mapEditor.laneWidth = 2;
    globalThis.before = JSON.stringify(captureMapEditorSnapshot());
    move([5, 3]);
    globalThis.preview = pairedLanePlacement(state.mapEditor.placementPoint);
  `, c);
  assert.equal(vm.runInContext('JSON.stringify(captureMapEditorSnapshot()) === before', c), true);
  assert.equal(vm.runInContext('state.mapEditor.undoStack.length', c), 0);
  assert.equal(vm.runInContext('preview.centerline.length', c), 4);
  assert.notEqual(vm.runInContext('preview.left_bound[2][0]', c), 4);
  vm.runInContext('down([5, 3])', c);
  for (const field of ['left_bound', 'right_bound', 'centerline']) {
    assert.equal(vm.runInContext(`JSON.stringify(activeEditorLane().${field}) === JSON.stringify(preview.${field})`, c), true);
  }
  assert.equal(vm.runInContext('state.mapEditor.placementPoint', c), null);
  assert.equal(vm.runInContext('state.mapEditor.undoStack.length', c), 1);
});

test('lane placement handles the first point, ignores near duplicates, and clears on leave or undo', () => {
  const c = pointerEditorContext();
  vm.runInContext(`
    state.mapEditor.drawingLane = true;
    Object.assign(activeEditorLane(), {left_bound:[],right_bound:[],centerline:[]});
    move([0,0]);
  `, c);
  assert.equal(vm.runInContext('pairedLanePlacement(state.mapEditor.placementPoint).centerline.length', c), 1);
  vm.runInContext('down([0,0]); move([0.001,0]);', c);
  assert.equal(vm.runInContext('state.mapEditor.placementPoint', c), null);
  vm.runInContext('move([2,0]); handleMapEditorPointerUp({type:"pointerleave"});', c);
  assert.equal(vm.runInContext('state.mapEditor.placementPoint', c), null);
  vm.runInContext('move([2,0]); down([2,0]); move([3,1]); undoMapEditor();', c);
  assert.equal(vm.runInContext('state.mapEditor.placementPoint', c), null);
  assert.equal(vm.runInContext('activeEditorLane().centerline.length', c), 1);
});

test('lane placement overlay draws a translucent footprint and dashed bounds without leaking canvas style', () => {
  const c = pointerEditorContext();
  const events=[];
  c.previewCanvas = {
    save(){events.push('save');},restore(){events.push('restore');},
    beginPath(){},moveTo(){},lineTo(){},closePath(){},arc(){},stroke(){},
    fill(){events.push(this.fillStyle);},setLineDash(value){events.push(Array.from(value));},
  };
  vm.runInContext(`
    state.mapEditor.drawingLane = true; move([5,3]);
    drawLanePlacementPreview(previewCanvas, state.selectedMapDetail, p => p, 1);
  `, c);
  assert.ok(events.includes('rgba(87, 199, 194, 0.14)'));
  assert.ok(events.some(item => Array.isArray(item) && item[0] === 7 && item[1] === 5));
  assert.equal(events.filter(item => item === 'save').length, events.filter(item => item === 'restore').length);
});

test('move mode ignores blank clicks and drags each boundary independently with undo', () => {
  for (const field of ['left_bound', 'right_bound']) {
    const c = pointerEditorContext();
    vm.runInContext(`
      setMapEditorField('${field}');
      globalThis.before = JSON.stringify(cloneEditorLane(activeEditorLane()));
      down([1,3]); move([1,4]); up();
    `, c);
    assert.equal(vm.runInContext('JSON.stringify(cloneEditorLane(activeEditorLane())) === before', c), true);
    assert.equal(vm.runInContext('state.mapEditor.undoStack.length', c), 0);
    const y = field === 'left_bound' ? 1 : -1;
    vm.runInContext(`down([2,${y}]); move([2,${y * 2}]); up();`, c);
    assert.equal(vm.runInContext(`activeEditorLane().${field}[1][1]`, c), y * 2);
    const other = field === 'left_bound' ? 'right_bound' : 'left_bound';
    assert.equal(vm.runInContext(`activeEditorLane().${other}[1][1]`, c), -y);
    assert.equal(vm.runInContext('activeEditorLane().centerline[1][1]', c), y / 2);
    vm.runInContext('undoMapEditor()', c);
    assert.equal(vm.runInContext('JSON.stringify(activeEditorLane()) === before', c), true);
    vm.runInContext('redoMapEditor()', c);
    assert.equal(vm.runInContext(`activeEditorLane().${field}[1][1]`, c), y * 2);
  }
});

test('add mode inserts a paired station but never drags existing or new points', () => {
  const c = pointerEditorContext();
  vm.runInContext(`setMapEditorPointMode('add'); down([2,1]); move([2,3]); up();`, c);
  assert.equal(vm.runInContext('activeEditorLane().left_bound[1][1]', c), 1);
  assert.equal(vm.runInContext('state.mapEditor.undoStack.length', c), 0);
  vm.runInContext('down([1,1.5]); move([1,4]); up();', c);
  assert.equal(vm.runInContext('activeEditorLane().left_bound.length', c), 4);
  assert.equal(vm.runInContext('activeEditorLane().right_bound.length', c), 4);
  assert.equal(vm.runInContext('activeEditorLane().left_bound[1][1]', c), 1.5);
  vm.runInContext('undoMapEditor()', c);
  assert.equal(vm.runInContext('activeEditorLane().left_bound.length', c), 3);
});

test('canvas double-click and right-click do not delete lane points in either mode', () => {
  for (const mode of ['move', 'add']) {
    const c = pointerEditorContext();
    vm.runInContext(`
      setMapEditorPointMode('${mode}');
      down([2,1]); up(); down([2,1]); up();
      handleMapEditorDoubleClick({point:[2,1], preventDefault(){}});
      handleMapEditorContextMenu({point:[2,1], preventDefault(){}});
    `, c);
    assert.equal(vm.runInContext('activeEditorLane().left_bound.length', c), 3);
    vm.runInContext('deleteSelectedEditorPoint()', c);
    assert.equal(vm.runInContext('activeEditorLane().left_bound.length', c), 2);
  }
});

test('finishing paired drawing switches to move and changing mode cancels a drag', () => {
  const c = pointerEditorContext();
  vm.runInContext(`state.mapEditor.drawingLane = true; state.mapEditor.pointMode = 'add'; finishPairedLane();`, c);
  assert.equal(vm.runInContext('state.mapEditor.pointMode', c), 'move');
  vm.runInContext(`down([2,1]); setMapEditorPointMode('add'); move([2,3]); up();`, c);
  assert.equal(vm.runInContext('activeEditorLane().left_bound[1][1]', c), 1);
});

test('manual centerline is preserved when a boundary moves', () => {
  const c = pointerEditorContext();
  vm.runInContext(`setManualCenterline(true); down([2,1]); move([2,2]); up();`, c);
  assert.equal(vm.runInContext('activeEditorLane().centerline[1][1]', c), 0);
});
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

test('curve connector preserves lane identities, endpoint geometry and successor', () => {
  const a=lane([[0,0],[1,0]]), b={...lane([[3,2],[3,3]]),id:'b'};
  const before=JSON.stringify([a,b]);
  const c=G.connector(a,b,'turn','curve');
  assert.equal(JSON.stringify([a,b]),before);
  assert.deepEqual(c.successor_ids,['b']);
  assert.deepEqual(c.centerline[0],[1,0]);
  assert.deepEqual(c.centerline.at(-1),[3,2]);
  assert.deepEqual(c.left_bound[0],a.left_bound.at(-1));
  assert.deepEqual(c.right_bound.at(-1),b.right_bound[0]);
  assert.ok(c.centerline[1][0]>1);
  assert.ok(c.centerline.at(-2)[1]<2);
});

test('straight connector and invalid closed/zero-gap connection', () => {
  const a=lane([[0,0],[1,0]]), b={...lane([[3,0],[4,0]]),id:'b'};
  const c=G.connector(a,b,'connector','straight');
  assert.ok(c.centerline.every(p=>p[1]===0));
  assert.throws(()=>G.connector({...a,closed_loop:true},b,'bad'));
  assert.throws(()=>G.connector(a,{...b,centerline:[[1,0],[2,0]]},'bad'));
});

test('editor connection survives undo/redo', () => {
  const c=editorContext();
  vm.runInContext(`
    state.mapEditor.lanes.push({id:'b',closed_loop:false,centerline:[[4,0],[5,0]],left_bound:[[4,1],[5,1]],right_bound:[[4,-1],[5,-1]]});
    document = {getElementById: id => id === 'network-target' ? {value:'b'} : null};
    connectEditorLane('direct');
  `,c);
  assert.deepEqual(Array.from(vm.runInContext('activeEditorLane().successor_ids',c)),['b']);
  vm.runInContext('undoMapEditor()',c);
  assert.equal(vm.runInContext('(activeEditorLane().successor_ids || []).length',c),0);
  vm.runInContext('redoMapEditor()',c);
  assert.deepEqual(Array.from(vm.runInContext('activeEditorLane().successor_ids',c)),['b']);
});
