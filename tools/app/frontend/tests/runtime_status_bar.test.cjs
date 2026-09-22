const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const source = fs.readFileSync(path.join(__dirname, '../runtime.js'), 'utf8');
function setup() {
  const bar = {innerHTML:''};
  const ctx = vm.createContext({
    localStorage:{getItem:()=>null,setItem:()=>{}},setInterval:()=>{},
    state:{tab:'maps'},render:()=>{throw Error('must not redraw map');},
    esc:value=>String(value ?? '').replaceAll('<','&lt;'),
    document:{querySelector:selector=>selector==='#runtime-status-bar'?bar:null},
    api:async()=>({bag:{state:'recording'}}),
  });
  vm.runInContext(source,ctx);
  vm.runInContext("runtimeConfig.host='jetson';runtimeResult={container_running:true,state:'running'};runtimeLastUpdate='12:00:00';",ctx);
  return {ctx,bar};
}
test('recording refresh on map page updates only the shared bar, failure becomes unknown',async()=>{
  const {ctx,bar}=setup();
  await ctx.runtimeRefreshBag();
  assert.match(bar.innerHTML,/● 記録中/);
  assert.match(bar.innerHTML,/確認時点 12:00:00/);
  ctx.api=async()=>{throw Error('unreachable');};
  await ctx.runtimeRefreshBag();
  assert.match(bar.innerHTML,/記録未確認/);
  assert.doesNotMatch(bar.innerHTML,/● 記録中/);
});
test('late recording result from old connection is discarded',async()=>{
  const {ctx,bar}=setup();
  let resolve;
  ctx.api=()=>new Promise(done=>{resolve=done;});
  const pending=ctx.runtimeRefreshBag();
  vm.runInContext("runtimeConfig.host='other';runtimeResult=null;runtimeLastUpdate='';runtimeBag={state:'unknown'};",ctx);
  resolve({bag:{state:'recording'}});
  await pending;
  assert.match(bar.innerHTML,/other/);
  assert.doesNotMatch(bar.innerHTML,/● 記録中|プロセス 実行中/);
});
test('summary follows effective payload when map use and extraction limit change',()=>{
  const element={outerHTML:''};
  const ctx=vm.createContext({window:{},localStorage:{getItem:()=>null},setInterval:()=>{},document:{getElementById:()=>element},console});
  const app=require('./console_source.cjs').readAppSource();
  vm.runInContext(app.slice(0,app.indexOf('\nwindow.')),ctx);
  vm.runInContext(`
    state.tab='bag-analysis';state.analysis.selectedMapPath='/maps/course';state.analysis.selectedBagPath='/record/lap';
    state.analysis.selectedImageTopics=['/camera/left','/camera/right'];state.analysis.primaryImageTopic='/camera/right';
    state.analysis.analysisPreset='telemetry';bindAnalysisPreflight=()=>{};scheduleAnalysisPreflight=()=>{};
  `,ctx);
  ctx.updateAnalysisOption('maxFps',999);
  assert.match(element.outerHTML,/60 FPS/);
  assert.match(element.outerHTML,/使用しない/);
  assert.doesNotMatch(element.outerHTML,/\/maps\/course/);
  vm.runInContext("state.analysis.analysisPreset='map_vslam'",ctx);
  ctx.updateAnalysisOption('maxFps',5);
  assert.match(element.outerHTML,/5 FPS/);
  assert.match(element.outerHTML,/\/maps\/course/);
  assert.match(element.outerHTML,/主カメラ<\/dt><dd title="\/camera\/right"/);
});
