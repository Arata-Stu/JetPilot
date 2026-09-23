const test = require('node:test');
const assert = require('node:assert/strict');
const vm = require('node:vm');
const fs = require('node:fs');
const path = require('node:path');
function context() {
  const c = vm.createContext({
    esc: s => String(s ?? '').replaceAll('&','&amp;').replaceAll('<','&lt;').replaceAll('"','&quot;'),
    state: { e2eWorkspace: 'section-multihead', rosbags: [{name:'bag',path:'/bags/a'}], tasks: [] },
    render(){}, compactLocalDateTime:()=> '0923-1200', renderTaskTable:()=>'<table>tasks</table>',
  });
  vm.runInContext(fs.readFileSync(path.join(__dirname,'../section_multihead_ui.js'),'utf8'),c);
  return c;
}
test('three default heads and complete workflow render',()=>{
  const c = context();
  assert.equal(vm.runInContext('sectionMultihead.heads.length',c),3);
  const html = vm.runInContext('renderSectionMultihead()',c);
  for (const text of ['データセットを作成','全headを学習','選択headで解析','Jetsonへ転送','multiple']) assert.ok(html.includes(text));
});
test('head assignment inherits dataset throttle and allows adding heads',()=>{
  const c=context();
  vm.runInContext("sectionMultihead.assets.datasets=[{path:'/d',recommended_throttle:.27,sections:['a','b']}];smDataset(0,'/d');smAddHead()",c);
  assert.equal(vm.runInContext('sectionMultihead.heads[0].throttle',c),.27);
  assert.equal(vm.runInContext('sectionMultihead.heads.length',c),4);
});
test('changing maps clears stale section selection',()=>{
  const c=context();
  vm.runInContext("sectionMultihead.sections=['old'];smSelectMap('/maps/new/hd_map.yaml')",c);
  assert.equal(vm.runInContext('sectionMultihead.sections.length',c),0);
  assert.equal(vm.runInContext('sectionMultihead.localizationMap',c),'/maps/new');
});
test('section multi select and model head selection',()=>{
  const c=context();
  vm.runInContext("smToggle('sections','a',true);smToggle('sections','b',true);smToggle('sections','a',false);sectionMultihead.assets.runs=[{path:'/m',heads:[{name:'curve'}]}];smRun('/m')",c);
  assert.equal(vm.runInContext('sectionMultihead.sections.join()',c),'b');
  assert.equal(vm.runInContext('sectionMultihead.analysisHead',c),'curve');
});
test('dataset and map names are escaped',()=>{
  const c=context();
  vm.runInContext("sectionMultihead.assets.maps=[{path:'/x',name:'<script>bad</script>',sections:[]}]",c);
  assert.ok(!vm.runInContext('renderSectionMultihead()',c).includes('<script>bad'));
});
