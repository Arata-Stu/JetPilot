const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const source = fs.readFileSync(path.join(__dirname, '../app.js'), 'utf8');
function functionSource(name) {
  const start = source.indexOf(`function ${name}(`);
  const end = source.indexOf('\nfunction ', start + 1);
  return source.slice(start, end < 0 ? undefined : end);
}
test('map build sends selected model and dimensions to preflight and API payload', () => {
  const fields = {
    'build-rosbag': '/record/bag', 'build-steps': 'edex compute_poses cuvgl',
    'build-vgl-model': '/workspaces/tools/aliked_workspace/artifacts/424x240/runtime_models',
    'build-vgl-width': '424', 'build-vgl-height': '240',
  };
  const context = { $: id => ({ value: fields[id] }), outputMapDir: () => '/map/new',
    selectedCameraTopicConfig: () => '/config/camera.yaml' };
  vm.createContext(context);
  vm.runInContext(functionSource('mapBuildPreflightPayload'), context);
  const payload = context.mapBuildPreflightPayload();
  assert.equal(payload.vgl_image_width, 424);
  assert.equal(payload.vgl_image_height, 240);
  assert.equal(payload.output_model_dir, fields['build-vgl-model']);
});
test('map form exposes model and image size fields', () => {
  const form = functionSource('renderMapBuildForm');
  for (const id of ['build-vgl-model', 'build-vgl-width', 'build-vgl-height']) {
    assert.ok(form.includes(`id="${id}"`));
  }
});
