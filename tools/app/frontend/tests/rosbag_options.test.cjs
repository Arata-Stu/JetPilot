const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const source = fs.readFileSync(path.join(__dirname, '../app.js'), 'utf8');

function functionSource(name) {
  const start = source.indexOf(`function ${name}(`);
  const end = source.indexOf('\nfunction ', start + 1);
  return source.slice(start, end < 0 ? undefined : end);
}

test('rosbag picker preserves record-relative hierarchy and groups categories', () => {
  const context = {
    state: {
      config: { record_root: '/workspaces/record' },
      rosbags: [
        {
          name: 'run_001',
          display_name: 'run_001',
          relative_path: 'competition/dry/run_001',
          path: '/workspaces/record/competition/dry/run_001',
        },
        {
          name: 'run_001',
          display_name: 'run_001',
          relative_path: 'development/wet/run_001',
          path: '/workspaces/record/development/wet/run_001',
        },
      ],
    },
    esc: (value) => String(value),
  };
  vm.createContext(context);
  vm.runInContext(functionSource('rosbagRelativePath'), context);
  vm.runInContext(functionSource('rosbagOptionLabel'), context);
  vm.runInContext(functionSource('renderRosbagOptions'), context);

  const html = context.renderRosbagOptions('/workspaces/record/development/wet/run_001');

  assert.match(html, /optgroup label="record \/ competition"/);
  assert.match(html, /competition \/ dry \/ run_001/);
  assert.match(html, /optgroup label="record \/ development"/);
  assert.match(html, /development \/ wet \/ run_001/);
  assert.match(html, /value="\/workspaces\/record\/development\/wet\/run_001" selected/);
});
