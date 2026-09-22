const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const source = require('./console_source.cjs').readAppSource();

function functionSource(name) {
  const start = source.indexOf(`function ${name}(`);
  const end = source.indexOf('\nfunction ', start + 1);
  return source.slice(start, end < 0 ? undefined : end);
}

test('Jetson rosbags are grouped by their directory below the remote record root', () => {
  const context = {
    state: {
      config: { jetson_record_root: '/home/pilot/record' },
      jetsonTarget: null,
    },
  };
  vm.createContext(context);
  vm.runInContext(functionSource('jetsonRosbagRelativePath'), context);
  vm.runInContext(functionSource('jetsonRosbagGroups'), context);

  const groups = context.jetsonRosbagGroups([
    { name: 'run-2', path: '/home/pilot/record/competition/dry/run-2' },
    { name: 'run-1', path: '/home/pilot/record/competition/dry/run-1' },
    { name: 'run-3', path: '/home/pilot/record/development/wet/run-3' },
  ]);

  assert.deepEqual(
    JSON.parse(JSON.stringify(groups)),
    [
      {
        path: 'competition/dry',
        sequences: [
          { name: 'run-2', path: '/home/pilot/record/competition/dry/run-2' },
          { name: 'run-1', path: '/home/pilot/record/competition/dry/run-1' },
        ],
      },
      {
        path: 'development/wet',
        sequences: [
          { name: 'run-3', path: '/home/pilot/record/development/wet/run-3' },
        ],
      },
    ],
  );
});

test('Jetson rosbag grouping falls back to the immediate parent for an unknown root', () => {
  const context = {
    state: {
      config: { jetson_record_root: '/different/root' },
      jetsonTarget: null,
    },
  };
  vm.createContext(context);
  vm.runInContext(functionSource('jetsonRosbagRelativePath'), context);
  vm.runInContext(functionSource('jetsonRosbagGroups'), context);

  const groups = context.jetsonRosbagGroups([
    { name: 'run-1', path: '/mnt/archive/session-a/run-1' },
  ]);

  assert.equal(groups[0].path, 'session-a');
});
