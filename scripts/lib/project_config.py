"""Shared, non-executable host configuration for shell tools and Console."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shlex

KEYS = frozenset(('ROS2_WS', 'PYTHON_WS', 'RECORD_ROOT', 'MAP_ROOT',
                  'JETSON_REMOTE_USER', 'JETSON_REMOTE_IPS', 'JETSON_WORKSPACE_ROOT',
                  'JETSON_MAP_ROOT', 'JETSON_RECORD_ROOT'))


def load_environment(root: Path, environ=None) -> dict[str, str]:
    environ = os.environ if environ is None else environ
    values = {}
    local = Path(environ.get('JETPILOT_HOST_CONFIG', root / 'config' / 'host.local.json')).expanduser()
    if not local.is_absolute():
        local = root / local
    for path in (root / 'config' / 'host.json', local):
        if not path.exists():
            if path == local and 'JETPILOT_HOST_CONFIG' in environ:
                raise ValueError(f'host configuration does not exist: {path}')
            continue
        data = json.loads(path.read_text())
        if not isinstance(data, dict) or set(data) - KEYS:
            raise ValueError(f'{path}: unsupported configuration keys')
        if any(not isinstance(value, str) or not value.strip() or '\x00' in value or '\n' in value for value in data.values()):
            raise ValueError(f'{path}: values must be nonempty single-line strings')
        values.update(data)
    values.update({key: environ[key] for key in KEYS if key in environ})
    for key, folder in (('ROS2_WS', 'ros2_ws'), ('PYTHON_WS', 'python_ws'), ('RECORD_ROOT', 'record'), ('MAP_ROOT', 'map')):
        values.setdefault(key, str(root / folder))
    values.setdefault('JETSON_REMOTE_USER', 'tamiya')
    values.setdefault('JETSON_REMOTE_IPS', '192.168.55.1 192.168.11.190 10.42.0.1 192.168.11.11')
    values.setdefault('JETSON_WORKSPACE_ROOT', f'/home/{values["JETSON_REMOTE_USER"]}/workspaces/JetPilot')
    values.setdefault('JETSON_MAP_ROOT', values['JETSON_WORKSPACE_ROOT'].rstrip('/') + '/map')
    values.setdefault('JETSON_RECORD_ROOT', values['JETSON_WORKSPACE_ROOT'].rstrip('/') + '/record')
    return {**environ, **values}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    args = parser.parse_args()
    environment = load_environment(args.root.resolve())
    for key in sorted(KEYS):
        print(f'export {key}={shlex.quote(environment[key])}')
