"""Image-free tuning snapshots and a fixed SSH transport (standard library only)."""
from __future__ import annotations

import hashlib
import json
import math
import shlex
import subprocess

from . import map_detail as maps
from .security import validate_ssh_target

MAX_BYTES = 8 * 1024 * 1024
FIELDS = ('s_m', 'x_m', 'y_m', 'psi_rad', 'kappa_radpm', 'vx_mps', 'ax_mps2')


def encoded(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def map_identity(root):
    """Portable content identity; unlike mtime, survives rsync and copying."""
    digest = hashlib.sha256()
    found = False
    for name in ('cuvgl_map', 'cuvslam_map', 'vslam_reference_snapshot.json', 'vslam_landmarks.yaml'):
        path = root / name
        files = sorted(p for p in path.rglob('*') if p.is_file()) if path.is_dir() else [path] if path.is_file() else []
        for item in files:
            found = True
            digest.update(str(item.relative_to(root)).encode() + b'\0')
            with item.open('rb') as handle:
                for chunk in iter(lambda: handle.read(1024 * 1024), b''):
                    digest.update(chunk)
            digest.update(b'\0')
    if not found:
        raise ValueError('自己位置推定用マップがありません。VSLAM/VGLマップを用意してください。')
    return digest.hexdigest()


def prepare_snapshot(config, body):
    root = maps._resolve_custom_line_map(config, body)
    layout = maps._custom_line_hd_layout(root)
    kind = body.get('line', 'centerline')
    manifest = {}
    closed = layout['closed_loop']
    if kind in ('centerline', 'raceline'):
        if kind == 'centerline':
            hd, _ = maps._read_hd_map(root / f'{root.name}_hd_map.yaml')
            lane = next(lane for lane in hd['lanes'] if lane.get('primary') or lane['id'] == hd['primary_lane_id'])
            raw = [{'x_m': p[0], 'y_m': p[1]} for p in lane['centerline']]
        else:
            _, raw = maps._read_custom_line_source(root, 'raceline', 1.0)
    else:
        line_id = maps._require_custom_line_id({'id': str(kind).removeprefix('custom:')})
        manifest = maps._read_custom_line_manifest(maps._custom_line_path(root, line_id))
        raw = manifest['points']
        closed = maps._manifest_closed_loop(manifest)
    default = maps._custom_line_default_speed(body.get('speed_mps', manifest.get('default_speed_mps', 1.0)))
    overrides = body.get('section_speeds_mps')
    if overrides is None:
        overrides = manifest.get('section_speeds_mps', {
            str(section['id']): section['speed_override_mps']
            for section in layout['sections'] if section.get('speed_override_mps') is not None
        })
    overrides = maps._custom_line_section_speeds(overrides, layout)
    constraints = maps._custom_line_constraints({}, manifest.get('constraints'))
    points = maps._custom_line_points(raw, closed)
    trajectory, _, validation, compiled = maps._compile_custom_line(root, points, closed, default, overrides, constraints)
    if not validation['valid']:
        raise ValueError(validation['issue'])
    hd, _ = maps._read_hd_map(root / f'{root.name}_hd_map.yaml')
    if maps.load_yaml(root / f'{root.name}_hd_map.yaml').get('frame_id', 'map') != 'map':
        raise ValueError('実車調整は map 座標のHD Mapが必要です。')
    snapshot = {
        'format': 1, 'map_id': map_identity(root), 'map_name': root.name,
        'line': kind, 'display_name': manifest.get('name', kind), 'closed': closed,
        'frame_id': 'map', 'default_speed_mps': default, 'section_speeds_mps': overrides,
        'constraints': constraints, 'hd_map': {key: value for key, value in hd.items() if key != 'path'},
        'points': [[row[key] for key in FIELDS] for row in trajectory],
        'sections': compiled['context']['sections'],
    }
    snapshot['revision'] = hashlib.sha256(encoded(snapshot)).hexdigest()
    validate_snapshot(snapshot, snapshot['map_id'])
    return snapshot


def validate_snapshot(snapshot, expected_map_id):
    if len(encoded(snapshot)) > MAX_BYTES:
        raise ValueError('調整データが8 MiBを超えています。点数を減らしてください。')
    if snapshot.get('format') != 1 or snapshot.get('frame_id') != 'map':
        raise ValueError('unsupported tuning format/frame')
    if not expected_map_id or snapshot.get('map_id') != expected_map_id:
        raise ValueError('自己位置推定用マップがJetsonと一致しません。')
    content = {key: value for key, value in snapshot.items() if key != 'revision'}
    if snapshot.get('revision') != hashlib.sha256(encoded(content)).hexdigest():
        raise ValueError('snapshot revision mismatch')
    if not isinstance(snapshot.get('closed'), bool):
        raise ValueError('closed must be boolean')
    rows = snapshot.get('points', [])
    if not (3 if snapshot['closed'] else 2) <= len(rows) <= 20000:
        raise ValueError('invalid trajectory point count')
    previous = -1.0
    for row in rows:
        if len(row) != 7 or any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in row):
            raise ValueError('invalid trajectory point')
        if row[0] <= previous or row[5] < 0 or row[5] > 10:
            raise ValueError('invalid station or speed')
        previous = row[0]
    if rows[0][0] != 0:
        raise ValueError('trajectory must start at station zero')
    return snapshot


def remote_request(config, body, action, payload=None):
    if action not in ('status', 'apply', 'rollback', 'active'):
        raise ValueError('unsupported tuning action')
    config.state_dir.mkdir(parents=True, exist_ok=True)
    target = validate_ssh_target(str(body.get('user') or config.jetson_user), str(body.get('host') or config.jetson_ips[0]))
    # Only a fixed loopback endpoint is accessible; no client-supplied shell or URL.
    script = "import sys,urllib.request,urllib.error; data=sys.stdin.buffer.read(); req=urllib.request.Request('http://127.0.0.1:8781/" + action + "',data=data,headers={'Content-Type':'application/json'});\ntry:\n r=urllib.request.urlopen(req,timeout=8); print(r.read().decode())\nexcept urllib.error.HTTPError as e:\n print(e.read().decode())"
    result = subprocess.run(
        ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=3', '-o', 'ControlMaster=auto',
         '-o', 'ControlPersist=30', '-o', f'ControlPath={config.state_dir}/tuning-%C', target,
         'python3 -c ' + shlex.quote(script)],
        input=encoded(payload or {}), capture_output=True, timeout=12,
    )
    if result.returncode:
        raise ValueError('Jetsonの実車調整サービスに接続できません: ' + result.stderr.decode(errors='replace')[-600:])
    response = json.loads(result.stdout)
    if response.get('error'):
        raise ValueError(response['error'])
    return response
