"""Validated SSH transport for the screen / Docker / tmux runtime."""
from __future__ import annotations

import json
import math
from pathlib import Path
import re
import shlex
import subprocess

from .security import validate_remote_absolute_path, validate_ssh_target


CONTROLLER_PARAMETERS = ('algorithm', 'min_lookahead_m', 'max_lookahead_m', 'lookahead_speed_gain_s', 'max_steering_angle_rad', 'max_steering_command', 'map_lateral_error_gain', 'mpc_path_error_weight', 'mpc_heading_error_weight', 'mpc_steering_weight', 'throttle_kp', 'throttle_ki', 'throttle_kd', 'throttle_feedforward', 'brake_kp', 'max_throttle_command', 'max_brake_command', 'max_target_speed_mps', 'max_steering_rate_per_s')

def settings(body):
    result = {}
    result['target'] = validate_ssh_target(str(body.get('user', '')), str(body.get('host', '')))
    for key, default in (('container', ''), ('container_user', ''), ('screen', 'jetpilot-web'), ('session', 'jetpilot-web')):
        value = str(body.get(key, default))
        pattern = r'[A-Za-z0-9_][A-Za-z0-9_-]{0,63}' if key in ('screen', 'session') else r'[A-Za-z0-9_][A-Za-z0-9_.-]{0,63}'
        if not re.fullmatch(pattern, value):
            raise ValueError(f'{key}: 名前に使えない文字が含まれています（screen/tmuxは英数字・_・-）')
        result[key] = value
    for key, default in (('bringup', '/workspaces/scripts/bringup.sh'), ('host_workspace', ''), ('model', ''), ('map', ''), ('bag', '')):
        value = str(body.get(key, default)).strip()
        result[key] = validate_remote_absolute_path(value, label=key) if value else ''
    if not result['bringup']:
        raise ValueError('bringupのパスが必要です')
    preset = str(body.get('preset', 'record'))
    if preset not in ('record', 'e2e', 'drive', 'runtime', 'competition', 'tuning', 'offline-vslam', 'offline-vslam-map', 'offline-localization'):
        raise ValueError('未対応の起動用途です')
    result['preset'] = preset
    for key, default in (('sensor', 'realsense'), ('vehicle', 'jpbb')):
        value = str(body.get(key, default))
        if not re.fullmatch(r'[A-Za-z0-9_-]+', value):
            raise ValueError(f'{key}が不正です')
        result[key] = value
    for key, default in (('rgb_fps', 30), ('infra_fps', 60)):
        value = int(body.get(key, default))
        if value not in (0, 30, 60, 90):
            raise ValueError('HzはOFF / 30 / 60 / 90から選んでください')
        result[key] = value
    for key, default in (('evs_window_ms', 50.0), ('evs_stride_ms', 10.0)):
        value = float(body.get(key, default))
        if not math.isfinite(value) or not 1.0 <= value <= 1000.0:
            raise ValueError('EVSの蓄積窓とスライド幅は1〜1000 msで指定してください')
        result[key] = value
    if result['evs_stride_ms'] > result['evs_window_ms']:
        raise ValueError('EVSのスライド幅は蓄積窓以下にしてください')
    value = float(body.get('throttle', 0.2))
    if not math.isfinite(value) or not 0 <= value <= 1:
        raise ValueError('固定スロットルは0〜1で指定してください')
    result['throttle'] = value
    result['fixed'] = body.get('fixed') is True
    return result


def bringup_args(s):
    args = ['bash', s['bringup'], s['preset'], '--yes']
    if s['preset'].startswith('offline-'):
        if not s['bag']:
            raise ValueError('再生するbagの絶対パスが必要です')
        args += ['--bag', s['bag']]
    else:
        args += ['--vehicle', s['vehicle'], '--sensor-kit', s['sensor']]
        for key in ('rgb_fps', 'infra_fps'):
            args += ['--set', f'sensor_kit_{key}:={s[key]}']
        if 'event' in s['sensor'] or 'silky' in s['sensor']:
            args += [
                '--set', f'sensor_kit_silky_evcam_event_image_fps:={1000.0 / s["evs_stride_ms"]}',
                '--set', f'sensor_kit_silky_evcam_event_image_window_ms:={s["evs_window_ms"]}',
                '--set', f'sensor_kit_silky_evcam_event_image_stride_ms:={s["evs_stride_ms"]}',
            ]
    if s['preset'] in ('runtime', 'competition', 'tuning', 'offline-vslam-map', 'offline-localization') and not s['map']:
        raise ValueError('この用途には地図の絶対パスが必要です')
    if s['map']:
        args += ['--map', s['map']]
    if s['preset'] == 'e2e':
        if not s['model']:
            raise ValueError('モデルディレクトリを指定してください（metadataで制御方式を判定します）')
        args += ['--e2e-model', s['model']]
    if s['preset'] == 'record':
        args += ['--set', f'teleop_fixed_throttle_mode:={str(s["fixed"]).lower()}']
    if s['preset'] in ('record', 'e2e'):
        args += ['--set', f'fixed_throttle:={s["throttle"]}']
    return args


def request(body, action):
    if action not in ('status', 'prepare', 'start', 'stop', 'record-start', 'record-stop', 'preview', 'bag-status', 'param-get', 'param-set', 'camera-get', 'camera-set', 'evs-get', 'evs-set'):
        raise ValueError('未対応の操作です')
    s = settings(body)
    if action in ('start', 'preview'):
        s['command'] = bringup_args(s)
    if action == 'preview':
        return {'command': shlex.join(s['command']), 'message': '起動内容を確認しました。まだ実行していません。'}
    if action in ('param-get', 'param-set'):
        node, parameter = str(body.get('node', '')), str(body.get('parameter', ''))
        allowed = {(n, p) for n in ('/teleop_cmd_node', '/e2e_control_decoder')
                   for p in ('fixed_throttle', 'steering_scale', 'steering_offset')}
        allowed.add(('/teleop_cmd_node', 'throttle_scale'))
        allowed.update(('/path_tracking_controller_node', p) for p in CONTROLLER_PARAMETERS)
        if (node, parameter) not in allowed:
            raise ValueError('この項目はWebからの動的調整に対応していません')
        s.update(node=node, parameter=parameter)
        if action == 'param-set':
            if parameter == 'algorithm':
                value = str(body.get('value', ''))
                if value not in ('pure_pursuit', 'map_pursuit', 'kinematic_mpc'):
                    raise ValueError('未対応のcontrollerです')
            else:
                value = float(body.get('value', 'nan'))
                low, high = (-1, 1) if parameter == 'steering_offset' else (0, 3) if parameter == 'steering_scale' else (0, 1)
                if node == '/path_tracking_controller_node': low, high = 0, 100
                if not math.isfinite(value) or not low <= value <= high:
                    raise ValueError(f'調整値は{low}〜{high}で指定してください')
            s['value'] = value
    if action in ('camera-get', 'camera-set'):
        stream = str(body.get('stream', 'rgb'))
        if stream not in ('rgb', 'infra'):
            raise ValueError('RGBまたはInfraを指定してください')
        s.update(node='/realsense', stream=stream)
        if action == 'camera-set':
            fps = int(body.get('fps', 0))
            if fps not in (30, 60, 90):
                raise ValueError('Hzは30 / 60 / 90で指定してください')
            s['fps'] = fps
    if action in ('evs-get', 'evs-set'):
        s['node'] = '/event_camera/event_preprocessor'
    if action in ('bag-status', 'param-get', 'param-set', 'camera-get', 'camera-set', 'evs-get', 'evs-set'):
        s['ros_script'] = Path(__file__).with_name('remote_runtime_ros.py').read_text()
    s['action'] = action
    agent = Path(__file__).with_name('remote_runtime_agent.py').read_text()
    try:
        response = subprocess.run(
            ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', s['target'],
             'python3 -c ' + shlex.quote(agent)],
            input=json.dumps(s), text=True, capture_output=True, timeout=25,
        )
    except subprocess.TimeoutExpired as exc:
        raise ValueError('SSH操作の応答がありません。結果は未確認です。「状態・ログ更新」で確認してください。') from exc
    if response.returncode:
        raise ValueError('SSH操作に失敗しました: ' + response.stderr[-1500:])
    try:
        result = json.loads(response.stdout)
    except ValueError as exc:
        raise ValueError('Jetsonからの応答が不正です: ' + response.stdout[-500:]) from exc
    if result.get('error'):
        raise ValueError(result['error'])
    return result
