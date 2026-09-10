"""Executed through SSH on the Jetson host; only Python standard library required."""
import fcntl
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys


def run(args, check=True):
    p = subprocess.run(args, text=True, capture_output=True, timeout=15)
    if check and p.returncode:
        raise RuntimeError((p.stderr or p.stdout or f'command failed: {args[0]}')[-2000:])
    return p


def execute(s):
    # Serializes operations across tabs, including sessions sharing a screen.
    directory = Path.home() / '.local/state/jetpilot-web'
    directory.mkdir(parents=True, exist_ok=True, mode=0o700)
    if s['action'] == 'bag-status':
        return execute_locked(s, directory)
    with (directory / 'runtime.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('別の操作を実行中です。状態を更新してから再操作してください。')
        return execute_locked(s, directory)


def execute_locked(s, directory):
    configured_container = s['container']
    container = configured_container
    session = s['session']
    screen = s['screen']

    def inspect_container(name):
        response = run(['docker', 'inspect', '-f', '{{.State.Running}}', name], False)
        return response.returncode == 0 and response.stdout.strip() == 'true', response

    alive, _ = inspect_container(container)
    if not alive:
        listing = run(['docker', 'ps', '--format', '{{.Names}}'], False)
        if listing.returncode:
            detail = (listing.stderr or listing.stdout or 'docker ps failed')[-1500:]
            raise RuntimeError('Jetson上のDocker一覧を確認できません: ' + detail)
        candidates = []
        for name in listing.stdout.splitlines():
            name = name.strip()
            if not name:
                continue
            probe = run(['docker', 'exec', name, 'test', '-f', s['bringup']], False)
            if probe.returncode == 0:
                candidates.append(name)
        if len(candidates) == 1:
            container = candidates[0]
            alive = True
        elif len(candidates) > 1:
            raise RuntimeError(
                'JetPilot候補のDockerコンテナが複数あります。詳細設定で名前を選択してください: '
                + ', '.join(candidates)
            )

    base = ['docker', 'exec', '-u', s['container_user'], container]

    def tmux(*args, check=True):
        return run(base + ['tmux', *args], check)

    def running_container():
        return inspect_container(container)[0]

    def screen_exists():
        p = run(['screen', '-ls'], False)
        return bool(re.search(r'\d+\.' + re.escape(screen) + r'\s', p.stdout))

    def activation_command():
        workspace = Path(s['host_workspace'] or Path.home())
        if not workspace.is_dir():
            raise RuntimeError('Jetsonホスト上の作業ディレクトリが存在しません。')
        ros_ws_candidates = [workspace / 'ros2_ws']
        if workspace.name == 'ros2_ws':
            ros_ws_candidates.insert(0, workspace)
        ros_ws_candidates.append(Path.home() / 'workspaces' / 'JetPilot' / 'ros2_ws')
        ros_ws = next((path for path in ros_ws_candidates if path.is_dir()), None)
        if ros_ws is None:
            raise RuntimeError(
                'Jetsonホスト上のROS 2ワークスペースを特定できません。'
                '作業ディレクトリにJetPilotのproject rootを指定してください。'
            )
        return (
            'cd ' + shlex.quote(str(workspace))
            + ' && export ISAAC_ROS_WS=' + shlex.quote(str(ros_ws))
            + ' ISAAC_DIR=' + shlex.quote(str(ros_ws))
            + ' && isaac-ros activate; printf "\\nDocker起動処理が終了しました\\n"; exec bash'
        )

    message = ''
    action = s['action']
    screen_log = directory / (screen + '.log')
    if action == 'prepare':
        existing_screen = screen_exists()
        previous_log = ''
        if screen_log.is_file():
            previous_log = screen_log.read_text(errors='replace')[-16000:]
        retry_missing_environment = (
            existing_screen
            and not alive
            and 'ISAAC_ROS_WS or ISAAC_DIR environment variable is not set' in previous_log
        )
        if retry_missing_environment:
            run(['screen', '-S', screen, '-X', 'quit'])
            existing_screen = False
        if existing_screen:
            message = '既存のscreenを再利用します。コンテナの状態と環境準備ログを確認してください。'
        else:
            command = activation_command()
            run(['screen', '-L', '-Logfile', str(directory / (screen + '.log')),
                 '-dmS', screen, 'bash', '-lc', command])
            message = (
                '環境変数不足で失敗したscreenを再作成し、Dockerの準備を再実行しました。'
                if retry_missing_environment else
                'screenで環境の準備を開始しました。Dockerの起動完了まで自動で再確認します。'
            )
    alive = running_container()
    if action in ('bag-status', 'param-get', 'param-set', 'camera-get', 'camera-set', 'evs-get', 'evs-set'):
        if not alive:
            if action == 'bag-status':
                return {'bag': {'state': 'unknown', 'message': 'コンテナが起動していません'}}
            raise RuntimeError('コンテナが起動していません')
        setup = str(Path(s['bringup']).parent.parent / 'ros2_ws/install/setup.bash')
        payload = {key: s[key] for key in ('action', 'node', 'parameter', 'value', 'stream', 'fps', 'evs_window_ms', 'evs_stride_ms') if key in s}
        command = ['timeout', '12', 'python3', '-c', s['ros_script'], json.dumps(payload)]
        response = run(base + ['bash', '-lc', 'source ' + shlex.quote(setup) + ' && ' + shlex.join(command)])
        return json.loads(response.stdout)
    exists = False
    pane = ''
    dead = False
    owned = False
    if alive:
        probe = tmux('has-session', '-t', '=' + session, check=False)
        exists = probe.returncode == 0
        if exists:
            owned = tmux('show-options', '-qv', '-t', '=' + session, '@jetpilot_web', check=False).stdout.strip() == '1'
            lines = tmux('list-panes', '-t', '=' + session + ':bringup', '-F', '#{pane_id} #{pane_dead}').stdout.splitlines()
            pane, value = lines[0].split()
            dead = value == '1'
    if action in ('start', 'stop', 'record-start', 'record-stop') and not alive:
        raise RuntimeError('コンテナが起動していません。先に環境を準備してください。')
    if action in ('start', 'stop', 'record-start', 'record-stop') and exists and not owned:
        raise RuntimeError('同名の手動tmuxセッションがあります。実機画面専用の名前を指定してください。')
    if action == 'start':
        if exists and not dead:
            current_command = tmux(
                'display-message', '-p', '-t', pane, '#{pane_current_command}', check=False
            ).stdout.strip()
            if owned and current_command == 'tmux':
                tmux('kill-session', '-t', '=' + session)
                exists = False
            else:
                raise RuntimeError('bringupセッションは実行中です。先に停止してください。')
        # Validate the actual remote launcher/model before creating a session.
        run(base + ['bash', '-lc', shlex.join(s['command'] + ['--dry-run'])])
        if exists:
            tmux('kill-session', '-t', '=' + session)
        tmux('new-session', '-d', '-s', session, '-n', 'bringup')
        tmux('set-option', '-t', '=' + session, '@jetpilot_web', '1')
        tmux('set-option', '-w', '-t', session + ':bringup', 'remain-on-exit', 'on')
        pane = tmux(
            'list-panes', '-t', '=' + session + ':bringup', '-F', '#{pane_id}'
        ).stdout.splitlines()[0].strip()
        launch_command = 'exec bash -lc ' + shlex.quote(shlex.join(s['command']))
        tmux('respawn-pane', '-k', '-t', pane, launch_command)
        message = '起動要求を送信しました。稼働状態とログを確認してください。'
    elif action == 'stop' and exists and not dead:
        tmux('send-keys', '-t', pane, 'C-c')
        message = '終了要求を送信しました。Dockerとscreenは残します。停止完了は状態更新で確認してください。'
    elif action in ('record-start', 'record-stop'):
        if not exists or dead:
            raise RuntimeError('bringupが実行されていません。')
        setup = str(Path(s['bringup']).parent.parent / 'ros2_ws/install/setup.bash')
        command = 1 if action == 'record-start' else 2
        ros = ['timeout', '10', 'ros2', 'topic', 'pub', '--once', '/bag/request', 'jetpilot_msgs/msg/BagRequest', '{command: ' + str(command) + ', label: web}']
        run(base + ['bash', '-lc', 'source ' + shlex.quote(setup) + ' && ' + shlex.join(ros)])
        message = '記録操作を送信しました。実際の記録状態はbag managerのログで確認してください。'
    detected_message = ''
    if container != configured_container:
        detected_message = f'Dockerコンテナ「{container}」を自動検出しました。'
    result = {'container': container, 'container_running': running_container(),
              'screen_running': screen_exists(),
              'message': '\n'.join(part for part in (detected_message, message) if part),
              'state': 'not_started', 'log': ''}
    if action == 'start':
        result['command'] = shlex.join(s['command'])
    if screen_log.is_file():
        with screen_log.open('rb') as log:
            log.seek(max(0, screen_log.stat().st_size - 16000))
            result['environment_log'] = log.read(16000).decode(errors='replace')
    if result['container_running']:
        p = tmux('list-panes', '-t', '=' + session + ':bringup', '-F', '#{pane_id} #{pane_dead} #{pane_dead_status}', check=False)
        if p.returncode == 0 and p.stdout.strip():
            fields = p.stdout.splitlines()[0].split()
            result['state'] = 'exited' if fields[1] == '1' else 'running'
            result['exit_code'] = fields[2] if len(fields) > 2 else None
            result['log'] = tmux('capture-pane', '-p', '-t', fields[0], '-S', '-300', check=False).stdout[-40000:]
        elif p.stderr and 'no server running' not in p.stderr and 'can\'t find' not in p.stderr and 'error connecting' not in p.stderr:
            result['message'] += '\n' + p.stderr[-1000:]
    return result


if __name__ == '__main__':
    try:
        print(json.dumps(execute(json.load(sys.stdin)), ensure_ascii=False))
    except Exception as exc:
        print(json.dumps({'error': str(exc)}, ensure_ascii=False))
