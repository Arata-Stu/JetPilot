"""Create locked Linux environments and export the same locks for Docker."""
from __future__ import annotations
import argparse
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
EXPORTS = {
    'calibration': [(None, 'requirements-multi-sensor-calibration.txt')],
    'training': [('gpu', 'requirements-training-gpu-amd64.txt'), ('tools', 'requirements-training-amd64.txt')],
    'analysis': [(None, 'requirements-analysis.txt')],
}


def main(argv=None):
    parser = argparse.ArgumentParser(description='uv.lock を正として Python 環境を管理')
    parser.add_argument('command', choices=('sync', 'lock', 'check', 'export'))
    parser.add_argument('environment', choices=tuple(EXPORTS))
    parser.add_argument('--python', default=os.environ.get('JETPILOT_ENV_PYTHON', '/usr/bin/python3'))
    parser.add_argument('--venv', type=Path)
    parser.add_argument('--check', action='store_true', help='export の差分だけを検査')
    args = parser.parse_args(argv)
    uv = os.environ.get('UV_BIN') or shutil.which('uv')
    if not uv:
        parser.error('uv not found; use the JetPilot container or install uv')
    project = ROOT / 'python_ws/environments' / args.environment
    base = [uv, '--project', str(project), '--no-python-downloads']
    if args.command in ('lock', 'check'):
        command = base + ['lock', '--python', args.python]
        if args.command == 'check':
            command += ['--check', '--offline']
        return subprocess.run(command).returncode
    if args.command == 'export':
        mismatch = False
        for group, filename in EXPORTS[args.environment]:
            command = base + ['export', '--locked', '--offline', '--python', args.python,
                              '--no-header', '--no-annotate', '--no-emit-project']
            if group:
                command += ['--only-group', group]
            result = subprocess.run(command, capture_output=True, text=True, check=True)
            generated = '# Generated from python_ws/environments/' + args.environment + '/uv.lock; do not edit.\n' + result.stdout
            # Keep a parent-repository export, then stage the identical file in
            # the separately versioned CLI Docker context where applicable.
            paths = [project / filename]
            docker = ROOT / 'tools/isaac-ros-cli/docker'
            if args.environment != 'analysis' and docker.is_dir():
                paths.append(docker / filename)
            for path in paths:
                if args.check:
                    if not path.exists() or path.read_text() != generated:
                        print(f'OUTDATED: {path}', file=sys.stderr)
                        mismatch = True
                else:
                    path.write_text(generated)
                    print(f'Exported: {path}')
        return int(mismatch)
    if sys.platform != 'linux':
        parser.error('sync は Linux の ROS/解析環境で実行してください。Mac では lock/check/export のみ使用します')
    if args.environment == 'training' and platform.machine() not in ('x86_64', 'amd64'):
        parser.error('training は x86_64 専用です。Jetson には導入しません')
    defaults = {'training': Path('/opt/env'), 'calibration': Path('/opt/multi_sensor_calibration_env'),
                'analysis': ROOT / '.venvs/analysis'}
    venv = (args.venv or defaults[args.environment]).expanduser().resolve()
    if venv == ROOT or venv == Path('/') or venv == Path(sys.prefix).resolve():
        parser.error('refusing to sync into the project root or system interpreter')
    environment = {**os.environ, 'UV_PROJECT_ENVIRONMENT': str(venv)}
    if not (venv / 'pyvenv.cfg').exists():
        command = [uv, 'venv', '--python', args.python, '--no-python-downloads', str(venv)]
        if args.environment == 'calibration':
            command.append('--system-site-packages')
        subprocess.run(command, check=True, env=environment)
    elif args.environment == 'calibration' and 'include-system-site-packages = true' not in (venv / 'pyvenv.cfg').read_text().lower():
        parser.error('calibration requires a venv created with --system-site-packages; choose a new --venv')
    # Preserve source-installed helpers and ROS-visible packages. Locked
    # dependencies are still installed at exactly their locked versions.
    subprocess.run(base + ['sync', '--locked', '--inexact', '--python', args.python], env=environment, check=True)
    if args.environment == 'calibration':
        subprocess.run([str(venv / 'bin/python'), '-c',
                        "import cv2, numpy, rosbags, yaml; "
                        "assert numpy.__version__ == '1.26.4'; "
                        "from metavision_core.event_io import EventsIterator; "
                        "print('Calibration environment OK; NumPy', numpy.__version__, 'OpenCV', cv2.__version__)"], check=True)
    print(f'Environment: {venv}')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except subprocess.CalledProcessError as exc:
        print(exc.stderr or str(exc), file=sys.stderr)
        raise SystemExit(exc.returncode)
