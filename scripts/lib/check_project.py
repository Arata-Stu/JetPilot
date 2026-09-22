"""Explicit validation profiles; quick never imports third-party Python packages."""
from __future__ import annotations
import argparse
import ast
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]


def run(command, **kwargs):
    print('+ ' + ' '.join(map(str, command)), flush=True)
    subprocess.run(command, cwd=ROOT, check=True, **kwargs)


def main(argv=None):
    parser = argparse.ArgumentParser(description='JetPilot validation profiles')
    parser.add_argument('profile', choices=('quick', 'frontend', 'locks', 'ros', 'gpu', 'jetson'), nargs='?', default='quick')
    parser.add_argument('--packages', nargs='+', help='ros profile package selection')
    args = parser.parse_args(argv)
    environment = {**os.environ, 'PYTHONDONTWRITEBYTECODE': '1'}
    if args.profile == 'quick':
        # Only tracked project files plus newly created first-party maintenance modules.
        tracked = subprocess.check_output(['git', 'ls-files', '-z'], cwd=ROOT).decode().split('\0')
        paths = {ROOT / name for name in tracked if name}
        for folder in ('scripts', 'tests', 'tools/app/backend/jetpilot_console'):
            paths.update((ROOT / folder).rglob('*.py'))
        paths.update((ROOT / 'scripts').rglob('*.sh'))
        for path in sorted(paths):
            if not path.is_file():
                continue
            if path.suffix == '.py':
                ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
            elif path.suffix == '.sh':
                run(['bash', '-n', str(path)], env=environment)
        for folder in ('tests', 'tests/scripts', 'tools/app/backend/tests'):
            env = {**environment, 'PYTHONPATH': str(ROOT / 'tools/app/backend') + os.pathsep + str(ROOT / 'scripts/lib')}
            run([sys.executable, '-S', '-m', 'unittest', 'discover', '-s', folder, '-p', 'test_*.py'], env=env)
    elif args.profile == 'frontend':
        node = os.environ.get('NODE_BIN') or shutil.which('node')
        if not node:
            parser.error('Node.js is required for frontend tests; quick does not require it')
        run([node, '--test', *map(str, sorted((ROOT / 'tools/app/frontend/tests').glob('*.test.*')))], env=environment)
    elif args.profile == 'locks':
        for name in ('training', 'calibration', 'analysis'):
            run([str(ROOT / 'scripts/maintenance/python_env.sh'), 'check', name], env=environment)
            run([str(ROOT / 'scripts/maintenance/python_env.sh'), 'export', name, '--check'], env=environment)
    elif args.profile == 'ros':
        if sys.platform != 'linux' or not shutil.which('colcon'):
            parser.error('ros profile requires a sourced Linux ROS environment')
        from project_config import load_environment
        workspace = Path(load_environment(ROOT)['ROS2_WS'])
        command = ['colcon', 'test', '--return-code-on-test-failure']
        if args.packages:
            command += ['--packages-select', *args.packages]
        subprocess.run(command, cwd=workspace, check=True)
        subprocess.run(['colcon', 'test-result', '--verbose'], cwd=workspace, check=True)
    elif args.profile == 'gpu':
        if sys.platform != 'linux':
            parser.error('gpu profile runs only in the x86_64 Linux training environment')
        python = os.environ.get('JETPILOT_TRAINING_PYTHON', '/opt/env/bin/python')
        run([python, '-c', 'import torch, onnxruntime as ort; assert torch.cuda.is_available(), "CUDA unavailable"; assert "CUDAExecutionProvider" in ort.get_available_providers(); x=torch.ones(8, device="cuda"); assert x.sum().item()==8; print(torch.cuda.get_device_name(0)); print(ort.get_available_providers())'])
    else:
        if sys.platform != 'linux' or not shutil.which('ros2'):
            parser.error('jetson profile requires a running, sourced ROS environment')
        # Read-only discovery. It never launches nodes or commands actuators.
        run(['ros2', 'node', 'list'])
        run(['ros2', 'topic', 'list', '-t'])
        print('Discovery only: sensor rates, calibration and inference accuracy require an explicit on-device test.')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except subprocess.CalledProcessError as exc:
        raise SystemExit(exc.returncode)
