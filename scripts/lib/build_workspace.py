"""ROS build entry point. No ROS imports are needed for planning or --dry-run."""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import platform
import re
import shlex
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET


def package_names(workspace: Path) -> list[str]:
    result = set()
    for directory, children, files in os.walk(workspace / 'src'):
        children[:] = [name for name in children if name not in {'.git', 'build', 'install', 'log'}]
        if 'COLCON_IGNORE' in files:
            children.clear()
            continue
        if 'package.xml' in files:
            name = ET.parse(Path(directory) / 'package.xml').getroot().findtext('name')
            if name:
                name = name.strip()
                if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name):
                    raise ValueError(f'invalid package name in {directory}/package.xml: {name}')
                result.add(name)
            children.clear()
    return sorted(result)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='JetPilot ROS workspace build')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--all', action='store_true', help='build every package')
    mode.add_argument('--packages', nargs='+', metavar='PACKAGE')
    parser.add_argument('--workspace', type=Path, default=Path(os.environ.get('ROS2_WS', Path(__file__).resolve().parents[2] / 'ros2_ws')))
    parser.add_argument('--no-deps', action='store_true', help='select only named packages; default includes workspace dependencies')
    parser.add_argument('--jobs', type=int, default=2 if platform.machine() in ('aarch64', 'arm64') else None)
    parser.add_argument('-c', '--clean', action='store_true', help='remove selected package outputs (all outputs in --all mode)')
    parser.add_argument('--clean-all', action='store_true', help='explicitly remove the entire workspace build/install/log')
    parser.add_argument('--no-ccache', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    if args.jobs is not None and args.jobs < 1:
        parser.error('--jobs must be positive')
    workspace = args.workspace.expanduser().resolve()
    if not (workspace / 'src').is_dir():
        parser.error(f'workspace src not found: {workspace}')
    try:
        available = package_names(workspace)
    except (ValueError, ET.ParseError) as exc:
        parser.error(str(exc))
    if not args.all and not args.packages:
        if not sys.stdin.isatty():
            parser.error('non-interactive use requires --all or --packages PACKAGE ...')
        print('ビルド対象: 1) 全体  2) パッケージ選択  Enter) 中止')
        choice = input('番号: ').strip()
        if choice == '1':
            args.all = True
        elif choice == '2':
            if shutil.which('fzf'):
                selection = subprocess.run(['fzf', '--multi', '--bind', 'space:toggle'], input='\n'.join(available), text=True, stdout=subprocess.PIPE)
                if selection.returncode:
                    return 1
                args.packages = selection.stdout.split()
            else:
                print('\n'.join(available))
                args.packages = input('パッケージ名を空白区切りで入力: ').split()
            if not args.packages:
                return 1
        else:
            return 0
    unknown = set(args.packages or ()) - set(available)
    if unknown:
        parser.error(f'unknown packages: {", ".join(sorted(unknown))}')
    if args.no_deps and not args.packages:
        parser.error('--no-deps requires --packages')
    removals = []
    if args.clean_all or (args.clean and args.all):
        removals = [workspace / name for name in ('build', 'install', 'log')]
    elif args.clean:
        layout = workspace / 'install' / '.colcon_install_layout'
        if layout.exists() and layout.read_text().strip() == 'merged':
            parser.error('merged install cannot be cleaned per package; use --clean-all')
        removals = [workspace / output / package for package in args.packages for output in ('build', 'install')]
    for path in removals:
        if not path.parent.resolve().is_relative_to(workspace):
            parser.error(f'clean path escapes workspace through a symlink: {path}')
    command = ['colcon', 'build', '--symlink-install']
    if args.packages:
        command += ['--packages-select' if args.no_deps else '--packages-up-to', *args.packages]
    if args.jobs:
        # Avoid multiplying package concurrency by compiler concurrency on Jetson.
        command += ['--parallel-workers', '1']
    command += ['--cmake-args', '-DCMAKE_BUILD_TYPE=Release']
    if not args.no_ccache and shutil.which('ccache'):
        command += ['-DCMAKE_C_COMPILER_LAUNCHER=ccache', '-DCMAKE_CXX_COMPILER_LAUNCHER=ccache']
    for path in removals:
        print(f'Clean: {path}', flush=True)
    print(f'Workspace: {workspace}\n{shlex.join(command)}', flush=True)
    if args.jobs:
        print(f'CMAKE_BUILD_PARALLEL_LEVEL={args.jobs}', flush=True)
    if args.dry_run:
        return 0
    if not shutil.which('colcon'):
        parser.error('colcon not found; run in the ROS environment')
    for path in removals:
        if path.is_symlink():
            path.unlink()
        elif path.exists():
            shutil.rmtree(path)
    environment = os.environ.copy()
    if args.jobs:
        environment['CMAKE_BUILD_PARALLEL_LEVEL'] = str(args.jobs)
    return subprocess.run(command, cwd=workspace, env=environment).returncode


if __name__ == '__main__':
    raise SystemExit(main())
