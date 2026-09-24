"""Manage JetPilot itself and repositories in packages.repos using only stdlib."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import subprocess
import sys


def read_manifest(path: Path) -> dict[str, dict[str, str]]:
    # Deliberately accept only the simple vcstool schema used by this project.
    # Reject unfamiliar YAML constructs rather than guessing a repository path.
    repos = {}
    current = None
    header = False
    for number, raw in enumerate(path.read_text().splitlines(), 1):
        if not raw.strip() or raw.lstrip().startswith('#'):
            continue
        if raw == 'repositories:' and not header:
            header = True
            continue
        match = re.fullmatch(r'  ([A-Za-z0-9_./-]+):\s*', raw)
        if match and header:
            current = match[1]
            if current in repos or Path(current) == Path('.') or Path(current).is_absolute() or '..' in Path(current).parts:
                raise ValueError(f'{path}:{number}: duplicate or unsafe path')
            repos[current] = {}
            continue
        match = re.fullmatch(r'    (type|url|version):\s*(.+?)\s*', raw)
        if match and current:
            key, value = match.groups()
            if key in repos[current]:
                raise ValueError(f'{path}:{number}: duplicate {key}')
            if value.startswith('"'):
                value = json.loads(value)
            elif value.startswith("'") and value.endswith("'"):
                value = value[1:-1].replace("''", "'")
            if not isinstance(value, str) or not value or value.startswith('-'):
                raise ValueError(f'{path}:{number}: invalid {key}')
            repos[current][key] = value
            continue
        raise ValueError(f'{path}:{number}: unsupported .repos syntax')
    for name, item in repos.items():
        if set(item) != {'type', 'url', 'version'} or item['type'] != 'git':
            raise ValueError(f'{name}: requires type: git, url and version')
    if not repos:
        raise ValueError('manifest contains no repositories')
    return repos


def git(path: Path, *args: str, check=True):
    return subprocess.run(['git', '-C', str(path), *args], text=True,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=check)


def inspect(path: Path) -> tuple[str, str, bool]:
    top = Path(git(path, 'rev-parse', '--show-toplevel').stdout.strip()).resolve()
    if top != path.resolve():
        raise ValueError(f'{path}: not an independent repository')
    revision = git(path, 'rev-parse', 'HEAD').stdout.strip()
    branch = git(path, 'symbolic-ref', '--quiet', '--short', 'HEAD', check=False).stdout.strip()
    dirty = bool(git(path, 'status', '--porcelain').stdout)
    return revision, branch, dirty


def print_pull_summary(results, dry_run: bool) -> None:
    print('\n=== pull 結果' + ('（dry-run・更新未実行）' if dry_run else '') + ' ===', flush=True)
    states = ('更新済み', '変更なし', '実行予定', 'スキップ', '失敗')
    print(' / '.join(f'{state}: {sum(item[1] == state for item in results)}' for state in states))
    for state in states:
        entries = [item for item in results if item[1] == state]
        if not entries:
            continue
        print(f'\n{state}:')
        for name, _, detail in entries:
            print(f'  - {"JetPilot (.)" if name == "." else name}')
            for line in detail.splitlines():
                print(f'      {line}')
    print(flush=True)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='JetPilot本体と外部リポジトリの一覧・更新・取得・コミット記録')
    parser.add_argument('command', choices=('status', 'pull', 'import', 'lock'), nargs='?', default='status')
    parser.add_argument('repositories', nargs='*', help='対象パス。本体は .（status/pullのみ）。省略時のstatus/pullは本体も含む')
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--manifest', type=Path, help='既定: packages.repos。再現時は packages.lock.repos')
    parser.add_argument('--output', type=Path, help='lock の保存先。既定: packages.lock.repos')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    if args.command == 'lock' and args.repositories and not args.output:
        parser.error('部分 lock には --output を指定してください。全体 lock の上書きを防ぎます')
    root = args.root.resolve()
    try:
        include_root = args.command in ('status', 'pull')
        repos = {} if include_root and args.repositories == ['.'] else read_manifest(args.manifest or root / 'packages.repos')
        if include_root:
            repos = {'.': {}, **repos}
        if set(args.repositories) - repos.keys():
            raise ValueError('unknown repository selection')
        selected = args.repositories or list(repos)
        locked = {}
        failures = 0
        results = []
        for name in selected:
            path = root / name
            item = repos[name]
            try:
                if not path.resolve().is_relative_to(root):
                    raise ValueError(f'{name}: path escapes workspace')
                if not path.exists():
                    if args.command != 'import':
                        print(f'MISSING  {name} (repos.sh import で取得)')
                        failures += 1
                        results.append((name, '失敗', '未取得です。repos.sh import で取得してください'))
                        continue
                    print(f'CLONE    {name} @ {item["version"]}', flush=True)
                    if args.dry_run:
                        continue
                    path.parent.mkdir(parents=True, exist_ok=True)
                    subprocess.run(['git', 'clone', '--no-checkout', '--', item['url'], str(path)], check=True)
                    # Resolve the requested revision, preserving a branch when it exists.
                    remote = git(path, 'show-ref', '--verify', f'refs/remotes/origin/{item["version"]}', check=False)
                    if remote.returncode == 0:
                        local = git(path, 'show-ref', '--verify', f'refs/heads/{item["version"]}', check=False)
                        if local.returncode == 0:
                            git(path, 'checkout', item['version'])
                        else:
                            git(path, 'checkout', '--track', f'origin/{item["version"]}')
                    else:
                        git(path, 'checkout', '--detach', item['version'])
                revision, branch, dirty = inspect(path)
                label = branch or 'DETACHED (固定コミット/タグ)'
                display_name = 'JetPilot (.)' if name == '.' else name
                print(f'{"DIRTY" if dirty else "CLEAN":8} {display_name}  {label}  {revision[:12]}', flush=True)
                tracking = git(path, 'rev-list', '--left-right', '--count', 'HEAD...@{upstream}', check=False)
                if tracking.returncode == 0:
                    ahead, behind = tracking.stdout.split()
                    print(f'  upstream cache: ahead={ahead}, behind={behind}')
                if args.command == 'pull':
                    if not branch:
                        print('  SKIP: 固定コミット/タグを維持しました')
                        results.append((name, 'スキップ', '固定コミット/タグを維持しました'))
                        continue
                    if dirty:
                        print('  INFO: 未コミット変更を残して更新を試みます。上書きが必要な場合はGitが停止します')
                    upstream = git(path, 'rev-parse', '--abbrev-ref', '@{upstream}', check=False)
                    if upstream.returncode:
                        print('  SKIP: upstream が未設定です')
                        failures += 1
                        results.append((name, '失敗', 'upstream が未設定のためpullできません'))
                        continue
                    if args.dry_run:
                        print(f'  PLAN: git pull --ff-only ({upstream.stdout.strip()})')
                        results.append((name, '実行予定', upstream.stdout.strip()))
                    else:
                        result = git(path, '-c', 'merge.autostash=false', '-c', 'rebase.autostash=false',
                                     'pull', '--no-rebase', '--ff-only')
                        print(result.stdout.strip())
                        current = git(path, 'rev-parse', 'HEAD').stdout.strip()
                        state = '更新済み' if current != revision else '変更なし'
                        detail = f'{revision[:12]} → {current[:12]}' if current != revision else current[:12]
                        results.append((name, state, detail))
                elif args.command == 'lock':
                    if dirty:
                        raise ValueError('未コミット変更は lock に再現できません。先にコミットしてください')
                    origin = git(path, 'remote', 'get-url', 'origin').stdout.strip()
                    if origin.removesuffix('.git').rstrip('/') != item['url'].removesuffix('.git').rstrip('/'):
                        raise ValueError('origin と manifest の URL が異なります。先に manifest を更新してください')
                    locked[name] = {**item, 'version': revision}
            except (ValueError, OSError, subprocess.CalledProcessError) as exc:
                detail = (exc.stderr or exc.stdout or str(exc)) if isinstance(exc, subprocess.CalledProcessError) else str(exc)
                print(f'ERROR    {name}: {detail}', file=sys.stderr)
                failures += 1
                results.append((name, '失敗', detail.strip()))
        if args.command == 'pull':
            print_pull_summary(results, args.dry_run)
        if args.command == 'lock' and not failures:
            lines = ['# Exact commits; commit this file with packages.repos.', 'repositories:']
            for name, item in locked.items():
                lines += [f'  {name}:', '    type: git', f'    url: {json.dumps(item["url"])}', f'    version: {json.dumps(item["version"])}']
            output = args.output or root / 'packages.lock.repos'
            if args.dry_run:
                print('\n'.join(lines))
            else:
                temporary = output.with_name(output.name + '.tmp')
                temporary.write_text('\n'.join(lines) + '\n')
                temporary.replace(output)
                print(f'Saved: {output}')
        return 1 if failures else 0
    except (ValueError, OSError) as exc:
        parser.error(str(exc))


if __name__ == '__main__':
    raise SystemExit(main())
