#!/usr/bin/env python3
"""標準ライブラリだけで収録ファイルを監査。内容の正常性は別途検証する。"""
import argparse
import json
from pathlib import Path


def audit_session(session):
    session = Path(session)
    mcaps = sorted(session.glob('*.mcap'))
    raws = sorted(session.glob('*.raw'))
    files = [session / 'metadata.yaml', *mcaps, *raws]
    issues = []
    if not mcaps:
        issues.append('missing_mcap')
    if not raws:
        issues.append('missing_raw')
    for raw in raws:
        files.append(Path(str(raw) + '.metadata.yaml'))
    for path in files:
        if not path.is_file():
            issues.append('missing:' + path.name)
        elif path.stat().st_size == 0:
            issues.append('empty:' + path.name)
    directions = [value for value in ('left', 'right', 'none')
                  if f'_dir-{value}_' in session.name]
    direction = directions[0] if len(directions) == 1 else None
    return dict(
        recording_id=session.name, session_path=str(session.resolve()),
        planned_direction_from_filename=direction,
        # ファイル名は予定条件のみ。映像を見て実際のラベルを付ける。
        scene_label='unknown', split='unassigned',
        evaluation_start_ns=None, evaluation_end_ns=None,
        evaluation_clock_id=None, sync_quality='unchecked',
        content_quality='unchecked', issues=issues,
        file_status='incomplete' if issues else 'present_not_validated',
        files=[dict(path=str(p.resolve()),
                    size_bytes=p.stat().st_size if p.is_file() else None)
               for p in files],
    )


def discover(root):
    root = Path(root)
    # metadata欠損やRAWだけ残った中断記録も発見する。
    candidates = set()
    for pattern in ('metadata.yaml', '*.mcap', '*.raw'):
        candidates.update(p.parent for p in root.rglob(pattern) if p.is_file())
    return sorted(candidates)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if not args.root.is_dir():
        parser.error('--rootには記録ディレクトリを指定してください')
    if args.output.exists():
        parser.error('出力は既存ファイルを上書きしません。別名を指定してください')
    sessions = [audit_session(p) for p in discover(args.root)]
    report = dict(
        schema_version=1, audit_mode='file_inventory_only',
        limitation='画像・イベントの復号、トピック内容、LED同期は未検証',
        session_count=len(sessions), sessions=sessions,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    incomplete = sum(bool(s['issues']) for s in sessions)
    print(f'記録数: {len(sessions)} / ファイル不足・空ファイル: {incomplete}')
    print('ファイルが揃っていても、内容と同期の正常性は未確認です。')
    return 2 if incomplete or not sessions else 0


if __name__ == '__main__':
    raise SystemExit(main())
