"""Summarize existing trtexec measurements; no TensorRT or GPU imports."""
import argparse
from pathlib import Path
import re


def summarize(text: str) -> str:
    if '=== Performance summary ===' not in text:
        raise ValueError('推論統計がログにありません。trtexecの出力を確認してください。')
    section = text.rsplit('=== Performance summary ===', 1)[1]
    rows = []
    labels = (
        ('GPU Compute Time', 'GPU計算'),
        ('Latency', '転送込み'),
        ('H2D Latency', 'CPU→GPU転送'),
        ('D2H Latency', 'GPU→CPU転送'),
        ('Enqueue Time', 'CPU投入処理'),
    )
    for key, label in labels:
        match = re.search(r'(?:^|\n)(?:\[[^\]\n]*\]\s*)*' + re.escape(key) + r':\s*([^\n]+)', section)
        if not match:
            continue
        values = dict(re.findall(r'(mean|median|percentile\(95%\)|percentile\(99%\))\s*=\s*([0-9.eE+\-]+)\s*ms', match[1]))
        if 'mean' not in values:
            continue
        cells = [values.get(field, '—') for field in ('mean', 'median', 'percentile(95%)', 'percentile(99%)')]
        rows.append(f'{label}: 平均 {cells[0]} / 中央値 {cells[1]} / P95 {cells[2]} / P99 {cells[3]} ms')
    if not rows:
        raise ValueError('推論時間を読み取れませんでした。元のログを確認してください。')
    throughput = re.search(r'Throughput:\s*([0-9.eE+\-]+)\s*qps', section)
    if throughput:
        rows.append(f'処理数: {throughput[1]} 推論/秒（trtexecの実行条件）')
    rows.extend('注意: ' + line.split('[W]', 1)[1].strip() for line in section.splitlines() if '[W]' in line)
    return '\n'.join(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', nargs='+', type=Path)
    args = parser.parse_args()
    failed = False
    print('\n=== TensorRT 推論時間のまとめ ===', flush=True)
    for path in args.logs:
        print(f'\n対象: {path.name}\nログ: {path}', flush=True)
        try:
            print(summarize(path.read_text(errors='replace')))
        except (OSError, ValueError) as error:
            print(f'統計表示エラー: {error}')
            failed = True
    print('\nP95/P99: 計測の95%/99%がこの時間以下。')
    print('このGPUでのモデル単体の計測です。画像取得・前後処理・ROS通信は含みません。')
    if len(args.logs) > 1:
        print('分割エンジンを個別に計測した結果です。全体の遅延やFPSを表すものではありません。')
    return int(failed)


if __name__ == '__main__':
    raise SystemExit(main())
