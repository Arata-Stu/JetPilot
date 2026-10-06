# 局所検知の監査：警報が長くつながる理由を調べる

`local_dev_trial01` のRGBは、負例・全正例とも発進後の候補が長く続いた。
候補回数だけを見ず、警報が続いた時間と場所を確認する。
結果の数値と解釈は [rc_popout_local_detection.md](rc_popout_local_detection.md) に保存した。

## 2026-10-06：監査第1試行の実データ結果

ユーザー提示の集計では10ケースが成功し、`review_errors.json` は空配列。
元の数値は [summary.csv](evidence/rc_popout_20260930/local_review_trial01/summary.csv) に保存した。
ここで実際に読めたのは集計値であり、実データのNPZ・ペア別CSV・比較図はまだこちらで見ていない。

| session | RGB走行中ON率 | EVS走行中ON率 | RGBの独立ペア反応数 | EVSの独立ペア反応数 |
|---|---:|---:|---:|---:|
| `t_0.2-none` | 96.04% | 17.21% | 162 | 10 |
| `test_01` | 95.46% | 41.87% | 224 | 98 |
| `test_05` | 95.58% | 30.16% | 228 | 93 |
| `test_11` | 95.79% | 39.03% | 211 | 100 |
| `test_14` | 95.07% | 40.21% | 227 | 81 |

ON率は走行指令区間内の準備済み観測時間に対する割合。独立ペア反応数は全保存区間の値で、互いに重複するペアも含む。

確認できたこと：

- RGBは正例・負例のどちらでも約95～96%の時間でON。現設定の全体警報は飛び出し有無を区別できていない。
- RGBの全体警報中に最大ペアが41～51回切り替わっている。隣接ペア同士の微小な順位変動も含むため、その回数を独立物体の数とは読まない。
- 正例RGBの初出現周辺で新たに始まったペア反応は26・49・62・55件で、全て既存の全体警報中に始まっている。EVSも8件中7件、33件中32件、52件中51件、43件中43件が同様。
- 従って元の全体候補一覧が、各場所の反応開始を隠していることは確認できた。ただし、それらが対象車両への反応かは集計から分からない。
- 負例RGBにも162件の独立ペア反応がある。候補を空間ごとに分離するだけでは背景への反応は除去できない。
- EVSは負例より正例で走行中ON率が大きいが、対象の検知時刻や先行性の証明にはならない。負例の走行中ON率17.21%も残っている。
- float32閾値近傍カウントは全ケース0で、今回の±1e-8の監査範囲では境界値の丸め問題は報告されていない。

次の判断には元のタイル時系列が必要。現在の0.05という開始閾値とスケール下限は未調整の初期値であり、
この集計だけで「閾値を変えれば分離できる」「活動量特徴では不可能」のどちらも確定しない。

次は調整用5件の保存NPZとメタデータをローカルへ受け取り、背景と出現付近のスコアの重なりを直接調べる。
まず現方式の空間・時間分布と負例基準の閾値候補を数値で確認し、正例の出現前誤反応・出現後候補の位置・遅れを併記する。
設定を変える場合も調整用だけで選び、全体警報と空間ごとの候補を分けて記録する。
背景の反応と対象付近の反応が分離しなければ、変化量に加えて動きの方向などの特徴を検討する。現時点ではその追加方式を実装・実証したとは扱わない。
ROIや繰り返し現れる背景位置を結果に合わせて切り捨てたり、初出現時刻を検知開始の条件にしたりしない。評価用は使用しない。

## 調整用データの受け渡し

2026-10-06に以下のZIPを受領し、[時系列の直接解析結果](rc_popout_development_bundle_analysis_20261006.md)を保存した。
下記は受け渡し手順の記録であり、今回のZIPを再作成する必要はない。

次のコマンドは保存済みの調整用5件のタイル時系列と設定・結果をZIPへまとめる。
元ファイルは変更せず、RAW・動画・評価用時系列を含めない。既存ZIPがあれば上書きせず停止する。
受け取り後は局所検知結果に記録された入力ハッシュと照合し、保存当時のデータとして解析する。
このZIPだけで現在のRAW・同期YAML・注釈との一致まで再検証したとは扱わない。

```bash
python3 - <<'PY'
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

root = Path("/workspaces/record/09-30/analysis")
sessions = ["t_0.2-none", "test_01", "test_05", "test_11", "test_14"]
paths = [root / "tile_dev_trial01/run_config.json"]
for name in ("run_config.json", "detector_parameters.json", "summary.csv", "candidates.csv"):
    paths.append(root / "local_dev_trial01" / name)
for name in ("summary.csv", "review_errors.json"):
    paths.append(root / "local_review_trial01" / name)
for session in sessions:
    for name in ("rgb_tiles.npz", "evs_tiles.npz", "tiles.json", "result.json"):
        paths.append(root / "tile_dev_trial01" / session / name)
    paths.append(root / "local_dev_trial01" / session / "result.json")

missing = [str(p) for p in paths if not p.is_file()]
if missing:
    raise SystemExit("Missing files:\n" + "\n".join(missing))
out = root / "development_debug_bundle01.zip"
with ZipFile(out, "x", compression=ZIP_DEFLATED) as bundle:
    for p in paths:
        bundle.write(p, p.relative_to(root))
print(f"Saved: {out}\nFiles: {len(paths)} / Size: {out.stat().st_size / 1024**2:.1f} MiB")
PY
```

このZIPを会話へ添付する。SSHによる取得は行わない。

## 実行

今回の追加ファイルを解析側へ反映してから実行する。
**検知器と閾値は変更しない。RAW、元タイル抽出、検知処理の再実行は不要。**
NumPyと、保存済みの局所検知結果、元タイル出力の `run_config.json`・`tiles.json` が必要。

```bash
cd /workspaces

bash scripts/experiments/review_rc_popout_local_changes.sh \
  --local-dir /workspaces/record/09-30/analysis/local_dev_trial01 \
  --output /workspaces/record/09-30/analysis/local_review_trial01

cat /workspaces/record/09-30/analysis/local_review_trial01/summary.csv
cat /workspaces/record/09-30/analysis/local_review_trial01/review_errors.json
```

`index.html` を開くと、各正例と負例の局所状態を比較できる。
処理するのは固定した調整用5記録のみ。保存先が存在すれば中止する。
元の結果は上書きしない。元の全体警報はCSVを正とし、ここで再計算した別の警報に置き換えない。
保存された状態同士の時刻・列・スコア整合を検査し、読み込んだファイルのハッシュを残す。
これは保存当時の結果の監査であり、現在の同期・注釈でRAWを解析し直す処理ではない。

## 確認する3点

1. **警報の占有時間。** 候補が3回から1回に減っても、長くつながっただけなら改善ではない。
2. **警報中の最大ペアの切替。** 最大値を与える場所が変わり続けていれば、1件の長い候補を同じ物体の追跡とは読めない。
3. **全体警報中に別ペアで始まった反応。** 対象付近に新しい反応があっても、全体警報がONなら元の候補一覧には別候補として現れない。

3が見つかっても、それ自体は新方式の検知成功ではない。背景候補も増えるため、空間ごとの候補管理と背景抑制を分けて考える。
対象付近の変化がペア状態にも残っていないなら、候補の統合方法だけを直しても解決しない。
現在の特徴量・背景推定を見直す判断に使う。

## 出力

| ファイル | 内容 |
|---|---|
| `summary.csv`, `summary.json` | 全体警報の占有時間・割合、走行指令中の占有時間、場所の切替回数、独立ペア反応の集計 |
| `pair_episodes.csv` | 全時間・全ペアの独立したヒステリシス反応。ペアIDと座標、開始・終了、全体警報中の開始かどうか |
| `onset_pair_starts.csv` | RGB初出現−150～+250 ms内に始まったペア反応。診断の表示用であり正解対応付けではない |
| `index.html`, `<session>.svg` | RGB初出現−100／+100 msと、負例の同じ発進経過時刻のタイル状態 |
| `review_errors.json` | 図のエラー。成功なら `[]` |
| `input_hashes.json`, `run_config.json` | 入力と監査コードのハッシュ、元設定 |

主な集計列は次の通り。

- `global_episodes`：元検知器の候補数。
- `global_active_s` / `global_active_fraction`：準備済み観測時間中の、元の全体警報がONの秒数／割合。
- `drive_active_s` / `drive_active_fraction`：走行指令区間内の同指標。実測の車両運動区間とは区別する。
- `pre_onset_active_s`：初出現の30 ms前までの全体警報ON時間。準備済み全保存区間が対象。
- `winning_pair_changes_during_alarm`：全体警報が続く隣接サンプル間で、最大ペアIDが変わった回数。微小な順位変化も含み、物体数ではない。
- `independent_pair_episodes`：各ペアを別々に扱った反応数。隣接・重複するペアで同じ現象が重複して数えられる。
- `pair_starts_during_existing_global_alarm`：直前サンプルから全体警報が継続中なのに、新たに開始閾値を超えたペア反応数。
- `onset_window_pair_starts` と `onset_window_starts_during_global_alarm`：初出現−150～+250 msの診断窓内に限定した上記の回数。

占有時間は現在サンプルの状態を次サンプルまで保持した区間 `[t_i, t_(i+1))` の和。
両端が準備済みの区間だけを数え、欠測・リセット・interval境界をまたがず、最後のサンプル以降を推測で補わない。未準備時間を成功時間として数えない。
元検知器の `ready_observed_seconds` は直前ステップで積算するため、準備開始や境界の1サンプル分で分母が異なり得る。
空間的に重なるペアの継続時間を足して、全体警報の時間にすることはしない。

## 精度と解釈

独立ペアの再現には保存済み `cusum_s`（float32）を使う。元検知器はfloat64で判定しているため、閾値付近では1サンプル違う可能性がある。
`near_float32_threshold_values` は開始／終了閾値から1e-8以内のペア値の個数。この再現は診断用で、正式な検知時刻にはしない。
元CSVの全体スコアと、保存状態から得た全ペア最大値がfloat32相当の精度で一致しなければ中止する。

図は各目標時刻以下の最後の準備済みサンプルを使い、未来側のフレームを選ばない。データがない時間は欠測表示する。
色は積分値／開始閾値で、2以上を同じ色にする。赤枠は各タイルの値が開始閾値以上であることだけを意味する。
部分タイルもセル全体を着色するが、元スコアは有効画素だけから計算されている。
負例との時刻合わせは発進指令基準であり、車両位置や背景画像が一致する保証はない。
対象位置は映像で照合する。反復して現れる背景タイルを、この結果だけで除外マスクにしてしまわない。

## 検証

```bash
python3 -m unittest discover -s tests -p 'test_rc_popout_local_review.py' -v
bash -n scripts/experiments/review_rc_popout_local_changes.sh
```

合成データで、場所を変えた反応が全体警報としてつながる場合、ペア反応と全体占有時間を区別できることを確認する。
欠測・準備状態・境界、過去側サンプルの選択、float32閾値境界、入力不整合の拒否、元結果の保存も検証した。
合成比較図の描画確認済み。実データの監査集計は上記ユーザー提示結果で確認した。
