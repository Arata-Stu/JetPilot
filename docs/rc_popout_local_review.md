# 局所検知の監査：警報が長くつながる理由を調べる

`local_dev_trial01` のRGBは、負例・全正例とも発進後の候補が長く続いた。
候補回数だけを見ず、警報が続いた時間と場所を確認する。
結果の数値と解釈は [rc_popout_local_detection.md](rc_popout_local_detection.md) に保存した。

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
合成比較図の描画確認済み。実データの監査はユーザー実行待ち。
