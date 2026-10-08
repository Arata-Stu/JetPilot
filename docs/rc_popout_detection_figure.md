# 実測データを使った検知方法の図

ポスター用の文字なし図を生成する。調整用記録の実際のRGBと、保存済みRAW由来の32×32画素活動データを使用し、固定済みの背景モデルと検知条件を再実行する。モデルの再学習、しきい値調整、注釈や同期ファイルの変更は行わない。

## 実行（ノートPCのDocker内）

```bash
cd /workspaces
bash scripts/experiments/render_rc_popout_detection_figure.sh \
  --record-root /workspaces/record/09-30/dynamic \
  --session test_05 \
  --output /workspaces/record/09-30/analysis/detection_figure_real_trial01
```

既存の校正用Python環境を使用する。NumPy、OpenCV、Matplotlib、PillowとRGBのrosbag読取り環境が必要。RAWの再デコードは不要。

既定では `/workspaces/record/09-30/analysis/development_debug_bundle01.zip` を読む。別の場所に保存した場合は `--bundle` で指定する。アーカイブのSHA-256は固定済み解析の入力と照合する。対象は `test_01`、`test_05`、`test_11`、`test_14`。どれも調整用記録であり、この図は新しい評価成績ではない。

`--preflight` を付けると入力同一性の確認だけを行い、RGBのデコードやファイル出力は行わない。既存の出力先は上書きしない。

## 出力

- `detection_method_real.png`：白背景、3600×1320画素、図中文字なし。
- `detection_method_real.svg`：RGB画像を内包した図。矢印・区画・グラフはベクトル。
- `observed_density.png` / `positive_residual.png` / `integrated_residual.png`：上段3枚の個別素材。
- `estimated_background.png`：背景モデルの推定値だけを示す区画図。
- `rgb_full.png` / `rgb_roi.png`：実際のRGBをEVS座標へ投影した画像。
- `index.html`：図と読み方の確認用。説明文は図の外に置く。
- `summary.json` / `source_values.npz`：元データ識別、時刻、表示尺度、数値。

図の上段は「活動密度 → 背景との差 → 時間積算と候補区画」。下段は背景推定マップと、候補2区画の直前150 msの積算履歴。青が活動密度、橙が残差・積算値、橙の枠が固定済み検知時刻に選ばれた隣接2区画を示す。枠は物体のbboxやセグメンテーションではない。

図外キャプション例：

> 実測EVS活動から背景との差を求め，区画ごとに時間積算した例．枠は検出候補となった隣接区画を示す．RGB画像は位置関係を示すために重ねた．

## 時刻・座標の扱い

- 保存済み候補時刻と隣接2区画を再現できた場合だけ出力する。
- RGBはそのEVS時刻以前で最も新しい取得フレーム。未来のフレームや補間画像を使用しない。両者の差を `rgb_age_ms` に保存する。
- RGBの記録原点と、保存済みRGB活動サンプルの全時刻を、元rosbagのタイムスタンプと照合する。
- 注釈・校正・同期は解析時のハッシュと照合する。同期は手動／自動の名前ではなく、固定済み解析と一致する内容で選ぶ。現在の同期を変更済みの場合、解析時と同じ同期がなければ停止する。
- 校正済み640×480 EVS座標、rotation-only投影と共通ROI（x=0, y=70, width=640, height=272）を維持する。境界区画の有効画素数も確認する。視差は残る。
- 活動と背景推定は同一の色尺度。標準化残差と積算値はそれぞれ別尺度であり、色の濃さを列間で直接比較しない。積算値の濃さは検知しきい値で飽和する。
- RGB背景上の色は実測／推定された区画の数値であり、画素単位で背景除去したイベント画像ではない。背景推定の小図にはRGBを重ねず、対象車を背景と誤読しにくくする。

## ローカル検証の範囲

保存済み調整用アーカイブによる固定済み候補・数値の再現、過去RGBフレームの選択、校正投影と区画の有効面積、入力不一致の拒否、文字なし出力を検証した。元rosbagを使った最終生成は、上記コマンドをノートPCのDocker内で実行して確認する。
