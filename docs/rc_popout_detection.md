# 共通ROIを使う活動量ベースライン

2026-09-30時点の実験結果は[予備評価の結果まとめ](rc_popout_results_20260930.md)を参照。

`tools/rc_popout_detection.py`は`common_detection_roi.json`の全sessionを逐次処理します。
各sessionの`sequence_annotations.json`から評価区間とRGB初出現を読みます。
個別の`rois`ではなく共通設定の`roi`を採用し、注釈ファイルは変更しません。
UIに共通ROIを表示する接続は、このコマンドには含まれません。

## 実行

Docker内で更新したroot側のコードを反映して実行します。新しいROSビルドは不要です。

```bash
cd /workspaces
bash scripts/experiments/run_rc_popout_detection.sh \
  --config /workspaces/record/09-28/common_detection_roi.json \
  --output /workspaces/record/09-28/analysis/detection_trial01 \
  --rgb-threshold 0.001 \
  --evs-threshold 20
```

上記閾値は動作確認用の仮値で、研究結果用に検証された値ではありません。
出力先が存在する場合は停止します。試行ごとに新しい出力先を指定してください。
処理が失敗した記録はsummaryへ理由を記録し、残りを続けます。失敗があれば終了コード1です。

## 測定方法

- RGB：元bagの取得時刻で更新。歪み補正し、指定深度でEVS座標へ投影します。
  同じ評価区間内の前のRGBとのグレースケール差が15/255以上の有効画素割合です。
  `--rgb-pixel-delta`で画素差、`--rgb-threshold`で割合閾値を指定します。
- EVS：元RAWの各イベント位置を歪み補正したEVS座標へ対応付け、共通ROI内の
  イベントを極性によらず数えます。同じ画素の複数イベントも数えます。
  LED同期を適用した共通時刻で既定1 ms刻み、直前2 msのイベント数を計算します。
  時間窓は`--step-ms × --window-bins`です。未来のイベントは使いません。
- ROI：共通の帯状矩形と、校正から再計算した両センサの有効領域との交差を使います。
  校正ハッシュ・投影行列・同期ハッシュ・RGB時刻原点の不一致はエラーにします。
- 評価区間は終了排他。exclude区間を除外し、区間境界ではRGB差分とEVS窓をリセットします。
  各区間の最初のRGBと最初のEVS窓完成までにはスコアがありません。
- 閾値以上が連続する期間を1活動エピソードとし、立ち上がりを候補時刻にします。
  チャタリング抑制や車両分類はまだありません。

RGBとEVSのスコアは単位が異なり、閾値の数値自体を揃える必要はありません。
調整用記録と評価用記録を分け、センサごとの閾値を固定してから比較してください。
全18件を見て閾値を調整した結果は探索的な結果であり、独立した性能評価ではありません。

## 出力

- `run_config.json`：共通ROI、ハッシュ、全パラメータ
- `summary.csv` / `summary.json`：全記録の結果・失敗理由
- `index.html`：結果一覧と波形へのリンク
- `<session>/rgb_scores.csv` / `evs_scores.csv`：元記録相対秒での全スコア
- `<session>/scores.svg`：スコア波形、赤の閾値線、緑のRGB初出現線
- `<session>/result.json`：全候補時刻、有効画素数、活動エピソード数など

`first_trigger_minus_rgb_onset_ms`が負でも、それだけで早期検知成功とは言えません。
最初の閾値超過が背景・ノイズである可能性があります。全候補と波形・映像を確認してください。
初出現のない記録は`no_rgb_onset`と表示し、記録条件を自動で断定しません。
飛び出しなしと確認した記録では、活動エピソードを誤警報候補として調べられます。
`episodes_per_minute`の分母は注釈された評価時間で、冒頭のウォームアップ時間も含みます。

これはオフラインの単純な活動量ベースラインで、車の認識や通信・推論を含む実機遅延ではありません。
同期精度・視差・RGBの観測間隔による不確かさは別途確認してください。
RAWの最初・最後のイベントが評価区間を覆わなければ、欠損を無活動と扱わずエラーにします。

## RAWイベントの順序

イベントの格納順がタイムスタンプ順でなくても、各イベントをその時刻のビンへ加算します。
バッチ内・バッチ間の順序逆転の両方に対応し、時刻の変更・イベント削除はしません。
評価区間終了後のバッチに遅れて含まれるイベントを取りこぼさないようRAW全体を走査します。
そのため、短い評価区間でもRAWのデコード時間がかかります。
`result.json`の`event_ordering`に逆転回数と隣接イベントの最大逆転幅（µs）を記録します。
この処理は格納順への対応であり、時計のリセットや壊れたタイムスタンプを補正するものではありません。
大きな逆転幅がある場合は診断値と元記録を確認してください。

## 空間的なまとまりとの比較

```bash
bash scripts/experiments/run_rc_popout_detection.sh \
  --config /workspaces/record/09-28/common_detection_roi.json \
  --output /workspaces/record/09-28/analysis/detection_trial04_spatial \
  --rgb-threshold 0.001 --evs-threshold 20 \
  --spatial --tile-px 32 --min-active-pixels 3 --spatial-threshold 20
```

RGBと従来のEVS全ROIスコアはそのまま残します。追加方式はEVS座標原点に固定した
32×32 pxタイルに区切り、同じ過去2 ms窓で3画素以上が活動したタイルの最大イベント数を
スコアとします。閾値20以上で検知候補となります。20イベントと3画素は必ず同じタイルの
条件です。重複イベントはイベント数には数え、画素数は時間窓全体で一度だけ数えます。
共通ROI外は除外し、極性は合算します。元イベントは削除・補正しません。

上記は仮設定です。単一画素の集中発火や分散した活動を抑える狙いですが、複数画素の
局所ノイズは通過することがあります。タイル境界に車がまたがるとスコアが下がる場合も
あります。車の分類や一般的なノイズ除去を保証する方式ではありません。検知時刻に
過去への補正は加えず、空間条件を満たした時点で比較します。

追加出力:

- `summary.csv`：`evs_spatial_first_trigger_s`, `evs_spatial_episodes`
- `<session>/evs_spatial_scores.csv`：条件付き最大タイルイベント数、条件なし最大タイル
  イベント数、最大タイル活動画素数（後者2つの最大値は同じタイルとは限りません）
- `<session>/spatial_scores.svg`：RGBと局所EVSの波形
- `index.html`：通常・局所方式の波形リンクと候補時刻
- `result.json`の`score_distribution`：各方式の50/95/99/99.9百分位点と最大値
- `background_distribution.csv`：RGB初出現が未設定の記録の分布と活動回数。
  今回は飛び出しなし6記録を想定しますが、未注釈の記録を混ぜないでください。

長さの異なる各記録を分けて分布を示します。背景記録を見て調整した後は、同じ記録だけを
使って独立した性能評価と主張せず、固定条件で別の記録にも検証してください。
空間方式はROI内イベントの画素・時間情報を保持して集計するため、従来よりメモリを使います。

## 自車移動：発進指令を基準にした区間別解析

LEDの配置・撤去に要した時間を条件比較に混ぜないため、既存の活動量CSVを
`/vehicle/control_cmd`で区切り直す。開始・終了の注釈、元のスコア、RAWは変更しない。

```bash
cd /workspaces
bash scripts/experiments/analyze_rc_popout_motion.sh \
  --scores-dir /workspaces/record/09-30/analysis/background_trial01 \
  --output /workspaces/record/09-30/analysis/motion_trial01 \
  --pre-drive-s 1.0
```

出力:

- `phase_summary.csv`: 発進前1秒、正スロットル指令中、brake指令中それぞれの活動量分布・
  閾値超過時間割合・エピソード数/秒。`complete=false`は有効区間不足や発進前の中立確認不足。
- `<session>/aligned_scores.svg`: 発進指令を0秒とした波形。指令段階の境界とRGB初出現を表示。
- `<session>/*_aligned_scores.csv`: 元記録時刻と発進相対時刻、phase付きの既存スコア。
  phase列はスコアの終端時刻で分類するが、区間別集計からは境界をまたぐ観測窓を除く。
- `<session>/commands.csv` / `alignment.json`: 元指令、切替時刻、同期基準、走行時の指令範囲。
- `summary.json` / `index.html`: 成否と波形へのリンク。

時刻は注釈と同じbag/header選択に揃え、先頭RGB時刻を引く。設定した走行秒数や
フォルダ名から発進を推定しない。指令がない、複数走行が評価区間に重なる、
発進・終了の遷移を観測できない、走行付近の指令間隔が既定0.1秒を超える場合はエラー。
`brake=0`で中立へ戻した記録にはbrake区間を捏造しない。

スコア観測窓がphase内に収まるものだけを集計する。RGBは前スコア時刻を直前フレームの
時刻として使用し、最初のスコアは観測窓の始点がCSVにないため除く。EVSは元設定の
窓幅を使う。除外区間をまたぐ集計は行わない。
閾値超過時間はスコアを次のサンプルまで保持して近似し、RGBは最大0.1秒、EVSは
更新周期の1.5倍で打ち切る。最後のサンプル以降は補間しない。割合と回数/秒の分母は
`observed_score_seconds`、その有効区間に対する割合は`score_coverage_fraction`に残す。
低coverageや不完全なphaseを完全な記録と同列に比較しない。

これは指令上の区間であり、実際の発進・停止とは区別する。スロットル0.1と0.2の
走行時間・加速過程の違いは残るため、同一速度や同一走行位置の比較とは主張しない。
飛び出しありも、通常の解析で活動量CSVを作った後に同じコマンドを使用できる。
RGB初出現が注釈されていれば発進からの相対時刻も出力する。
