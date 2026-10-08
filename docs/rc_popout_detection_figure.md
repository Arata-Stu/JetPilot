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

## RGBの折れ線も表示する

同じ区画、同じ時間範囲のRGBとEVSの積算履歴を出力できる。折れ線のみなので、転送済みアーカイブがあれば元rosbagを読む必要はない。

```bash
cd /workspaces
bash scripts/experiments/render_rc_popout_detection_traces.sh \
  --bundle /workspaces/record/09-30/analysis/development_debug_bundle01.zip \
  --session test_05 \
  --before-ms 100 \
  --after-ms 500 \
  --output /workspaces/record/09-30/analysis/detection_traces_extended_trial01
```

重ね描きした **`traces_overlay.png/.svg`**、RGB単体 `trace_rgb.png/.svg`、EVS単体 `trace_evs.png/.svg`、左右に並べた `traces_rgb_evs.png/.svg` を生成する。軸ラベル・数値目盛り・凡例を付ける。左右表示では左がRGB、右がEVS。

- 既存EVS図の候補区画と同じ2区画（test_05では177・178）を固定し、既定ではRGB初出現の100 ms前〜500 ms後を表示する。RGB自身が選んだ候補位置ではない。
- 横軸はRGB初出現を0 msとし、50 ms刻みの目盛りを付ける。0 msの縦点線はRGB初出現、橙の丸は保存済みEVS候補（test_05では+105.1 ms）。表示区間を延長しても検知マーカーを末尾へ移動しない。区間外ならマーカーを描かない。
- `--before-ms` と `--after-ms` で範囲を指定する。保存済み評価データの範囲を超える指定や、欠測・リセットをまたぐ区間は拒否し、黙って切り詰めない。
- 元サンプルをそのまま使用し、RGBの小さな点でフレームごとの値を示す。検知後の観測もその取得時刻で表示し、再サンプリングや平滑化を加えない。
- 両図の横軸・縦軸の表示範囲は同じ。縦軸は各センサの積算値をそれぞれの固定しきい値で割った値で、点線は1。元のEVS図右下の非正規化グラフとは表示単位が異なる。
- 青がRGB、橙がEVS。実線が区画A（177）、破線が区画B（178）。同じA/Bは同じ画像位置を表す。
- test_05の固定済み条件では、全評価区間のRGB候補は0件、EVSは1件。RGBにも検知したような終点マーカーを付けない。
- test_05の−100〜+500 msでは、RGBの区画A/Bの最大しきい値比は約0.253/0.272。RGBの保存済み評価データは初出現の約−5.918〜+3.169 sで、全区画の全隣接ペアを対象にした最大検知スコア比は約0.966（しきい値未満）。この図の2本だけでなく、固定済み検知器の全区間出力でもRGB候補0件を確認した。
- EVSで事後選択した区画の挙動を説明する図であり、この2本だけでRGB全体の検出性能を評価しない。背景モデル・しきい値はセンサごとに異なる。しきい値比は確率や反応時間ではない。

元の値は `rgb_trace_values.csv/.npz` と `evs_trace_values.csv/.npz`、入力識別・しきい値・時刻は `summary.json` に保存する。

従来の文字なし図（`poster_detection_traces_20261009/test_05/`）と150 msの重ね描き版（`test_05_overlay/`）は保存してある。600 msに拡張した版は `poster_detection_traces_20261009/test_05_extended/`。`from_evs_candidate_ms` と `from_rgb_onset_ms` の両方を保存する。検知器の入力・設定・しきい値は共通で、表示区間だけを広げている。

## 静止100%のRGB・EVS判定スコアを重ねる

両方に車両対応の候補があった `popout-0928-static-100_01` を表示する。ノートPCのDocker内で実行する。

```bash
cd /workspaces
bash scripts/experiments/render_rc_popout_static_traces.sh \
  --static-dir /workspaces/record/09-28/analysis/grid_background_static_trial01 \
  --session popout-0928-static-100_01 \
  --before-ms 100 \
  --after-ms 500 \
  --output /workspaces/record/09-28/analysis/static100_detection_traces_trial01
```

主図は `traces_overlay.png/.svg`、確認ページは `index.html`。元の数値、全候補時刻、区画ID、入力ハッシュは `rgb_trace_values.csv/.npz`、`evs_trace_values.csv/.npz`、`summary.json` に保存する。既存の出力先は上書きしない。より拡大する場合は `--before-ms 50 --after-ms 150` と別の出力先を指定する。

- **RGB・EVS各1本の実際の判定スコア**を、各センサの固定しきい値で割って描く。各隣接2区画の積算値の小さい方をペアのスコアとし、その全ペア中の最大値を使用する。
- 以前の4本の図は同じ固定2区画の履歴だったが、今回の図では各センサの検知器全体の判定を表示する。RGBとEVSで選択する区画が異なっても、保存済みの検知時刻との対応を保てる。
- 青はRGB、橙はEVS。RGB初出現が横軸の0 ms、水平破線の1がしきい値。小さい青点はRGBの元サンプル、白抜きの丸と色付き縦線は表示範囲内の全候補開始時刻。未来フレームへの置換や平滑化はしない。
- 保存済み候補表では、EVSは **+9.834 ms（区画162,182）**、RGBは **+50.291 ms（区画162,163）**。これは指定した1試行の値。静止12試行中でEVS先行幅が最大の例なので、代表値として扱わず、集計結果は別に示す。
- 保存済みの活動配列から固定モデルの推論を再実行し、保存されたスコア全時系列と候補開始を照合してから描画する。再学習・しきい値変更・RAW/RGB再デコードは不要。
- 既存の静止映像レビューと同じ検証を用いるため、完了済み18記録の解析フォルダと、解析時の注釈・同期・校正・記録ファイルのメタデータが必要。変更を検出した場合は停止し、別の同期や注釈で黙って描き直さない。
- Mac側では合成データによる保存結果の再現・時刻/区画対応・PNG/SVG生成・入力不一致拒否を検証。実測静止スコア配列からの最終生成は上のコマンドを実行する。

## RGB写真を使わず、実イベント像で背景差分を説明する

方法説明用に、入力イベント像と区画の残差を濃淡で表したイベント像を並べる。既定の `--layout compact` は前後2枚の横並びで、左だけに薄いグリッドを置く。図内に文章・折れ線・検知時刻・候補枠は入れない。対象は既存の調整用記録 `test_05`。従来の下段に背景区画図を置く版は `--layout model` で生成できる。

```bash
cd /workspaces
bash scripts/experiments/render_rc_popout_event_figure.sh \
  --record-root /workspaces/record/09-30/dynamic \
  --session test_05 \
  --output /workspaces/record/09-30/analysis/detection_figure_events_trial01
```

この版は **Metavision SDKでRAWを読み直す**。以前のRGB中心の図と同じ固定済みEVS候補時刻・共通ROIを使用し、既定の表示蓄積は過去2 ms。全RAWを最後まで読み、遅れて現れるタイムスタンプも元解析と同じ時間ビンに加算する。記録全体のイベントをメモリへ蓄積せず、指定窓の画素ごとの個数だけを保持する。歪み補正後に丸めた座標とROI判定も元解析に合わせ、全区画のイベント数がアーカイブと一致した場合だけ画像を保存する。RGBやグリッドの個数から架空のイベント点は作らない。

### 表示だけ20 ms蓄積する

イベント点が疎で見えにくい場合は、左右とも同じ20 msを表示する。検知器の過去2 ms窓・背景モデル・しきい値・候補時刻は固定したまま。Docker内で実行する。

```bash
cd /workspaces
bash scripts/experiments/render_rc_popout_event_figure.sh \
  --record-root /workspaces/record/09-30/dynamic \
  --session test_05 \
  --layout compact \
  --display-window-ms 20 \
  --output /workspaces/record/09-30/analysis/detection_figure_events_20ms_trial01
```

- 可視化区間は候補時刻で終わる過去20 ms。候補時刻より後のイベントは含めない。
- 重複しない2 msブロックを10個抽出し、**各ブロックの各区画の個数**を元の解析配列と照合する。1 ms刻みの重複窓を足して二重計上することはない。
- 右図では各ブロックのイベントに、そのブロック末尾の固定済み残差を使う。最後の時刻の残差を過去20 ms全体へ一括で適用しない。同じ画素・極性に複数回発火がある場合は、発火回数で重み付けした平均の濃さとする。
- 表示コントラストは表示対象の全ブロックに共通の最大残差で正規化する。図中文字は増やさず、`index.html` と `summary.json` に **表示20 ms／検知2 ms** を明記する。
- `--display-window-ms 10` なら10 ms。指定は検知窓の整数倍（現在は2 msの倍数）、上限100 ms。長い蓄積はcompact版で使用する。保存済み評価区間の境界や欠測・リセットを跨ぐ指定は拒否する。
- test_05の固定アーカイブでは、同じ終端時刻で2 ms=673件、10 ms=2,663件、20 ms=7,183件。これは活動配列から確認した件数で、20 msの点座標と最終画像はRAWでの生成時に照合する。
- `source_values.npz` は、表示全体の極性別画素カウントに加え、`history_*` にブロック別カウント・時刻・残差を保存する。従来の `observed_density` 等は候補時刻で終わる検知2 ms窓の値として維持する。
- 既存の2 ms図には前の18 ms分の座標がないため、`--from-figure` だけで20 msへ延長することはできない。初回は上のRAWコマンドを使う。生成した20 ms版を転送すれば、その後のレイアウト変更は `--from-figure` で行える。

ポスターの図外キャプションには「可視化のイベント蓄積：20 ms（検知窓：2 ms）」を併記する。検知遅延の比較用には、別の判定スコアと候補時刻の図を使う。

- **`detection_method_events.png/.svg`**：イベント入力 → 区画の残差による濃淡表示。compact版では背景モデルの小図を省く。
- **`events_grid.png/.svg`**：実イベント像＋薄い32×32画素グリッド。
- **`residual_events_grid.png/.svg`**：同じイベント点を区画の正の標準化残差に応じて表示。互換性のためファイル名は維持するが、compact版では格子線を描かない。
- `estimated_background_grid.png/.svg`：背景活動の推定。背景のイベント座標を復元した画像ではない。
- `events.png` / `residual_events.png`：グリッドなしの個別素材。
- `index.html` / `summary.json` / `source_values.npz`：図外の説明・窓時刻・RAW識別・区画照合結果・極性別の実画素カウント・重み。

青はOFF、赤はONイベント。両極性が重なる画素は混色。図では発火した画素を表示するが、数値では同一画素の複数イベントも保持する。点の膨張・RGB画像の重畳・候補部分だけの切り抜きは行わない。

各ブロックのイベント点の透明度は `clip(positive_residual / residual_display_max, 0, 1)`。model版は固定済みの `z_clip=10`、compact版は表示対象の最大残差を表示上限とする。test_05の2 ms図では最大残差が約3.901なので、compact版はmodel版より濃く表示する。**この変更は表示コントラストだけで、元の残差値・検知スコア・しきい値を変更しない。** compact版の濃さはシーン間比較には使わない。

**区画の残差を実イベント点上で可視化したもので、画素単位の背景除去・セグメンテーションを実現したという意味ではない。** 同じ区画内の車両と背景のイベントは同じ重みになる。実際の判定には、この区画残差の時間積算と隣接区画の評価が続く。

### 転送済み実データからの再描画

生成結果にある `summary.json` と `source_values.npz` から、RAWを再度読まずに描き直せる。固定アーカイブとモデル、候補時刻、転送ファイルのハッシュ、全区画のイベント数・残差値を照合する。NumPy・Matplotlib・Pillowが必要で、ROSやMetavisionは不要。リポジトリ直下から実行する例：

```bash
bash scripts/experiments/render_rc_popout_event_figure.sh \
  --from-figure /Users/at/Downloads/scp/detection_figure_events_trial01 \
  --session test_05 \
  --layout compact \
  --output /tmp/detection_figure_events_compact
```

既定のアーカイブはチェックアウト内の `record/09-30/analysis/development_debug_bundle01.zip`。別の場所にある場合は `--bundle` を指定する。出力先は毎回新しい場所にする。

2026-10-09に受領した実測673件を用いてcompact版を生成し、元の全区画の値との一致と画像を確認した。保存先は `docs/evidence/rc_popout_20260930/poster_event_compact_20261009/`。

図外キャプション例：

> 実イベント像を区画に分け，背景活動の推定からの偏差を求める．右図は区画の偏差をイベント点の濃さで示す．

ローカルでは、時間ビン境界、重複イベント、後続バッチの遅延イベント、元の区画個数との照合、実在しない画素に点を追加しない描画、文字なし出力を合成入力で検証。実RAWによる生成・画質確認はノートPCのDocker内で行う。
