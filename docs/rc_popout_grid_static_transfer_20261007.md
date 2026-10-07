# 固定grid背景方式の静止記録への適用

更新：2026-10-08。静止18記録の実行結果を受領し、36行すべて正常終了、飛び出しあり12件で両センサとも各1候補、飛び出しなし6件で候補0件を確認した。[集計と解釈](rc_popout_grid_background_static_results_20261008.md)を参照。候補位置の映像照合はこれから行う。

自車移動条件で固定した背景モデル・閾値を、静止条件へ変更せず適用する。静止条件に合わせて再調整する実験ではなく、同じ方式が両条件でどこまで働くかを確認する。

## 対象と固定条件

既存の `/workspaces/record/09-28/common_detection_roi.json` を入口とし、注釈が参照するRGB/RAW・LED同期・評価区間を使用する。

| 接頭辞 | 左 | 右 | 飛び出しなし |
|---|---|---|---|
| `popout-0928-static` | `_01`〜`_03` | `_04`〜`_06` | `_07`〜`_09` |
| `popout-0928-static-100` | 番号なし・`_01`・`_02` | `_03`〜`_05` | `_06`〜`_08` |

飛び出しあり12件・なし6件。20%の番号なし記録は従来どおり予備収録として除外する。20%・100%はプロポ設定であり、計測した速度ではない。これらは過去の静止解析に使用済みであり、完全に未知のテスト集合とは呼ばない。

固定パッケージは [grid_background_frozen_20261006](evidence/rc_popout_20260930/grid_background_frozen_20261006/)。モデル・閾値・抽出コードのハッシュを検査し、不一致なら処理を止める。

- 歪み補正後のEVS座標640×480、rotation-only投影。
- ROI：`x=0, y=70, width=640, height=272`。32×32画素、同じタイルID・有効画素数を要求する。
- RGB：連続フレームの輝度差15以上の画素数。EVS：直前2 msのイベント数、1 ms刻み。
- 背景モデル2個と各センサの閾値をそのまま使用する。実際に静止していることを理由に `pre_drive` モデルを強制しない。各観測で背景再構成誤差に基づく既存の選択処理を使う。
- 発進時刻は作らない。全保存評価区間を処理し、RGB初出現時刻や方向は推論に使わない。

旧静止解析はROI `x=0,y=150,width=640,height=163`、fixed-depth 2.2 mである。既存のスコア配列を流用せず、**固定方式の入力条件で元のRGB/RAWから再抽出する**。元の設定・注釈・同期ファイルは書き換えない。

時間だけを保存した注釈では `spatial` が `null` または未保存の場合がある。その場合も共通設定の校正ファイルと固定パッケージのハッシュが一致すれば、保存された評価区間・RGB初出現時刻を使って処理する。注釈側の投影を推測して補完せず、`annotation_spatial_status=not_recorded`、`annotation_geometry_changed=null` と記録し、端末にも表示する。共通設定の `spatial` まで欠損している場合や、保存済みの注釈側校正が不一致の場合は停止する。

## 実行

追加した [Python](../tools/evaluate_rc_popout_grid_static.py) と [シェル](../scripts/experiments/evaluate_rc_popout_grid_static.sh) をデータ側の `/workspaces` に反映して実行する。SSHによる操作は行わず、以下は利用者側で実行するコマンド。

まずファイル・注釈・固定パッケージの整合性を確認する。`--preflight` は動画やRAWをデコードせず、出力も作らない。

```bash
cd /workspaces

bash scripts/experiments/evaluate_rc_popout_grid_static.sh \
  --config /workspaces/record/09-28/common_detection_roi.json \
  --output /workspaces/record/09-28/analysis/grid_background_static_trial01 \
  --preflight
```

`Preflight passed` が表示されたら本処理を実行する。ROS・Metavision環境で元記録を読み、タイル抽出と固定モデルによる推論を行う。

```bash
cd /workspaces

bash scripts/experiments/evaluate_rc_popout_grid_static.sh \
  --config /workspaces/record/09-28/common_detection_roi.json \
  --output /workspaces/record/09-28/analysis/grid_background_static_trial01

cat /workspaces/record/09-28/analysis/grid_background_static_trial01/summary.csv
cat /workspaces/record/09-28/analysis/grid_background_static_trial01/errors.json
```

固定パッケージの既定パスは、このcheckout内の `docs/evidence/rc_popout_20260930/grid_background_frozen_20261006`。既存出力へ上書きしない。失敗後に入力を修正して再実行する場合も `trial02` など新しい出力名を使う。タイル抽出の配列見積りが512 MiBを超える場合は停止する。メモリに余裕があれば `--max-memory-mb` を明示できるが、これはROI・時間窓・閾値を変更しない。

初版の事前確認では `spatial: null` の注釈を辞書として参照すると `error: 'NoneType' object is not subscriptable` になった。このケースに対応し、必須設定の欠損にはファイル名・JSON項目名を付けるよう修正した。別の読み込み失敗が残る場合は、同じコマンドに `--debug` を付けて実際の例外発生行を確認できる。`--preflight` は出力を作らないため、その失敗だけなら同じ出力先で再確認できる。

## 出力の読み方

- `summary.csv/json`：18記録×RGB/EVSの36行。処理失敗は `status=failed` として残し、候補0件に置き換えない。
- `candidates.csv`：全候補。時刻は元記録の最初のRGB時刻を0秒とする。移動条件の `from_drive_s` とは異なる。
- `first_candidate_minus_onset_ms`：全評価区間の最初の候補とRGB初出現の差。早期誤警報も含めて保持する。
- `first_candidate_after_onset_minus_onset_ms`：初出現以後で最初の候補との差。これは診断用の集計であり、それ以前の警報を無視した検知成績にはしない。
- `candidates_before_onset`、`candidates_before_guard`、`active_at_rgb_onset`：初出現前の候補数・固定guard以前の候補数・初出現時に既に警報中だったかを併記する。RGB初出現直前のEVS候補は、同期・注釈の分解能と位置を確認して解釈する。
- `ready_observed_seconds`：欠測・境界のリセットを除いた観測時間。`active_seconds` は観測が連続した区間の左端の警報状態で積算し、欠測時間を警報継続に加えない。飛び出しなしの候補数と併せて見る。
- `selected_background_model`：選択された既存モデル名。`drive` と出ても静止記録が物理的に動いたという意味ではない。
- `baseline_summary.csv`：**今回と同一の投影・ROIで再計算した単純活動量方式**。RGB閾値0.001、EVS閾値20。以前の空間条件付きEVS方式や、旧投影の数値とは区別する。
- 各記録フォルダ：タイル配列、スコア、背景再構成・残差、全候補、元注釈のハッシュを保存。`preflight.json` に入力と旧設定／新設定、`run_config.json` に実行状態を記録する。

大きなRAW・MCAPはパス・サイズ・更新時刻で実行中の変更を検査する。転送前後の内容一致を保証する全ファイルのハッシュ検証ではない。注釈・同期・設定など小さなファイルはSHA-256を検査する。

## 結果取得後の確認

1. 36行が揃い、失敗がないことを確認する。処理に成功しても検知に成功したとは限らない。
2. 飛び出しなし6件の候補と観測時間、飛び出しあり12件の早期警報・初出現付近の候補を整理する。
3. 同じrotation-only条件の確認映像で候補位置を照合する。旧注釈のRGB取得時刻は保持しているが、投影・共通視野が変わるため、初出現の見え方も再確認する。検知結果に合わせて初出現を移動しない。
4. 静止と移動の結果を、候補位置・誤警報・反応時刻に分けて比較する。候補を車両検知と確定せず、閾値の再調整は別の探索実験として扱う。

## 静止結果の枠付き確認動画（2026-10-08追加）

[静止用ラッパー](../scripts/experiments/render_rc_popout_grid_static_reviews.sh)と
[確認ツール](../tools/rc_popout_grid_static_videos.py)を追加した。入力は保存済みの静止解析フォルダで、記録ルートは各注釈から取得する。移動用の評価splitや発進時刻を流用しない。

```bash
cd /workspaces

bash scripts/experiments/render_rc_popout_grid_static_reviews.sh \
  --static-dir /workspaces/record/09-28/analysis/grid_background_static_trial01 \
  --output /workspaces/record/09-28/analysis/grid_background_static_review_trial01 \
  --prepare-previews
```

`--prepare-previews` は解析と一致する動画がない記録だけ、既存の `generate_rc_popout_common_views.sh` でrotation-only動画を用意する。動画生成には従来と同じROS・Metavision・LED同期エクスポートが必要。一致する動画がある場合は再利用し、切り出しにはOpenCVとffmpegを使う。保存済み注釈が参照する同期と一致しなければ描画を拒否し、同期・注釈を自動で書き換えない。

同じコマンドに `--dry-run` を付けると、整合性検査と不足動画の生成コマンド表示だけを行う。`--prepare-previews` を併記してもdry-runでは動画や出力を作らない。不足時は終了コード1となる。

18記録・両センサの全結果を検査し、候補0件も一覧へ残す。入力候補数を24件に固定せず、早期・負例も含む保存済み全候補を処理する。今回受領したCSVでは12正例×2センサ＝24候補。凍結ファイル、解析コード、元注釈・同期・校正、保存済みタイル配列、alarm配列と候補時刻・座標・スコア・モデル名の整合を確認する。検出の再推論や学習は行わない。

出力は `index.html`、`summary.json`、`review_errors.json` と、各記録の `rgb_c001/`・`evs_c001/` 内の `candidate_review.mp4`、`contact_sheet.png`、`frames.csv`、`summary.json`。表示は録画基準秒、前後各0.3秒、4倍スロー。発進時刻は作らず `Static recording` と表示する。枠は候補開始時の固定タイルであり、追跡や警報継続表示ではない。

RGB初出現がクリップに含まれる場合は初出現前後もcontact sheetに載せる。含まれない早期候補・負例候補を除外せず、候補前後を表示する。EVS表示窓は元動画の設定を明示し、厳密なEVS初出現時刻の測定には使わない。`pre_drive/drive` は選択された背景モデル名で、実際の運動状態ではない。

まず `popout-0928-static_01`（20%左）と `popout-0928-static_04`（20%右）を条件内の先頭例として確認し、その後全候補を照合する。枠内が車体か背景か、初出現が遮蔽物端か画像端か、上下・左右の位置ずれとグリッド境界との関係を見る。良い結果だけを選んで評価しない。

新規の静止レビュー6件と、既存の静止評価9件・候補動画9件・移動評価動画6件のテスト計30件が成功。静止の全候補保持、ゼロ候補の一覧、入力不一致の拒否、dry-runの無書込、静止表示、実際のH.264生成・復号とフレーム対応を確認した。実データの動画生成・候補位置の判定は利用者側で上記を実行してから行う。

```bash
PYTHONPATH=tools:tests python3 -m unittest \
  test_rc_popout_grid_static_videos test_rc_popout_grid_static \
  test_rc_popout_candidate_videos test_rc_popout_grid_evaluation_videos -v
```

## 静止評価のローカル検証

静止用9件と既存の固定評価・grid背景方式のテストを合わせて23件成功。合成の抽出入力から18記録・36行の生成、固定値の不変、再適合禁止、発進時刻を作らない集計、注釈が候補時刻を変えないこと、入力変更の拒否、タイル活動量の保存、処理失敗の明示を確認した。時間だけの注釈の `spatial: null`／欠落を保持した評価、必須項目の欠損をデコード前に拒否すること、`--debug` による例外保持も含む。

元RGB/RAWのデコード境界は合成入力に置き換えている。実機環境でのROS・Metavisionデコード、静止18記録の候補数・位置・反応時刻は上記コマンドによる確認が必要。
