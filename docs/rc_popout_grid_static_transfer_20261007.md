# 固定grid背景方式の静止記録への適用

更新：2026-10-07。実装・合成データによる検証まで完了。**静止18記録での実行結果は未取得**。

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

## ローカル検証

新規6件と既存の固定評価・grid背景方式のテストを合わせて20件成功。合成の抽出入力から18記録・36行の生成、固定値の不変、再適合禁止、発進時刻を作らない集計、注釈が候補時刻を変えないこと、入力変更の拒否、タイル活動量の保存、処理失敗の明示を確認した。

元RGB/RAWのデコード境界は合成入力に置き換えている。実機環境でのROS・Metavisionデコード、静止18記録の候補数・位置・反応時刻は上記コマンドによる確認が必要。
