# grid背景モデル：固定設定での内部評価手順

2026-10-06。候補映像の確認後、調整用5記録から得たモデル・閾値倍率2を固定し、内部評価用10記録へ適用する経路を追加した。

2026-10-07追記：データ側で実行した10記録×2センサの完了CSVを受領した。[初回評価結果](rc_popout_grid_background_evaluation_results_20261007.md)を参照。以下はその再現手順であり、既存の結果を再実行・上書きする必要はない。

## 固定したもの

[固定パッケージ](evidence/rc_popout_20260930/grid_background_frozen_20261006/freeze.json)に、背景モデル・検知設定・入力定義・区分表・コードハッシュと調整時の出力を保存した。

- 背景作成用負例：`t_0.2-none`。評価用負例で作り直さない。
- EVS閾値：`0.06090278921343967`、RGB閾値：`0.19765397027910442`。積分スコアの閾値であり、検知遅延ではない。
- ROI：EVS座標 `(x=0, y=70, width=640, height=272)`、32×32画素タイル。
- rank=3、積分時定数50 ms、終了比率0.5。各センサのモデル・尺度を調整時のまま使用する。
- EVSは過去2 ms、1 ms刻み。RGBは元の連続フレーム対、変化画素の閾値15。

`detector_parameters.json` と `background_models.json` は調整用成果物のバイト列をそのままコピーした。前者の `status` は履歴として `development_candidate_not_evaluated` のまま保持し、今回の固定状態は `freeze.json` の `frozen_for_internal_evaluation` で表す。これは評価の成功を意味しない。

設定選択には調整用正例の確認が入っている。評価区分も別手法の集計では以前に確認済みであり、完全に未知のテストデータではない。[区分表](evidence/rc_popout_20260930/development_evaluation_split_v1.json)を変更せず内部評価と呼ぶ。

| 条件 | 今回処理する記録 |
|---|---|
| 自車0.2、飛び出しなし | `t_0.2-none_01`, `t_0.2-none_02` |
| 20%左 | `test_02`, `test_04` |
| 20%右 | `test_06`, `test_07` |
| 100%左 | `test_12`, `test_13` |
| 100%右 | `test_15`, `test_16` |

20%・100%はプロポ設定であり、実測速度ではない。

## Linux側での実行

今回のコードと `docs/evidence/rc_popout_20260930/grid_background_frozen_20261006/` をデータ側の `/workspaces` に反映してから実行する。固定パッケージは作成済みのため、データ側で再学習や再固定を行う必要はない。

まず評価用10件のタイルを抽出する。既存の同期・共通ROI・時間範囲を使い、変更がないことを確認してからRGB/RAWを読む。

```bash
cd /workspaces

bash scripts/experiments/export_rc_popout_tiles.sh \
  --motion-dir /workspaces/record/09-30/analysis/detection_motion_trial01 \
  --subset evaluation \
  --frozen-dir /workspaces/docs/evidence/rc_popout_20260930/grid_background_frozen_20261006 \
  --tile-px 32 \
  --output /workspaces/record/09-30/analysis/tile_eval_trial01
```

抽出が完了してから固定モデルで評価する。このコマンドはNumPyを使用し、ROS・RAWの再読取やMatplotlibを必要としない。

```bash
cd /workspaces

bash scripts/experiments/evaluate_rc_popout_grid_background.sh run \
  --tile-dir /workspaces/record/09-30/analysis/tile_eval_trial01 \
  --frozen-dir /workspaces/docs/evidence/rc_popout_20260930/grid_background_frozen_20261006 \
  --output /workspaces/record/09-30/analysis/grid_background_eval_trial01

cat /workspaces/record/09-30/analysis/grid_background_eval_trial01/summary.csv
```

出力先が存在する場合は停止する。途中失敗した出力を完了と扱わず、修正後は新しい出力先で実行する。抽出不完了の場合は `tile_eval_trial01/summary.json` のエラーを先に確認する。設定・コード・同期等の不一致をハッシュ編集で回避しない。

## 保存・集計内容

- `summary.csv` / `summary.json`：10記録×2センサの20行。全保存区間の候補数、走行指令区間の候補数・観測秒数・警報継続秒数、最初の候補とRGB初出現の差など。
- `candidates.csv`：早期警報も含む全候補の開始・終了時刻と開始タイル。出現に近い候補だけを選ばない。
- 各記録の `*_background_scores.csv` / `*_background_maps.npz`：元の観測頻度でのスコア・背景推定・残差・積分状態・警報状態。
- `run_config.json`：固定パッケージと入力のハッシュ、対象記録、NumPy版、実行状態。正常終了時のみ `status=complete`。
- `index.html`：結果一覧。今回の段階では候補を車両と照合した枠付き動画は自動生成しない。

`candidates_before_guard` はRGB初出現−30 msより前の候補数。`candidates_before_onset` はガードなしの初出現前候補数。`candidates_onset_to_250ms` は初出現から250 ms以内に開始した候補の数であり、車両検知の成功数ではない。

`active_at_rgb_onset` は初出現以前で最後に利用可能だった判定状態。`onset_state_age_ms` でその古さを確認する。観測が遠い・無効なら空欄とする。出現前から続く警報を、初出現付近の新たな検知と区別するための診断値である。

`ready_observed_seconds` は有効で連続したサンプル間の秒数、`drive_ready_seconds` はその走行指令区間との共通部分。撮影全体の長さだけで誤警報頻度を比較せず、走行区間の値と全区間の値を分けて報告する。候補が0件でも観測できていない時間が長ければ、検知器が正常に監視していたとは扱わない。

評価時には背景モデル適合・閾値再計算・倍率探索を呼ばず、CLIでも上書き指定を受け付けない。入力の活動から背景モデルを選ぶ既存処理は維持し、対象記録の発進時刻・初出現・方向は報告にのみ使う。評価用の抽出では出現後最大タイルを選ぶ探索用パネルも作らない。

## 検証範囲

- 新規評価経路6テストと、関連する既存41テストが通過。固定ファイル変更・コード変更・入力不一致の拒否、評価10件のみの処理、再適合関数を呼ばないこと、注釈を変えてもスコア・候補時刻が不変であること、抽出から評価までの合成データ連携を確認した。
- 手元の調整用実データ5件×RGB/EVSを固定モデルで再計算し、**全保存配列と全候補が既存出力と完全一致**した。[再計算の検証結果](evidence/rc_popout_20260930/grid_background_frozen_20261006/replay_verification.json)。
- コアの検知・背景適合コードは変更していない。タイル抽出の数値処理も維持し、評価区分と固定設定の確認を追加した。
- 2026-10-07にLinux側実行の評価用20行の完了CSVを受領し、数値を集計した。リモートの配列・モデルハッシュ・映像は独立検証していない。静止条件への転用も未検証。SSH操作は行っていない。

結果を受領したら、負例の誤警報、正例の早期警報・候補なし、候補と車体の空間的対応を確認する。評価結果を見て調整した場合は探索として別記し、同じ10件を独立な最終評価とは呼ばない。

## 固定パッケージの作成履歴

今回ローカルで実行した作成コマンド。通常の評価時に繰り返すものではない。

```bash
python3 tools/evaluate_rc_popout_grid_background.py freeze \
  --development-dir docs/evidence/rc_popout_20260930/grid_background_20261006 \
  --output docs/evidence/rc_popout_20260930/grid_background_frozen_20261006
```

背景モデルは保存済みJSONからコピーし、再推定しない。既存の調整コード・モデル・閾値・区分表・完了状態の対応を検査してから固定する。
