# RGB・EVS活動量の逐次変化検知（全ROI・第1段階）

`tools/rc_popout_change_detection.py` は既存の発進基準スコアCSVを読み、過去の中央値・MADからの上方偏差を時間積分する。
学習済みモデル、ROS、Metavision、NumPyは不要。Python 3の標準ライブラリだけを使う。
タイル別の出力・運動補償はまだ含めない。

## 2026-10-06：第1試行の結果と次の比較

ユーザー提示の `change_dev_trial01` の集計・候補JSON診断による。元CSVをこちらで再読込した結果ではない。
調整用5記録×2方式の全10ケースで、最初の候補が発進指令から約67～99 ms後に発生した。
正例4件はRGB・EVSともに、発進からの候補がRGB初出現時刻まで継続していた。

| session | RGB初出現 [s] | RGB最初の候補の終了 [s] | EVS最初の候補の終了 [s] | 初出現時のEVS累積値 |
|---|---:|---:|---:|---:|
| `test_01` | 1.549740 | 1.5999 | 2.5794 | 0.221659 |
| `test_05` | 1.530801 | 1.5812 | 2.5829 | 0.501613 |
| `test_11` | 1.693439 | 2.1798 | 2.6655 | 0.498936 |
| `test_14` | 1.646375 | 2.0940 | 2.5309 | 0.419228 |

秒数は発進指令基準。累積値はRGB初出現時刻以下の最後のEVSサンプル。
負例 `t_0.2-none` でも、最初の候補はRGBが0.0796～2.0231秒、EVSが0.0952～3.4892秒続いた。
初版は累積値がゼロに戻るまで再検知しないため、初出現時の別の変化も同じエピソードに吸収される。
保持による再検知不能は確認できたが、保持だけを直せば対象を分離できると保証する結果ではない。

### 次に実行する調整用コマンド

旧出力は保持し、`change_dev_trial02` へ出す。既存の背景窓・スケール下限・開始閾値は変えず、減衰と終了閾値を明示する。

```bash
cd /workspaces

bash scripts/experiments/analyze_rc_popout_changes.sh \
  --motion-dir /workspaces/record/09-30/analysis/detection_motion_trial01 \
  --subset development \
  --decay-tau-s 0.10 \
  --release-ratio 0.5 \
  --output /workspaces/record/09-30/analysis/change_dev_trial02

cat /workspaces/record/09-30/analysis/change_dev_trial02/summary.csv
cat /workspaces/record/09-30/analysis/change_dev_trial02/candidates.csv
```

評価用10記録はまだ実行しない。先頭候補が発進時に残ることは想定内で、それを削除した改善にはしない。
確認するのは候補の保持が解消するか、正例の出現付近に追加の候補があるか、負例や出現前にも同様の追加候補が増えるかである。
全ROIで対象と背景を分離できなければ、全体閾値の探索を続けずタイル別へ進む。

コードは `rolling_median_mad_time_cusum_v2`。旧 `detector_parameters.json` はコード・方式の不一致として受け付けない。
新しい調整runの設定を保存し、将来の評価は最終的に採用したrunの設定を使用する。

## まず実行するコマンド：調整用5記録

リポジトリの変更をデータ側に反映した後、コンテナ内で実行する。SSHによる自動実行はしない。

```bash
cd /workspaces

bash scripts/experiments/analyze_rc_popout_changes.sh \
  --motion-dir /workspaces/record/09-30/analysis/detection_motion_trial01 \
  --subset development \
  --output /workspaces/record/09-30/analysis/change_dev_trial01

cat /workspaces/record/09-30/analysis/change_dev_trial01/summary.csv
```

対象は `t_0.2-none`, `test_01`, `test_05`, `test_11`, `test_14`。
既定の[区分JSON](evidence/rc_popout_20260930/development_evaluation_split_v1.json)に基づく。
評価用10記録のスコアや注釈は調整時には読み込まない。
出力先が存在すると中止し、既存の結果を上書きしない。

### 読み込むもの

- `<motion-dir>/<session>/rgb_aligned_scores.csv`, `evs_aligned_scores.csv`
- 同じ場所の `alignment.json`
- `<motion-dir>/run_config.json` が示す元スコアの `run_config.json`
- 元スコアの各 `result.json`、それが示す注釈JSONと同期YAML（変更検査用。RAW/bagは読まない）

元スコアの設定からEVSの蓄積窓を取得する。今回の元解析では2 msだが、値を固定で仮定しない。
元のスコアディレクトリを移動した場合は `--score-config /新しい場所/run_config.json` を指定する。
注釈・同期の参照パスも有効である必要がある。同期・注釈が更新されている場合、先に元のスコアと発進基準解析を再生成する。

## 出力

| ファイル | 内容 |
|---|---|
| `summary.csv` | センサ別の全候補数、最初の候補、RGB初出現との時刻差、初出現前後の候補数 |
| `candidates.csv` | 全候補の開始・終了・継続時間・指令phase・RGB初出現との差。正解対応付けは行わない |
| `summary.json` | 同上と指令phase別のサンプル数・準備済み数・候補数 |
| `index.html` | 各試行の波形へのリンク |
| `<session>/<sensor>_change_scores.csv` | 全時系列の元スコア、過去中央値・スケール、標準化偏差、累積値、候補・状態 |
| `<session>/<sensor>_change_scores.svg` | 全保存区間の波形。赤線＝候補、緑破線＝RGB初出現 |
| `<session>/<sensor>_drive_zoom.svg` | 発進付近の拡大。表示のみを切り出し、検知状態はリセットしない |
| `<session>/<sensor>_candidates.json` | 候補エピソードの開始・終了・終了理由 |
| `<session>/result.json` | 入力ハッシュ、元のalignment、センサ別集計 |
| `detector_parameters.json` | アルゴリズム、コードと区分のハッシュ、全検知設定 |
| `run_config.json` | 入力パス・元スコア設定のハッシュ・区分・実行設定 |

`first_candidate_minus_onset_s < 0` は「RGB初出現より前に候補が出た」という意味に限る。
対象に対応する候補かは未判定であり、EVSの先行検知成功と自動的に解釈しない。
候補の対応付け、見逃し判定、検知率の算出は本ツールでは行わない。
注釈から30 ms以上前の候補、±30 ms内、30 ms以上後を分けて示すが、この30 msは探索用の表示区分であり、同期誤差の保証でも検知の許容窓でもない。
`first_candidate_end_from_drive_s` は最初の候補の終了時刻。
`active_candidate_at_onset` は候補の開始以上・終了未満にRGB初出現が含まれるかを示す診断値であり、検知成功を意味しない。負例は空欄。

## 検知処理

時刻 `t` に利用可能なスコアを `x(t)` とする。
現在の観測窓の始点 `a(t)` は、RGBでは同じintervalの直前スコア時刻、EVSでは `t − 元解析の窓幅` とする。
RGBの最初の差分の始点は旧CSVにないため、interval開始・欠測直後の1点を背景推定に使用しない。

背景の参照スコアは、終端時刻が `[a(t) − history_s, a(t)]` に含まれる過去の値だけを使う。
重複するEVS観測窓に含まれる現在のイベントを、背景側へ混入させない。

```text
background = median(reference_scores)
scale      = max(scale_floor, 1.4826 * median(abs(reference_scores - background)))
z(t)       = clip((x(t) - background) / scale, -z_clip, z_clip)
C(t)       = max(0, C(previous) + (z(t) - drift_k) * delta_time_seconds)
```

- 背景履歴は候補が出ても更新を続ける。初出現注釈を用いた更新停止はしない。
- `C >= threshold_s` になった時刻を候補開始とする。窓の先頭や推定変化点へ時刻を戻さない。
- 候補の終了は `C <= threshold_s * release_ratio`。既定の `release_ratio=0` では初版と同じくゼロまで継続する。
- 終了時に累積値を強制的にゼロにしない。再度開始閾値に達したときだけ次の候補を出す。
- 大きな欠測・interval境界では背景と累積をリセットして再度準備期間を取る。
- 初出現時刻、発進時刻、指令phaseで検知を許可・抑制しない。`outside_phases`を含む保存済みCSVの全区間を処理する。
- delta_timeは直前サンプルから現在サンプルまでの秒数。現在値を用いる後退区間の数値積分であり、判定は必ず現在時刻に出す。
- `ready_observed_seconds` は準備完了サンプルの有効な直前ステップ秒数の和。欠測を埋めない。前準備で評価できない時間は候補ゼロの成功時間として扱わない。
- スコアは生値。変換やパーセンタイルへの非線形写像は入れていない。

### 減衰を指定した場合

`--decay-tau-s tau` が正なら、上の積分を以下に置き換える。

```text
retain = exp(-delta_time_seconds / tau)
C(t) = max(0, retain * C(previous)
              + tau * (1 - retain) * (z(t) - drift_k))
```

これは区間内の入力を一定とした `dC/dt = (z-k) - C/tau` の厳密な更新式で、更新回数が多いほど減衰する実装にはしない。
時定数0.1秒は検知の固定待ち時間でも、発進後の無視時間でもない。新しい入力がなければ過去の寄与が0.1秒ごとに約37%へ減衰する。
さらに `z=0` の背景であれば `-k` の分も減る。常に背景を上回る活動が続く場合は保持が続き得る。
`z <= z_clip`、初期値0より、減衰時の累積値は `tau * (z_clip-k)` 以下となる。
開始と終了の別閾値を使うことで、終了に厳密なゼロを要求せず、開始閾値近辺の細かな上下も抑える。
これらは発進の誤反応を除去する仕組みではなく、古い反応の長時間保持を抑える比較候補である。

CUSUMの考え方を用いたロバストな時間積分スコアであり、固定分布・独立サンプルを仮定した検定そのものではない。
理論的なp値や誤警報率は出さず、調整記録で実測する。
参考：[NIST CUSUM Control Charts](https://www.itl.nist.gov/div898/handbook/pmc/section3/pmc323.htm)。

### 未調整の初期値

| 引数 | 初期値 | 意味 |
|---|---:|---|
| `--history-s` | 0.30 | 背景参照履歴の最大時間 |
| `--min-history-s` | 0.20 | 参照スコアの最初～最後に必要な時間幅 |
| `--min-samples` | 8 | 背景推定に必要な最小サンプル数 |
| `--drift-k` | 1.0 | 累積を増やすために超える標準化偏差 |
| `--threshold-s` | 0.05 | 時間積分スコアの候補閾値 |
| `--z-clip` | 10.0 | 単発極値の寄与の上限 |
| `--rgb-scale-floor` | 0.001 | RGBスコア単位での背景スケール下限 |
| `--evs-scale-floor` | 20.0 | EVSイベント数単位での背景スケール下限 |
| `--rgb-max-gap-s` | 0.10 | RGBのリセットを行う欠測間隔 |
| `--evs-max-gap-s` | 0.003 | EVSのリセットを行う欠測間隔 |
| `--onset-guard-s` | 0.030 | 集計のみの初出現前後区分 |
| `--decay-tau-s` | 0.0 | 0は初版の非減衰積分。正なら累積の減衰時定数 |
| `--release-ratio` | 0.0 | 候補終了閾値／開始閾値。0以上1未満 |

これらは動作確認用の出発値であり、実記録で最適化・成功確認した値ではない。
`threshold_s=0.05`は50 msの固定待ち時間ではない。例えば一定の `z=6, drift_k=1` なら、累積が0から閾値に達する積分時間は約10 msになる（実際の出力は更新時刻に制限される）。
履歴幅が長すぎると発進を大きな異常として捉え、短すぎると対象の活動を背景へ取り込みやすい。
本方式が対象と背景を分離できるかは未確認。設定を増やして評価用記録に合わせない。

## 調整の手順

1. 最初のコマンドを実行し、`summary.csv` と走行付近の波形を確認する。
2. 負例 `t_0.2-none` と正例4記録の出現前候補を確認する。発進時の候補も隠さない。
3. 調整は新しい出力先を使い、変更する引数を明示する。全組み合わせの自動探索はしない。
4. 当面は候補時刻・背景中央値・累積値を並べ、活動量増加が背景との差として残るかを診断する。
5. 調整用でも分離できない場合は全ROIの閾値探索を続けず、タイル別処理へ進む。

## 設定固定後だけ実行するコマンド：評価用10記録

以下は調整終了後の手順。**現時点では実行しない。** `change_dev_trial01` は最終的に採用した調整runへ置き換える。

```bash
cd /workspaces

bash scripts/experiments/analyze_rc_popout_changes.sh \
  --motion-dir /workspaces/record/09-30/analysis/detection_motion_trial01 \
  --subset evaluation \
  --parameters /workspaces/record/09-30/analysis/change_dev_trial01/detector_parameters.json \
  --output /workspaces/record/09-30/analysis/change_eval_fixed01
```

評価用には保存済みパラメータを必須とし、引数での上書きを禁止する。コードまたは区分が変わっていれば中止する。
同一の設定で実行したことを追跡する仕組みであり、研究上の独立性を自動保証するものではない。
静止データへの適用は、同じ形式の入力と評価範囲を整備する別段階。本コマンドは現在の移動条件の区分を対象とする。

## ローカル検証

```bash
python3 -m unittest discover -s tests -p 'test_rc_popout_change_detection.py' -v
bash -n scripts/experiments/analyze_rc_popout_changes.sh
```

合成データで因果性、観測窓の分離、欠測とinterval境界、時間積分、設定固定、開発／評価の分離、同期の変更検出を検証する。
実データでの性能評価はLinux側の実行結果を受け取ってから行う。
