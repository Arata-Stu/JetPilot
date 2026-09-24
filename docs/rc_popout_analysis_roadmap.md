# RCカー飛び出し実験：解析ロードマップ

更新日: 2026-09-24

## 目的

同一のRCカー飛び出し刺激に対し、RGB検出器、EVS検出器、人間のボタン反応を、
共通の飛び出し開始時刻と誤警報条件で比較する。EVSの主張点は最終認識精度ではなく、
許容可能な誤警報率における初回安定検知の早さとする。

## 現在の資産

- 飛び出し記録: 36シーケンス
- 条件: 左右、RCカー速度10/20/30%、2種類の距離・開口幅構成
- 全記録の開始・終了付近に既知周期のLED点滅
- EVS: 640x480 RAW、default bias
- RGB: RealSense D455、848x480、60 Hz
- 既定空間校正: `ros2_ws/src/tool/multi_sensor_calibration/config/calibrations/rc_popout_default/`

## 比較で分離する時刻

| 時刻 | 意味 |
| --- | --- |
| `t_visible` | RCカー先端が遮蔽物から初めて見えた物理時刻 |
| `t_sensor_ready` | 比較対象の入力表現が利用可能になった時刻 |
| `t_model_output` | 検出器がスコアを出した時刻 |
| `t_stable` | 規定回数連続で閾値を超えた最初の時刻 |
| `t_human` | 人間がボタンを押した時刻 |

主指標は `t_stable - t_visible` とする。推論時間だけでなく、RGBフレーム待ち、EVSの
蓄積窓、前処理、推論、安定判定を含むend-to-end値も報告する。

## Phase 1: 全シーケンスの時間同期

各記録の開始側・終了側LEDを使い、シーケンスごとに次を保存する。

- 開始側オフセット
- 終了側オフセット
- affine drift [ppm]
- 対応した点灯・消灯edge数
- edge残差RMS
- 既知LED周期との一致品質
- 自動推定後の目視確認結果

集約表には最低限、次の列を持たせる。

```text
sequence,offset_start_ms,offset_end_ms,offset_anchor_ms,drift_ppm,
matched_edges,residual_rms_ms,quality_status,manual_status
```

### 使用方針

1. 主解析では各シーケンス固有の推定値を使用する。
2. 全体代表値には外れ値に強いmedianとMADを使用する。
3. 全体代表値は、LED検出に失敗した記録のfallbackと感度分析にのみ使用する。
4. mean、standard deviation、range、IQRも報告する。
5. global値を使った場合とper-sequence値を使った場合で結論が変わらないか確認する。

暫定的な安定判定は、シーケンス間MADが5 ms以内、全シーケンスのmedianからの最大偏差が
10 ms以内とする。ただし最終閾値は、EVS対RGBで主張する遅延差より十分小さいことを条件に
決める。60 Hz RGBの1フレームは約16.67 msなので、edge単体の残差と、多数edgeから得た
オフセット推定値の不確かさは分けて扱う。

## Phase 2: RGBとEVSの視野・解像度統一

比較用の共通キャンバスはEVSの640x480を第一候補とする。EVSを拡大して情報が増えたように
見せるのではなく、広角なRGBを校正済みEVSの仮想視野へremapし、共通有効領域maskを作る。

```text
EVS RAW 640x480 ───────────────┐
                               ├─ 共通EVS視野 640x480
RGB 848x480 ─ 回転・歪み補正 ─┘
```

カメラの光学中心が約42.16 mm離れているため、深度なしで全画面を完全一致させることは
できない。そこで用途を分ける。

- `rotation-only`: DSEC型。共通の角度視野を作るための定性的表示とモデル入力用crop。
- `fixed-depth/plane`: RCカーが遮蔽物から出る代表距離で一致させる定量確認用。
- `native`: 各センサ本来の視野・更新周期を保った実運用比較。

最終評価では`native`条件と`FOV/resolution matched`条件を分けて報告し、EVSの更新周期の
利点と、画角・画素数の違いを混同しない。

## Phase 3: 飛び出し開始時刻の付与

各シーケンスについて、遮蔽物端を基準にRCカー先端が現れる`t_visible`を付与する。
外部フォトゲートがないため、同期済みRGB・RAWイベント重畳を使う半手動annotationとし、
不確かさも記録する。

- RGB: 最後の非表示frameと最初の表示frameによる時刻区間
- EVS: その区間内で遮蔽物端から連続eventが生じ始めた時刻
- 保存値: best estimate、lower bound、upper bound、annotator confidence

EVSだけを見て開始時刻を決めるとEVSに有利になるため、RGBの区間制約と空間重畳を併用する。
可能なら一部を複数回annotationし、付与者内誤差を確認する。

## Phase 4: 人間の反応速度評価

人間には共通crop後のRGB映像を提示し、「飛び出しを認識したらボタンを押す」という単純検知を
課す。左右判断は主課題に含めず、必要なら別実験にする。

- 映像と応答UIを事前loadする
- 60 Hzなど一定の提示条件を維持する
- シーケンス順を参加者ごとにrandomizeする
- 飛び出しまでの待ち時間をjitterする
- 飛び出さないcatch trialを入れる
- 早押し、無反応、タブ非表示などの除外規則を事前定義する
- 同一人物の多数trialを独立参加者として数えない

人間の値は厳密な神経反応時間ではなく、「この提示系におけるボタン反応時間」と表現する。
参加者を募る場合は、所属機関の倫理審査・同意取得要否を事前に確認する。

## Phase 5: RGB・EVS検出器

### EVS

- 20ch histogram: 10 temporal bins x 2 polarities
- 40 ms rolling window、4 ms strideを基本候補
- ROI event-count非学習baseline
- 2ch ON/OFF CNN baseline
- 20ch CNNのTiny / Small / Base

### RGB

- 60 Hz frame入力
- 共通crop 640x480とnative 848x480を区別
- 単純なframe-differenceまたはmotion baseline
- EVSモデルと計算量を近づけた小型CNN

36シーケンスしかないため、連続frameをrandom splitしてはいけない。同一シーケンスがtrainと
testへ入ると重大なdata leakageになる。splitはシーケンス単位、可能なら距離・開口幅条件を
またぐleave-condition-outまたはgrouped cross-validationとする。ニューラルネットを一から
学習するには少量なので、結果はpilotとして扱い、augmentationまたは追加記録を検討する。

## Phase 6: RGB vs EVS vs 人間

全方式を同じ`t_visible`へ揃え、次を比較する。

- stable time-to-first-detectionのmedian、IQR、p95
- 8/16/32/64 ms時点のearly recall
- miss率
- false alarm / trial、false alarms / minute
- `t_visible`より前の予測的反応率
- 前処理・推論・判定のp50/p95/p99
- RGBとEVSのpaired latency差とbootstrap confidence interval
- 人間のparticipant別分布

閾値は同じfalse-alarm制約になるようvalidation dataで決め、test dataを見て変更しない。
「常に危険」と出す方式が最速になるため、反応時間だけを単独で比較しない。

EVSが人間を上回ったとする条件は、単に最速trialが人間より早いことではなく、例えば次を
同時に満たすこととする。

1. EVSのmedian stable TTFDが人間のmedianより短い。
2. EVSのp95または信頼区間を報告できる。
3. miss率とfalse alarm率が許容範囲内である。
4. 同じシーケンス集合に対する比較である。
5. センサ時刻から出力までの計算遅延を含めている。

主張は「限定されたRCカー飛び出し・単純反応課題において、EVS検出系が人間のボタン反応
より早い」とし、人間一般の危険認知能力を上回ったとは表現しない。

## 実行順

1. 全記録のinventoryを作成し、条件とファイル欠損を確認する。
2. LED同期を36シーケンスへbatch適用し、同期安定性を判定する。
3. 代表1試行でrotation-onlyとfixed-depthの共通視野を比較する。
4. 共通視野仕様を固定し、36試行へ前処理をbatch適用する。
5. `t_visible`annotation UIとmanifestを作る。
6. 人間反応UIを完成させ、pilotで表示・入力遅延を確認する。
7. 非学習baselineを先に評価する。
8. RGB CNN、2ch EVS CNN、20ch EVS CNNを評価する。
9. 同一false-alarm制約で三者比較と感度分析を行う。

最初の完了条件は、Phase 1の`sequence_sync.csv`と品質一覧を得ることである。
