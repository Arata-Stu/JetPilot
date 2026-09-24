# RCカー飛び出しシナリオにおけるEVS危険反応時間評価：研究提案

更新日: 2026-09-20

収録後の具体的な解析順、時間同期・共通視野・人間評価・検出器比較の判断基準は
[`rc_popout_analysis_roadmap.md`](rc_popout_analysis_roadmap.md)にまとめる。

提出済み題目:

> RCカー飛び出しシナリオにおけるイベントカメラおよび人間の危険反応時間比較評価に関する研究

## 1. 結論

本研究は、単に「イベントカメラは低遅延である」と示すのではなく、次の問いを中心に据える。

> RCカーの物理的な飛び出し開始から、20ch event histogramを入力とするニューラルネットワークが、許容可能な誤警報率で危険信号を出すまでに何msを要するか。また、イベントカメラの時間的利点を維持できるモデル規模の上限はどこにあるか。

比較対象としてRGBモデルと人間のボタン反応を置く。ただし、人間との比較は機械学習モデルの優劣を決める主評価ではなく、得られた遅延値に実用上の意味を与える参照評価とする。

本研究の主な価値は、新しいevent representationや最高認識精度ではなく、以下の組合せに置く。

1. 物理的な飛び出し開始を外部基準で定義したRCカー実験系
2. 40 ms window、4 ms stride、20ch histogramを用いる250 Hz危険検知
3. Jetson上の実測遅延に基づくaccuracy--latency Pareto評価
4. RGB、EVS、人間を共通の開始時刻で比較するtime-to-first-detection評価
5. 可能であれば、危険信号だけでなくブレーキ指令・車体減速までを含むオンライン評価

「20ch histogramを用いたイベント検知」自体には既に近い研究があり、新規性として主張しない。特に、10 temporal binsと2 polaritiesを結合した20ch stacked histogramは既存のevent-based object detection研究でも用いられている。

## 2. 研究上の問い

### RQ1: 早期検知

遮蔽物から対象RCカーが出現した時点から、EVS危険検知モデルが安定した危険信号を初めて出すまでの時間はどの程度か。

### RQ2: RGBとの相違

同一の物理刺激、同一の誤警報制約、同一計算機上で、EVSはRGBより早く危険を検知できるか。

### RQ3: モデル規模上限

モデルを拡大したときの検知性能向上と推論遅延増加の関係はどうなるか。250 Hzの4 ms周期を維持できる最大構成はどれか。

### RQ4: 人間との位置関係

同じ飛び出しに対する人間の危険反応時間分布の中で、EVSシステムの応答はどの位置にあるか。

### RQ5: 物理的効果

遅延差は、RCカーの反応距離、停止距離、衝突回避成功率にどの程度影響するか。

## 3. 仮説

- H1: EVSは、対象が遮蔽物から出始めた初期区間で、RGBより短いtime-to-first-detectionを示す。
- H2: 対象が十分に露出した後の最終的な認識精度ではRGBが同等以上でも、出現後8/16/32 msのearly recallではEVSが優位になる条件がある。
- H3: モデル規模を拡大すると検知性能は向上するが、ある規模を超えるとdeadline missによって実効的な初回検知時間が悪化する。
- H4: 20chの時間情報は、単純な2ch ON/OFF count imageよりも、同程度の誤警報率における初回検知の安定性を改善する。
- H5: EVSシステムの危険信号は人間のボタン反応より先に出る可能性があるが、その差は人間一般への優越ではなく、限定された実験条件での参照差として解釈すべきである。

## 4. 何を検知するか

一般物体検出のmAP競争は本研究の目的にしない。標準化された飛び出しシナリオについて、モデルは次を出力する。

\[
p_{\mathrm{hazard}}(t)
=
P(\text{対象が規定時間内にego走行領域へ侵入する}\mid E_{t-40\mathrm{ms}:t})
\]

最小構成は1出力の二値分類でよい。必要に応じて以下を追加する。

- 飛び出し方向: left / right
- 危険度: non-hazard / warning / emergency
- 対象位置の粗いheatmap
- TTCまたは走行領域侵入までの残余時間

本研究の題目に対しては、bounding box精度よりも危険信号の時刻が重要である。したがって、最初からYOLO型の一般検出器を作るより、小型CNNによる危険判定を先に完成させる。

## 5. 提案モデル

### 5.1 入力

- shape: `[1, 20, 120, 212]`
- window: 40 ms
- stride: 4 ms
- temporal bins: 10
- polarity: positive / negativeを分離
- channel order: `positive[0:10], negative[0:10]`

40 ms windowは40 ms待ってから推論するという意味ではない。rolling windowは常時更新され、飛び出し後の新しいeventは次の4 ms境界から最新binへ入る。ただし、実際に何ms分のeventを観測すれば安定検知できるかはモデルと対象条件に依存するため、物理的開始時刻から実測する。

### 5.2 最小ネットワーク

現行PilotNet backboneを危険検知用に変更する。

```text
20ch histogram
  -> temporal mixer (1x1 conv, 20 -> C)
  -> small spatial CNN
  -> global pooling
  -> hazard logit
```

20chへ最初から5x5 convolutionを適用すると、現行PilotNetでは最初の層が計算量の大部分を占める。概算では、20x120x212入力の現行構成を1 hazard出力に変更した場合、約35万parameter、約1.25億MACで、そのうち最初のconvolutionが約58%を占める。1x1 temporal mixerやdepthwise convolutionは候補になる。

ただし、1x1 temporal mixing自体も既存研究に近い考え方があり、アーキテクチャ新規性として強く主張しない。本研究では、Jetson上での実測deadlineと初回検知時間を基準に採否を決める。

### 5.3 比較するモデル

最低限、次を用意する。

1. ROI内event数の固定閾値: 非学習baseline
2. 2ch ON/OFF histogram CNN: 時間binを潰したbaseline
3. 20ch histogram CNN: 提案系
4. RGB CNN: frame baseline

モデル規模評価では、同じ20ch backboneのwidth multiplierを変える。

| 構成 | 例 | 目的 |
| --- | ---: | --- |
| Tiny | 0.25x | 推論時間下限と単純課題での十分性 |
| Small | 0.5x | 250 Hz有力候補 |
| Base | 1.0x | 現行PilotNet相当 |
| Large | 1.5x | 精度改善とdeadline超過点の確認 |

時間が限られる場合はTiny / Small / Baseの3点でPareto傾向を示す。parameter数やFLOPsは補助指標とし、上限判定にはTensorRT FP16の実測値を用いる。GPUでは演算構造により速度が変わるため、parameter数だけから上限を断定しない。

## 6. 時刻と評価指標の定義

### 6.1 主要時刻

| 記号 | 定義 |
| --- | --- |
| `t_release` | 飛び出し機構を作動させた時刻 |
| `t_visible` | 対象先端が遮蔽物を越え、センサ・人間から物理的に見え始めた時刻 |
| `t_hazard` | 対象の車体領域がegoの危険領域へ侵入し始めた時刻 |
| `t_detect` | scoreが閾値を初めて超えた時刻 |
| `t_stable` | scoreがK回連続して閾値を超えた最初の時刻 |
| `t_cmd` | brake / stop指令を発行した時刻 |
| `t_act` | 車体の減速が物理的に始まった時刻 |
| `t_human` | 人間が反応ボタンを押した時刻 |

`t_release`を反応時間の起点にしてはいけない。機構の立ち上がり時間が混入するため、主起点は`t_visible`とする。

### 6.2 主指標

\[
TTFD_{stable}=t_{stable}-t_{visible}
\]

一瞬だけ閾値を越える誤反応を除くため、K回連続条件を用いる。例えばK=2なら安定化のため最大4 ms程度が追加される。この追加分もシステム反応時間へ含める。

主に報告する値:

- stable TTFDのmedian / p95 / p99
- 8 / 16 / 32 / 64 ms時点のearly recall
- `t_hazard`までの検知成功率
- positive trialのmiss率
- negative / catch trialのfalse alarm率
- false alarms per minute
- 推論時間とend-to-end時間のp50 / p95 / p99 / max
- 4 ms deadline miss率、skipped snapshot数、queue drop数
- brake command latencyと、可能ならactuation latency

検知できなかったtrialを平均計算から除外しない。missとして別報告するか、右打切りデータとして扱う。

### 6.3 誤警報制約

「常に危険」と出すモデルは反応時間だけなら最速になる。したがって、validation setで閾値を固定し、同一のfalse positive constraintでEVSとRGBを比較する。

候補:

- negative trial当たりfalse alarm 5%以下
- false alarms per minuteを一定以下
- precisionを一定値以上に固定

閾値はtest setを見て調整しない。Maoらのdelay metric研究が指摘するように、時間遅延の評価にはfalse alarm制約が不可欠である。

## 7. 二つのlatency budget

### 7.1 250 Hz計算期限

全ての4 ms snapshotを処理する**将来の統合危険検知系**の条件は、概念的には次である。

\[
P99(T_{preprocess}+T_{inference}+T_{decision}) < 4\ \mathrm{ms}
\]

実際のモデル予算は次で決める。

\[
D_{model}
=4\ \mathrm{ms}
-P99(T_{non-model})
-M_{safety}
\]

`T_non-model`にはdecode、tensor publish/受渡し、postprocess、scheduler待ちを含める。既存native benchmarkの平均約0.294 ms/snapshotだけを差し引いて「残り3.7 ms」と断定してはいけない。live pipelineのp99を先に測定する。

モデル規模上限は次のように定義できる。

\[
s^*=\max_s\{s\mid P99(T_{cycle}(s))<4\ \mathrm{ms},
\;miss_{deadline}(s)=0\}
\]

60秒の反復runと10分soakの双方で確認する。

#### 現行実装についての重要な注意

現在のJetPilotで`deadline_ms: 4.0`が直接監視しているのは、非同期event preprocessorによるdecode、CUDA rolling state更新、20ch snapshot生成・publishまでであり、ニューラルネット推論、危険判定、command mux、車両interface、物理制動を含むend-to-end期限ではない。既存のcontrol decoderも既定deadlineは33.3 msで、出力はsteering / throttleを想定し、brakeは0に固定されている。

また、既定のcommand muxは100 Hz、VESC変換nodeは50 Hz、serial bridgeおよびPCA9685 vehicle出力は100 Hzである。タイマ位相が最悪の場合、周期待ちだけでもVESC経路では概ね最大30 ms、bridge / PCA9685経路では概ね最大20 msが加わり得て、さらに通信と車体の物理応答が続く。このため、既存構成のまま「飛び出しから4 ms以内にブレーキが作動する」とは主張できない。研究では、少なくとも次の三層を分離して計測する。

1. **表現期限**: 20ch snapshot生成が4 ms周期を維持できるか
2. **知覚期限**: sensor timestampからhazard decision publishまでのp99
3. **反応期限**: `t_visible`から`brake command`、さらに実減速開始まで

したがって、モデル規模の上限には二つの報告方法を用いる。

- 現行資産を用いた**前処理＋推論の実測Pareto**
- hazard runtimeを統合した後の**decisionまでの250 Hz適合上限**

後者を実装できない段階では、「4 ms以内の危険判定を達成」とせず、「4 ms入力周期に対する推論計算量の余裕を測定した」と表現する。

### 7.2 物理安全上の期限

4 msは更新周期の期限であり、飛び出し回避の安全期限そのものではない。直線停止へ単純化すると、利用可能な反応時間は概ね次で与えられる。

\[
T_{safe}
=
\frac{d_{available}-v^2/(2a)-d_{margin}}{v}
-T_{actuator}
\]

ここからsensor transport、preprocess、inference、decisionの遅延を消費する。実際には横方向からの侵入なので、対象の横速度とego走行領域へ入る時刻を使う。

モデルは次の二種類に分類する。

- **native-250Hz適合**: 全snapshotを4 ms周期で処理できる
- **safety-feasible**: 250 Hzには追従できないが、物理安全期限内かつRGBより早く反応できる

この区別により、「4 msを少し超えたからEVSに価値がない」という誤った結論を避ける。

## 8. RGBとの比較

RGBより高い最終accuracyを主目的にしない。EVSの有効性は、同じ誤警報率での初回安定検知時刻によって定義する。

\[
\Delta T = TTFD_{RGB}-TTFD_{EVS}
\]

\[
\Delta d = v_{ego}\Delta T
\]

`Delta T > 0`であればEVSが早く、`Delta d`によって何cmの反応距離に相当するかを示せる。

比較は少なくとも二条件に分ける。

### 8.1 Sensor-native条件

- EVS: 250 Hz出力
- RGB: 実カメラのnative frame rate
- 同一実装環境・実運用条件で比較

これは実用上の比較だが、更新レート差とrepresentation差が混ざる。

### 8.2 Rate / compute matched条件

- EVS出力をRGB timestampへsubsampleする、または共通の評価時刻に揃える
- backbone family、計算量、入力解像度を可能な範囲で合わせる
- sensor rateの差とrepresentationの差を切り分ける

同じPilotNet幅でも20chはRGB 3chより最初のconvolutionが重いため、「同一アーキテクチャ」を「同一計算量」と呼ばない。parameter、MACs、実測推論時間を併記する。

### 8.3 EVSが有力と判断する条件

次のどれかではなく、原則として全てを満たす条件を示す。

1. 同一false alarm constraintでstable TTFDがRGBより短い
2. `t_hazard`までのmiss率がRGB以下、または許容差内
3. end-to-end遅延差の信頼区間が報告できる
4. その差がRCカー速度に対して無視できない距離差になる

最終accuracyがRGBより低くても、early recallとTTFDで優位なら研究目的には合う。

## 9. 人間との比較

人間条件では、参加者をセンサに近い視点へ置き、飛び出しを認識したら手元ボタンを押してもらう。直接視認条件は表示遅延を含まず、人間側に有利な保守的比較になる。

可能なら、次を別条件として追加する。

- direct-view human: 現場を直接見る
- RGB-teleoperation human: 車載RGB映像を見てボタンを押す

後者にはcapture、伝送、display遅延が含まれるため、直接視認と混ぜない。

注意点:

- trial開始から飛び出しまでをランダム化する
- 飛び出さないcatch trialを含める
- 左右、速度、出現位置をランダム化する
- 機構音などの視覚以外の手掛かりを抑える
- 同一参加者の反復を独立sampleとして扱わない
- 参加者を集める場合は所属機関の倫理審査・同意取得要否を確認する
- 自分1名だけなら「人間一般」ではなくpilot referenceと表記する

人間の主要値は次である。

\[
T_{human}=t_{human}-t_{visible}
\]

参加者間差があるため、median、分布、participantごとの値を示し、可能ならparticipantをrandom effectとする。人間との比較は「同じ危険概念を同じ能力で判断した公平な知能比較」ではなく、限定された単純反応タスクとのシステム応答時間比較である。

## 10. RCカー飛び出し実験系

### 10.1 推奨構成

```text
遮蔽物
  └─ target RC car / ガイド付き移動台車
        └─ photogate at visibility boundary

ego RC car
  ├─ SilkyEvCam
  ├─ RGB camera
  ├─ Jetson
  └─ brake command / wheel speed logger

外部計測
  ├─ microcontroller trigger timestamp
  ├─ photogate timestamp
  └─ high-speed audit camera（可能なら）
```

targetは二台目RCカーでもよいが、再現性を上げるにはガイドレール、一定速度機構、または横方向の直線ガイドを用いる。飛び出し速度と位置の再現性は、モデル差より大きい実験誤差になり得る。

### 10.2 共通時刻

最良はphotogateのTTLを、Jetsonと独立logger、可能ならセンサ外部triggerへ同時入力する方法である。難しい場合は、event/RGB両方から見える同期マーカーとhigh-speed cameraで監査する。

EVS sensor timeとROS/host time、RGB hardware timeを同期せずに直接減算してはいけない。各clock domain間の対応、offset、jitterを保存する。

#### photogateを使用できない場合の低予算構成

photogateは必須ではない。モデル入力とは独立した側面監査映像で、遮蔽物の境界とtarget先端を同時に撮影し、targetが境界を初めて越えたframeを`t_visible`とする。

- 240 fps撮影なら1 frameは約4.17 ms
- 120 fps撮影なら1 frameは約8.33 ms
- 境界frameを一意に決められない場合は、最終不可視frameと初回可視frameの区間として報告する
- スマートフォンが可変frame rateの場合は、公称fpsではなく保存動画のtimestampを確認する

監査映像とJetson時刻の対応には、試行開始時にLEDを点灯させ、その変化を監査映像とEVS / RGBの画角端へ記録する方法を用いる。LED領域は学習入力からcropまたはmaskする。人間の直接視認試験では、LEDが反応の手掛かりにならないよう参加者から遮蔽する。

飛び出し機構のスイッチ時刻やRC送信指令は`t_release`として保存できるが、機構遅延が混入するため、それだけを`t_visible`とは呼ばない。側面映像から各trialの`t_visible - t_release`を測れば、機構遅延のばらつきも評価できる。

EVSとRGBの相対的な早さだけが目的なら、同期済みの同一trialについて

\[
\Delta T=t_{detect,RGB}-t_{detect,EVS}
\]

を計算すると、共通の`t_visible`は差分から相殺される。ただし、異なるsensor clockのoffsetとdriftはLED等で合わせる必要がある。絶対反応時間や人間との比較には、側面監査映像から求めた`t_visible`を使用する。

#### 赤外線photogateを使う場合のEVSリーク対策

SilkyEvCamのPROPHESEE event sensorは赤外LEDによるactive marker用途にも使われるため、赤外光を完全に不可視と仮定してはいけない。発光部、直接光、targetからの反射光がEVS画角へ入ると、飛び出しと同時に強いevent burstを生じ、モデルがphotogateを手掛かりにするデータリークとなる可能性がある。

推奨順は次の通りである。

1. スロット型photointerrupterと物理フラグをEVS画角外へ置き、赤外光をセンサ筐体内へ閉じ込める
2. 対向型break-beamを使う場合は、発光・受光部を画角外に置き、黒色tube / hoodで直接光と反射を抑える
3. photogate信号をSilkyEvCamのTrigger Inへ入力できる場合は、画像上のLEDではなく外部trigger eventとしてsensor clock上へ記録する。ただし電圧、極性、connector pinは機種資料で確認し、直接配線しない

採用前に、targetなしでphotogateだけを遮光・復帰させるnegative controlを繰り返し、EVSの入力ROIに同期eventが発生しないことを確認する。発生する場合は配置を変更し、単に学習画像から発光部をcropするだけで済ませない。

#### 外部センサを使えず、LEDだけを使える場合

LED遷移を飛び出し機構の作動信号と同時に発生させ、EVS raw stream上のLED ROIから`t_led`を取得する。LED ROIは20ch hazard modelへ入力する前に必ずmaskし、モデルが同期信号を危険判定の手掛かりとして利用できないようにする。RGB入力でも同じ領域をmaskする。

この構成で直接測れるのは原則として`t_release`相当であり、物理的な`t_visible`ではない。機構の作動遅延を別途測れない場合は、絶対値を「LED基準反応時間」と明記する。一方、同一trialにおけるEVSとRGBの検知差`t_detect,RGB - t_detect,EVS`では共通起点が相殺されるため、相対比較は維持できる。

LEDをtarget先端へ固定し、遮蔽物からtargetと同時に初めて見えるよう機械的に配置できるなら、LED初回出現を`t_visible`の代理とできる。ただしLEDはmodel crop外または固定mask領域に置き、人間の反応手掛かりにならないようにする。

LEDだけを点灯・消灯し、targetを動かさないnegative controlを実施する。hazard modelの出力が変化する場合はmaskまたは前処理が不十分であり、そのtrial設計を採用しない。

##### 着脱式LED同期板による推奨運用

LED回路を車体へ常設せず、記録の開始直後と終了直前だけ手でEVS / RGBの共通画角へ提示する方法を採用できる。これは映像制作のclapperboardに相当する光学的な同期方法である。

1. EVS / RGB / bagの記録を開始する
2. LED消灯状態でbreadboardを共通画角へ入れる
3. boardを静止させて0.5--1秒待つ
4. 識別可能な短・長パターンを2--3周期記録する
5. LEDを消灯し、boardを画角外へ除去する
6. 0.5--1秒のguard interval後に走行trialを開始する
7. 走行終了・車体停止後、LED消灯状態でboardを再び画角へ入れる
8. 静止後に同じパターンを2--3周期記録し、その後にbagを停止する

boardの挿入・除去はEVSに大量のmotion eventを生じるため、点滅区間を含めて同期前後のguard interval全体を学習・評価対象から除外する。開始と終了でLEDの画像位置が変わっても時刻対応には問題ないため、それぞれ別ROIでedgeを抽出する。

対応する複数のLED ON / OFF edgeから

\[
t_{EVS}=a\,t_{RGB}+b
\]

を当てはめ、開始・終了間のclock driftを補正する。fit residual、開始offset、終了offsetを保存し、残差が1 RGB frameを超える、timestampが逆行する、sensor / nodeが途中再起動する場合は、そのrecordingを単一の同期区間として扱わない。

この方法はEVSとRGBの時刻軸を対応付けるが、飛び出しそのものの`t_visible`を直接与えるものではない。絶対反応時間には同期後の映像上で遮蔽物境界を越えた時刻を別途annotateし、相対的なEVS--RGB検知差では共通起点が相殺されることを利用する。

### 10.3 条件

最低限、次を変える。

- target横速度: low / mid / high
- ego速度: stationary / low / nominal
- 出現方向: left / right
- 照明: bright / ordinary / low light
- 背景event密度: static / ego motionあり
- positive / negative / distractor / catch trial

特に、静止カメラで横から動体が出るだけではevent数で容易に解ける。最終評価には、ego motion、影、背景物体、走行領域外を通る非危険動体を入れ、単なる「eventが増えた」判定から危険判定へ引き上げる。

## 11. データ作成と分割

1. RGB、EVS、target trigger、photogate、ego control、wheel speedを同一sessionへ記録する。
2. `t_visible`と`t_hazard`を外部計測から付与する。
3. 4 ms境界ごとに20ch tensorとhazard labelを作る。
4. train / validation / testはframe単位ではなくsession、日、背景、target appearance単位で分割する。
5. 同一飛び出しの隣接tensorがtrainとtestへ跨がないようにする。
6. test thresholdはvalidationで固定したものを用いる。

label候補:

- `0`: target未出現または走行領域へ侵入しない
- `1`: targetが見え始め、現在軌道のままなら規定時間内に走行領域へ入る

飛び出し直前をpositiveにすると未来情報を要求するため、センサから観測不可能な区間へ正例を付けない。

## 12. オンライン評価の段階

### Stage A: 静止ego・危険信号のみ

- ego RCを停止
- targetを遮蔽物から飛び出させる
- EVS/RGB/humanのTTFDを比較
- actuatorを動かさない

これが最小の研究成立条件である。

### Stage B: 走行ego・shadow mode

- ego RCを一定低速で走行
- モデルは危険信号を出すが、自動ブレーキへ接続しない
- 人間または安全系が車両を停止
- ego motion下のfalse alarmとTTFDを評価

### Stage C: 閉ループブレーキ

- 危険信号をbrake commandへ接続
- `t_visible -> t_detect -> t_cmd -> t_act -> stop`を計測
- 停止距離、衝突率、最小距離を評価

targetは柔らかいダミーまたは壊れにくい小型車とし、速度制限、物理E-stop、走行範囲制限を設ける。人を実際の飛び出し対象にしない。

### Stage D: EVS-triggered RGB confirmation

将来提案として、EVSの軽量モデルが早期warningまたは予備制動を出し、RGBモデルが対象を確認して本制動へ移る構成を評価する。

```text
EVS 20ch small CNN
  -> early warning / pre-brake
  -> RGB confirmation
  -> full brake / avoidance
```

これは「EVSがRGBの最終認識精度を超える」必要がなく、両者の長所を使える。

## 13. 統計計画

- 同一物理条件のEVS/RGB比較はpaired designにする
- trialまたはsession単位のbootstrap 95% confidence intervalを用いる
- 非正規な反応時間にはpaired permutationまたはWilcoxon系を検討する
- 人間はparticipant内反復を考慮し、participant単位の要約またはmixed-effects modelを使う
- speed、lighting、directionを層別化する
- meanだけでなくmedian、p95、分布CDFを示す
- missとfalse alarmを遅延とは別に必ず示す
- online collision successは成功率と二項信頼区間を示す

sample数は先に恣意的に決めず、pilot trialから分散と効果量を見積もる。ポスター段階で十分な検出力が得られない場合は、p値より効果量とconfidence intervalを中心に報告する。

## 14. 先行研究との差分

### 14.1 重要な先行研究

| 研究 | 既に示されていること | 本研究との差分候補 |
| --- | --- | --- |
| Falanga et al., RA-L 2019 | perception latencyと安全速度の関係を理論化し、event cameraによるquadrotor障害物回避を実証 | 本研究はRCカー停止、学習型20ch detector、RGB・人間比較、モデル規模上限を扱う。ただし「実機回避そのもの」は新規ではない |
| Mao et al., ICCV 2019 | APだけではvideo detectorの検知遅延を表せず、false alarmを考慮したdelay metricが必要 | stable TTFDとfalse alarm制約をRC飛び出しへ適用する |
| Arakawa and Shiba, 2020 | DAVIS搭載car-like robotを100 Hzで制御し、投げ込まれた物体への停止を実機デモ | 同研究はtimestampを捨てた単純event image、外部MacへのWebSocket推論、定量的RGB/人間比較なし。本研究はonboard Jetson、20ch temporal bins、250 Hz、共通`t_visible`、end-to-end定量評価を狙う |
| Gehrig and Scaramuzza, CVPR 2023 (RVT) | 10時間bin x 2極性の20ch histogramを用い、Tiny / Small / Baseのaccuracy--latency trade-offを評価。T4上で2.3 / 3.0 / 3.7 msを報告 | 20chやモデル規模比較自体は既存。本研究は一般mAPではなく、RC飛び出しの物理時刻からの早期危険反応とJetson実装上の期限を扱う |
| Gehrig and Scaramuzza, Nature 2024 (DAGr) | RGB+eventの非同期検出によりframe間blind timeを埋め、低いperceptual latencyを示す | 非常に近い動機。論文自身はopen-loop評価と明記。本研究は小型実機でcommand/actuationまでのclosed-loop評価と人間参照を狙う |
| Falanga et al., Science Robotics 2020 | event cameraによる動的障害物回避を実機quadrotor上で閉ループ実行し、event受信から最初の回避指令まで約3.5 msを報告 | 「EVSの実機閉ループ回避」や「数ms応答」自体は新規でない。本研究は陸上RCの横方向飛び出し、20ch NNの規模上限、RGB・人間との共通刺激比較を扱う |
| Li et al., ECCV 2024 / EvTTC | event-aided TTC推定、実車・小型testbed、FCWを評価 | 前方接近TTCではなく遮蔽からの横方向飛び出し、危険信号TTFD、モデル計算期限を扱う |
| Khan et al., arXiv 2025 (EMF) | 2 polarities x 10 binsの20ch stacked histogramをDNN object detectionに使用し、accuracyとinference timeを評価 | 20ch自体は新規でない。本研究は小型hazard classifier、Jetson 4 ms deadline、物理飛び出しのonline応答へ焦点を移す |
| Huang et al., 2026 | DVSとRGBを用いた横断歩行者検知で、CARLAおよび実収録データ上の早期検知を評価 | 「飛び出し・横断対象をEVSでRGBより早く検知」だけでは新規性にならない。本研究はevent-only 20ch危険判定、物理`t_visible`、モデル規模、実機command / actuationを差分にする |
| DeepIPCv3, arXiv 2026 | DVS+LiDARで突然の歩行者横断を扱い、RGB系と比較しResponse Time Delayを報告 | 最も近い研究。offlineのみ、4 Hz収録で250 ms刻み、expert actionを時刻基準とする。本研究はRCで安全にonline化し、外部`t_visible`、4 ms event更新、実際のcommand/actuation、人間を共通起点で測る点を差分候補とする |

### 14.2 新規性として主張しないこと

- event cameraがmicrosecond級の時間分解能を持つこと
- event cameraがRGBよりmotion blurに強いという一般論
- event streamをtemporal binsへ変換すること
- 2 polarities x 10 binsの20ch stacked histogram
- CNNやTransformerでevent-based object detectionを行うこと
- event cameraをRC/robotの衝突回避へ使うこと
- event cameraで横断者をRGBより早く検知するという一般的主張
- event cameraによる数ms級の閉ループ回避
- accuracy--latency trade-offを調べるという一般概念

### 14.3 安全に主張できる新規性候補

> 20ch event histogramを250 Hzで生成するGPU常駐pipelineと小型危険検知CNNをRCカーへ実装し、物理的な飛び出し開始から危険信号・ブレーキ指令・車体応答までをオンラインで分解計測する。さらに、同一飛び出しに対するRGBモデルおよび人間の反応を、共通の外部時刻基準と誤警報制約のもとで比較し、4 ms周期を維持できるモデル規模上限を明らかにする。

2026-09-20時点の重点的な文献調査では、この全条件を同時に扱う研究は確認できていない。ただし、これはsystematic reviewによる「世界初」の証明ではないため、ポスターでは「世界初」「前例がない」と断定せず、「既存研究では十分に扱われていない」「本研究では実機オンライン条件まで拡張する」と表現する。

## 15. 結果別の着地点

### 15.1 EVSがRGBより早く、4 msも満たした場合

> 20ch EVSモデルは、同一false alarm constraintでRGBより早いstable hazard outputを示し、Jetson上で250 Hz deadlineを維持した。RCカー飛び出しに対する早期警戒の可能性が示された。

### 15.2 EVSは早いが4 msを満たさない場合

> 全snapshot処理には至らなかったが、sensor-native条件ではRGBより早い初回危険出力を得た。推論周期と早期検知性能のPareto境界を示した。

### 15.3 EVSのaccuracyがRGBより低い場合

> 完全露出後の認識性能はRGBが優れた一方、出現初期のearly recallまたは低照度条件ではEVSが早期警戒に寄与した。EVS-triggered RGB confirmationが妥当である。

### 15.4 EVSがRGBより早くない場合

> dense 20ch representationとニューラル推論によって、sensor-levelの時間的利点がsystem-levelでは失われる条件を定量化した。EVSを有効にするために許されるモデル計算量・推論時間の上限を示した。

これは否定的だが研究価値がある。センサ仕様だけからend-to-end低遅延を主張できないことを実機で示せる。

### 15.5 オンライン閉ループまで到達しない場合

> paired収録によるoffline TTFD、live pipeline latency、shadow-mode出力までを報告し、「衝突回避を実証した」ではなく「オンライン危険検知の実現可能性と潜在的利点を示した」と結論づける。

## 16. ポスターの推奨構成

### 背景

イベントカメラには低いsensor latencyがあるが、dense representationとニューラル推論を通した後も、その利点が危険反応時間として残るとは限らない。

### 目的

RCカー飛び出しに対する20ch EVS危険検知モデルのend-to-end応答を測定し、RGB・人間と比較するとともに、250 Hzを維持できるモデル規模上限を求める。

### 方法図

```text
physical pop-out (`t_visible`)
   ├─ EVS -> 20ch rolling histogram -> scaled CNN -> hazard -> brake
   ├─ RGB -> matched CNN -------------------------> hazard
   └─ Human -------------------------------------> button
```

### 主な図

1. `t_visible -> event -> tensor -> inference -> command -> actuation`のtimeline
2. モデル規模ごとのearly recall--p99 latency Pareto plot
3. EVS / RGB / humanのstable TTFD CDFまたはbox/violin plot
4. 速度ごとの反応距離差または停止成功率

### 結論

「EVSは高精度だった」ではなく、次の形式にする。

> 指定した誤警報率と実装条件のもとで、どのモデル規模までEVSの時間的利点が残るかを示した。

## 17. 実施優先順位

### 最優先

1. `t_visible`を独立に定義できる計測を作る。photogateがなければ側面240 fps映像と同期LEDを用い、約4.17 msの時間分解能と判定区間を明記する
2. positiveとcatch trialをpaired RGB/EVSで収録する
3. ROI threshold / 2ch CNN / 20ch Tiny CNNを比較する
4. validationで閾値を固定し、stable TTFDとfalse alarmを算出する
5. Jetson TensorRTでp99 latencyとdeadline missを測る

### 次点

6. width scalingでTiny / Small / Baseを比較する
7. moving egoのshadow-mode評価を行う
8. 人間のdirect-view button taskを行う

### 発展

9. closed-loop braking
10. RGB confirmationとの二段構成
11. TTCまたはcollision probability出力

「大きなモデルを先に完成させる」より、外部`t_visible`とfalse alarmを含む評価器を先に完成させる。評価器がなければ低遅延という主張を検証できない。

## 18. 現在のJetPilot資産との対応

- EVS native benchmark: `tools/evs_benchmark/`
- 40 ms / 4 ms / 20ch CUDA rolling pipeline: `ros2_ws/src/perception/jetpilot_e2e_inference/`
- 20ch PilotNet training preset: `python_ws/jetpilot_e2e_training/src/e2e_learning/conf/experiment/event_tensor_pilotnet.yaml`
- PilotNet backbone: `python_ws/jetpilot_e2e_training/src/e2e_learning/models/pilotnet.py`
- live TensorRT 250 Hz protocol: `tools/evs_benchmark/LIVE_EVALUATION.md`
- RGB+EVS simultaneous capture: `rgb-evs-benchmark` bringup preset
- 着脱式LEDによるRGB--EVS同期と手動`t_visible`注釈設計: `tools/rgb_evs_led_sync/`
- 記録済みRGB映像を用いた人間反応評価protocol: `tools/rgb_evs_led_sync/human_video_reaction_protocol.md`
- brake command recording: `/commands/motor/brake`およびbag manager

ただし、これらは現時点で「20ch危険検知から250 Hzで実制動する完成系」を意味しない。現行資産の境界は次の通りである。

- 4 ms deadlineの直接監視対象は20ch snapshot生成pipeline
- 既存学習labelとdecoderはsteering / throttle用で、hazard classを持たない
- 既存decoderのdeadline既定値は33.3 msで、brake出力は0固定
- command muxは既定100 Hz
- VESC vehicle interfaceは既定50 Hz、serial bridgeとPCA9685出力は既定100 Hz
- `/commands/motor/brake`は記録可能だが、hazard scoreから安全に上書きする経路は未実装
- JPBB用STM32 firmwareの編集可能なソースは現checkoutに含まれず、`jetpilot_bridge_interface/firmware`にはStep 8の`.hex`と説明だけがある。文書上の正本は外部`Arata-Stu/jetpilot_bridge_board` repository、記録されたbuild元commitは`f36fd306add45793dc73c1707112047ff8755eaa`である

不足している主要要素:

- hazard専用labelとdataset manifest
- hazard classifier head / export metadata
- hazard scoreと判定時刻をpublishするruntime node
- hazard判定を既存制御より優先してブレーキへ接続するsafety overrideとwatchdog
- physical `t_visible` logger
- RGB baselineの同一task学習・評価
- stable TTFD / early recall / false alarm解析
- sensor / decision / command / actuationを同一clockへ対応付けるtrace
- closed-loop safety interlock

## 19. 参考文献

1. Falanga, D., Kim, S., Scaramuzza, D., “How Fast Is Too Fast? The Role of Perception Latency in High-Speed Sense and Avoid,” IEEE RA-L, 2019. <https://rpg.ifi.uzh.ch/docs/RAL19_Falanga.pdf>
2. Mao, H., Yang, X., Dally, W. J., “A Delay Metric for Video Object Detection: What Average Precision Fails to Tell,” ICCV, 2019. <https://openaccess.thecvf.com/content_ICCV_2019/papers/Mao_A_Delay_Metric_for_Video_Object_Detection_What_Average_Precision_ICCV_2019_paper.pdf>
3. Arakawa, R., Shiba, S., “Exploration of Reinforcement Learning for Event Camera using Car-like Robots,” arXiv:2004.00801, 2020. <https://arxiv.org/abs/2004.00801>
4. Gehrig, M., Scaramuzza, D., “Recurrent Vision Transformers for Object Detection With Event Cameras,” CVPR, 2023. <https://openaccess.thecvf.com/content/CVPR2023/html/Gehrig_Recurrent_Vision_Transformers_for_Object_Detection_With_Event_Cameras_CVPR_2023_paper.html>
5. Gehrig, D., Scaramuzza, D., “Low-latency automotive vision with event cameras,” Nature, 2024. <https://www.nature.com/articles/s41586-024-07409-w>
6. Li, J. et al., “Event-Aided Time-to-Collision Estimation for Autonomous Driving,” ECCV, 2024. <https://www.ecva.net/papers/eccv_2024/papers_ECCV/papers/07043.pdf>
7. Sun, K. et al., “EvTTC: An Event Camera Dataset for Time-to-Collision Estimation,” IEEE RA-L, 2025. <https://nail-hnu.github.io/EvTTC/>
8. Khan, M. A. U., Khan, A. H., Dengel, A., “EMF: Event Meta Formers for Event-based Real-time Traffic Object Detection,” arXiv:2504.04124, 2025. <https://arxiv.org/abs/2504.04124>
9. Natan, O. et al., “DeepIPCv3: Event-Aware Multi-Modal Sensor Fusion for Sudden Pedestrian Crossing Avoidance,” arXiv:2606.01277, 2026. <https://arxiv.org/abs/2606.01277>
10. Makishita, H., Matsunaga, K., “The brake reaction time to a sudden hazard while driving,” Japanese Journal of Ergonomics, 2002. <https://doi.org/10.5100/jje.38.324>
11. Falanga, D. et al., “Dynamic obstacle avoidance for quadrotors with event cameras,” Science Robotics, 2020. <https://pubmed.ncbi.nlm.nih.gov/33022598/>
12. Huang, Y. et al., “Low Latency Crossing Pedestrian Detection by Dynamic Vision Sensor and RGB Camera,” Journal of Intelligent & Robotic Systems, 2026. <https://link.springer.com/article/10.1007/s10846-026-02361-5>

## 20. 最終的な研究メッセージ案

> イベントカメラの高い時間分解能は、それだけでは短い危険反応時間を保証しない。本研究では、20ch rolling histogramとニューラル危険検知をRCカーへ実装し、物理的な飛び出し開始から危険信号までを測定した。モデル規模、検知性能、推論遅延の関係を評価することで、250 Hz処理および早期飛び出し検知において許容されるモデル規模を示し、RGBおよび人間の反応時間との比較からその実用的な位置づけを明らかにする。
