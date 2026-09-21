# 記録済み飛び出し映像を用いた人間反応時間評価プロトコル

## 1. 評価対象

本プロトコルが測る対象は、現場を直接見る人間ではなく、記録済みRGB映像を画面上で見た参加者の危険反応である。

論文・ポスターでは次のように記述する。

> 同期収録したRCカー飛び出しscenarioのRGB映像を参加者へ提示し、targetがego走行領域へ侵入すると判断した時点でボタンを押す課題における反応時間を測定した。

EVS / RGB modelとの比較では、人間の値を厳密な知覚能力の優劣ではなく、表示・判断・運動応答を含むhuman referenceとして扱う。

## 2. 最小の研究質問

> 同一の飛び出しtrialに対し、EVS hazard model、RGB hazard model、RGB映像を視聴する人間は、どの時点で最初の有効な危険応答を生成するか。

主要比較:

- EVS stable hazard decision
- RGB stable hazard decision
- human key-down / button-down

必ずmissとfalse alarmを反応時間と併記する。

## 3. Scenario収録

RGBとEVSを同時収録し、各物理trialへ一意な`clip_id`を割り当てる。LED同期板はsessionの開始・終了だけに提示し、trial中は画角外へ除去する。

### 3.1 Positive trial

- targetが遮蔽物から出現する
- 現在軌道を維持するとego走行領域へ侵入する
- 左 / 右の双方
- target速度を複数設定する
- 出現までの待ち時間をランダム化する

### 3.2 Negative / catch trial

- targetが出現しない
- 遮蔽物内で動くが出現しない
- targetは見えるが走行領域へ入らない
- 影、背景物体、ego motionだけが生じる
- 走行領域外を横切る物体

単純なno-event映像だけをnegativeにすると、人間もmodelも「何か動いたら押す」で解ける。動きはあるが危険でないhard negativeを含める。

### 3.3 最初に固定する条件

初回pilotでは条件を増やしすぎない。

```text
ego: stationary
direction: left / right
target speed: low / high
lighting: one controlled condition
trial type: positive / hard negative / catch
```

pilotが成立した後、ego motion、照明、遮蔽物、target appearanceを増やす。

## 4. Dataset分割

連続frame単位でrandom splitしない。収録日、session、背景、target appearance単位でtrain / validation / testを分ける。

- train: EVS / RGB model学習用
- validation: model threshold、debounce、false alarm operating point決定用
- test: 最終model評価および人間提示用

人間へ提示するclipはtest splitだけとし、model threshold調整へ使用しない。

## 5. 人間提示用clip

各clipは次を満たす。

- 音声を除去する
- 同期LED区間を含めない
- model出力、box、timestamp文字を重畳しない
- 解像度、fps、画角、輝度処理を統一する
- positive / negativeでclip長から答えが分からないようにする
- 飛び出し前のlead-inを複数長からrandomに設定する
- 飛び出し後の長さを統一する

推奨lead-inは1.5--4.0秒の範囲でclipごとに変える。一定時刻に必ず飛び出す設計を避ける。

RGB captureが30 / 60 / 90 fpsでも、提示fpsはmonitor refreshとpresentation softwareが安定して再生できる条件へ固定する。例えば60 Hz monitorでは60 fps提示を基本とする。90 fps収録を60 fpsへ変換した場合は、どのframeを選んだかをmanifestへ保存する。

## 6. 提示software

一般的な動画playerではなく、次を満たす専用のlocal presentationを用いる。

- clipを開始前にmemoryへ読み込み、再生中decode stallを避ける
- vertical syncを有効にする
- 実際のframe flip時刻をmonotonic clockで記録する
- key-downまたはbutton-downの時刻を同じclockで記録する
- key repeatを無効化する
- fullscreenで通知、menu bar、cursorを隠す
- clip ID、順序、frame dropをlogする

初期実装候補はPsychoPyまたはPsychtoolboxである。通常のOpenCV windowはOS compositorとvsyncの時刻保証が弱いため、予備実験には使えても最終反応時間の正本にはしない。

## 7. 人間trialの時間定義

参加者が実際に見る刺激では、物理的`t_visible`ではなく、targetが初めて表示されたframeのflip時刻を起点とする。

```text
t_visible_display = targetが初めて見えるframeを表示したflip時刻
t_press = button key-downを取得した時刻
RT_human_video = t_press - t_visible_display
```

映像上の`t_visible_display`はRGBの`first_visible_frame_index`で固定する。

これはcapture、display、human motor responseを含む値であり、EVS sensor latencyと同一概念ではない。比較時はhuman video responseと明記する。

## 8. Trial判定

### Positive

- correct: `t_visible_display`以後、規定時間内にbutton-down
- anticipatory false alarm: `t_visible_display`より前にbutton-down
- miss: clip終了または規定時間までにbutton-downなし

### Negative / catch

- correct rejection: button-downなし
- false alarm: button-downあり

positive trialで押した試行だけを使って平均反応時間を計算しない。miss率とanticipatory responseを別に残す。

## 9. 実施手順

1. 研究説明、同意、参加者ID発行
2. 視距離、monitor、照明、button deviceを固定
3. 説明文を読み上げず、全参加者へ同じ文章で提示
4. 評価に使わないclipでpracticeを行う
5. positive / negative / catchを混ぜてrandom提示
6. 一定trial数ごとに休憩を入れる
7. 終了時に映像の予測可能性、見づらさ等を簡単に確認

課題文の例:

> 映像を見て、対象が自車の進行領域へ入り危険だと判断した時点で、できるだけ早くボタンを押してください。対象が見えても進行領域へ入らない場合は押さないでください。何も起きない映像もあります。

「何かが見えたら押す」ではなく、危険判断課題であることを明示する。

## 10. Trial数と参加者

最初は実装・課題理解を確認するpilotと、本評価を分ける。

### Pilot

- 研究者を含む少人数
- 目的はsoftware、課題文、clip難易度、frame timingの確認
- population-levelの結論を出さない

### 本評価

- 参加者数はpilotで得たparticipant間分散と効果量から決める
- 各参加者は同一clipを原則1回だけ見る
- clip数が多い場合は複数listへ分けて参加者間でcounterbalanceする
- 同一参加者の多数trialを参加者数として水増ししない

人を対象とするデータを収集する前に、所属機関の倫理審査、免除判定、同意取得、匿名化、撤回手順を確認する。承認前に本評価データを収集しない。

## 11. 保存schema

### 11.1 Clip manifest

```csv
clip_id,session_id,trial_type,direction,target_speed,source_fps,presentation_fps,first_visible_frame,lead_in_s,split
```

### 11.2 Human response

```csv
participant_id,clip_id,presentation_order,t_visible_flip_ns,response_ns,rt_ms,outcome,dropped_frames
```

`outcome`:

- `hit`
- `miss`
- `anticipatory`
- `correct_rejection`
- `false_alarm`

参加者識別子と同意書等の個人情報はresponse CSVから分離する。

## 12. EVS / RGB modelとの対応

同一`clip_id`に対して次を保存する。

```csv
clip_id,evs_detect_evs_s,rgb_detect_rgb_s,rgb_detect_evs_s,evs_outcome,rgb_outcome
```

LED markerから求めたclock mapでRGB modelの検知時刻をEVS clockへ変換する。

主要なmachine差:

```text
delta_rgb_evs = rgb_detect_evs_s - evs_detect_evs_s
```

人間については提示clock上の反応時間を使い、machine sensor clockへ無理に直接接続しない。物理trialの`first_visible_frame`を共通のclip anchorとして分布を比較する。

## 13. 主評価指標

- participantごとのmedian `RT_human_video`
- participantごとのfalse alarm率、miss率
- EVS / RGBのstable TTFD、false alarm率、miss率
- 8 / 16 / 32 / 64 msにおけるmachine early recall
- human / EVS / RGBの反応時間CDF
- 条件別のmedianとconfidence interval

人間の反復trialを独立sampleとして単純集計しない。participantごとに要約するか、participantとclipをrandom effectとして扱う。

## 14. 最小成立ライン

ポスター向けの最小構成:

1. stationary ego
2. 同期RGB / EVSのtest clip
3. positive、hard negative、catch
4. EVS modelとRGB modelを同一clipで評価
5. 人間はRGB映像を一回ずつ視聴
6. human hit / miss / false alarmと反応時間を記録
7. 表示遅延を未補正なら明記し、humanをreferenceとして扱う

この段階でも、「EVSの低いsensor latencyが、RGB modelおよびRGB映像視聴時のhuman responseに対して、危険出力時間としてどの位置にあるか」を示せる。
