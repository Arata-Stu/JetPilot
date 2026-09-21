# RGB--EVS LED同期・飛び出し時刻注釈設計

## 1. 目的と境界

この系では、Pro Micro、MOSFET、可視LEDを着脱式の光学同期板として使用する。

- LED板の役割: RGBとEVSの時刻軸のoffset / driftを推定する
- 手動注釈の役割: 遮蔽物からtargetが見え始めた時刻を記録する
- LED板が行わないこと: 飛び出し時刻を物理的に検出する

したがって、LED時刻を`t_visible`とは呼ばない。開始・終了markerを使って

```text
t_evs = scale * t_rgb + offset
```

を推定し、RGB timestampをEVS sensor-time domainへ写像する。

## 2. 対象ハードウェア

想定回路はPro MicroのD9でN-channel MOSFETを駆動するlow-side LED switchである。

```text
LED電源 -- LED -- 電流制限抵抗 -- Drain
                                      MOSFET
Pro Micro D9 -- gate抵抗 ----------- Gate
                                      |
                                 pull-down抵抗
                                      |
Source ----------------------------- GND
```

確認項目:

- MOSFET SourceがGNDへ接続されている
- Pro MicroとLED電源のGNDが共通である
- D9からGateの間に抵抗がある
- Gate--GND間にpull-down抵抗がある
- LED電源電圧と電流制限抵抗が適切である
- `HIGH`で点灯し`LOW`で消灯する

回路がhigh-side / active-lowの場合はfirmwareの`kLedActiveHigh`を`false`にする。

## 3. Firmware

[`firmware/pro_micro_led_sync/pro_micro_led_sync.ino`](firmware/pro_micro_led_sync/pro_micro_led_sync.ino)
は、resetまたはpower-on後に次の動作を一度だけ行う。

1. LEDを消灯
2. setup開始後1.5秒待つ
3. 非周期的な短・長patternを2回出力
4. LEDを消灯したまま待機

100 / 300 msだけのpatternは30 / 60 / 90 fpsのframe周期と整数比に近い。提案firmwareでは
137 / 211 / 293 / 163 / 421 / 251 / 179 msを組み合わせ、複数edgeが異なるframe位相へ入りやすくする。

Pro Microをportable USB電源で動かす場合、USB Serialは接続されていなくてもよい。接続されている場合だけ、各edgeの`micros()`を診断用に出力する。光学edgeが同期の正本であり、USB Serialの受信時刻は同期時刻として使わない。

Arduino IDEでは実機に合う`SparkFun Pro Micro`または`Arduino Leonardo`を選択する。5 V / 16 MHzと3.3 V / 8 MHzを取り違えるとUSBやclock設定が不正になるため、基板表記を確認する。

## 4. 収録手順

現在の`rgb-evs-benchmark`系では、RGBはrosbag2の
`/realsense/color/image_raw`、EVS payloadは同じsession directoryのOpenEB native RAWへ保存される。

各sessionを次の順序で記録する。

1. EVS native RAW、RGB rosbag、diagnosticsの記録を開始する
2. LED消灯状態で同期板をRGB / EVSの共通画角へ入れる
3. 同期板を静止させる
4. Pro Microをresetし、marker patternを最後まで記録する
5. LED消灯を確認し、同期板を画角外へ除去する
6. 1秒以上のguard intervalを置く
7. 飛び出しtrialを実施する
8. 走行終了後、ego / targetの双方が停止したことを確認する
9. LED消灯状態で同期板を再度共通画角へ入れて静止させる
10. Pro Microをresetし、終了markerを最後まで記録する
11. marker完了後に記録を停止する

開始と終了で同期板の画像位置が異なってもよい。解析時は別々のROIを指定する。

同期板の挿入、除去、手の動きはEVSへ大量のeventを発生させる。開始marker終了から走行開始まで、走行終了から終了marker開始までをguard intervalとし、同期区間とともに学習・危険検知評価から除外する。

## 5. LEDの撮影条件

- LEDを両カメラの共通画角へ置く
- 近づけすぎてRGBが広範囲に飽和しないようにする
- 必要なら拡散板を付け、LED chipではなく小さな均一面として見せる
- RGBのauto exposureがmarker中に大きく変化する場合、露光を固定するか輝度を下げる
- marker中はboardを静止させる
- pattern開始前後に十分な消灯区間を入れる
- 人間の反応trialでは同期板を画角外へ除去してから開始する

## 6. Edge抽出

### 6.1 RGB

開始markerと終了markerで別々にLED ROIを選び、frameごとの平均輝度を求める。
各edgeは一点ではなく、次の区間として保存する。

- `rgb_before_s`: 遷移前状態を最後に確認したframe timestamp
- `rgb_after_s`: 遷移後状態を最初に確認したframe timestamp

代表値には区間中央を使用し、半区間幅をRGB量子化不確かさとする。

### 6.2 EVS

同じLED ROIのON / OFF eventを0.5--1 ms binへ集計する。

- ON edge: 正極性event burst
- OFF edge: 負極性event burst

単一の最初のeventはhot pixelやbackground activityの影響を受けるため使わない。静止中baselineを超えるburst onset、またはcluster先頭の頑健な分位点をedge時刻とする。抽出したedge列がfirmware patternの順序と一致することを確認する。

## 7. Clock map

手動または自動抽出した対応edgeを次のCSVへ保存する。

```csv
marker,edge,state,rgb_before_s,rgb_after_s,evs_time_s
start,0,on,1.500000,1.533333,0.872410
start,1,off,1.633333,1.666667,1.009522
end,0,on,61.500000,61.533333,60.875104
end,1,off,61.633333,61.666667,61.012225
```

時刻はsession内の相対秒を推奨する。epoch秒でも計算できるが、CSVを人手で扱いやすくするため相対値を使う。

標準libraryだけで動くfit toolを用意している。

```bash
python3 tools/rgb_evs_led_sync/fit_clock_map.py \
  --edges sync_edges.csv \
  --output clock_map.json
```

出力modelは次である。

```text
t_evs = scale * t_rgb + offset_s
drift_ppm = (scale - 1) * 1e6
```

開始・終了それぞれ最低4 edge、全体で最低8 edgeを使用する。開始と終了がなく、片側markerだけの場合はoffset確認に留め、driftを主張しない。

### 合格条件

- 開始・終了patternを両sensorで一意に対応付けられる
- timestampがsession内で単調増加する
- sensor、driver、recording nodeの途中再起動がない
- dropped frame / RAW interruptionをdiagnosticsで確認する
- fit residualが各RGB edgeの半区間幅に概ね収まる
- residualが1 RGB frameを超えるedgeは映像を再確認する

clock mapの`scale`、`offset_s`、RMSE、最大絶対残差、開始offset、終了offsetをsession metadataとともに保存する。

## 8. 飛び出し時刻の手動注釈

### 8.1 Primary annotation: RGB区間

遮蔽物境界へ固定lineを引き、target先端について次を記録する。

- `rgb_last_hidden_s`: targetがまだ見えていない最後のRGB frame
- `rgb_first_visible_s`: targetが見えた最初のRGB frame

clock mapで両端をEVS timeへ変換する。

```text
t_visible_evs in [map(rgb_last_hidden), map(rgb_first_visible)]
```

一点値が必要な図では区間中央を使えるが、解析の正本には上下限を保持する。

### 8.2 Secondary annotation: event細粒度

遮蔽物境界付近のeventを細かいtime sliceで表示し、targetに帰属できるedgeが境界を初めて越えた時刻を`event_visible_s`として記録できる。

これは高い時間分解能を持つ一方、評価対象であるEVS自身から起点を決めるため独立ground truthではない。主解析のRGB区間を置き換えず、次の用途に限定する。

- RGB区間内での細粒度な参考値
- annotation sensitivity analysis
- early event形成過程の可視化

### 8.3 反応時間の報告

hazard detection時刻を`t_detect`とすると、RGB区間から得る反応時間は一点ではなく次の区間になる。

```text
TTFD in [t_detect - t_visible_upper, t_detect - t_visible_lower]
```

EVSとRGBの検知差は、両者を共通EVS clockへ写像した後、起点を使わず直接求める。

```text
delta_detection = map(t_detect_rgb) - t_detect_evs
```

この差では共通の`t_visible`が相殺されるため、EVS--RGB比較の主指標に向いている。

注釈は[`popout_annotations.example.csv`](popout_annotations.example.csv)を複製して記録する。

## 9. EVS動画の見返し方

表示用event imageの25 fpsだけで`t_visible`を決めない。native RAWから、遮蔽物境界の前後だけを次のような複数slice幅で再生成する。

- 1 ms: 最初のevent burst候補の探索
- 4 ms: 20ch modelの更新周期との対応
- 10 ms: 形状確認
- 20--40 ms: 人間が対象輪郭を確認するためのoverview

最初は20 ms程度で対象を見つけ、10 ms、4 ms、1 msへ狭めるcoarse-to-fine方式を用いる。単一hot pixelではなく、target輪郭として空間的に連続するevent clusterが遮蔽物境界を越えた時刻を選ぶ。

annotation時には次を保存する。

- RAW file名とsession ID
- ROI / 遮蔽物境界line
- slice幅
- polarity表示方法
- 選択したtimestamp
- 直前 / 直後の画像
- annotatorとconfidence

## 10. 解釈上の注意

- LED同期はsensor clock間の対応であり、物理的な飛び出し検出ではない
- RGB timestampがcapture時刻かhost受信時刻かをsessionごとに確認する
- LED fitが良くても、RGB exposureが長い場合はscene motionの時間ぼけが残る
- event-derived`t_visible`だけでEVSのTTFDを評価すると有利な定義になり得る
- 同じtrialのEVS--RGB検知差、RGB区間を用いた絶対TTFD、event細粒度TTFDを分けて報告する

人間へ記録済みRGB映像を提示する実験は、[`human_video_reaction_protocol.md`](human_video_reaction_protocol.md)に分離して定義する。
