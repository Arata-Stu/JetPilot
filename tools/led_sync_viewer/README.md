# LED Sync Inspector

## Python環境

既存containerで解析環境を作る場合は、container内で次を実行します。

```bash
cd /workspaces
scripts/setup/setup_multi_sensor_calibration_env.sh
source /workspaces/.venvs/multi_sensor_calibration/bin/activate

source /opt/ros/jazzy/setup.bash
cd /workspaces/ros2_ws
colcon build --packages-select multi_sensor_calibration --symlink-install
source /workspaces/ros2_ws/install/setup.bash
```

新しくbuildしたJetPilot imageには同じ環境が
`/opt/multi_sensor_calibration_env`として含まれます。一括export scriptは、image内の
環境、workspace内の一時環境の順で自動検出します。

Docker内で次を実行します。

```bash
scripts/experiments/led_sync_gui.sh
```

Macのbrowserで <http://localhost:8765> を開きます。データの開き方は2通りあります。

- `Mac・ローカル解析フォルダ`: Finderで`led_sync_data.json`と`preview/`を含む解析
  フォルダを選びます。JSONと画像をMacへコピー済みなら、serverを介さず読み込めます。
- `Docker解析を選ぶ`: serverが`/workspaces/record`配下の`led_sync_data.json`を自動で
  探し、一覧を表示します。Docker内のpathを手入力する必要はありません。

browserの安全制約により、ローカル側ではJSON単体ではなく解析フォルダを選択します。
これによりJSONから参照される`preview/rgb/`と`preview/evs/`も同時に利用できます。
新しい解析フォルダには`roi_data/`も含まれます。これはUIでLED位置を選び直したときに、
開始側・終了側それぞれのRGB輝度と1 ms EVSイベント数を再計算するためのデータです。

既定ではlocalhostだけで待ち受けます。Dockerがbridge networkでhost側へportを公開する
必要がある場合のみ、container起動時に`8765:8765`を公開したうえで次を使います。

```bash
scripts/experiments/led_sync_gui.sh --host 0.0.0.0
```

serverが読み出せるのは、既定では`/workspaces/record`配下の解析JSON、そのJSONが
参照するpreview画像、およびUIから生成したMP4だけです。

## v1/v2の一括export

まず非校正シーケンスを1本だけ確認する場合は、記録ディレクトリを明示します。

```bash
scripts/experiments/export_led_sync_batch.sh \
  --session /workspaces/record/evs-popup-v1/<session-directory>
```

出力先は
`/workspaces/record/evs-popup-v1/analysis/led_sync/<session-directory>/`です。

checkerboard校正sessionを除外し、`evs-popup-v1`と`evs-popup-v2`の各走行を順番に
処理します。既存の出力は自動的にskipされるため、途中で止まっても再実行できます。

```bash
scripts/experiments/export_led_sync_batch.sh
```

既定では各sessionの先頭・末尾12秒を60 fpsでpreview出力します。RGBの観測周期に
合わせて目視同期できる設定です。古いJSONを
preview付きで作り直す場合は`--force`を付けます。

動的ROIに対応する前に作った既存JSONには`roi_data/`がないため、一度だけ次を実行して
再エクスポートしてください。同期オフセットの推定はここでは行わず、UIの
「選択ROIで再解析」「自動推定を実行」で行います。

```bash
scripts/experiments/export_led_sync_batch.sh --force
```

LEDのROIが分かっている場合は、全画面より小さいROIを指定してください。

```bash
scripts/experiments/export_led_sync_batch.sh \
  --rgb-roi X,Y,W,H \
  --evs-roi X,Y,W,H
```

出力先は`evs-popup-vN/analysis/led_sync/<session>/`です。

GUI上部ではRGBとEVSを別々のシークバーで移動できます。同じLEDエッジを表示して
「この2枚を手動同期」を押すと、RGB時刻−EVS時刻が手動オフセットになります。
半自動推定後は「推定オフセットを表示へ反映」でEVS位置を固定したままRGB表示だけを
移動できます。「同期スロー再生」はEVS時刻を基準に推定式を適用し、対応するRGB画像を
0.1/0.25/0.5/1倍速で表示します。再生範囲は「現在位置±1秒」「開始区間」「終了区間」から
選べます。開始・終了区間は互いに独立して再生され、未出力の記録中央へ飛びません。
同じ点灯・消灯エッジが一致しているかを目視確認できます。
「同期−1コマ」「同期＋1コマ」はEVSプレビューを1コマ移動し、推定したオフセット・
ドリフトを適用したRGBフレームも同時に更新します。各画像下の±1はセンサごとの独立した
コマ送りなので、手動同期の微調整に使用できます。

「開始側」または「終了側」を選び、両方の画像上でLEDをドラッグして囲んだ後、
「選択ROIで再解析」を押すと、選んだ4つのROIから信号を作り直します。ROIは16 pxタイル
単位の空間集計を使うため、LEDより十分小さく、背景を含めすぎない矩形にしてください。
選択ROIと同期方法は「結果を書き出す」のJSONにも保存されます。

`roi_data/`がある記録では「LED ROIを自動検出」を使用できます。開始側・終了側、RGB・
EVSをそれぞれ独立に、16 pxタイル上の32・48・64 px正方形窓で探索します。単純な輝度・
event量ではなく、既知の2.3秒周期、点灯・消灯の極性、複数周期での再現性を採点します。
検出した4つのROIは即座に同期再解析へ使われ、枠と`高・中・低`の信頼度が画像上へ表示
されます。「開始側」「終了側」で各候補の強い点灯時刻へ移動して目視確認できます。
誤っている候補だけ従来どおりドラッグで囲み直し、「選択ROIで再解析」を実行してください。
自動検出の対応エッジ数、coverage、候補間の差は結果JSONにも保存されます。

「既知LED周期を使って対応付ける」は既定で有効です。Arduinoの点滅コードから得られる
2.3秒周期と、周期内の100・300・1000 msのエッジ間隔系列をRGB/EVSへ別々に当てはめ、
同じ周期・同じ位相のエッジだけを対応付けます。周期許容誤差は既定35 msです。比較実験や
任意の点滅を使う場合はチェックを外すと、従来の同極性最近傍対応に戻せます。

## 全区間のDSEC型重畳動画

Docker内の解析データを開いてオフセットを推定した後、
「全区間の重畳動画を生成」を押すと、現在の推定値を`time_sync_led.yaml`へ保存し、
既定のEVS/RGB空間校正を適用したMP4をバックグラウンドで生成します。深度は使わず、
DSEC型のrotation-only射影で正極性eventを青、負極性eventを赤としてRGBへ重ねます。
完成後は同じ画面に比較動画が表示され、全区間をシークできます。

生成先は次のversion付きdirectoryです。再生成しても以前の結果は上書きしません。

```text
/workspaces/record/evs-popup-vN/analysis/scenario_overlay/<session>/rotation_only_<timestamp>/
```

`rgb_vs_overlay.mp4`（RGBと重畳の左右比較）、`overlay_polarity.mp4`（重畳のみ）、
`polarity_only.mp4`（eventのみ）の3本を生成します。この機能は元のMCAP/RAWへアクセス
するためDocker読込時だけ利用できます。Macへコピーした解析フォルダは目視解析には
使えますが、動画の再生成には使えません。新しく生成する動画は、QuickTimeとbrowserに
対応するH.264・yuv420p・fast-start MP4として保存されます。旧`mp4v`動画はDocker内で
`scripts/experiments/normalize_mp4_for_macos.sh VIDEO.mp4`により変換できます。

## 全シーケンスの無人処理

次の専用scriptは、校正記録を除く`evs-popup-v1`と`evs-popup-v2`の全記録について、
空間タイルの不足時だけ前処理し、LED ROI自動検出、時刻同期、DSEC型全区間動画生成を
順番に実行します。最後に開始・終了それぞれのRGB/EVS ROIデバッグ画像と、ROI指標・
重畳動画を1画面で確認できるportable HTML reviewも生成します。

```bash
cd /workspaces
scripts/experiments/run_rc_popout_auto_pipeline.sh
```

完了済みの段階はskipするため、そのまま再実行して途中から継続できます。動画を作らず
ROI・同期だけ先に評価する場合は`--no-video`、1記録だけ試す場合は`--session DIR`を
使用します。既存の自動結果を更新する場合は`--force-sync`、以前の動画をbackupして
作り直す場合は`--force-video`です。

全体一覧は次に保存されます。

```text
/workspaces/record/evs-popup-analysis/auto_pipeline/summary.tsv
```

各記録の目視確認ページは次に保存され、`summary.tsv`の`review_page`列からも参照できます。

```text
/workspaces/record/evs-popup-vN/analysis/led_sync/<session>/review/index.html
```

同じdirectoryには`start_roi_debug.jpg`、`end_roi_debug.jpg`、センサ別ROI画像、
`review.json`も保存されます。HTMLと動画をMacへ持ち出す場合は、相対リンクを維持するため
対象の`analysis` directoryをまとめてコピーしてください。EVSのROI画像には、対応edgeの
前後120 msからROI内イベント量が最大のpreviewを選ぶため、瞬間的な点滅eventを目視しやすく
しています。判定は時刻検出品質`Timing`と候補位置の一意性`Location`を分けて表示します。

`review=yes`または信頼度`medium/low`だけを優先して目視確認します。各解析フォルダの
`auto_led_sync_result.json`はUIが自動的に読み込み、開始・終了の候補枠、信頼度、同期値を
復元します。動画は`analysis/scenario_overlay/<session>/rotation_only_auto/`に保存されます。
