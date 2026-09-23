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

serverが読み出せるのは、既定では`/workspaces/record`配下の解析JSONと、そのJSONが
参照するpreview画像だけです。

## v1/v2の一括export

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
同じ点灯・消灯エッジが一致しているかを目視確認でき、別途MP4を作る必要はありません。
「同期−1コマ」「同期＋1コマ」はEVSプレビューを1コマ移動し、推定したオフセット・
ドリフトを適用したRGBフレームも同時に更新します。各画像下の±1はセンサごとの独立した
コマ送りなので、手動同期の微調整に使用できます。

「開始側」または「終了側」を選び、両方の画像上でLEDをドラッグして囲んだ後、
「選択ROIで再解析」を押すと、選んだ4つのROIから信号を作り直します。ROIは16 pxタイル
単位の空間集計を使うため、LEDより十分小さく、背景を含めすぎない矩形にしてください。
選択ROIと同期方法は「結果を書き出す」のJSONにも保存されます。

「既知LED周期を使って対応付ける」は既定で有効です。Arduinoの点滅コードから得られる
2.3秒周期と、周期内の100・300・1000 msのエッジ間隔系列をRGB/EVSへ別々に当てはめ、
同じ周期・同じ位相のエッジだけを対応付けます。周期許容誤差は既定35 msです。比較実験や
任意の点滅を使う場合はチェックを外すと、従来の同極性最近傍対応に戻せます。
