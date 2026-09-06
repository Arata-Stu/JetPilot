# 相手車両のMap投影とFoxglove表示

## 今回の範囲

追跡ID付きYOLO検出のbox下端中央をCameraInfoで逆投影し、撮影時刻のTFを使って
路面との交点をMap座標へ変換します。相手の位置・ID別の軌跡をROS topicとして公開します。
位置は接地点の近似であり、車両中心/車軸位置や相手自身のodometryではありません。
速度・姿勢・ReID・追い越し制御は出力しません。可視化用で、plannerへは接続していません。

路面は撮影時刻の `base_footprint` を優先し、取得できなければ `tt02_ground_estimate` を使用します。
どちらもない場合は投影しません。Map Zを0と決めつけず、路面の位置・法線をTFから求めます。
カメラTFも最新値ではなく撮影時刻のものを使い、到着待ちは最大0.75秒です。
現行TT-02設定なら、55 mm支柱と仮のデッキ地上高25 mmを含む推定路面を利用できます。

## 起動

対象JetsonのROS/Isaac ROS環境でpackageを再ビルドしてください。
TensorRT engineの再生成は不要です。MacでのROS実行は検証していません。

```sh
cd /workspaces/ros2_ws
colcon build --packages-up-to jetpilot_object_detection jetpilot_system_launch
source install/setup.bash
```

モデルとMapを用意して、既存のlocalization起動へ可視化を追加します。
`course_a`とモデル名は実際のものに置き換えます。

```sh
scripts/bringup.sh localization --map /workspaces/map/course_a \
  --set enable_object_detection:=true \
  --set object_detection_model_root:=/workspaces/ros2_ws/models/yolov8/yolov8n_224_v1 \
  --set enable_opponent_projection:=true \
  --set enable_foxglove:=true
```

カメラ・YOLO・自己位置・車体TFを既に起動している場合、投影だけを起動できます。

```sh
ros2 launch jetpilot_object_detection opponent_projection.launch.py \
  detections_topic:=/perception/detections \
  camera_info_topic:=/realsense/color/camera_info \
  source_width:=424 source_height:=240
```

画像解像度はYOLOのsource寸法とCameraInfoのROI/binning後の寸法に一致させます。
Raw画像が既定です。補正済み画像で検出した場合は `image_geometry:=rectified`
（bringupでは `opponent_projection_image_geometry:=rectified`）を明示してください。
Rawはplumb_bob/rational_polynomialに対応し、その他の歪みモデルは拒否します。
rectifiedはR/Pを使いますが、補正ROIは未対応です。

## Foxglove

1. ROS Foxglove WebSocket接続で `ws://<JetsonのIP>:8767` を開く。
2. **3Dパネル**を追加し、Display frameを `map` にする。地理座標用のMapパネルではありません。
3. `/hd_map/lane_markers` と `/perception/opponents/markers` を表示する。
4. 自車も表示する場合は `/visual_slam/tracking/odometry` を追加する。
5. 個別のPathを選びたい場合は `/perception/opponents/track_0/path` などを有効にする。
   Markerにも軌跡が含まれるので、重複する場合はどちらかの線を非表示にする。

[Foxglove 3Dパネル](https://docs.foxglove.dev/docs/visualization/panels/3d)の標準ROS表示を使います。
配信許可には `/perception/opponents/` を追加済みです。独自のwhitelistを指定する環境では
このprefixも許可してください。

## 表示とtopic

| Topic | 型 | 内容 |
|---|---|---|
| `/perception/opponents/markers` | visualization_msgs/MarkerArray | 投影点、IDラベル、軌跡 |
| `/perception/opponents/track_0/path` ～ `track_7/path` | nav_msgs/Path | 各表示スロットの観測軌跡 |
| `/perception/opponents/status` | std_msgs/String（JSON） | 投影状態・拒否理由・slotと完全なtrack_idの対応 |

球は最新の推定接地点です。青色はbase_footprintの路面、橙色はTT-02推定路面を意味します。
両方とも位置自体は画像からの推定です。`Path`の姿勢は未推定で単位quaternionを入れているため、
矢印の向きとして解釈せず線・点で表示してください。

最後の観測から0.5秒経つと球を消し、軌跡・ラベルを薄くして `lost` を表示します。
保持は15秒・200点、最大8スロットです。スロットは空き/最古から再利用するため、
`track_0`自体は恒久的な個体IDではありません。完全な追跡IDはstatusに保存します。
消失後の再観測、投影点の急な飛び、路面の取得元変更では軌跡を切り、空白区間を補間しません。
時刻巻き戻し・カメラframe変更では履歴を破棄し、Map→odomの大きな補正も履歴切断の対象です。
markerには寿命を設定し、node停止後に現在位置が残り続けないようにしています。
Pathは履歴データとして残るため、停止中に現在位置として解釈しないでください。

初期設定では、bboxが画像端で切れる場合、追跡IDが未確定の場合、必要なCameraInfo/TFがない場合、
投影が水平線に近い場合、カメラから0.15～8 mの範囲外の場合は点を生成しません。
路面高さ・校正・bbox下端の誤差は遠方ほど位置へ強く影響します。近距離の既知位置から確認します。
IDが途中で入れ替われば軌跡も誤って結び付く可能性があり、ReIDによる恒久識別は未実装です。

## 記録と検証

標準bag設定にmarkers・status・既定8スロットのPathを追加しました。
記録済み表示はFoxgloveでMCAPを開いて確認できます。
旧bagにIDがない場合は追跡付き推論を再実行する必要があります。そこからMap投影を再計算するには、
同じ時刻のCameraInfo・TF・検出結果を投影nodeへ供給してください。
Bag Analysisの検出sidecarだけでは、投影に必要なTF一式は揃いません。

標準ライブラリのテスト:

```sh
python3 -S -m unittest discover \
  -s ros2_ws/src/perception/jetpilot_object_detection/test -p 'test_opponent*.py'
```

計算・履歴と、ROS message代替を使った出力処理を検証しています。
ROS実機ビルド、実カメラでの位置精度、Foxgloveへの実接続は未検証です。
