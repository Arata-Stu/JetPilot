# jetpilot_planning_manager

通常経路・矢印信号による分岐・衝突復帰を統括し、controller が購読する `/planning/*` を一箇所から配信する。

優先順位は `復帰 > 信号待ち/確定経路 > 通常経路`。信号が安定確定するまで `ready=false` とし、車両を停止させる。分岐 lane は、HD mapで必須指定したrelease sectionへ入るまで固定する。

競技用の `route_lane_selector_node` は、Mapごとの `competition_route.param.yaml` で
出力を `/planning/route/*` へ分離する。同梱の
`config/route_lane_selector.competition.param.yaml` を雛形として使う。
manager は selector が1周期で同じtimestampを付けた Path、typed profile、Diagnostics の3点を
同一世代として受け取った場合だけ採用する。信号確定・Junction定義変更・release時には旧世代を
即座に破棄し、選択laneが要求laneと一致する新世代が揃うまで `ready=false` を維持する。
同一内容のJunction周期再配信は無視し、位置・Section・信号ID・分岐先の実変更または削除だけを
新revisionとして扱う。

競技用planning一式は次で起動できる。

```bash
ros2 launch jetpilot_planning_manager competition_planning.launch.xml \
  route_config_file:=/workspaces/map/course_a/competition_route.param.yaml
```

Map Builder の Review は、各 Map 直下の同名ファイルを読み、Junction の
Left/Straight/Right ID が `lane_ids` に登録済みかを表示する。HD map 内に
経路が存在するだけでは実走行可能とは判定しない。さらに、managerへ接続する
`/planning/route/*` とdiagnostics、heartbeat、watchdogの設定も検証する。

信号YOLOは bringup へ以下を渡す。

```bash
enable_object_detection:=true \
object_detection_decoder_param_file:=/workspaces/ros2_ws/install/jetpilot_object_detection/share/jetpilot_object_detection/config/yolov8_signal.param.yaml \
object_detection_detections_topic:=/perception/signal/detections
```
