# rosbag replay safety

JetPilot の bag にはセンサーだけでなく、`/vehicle/control_cmd`、各制御入力、
operation mode の要求と状態も含まれます。そのため、同じ ROS graph 上で車両
interface と bag replay を同時に起動すると、過去の指令が実車へ届く可能性が
あります。

## Safe replay default

通常 replay では、録画された入力・制御・mode・低レベルactuator topicを
`/replay/...` へ強制的にremapします。対象には `/joy`、`/rc/channels`、各
`control_cmd`、`/vehicle/control_cmd`、operation mode、`/bag/request`、motor/servo
command、driverのcalibration commandが含まれます。launch引数で変更したvehicle、RC、
propoの入力topicも追加で隔離します。同時にlive Joy、teleop、RC serial、operation mux、
autonomous control nodeも起動しません。sensor、TF、localization、map toolは通常どおり
使用できます。録画済みの `/bag/status` に加え、`/initialpose`、localization trigger、
VGL pose、VSLAM pose hint、`/diagnostics`、localization statusもlive localizationへ
混入しないよう隔離します。これらを変更する場合は、managerのparameter fileだけでなく
対応する`bringup.launch.py`のlaunch引数も変更すると、replay隔離にも同じtopic名が反映
されます。

safe replay中は、この隔離を上書きできないよう `replay_additional_args` の
`--remap` / `-m` を拒否します。

## Vehicle guard

`bringup.launch.py` は、次の条件がすべて成立した場合に node を一つも起動せず
エラーで終了します。

- `enable_rosbag_replay:=true`
- `rosbag` が空でない
- `enable_vehicle:=true`
- `allow_unsafe_replay_with_vehicle:=false`（既定）

通常のオフライン再生では、車両 interface を必ず無効にします。

```bash
ros2 launch jetpilot_system_launch bringup.launch.py \
  enable_rosbag_replay:=true \
  rosbag:=/workspaces/record/session/take \
  enable_vehicle:=false \
  use_sim_time:=true
```

## Hardware-in-the-loop

意図的な HIL 試験だけ、隔離した ROS domain と物理的な安全措置を用意した上で
guard を解除できます。録画された制御topicも元の名前で再生する場合は、二つの
unsafe flagが必要です。

```bash
ROS_DOMAIN_ID=82 ros2 launch jetpilot_system_launch bringup.launch.py \
  enable_rosbag_replay:=true \
  rosbag:=/workspaces/record/session/take \
  enable_vehicle:=true \
  allow_unsafe_replay_control_topics:=true \
  allow_unsafe_replay_with_vehicle:=true
```

これらの override を通常の launch preset や script の既定値に入れてはいけません。
guard が確認できるのは同じ `bringup.launch.py` が起動する vehicle interface
だけです。別 terminal、別 container、別 launch のnodeまでは停止できないため、
オフライン解析は実車 graph と異なる `ROS_DOMAIN_ID` で行うのが原則です。
