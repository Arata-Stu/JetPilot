# jetpilot_operation

JetPilot の operation mode と command mux を担当する package です。自律、手動、プロポ直結の各指令を直接 vehicle driver に流さず、mode と timeout を見て1本の `/vehicle/control_cmd` に絞ります。

## Nodes

| Node | 役割 |
| --- | --- |
| `operation_mode_manager_node` | `/operation_mode/request` を受け、現在 mode を `/operation_mode/state` として reliable + transient-local で配信する |
| `command_mux_node` | mode に応じて `/auto/control_cmd`、`/teleop/control_cmd`、`/propo/control_cmd` から1つを選び `/vehicle/control_cmd` へ publish する |

## Topic契約

| 方向 | Topic | 型 | 用途 |
| --- | --- | --- | --- |
| input | `/operation_mode/request` | `jetpilot_msgs/msg/OperationModeRequest` | AUTO/MANUAL/STOP/PROPO の切替要求 |
| output | `/operation_mode/state` | `jetpilot_msgs/msg/OperationModeState` | 現在 mode。遅れて起動した node も受け取れる latched state |
| input | `/auto/control_cmd` | `jetpilot_msgs/msg/ControlCommand` | controller 由来の自律指令 |
| input | `/teleop/control_cmd` | `jetpilot_msgs/msg/ControlCommand` | joystick 由来の手動指令 |
| input | `/propo/control_cmd` | `jetpilot_msgs/msg/ControlCommand` | RC receiver 由来のプロポ指令 |
| output | `/vehicle/control_cmd` | `jetpilot_msgs/msg/ControlCommand` | vehicle interface へ渡す選択済み指令 |

control command 系 topic は `KeepLast(1)` の best-effort QoS です。最新値だけを使い、古い指令を queue しません。operation state は reliable + transient-local で、起動順に依存しないようにしています。

## Mux algorithm

`command_mux_node` は周期的に次の優先で出力を決めます。

1. mode が `AUTO` で、`/auto/control_cmd` が `command_timeout_s` 以内なら採用
2. mode が `MANUAL` で、`/teleop/control_cmd` が `command_timeout_s` 以内なら採用
3. `control_authority=jetson_mux` かつ mode が `PROPO` で、`/propo/control_cmd` が fresh なら採用
4. それ以外は steering/throttle/brake/reverse をすべて0にした停止指令

標準設定では `control_authority=hardware_mux` です。この場合 `PROPO` は Jetson の mux では出力せず、実車側の hardware mux に権限を残す想定です。`command_timeout_s` は通信途絶時の最終防衛線であり、vehicle driver 側にも独立した timeout を置きます。

## 起動

```bash
ros2 launch jetpilot_operation jetpilot_operation.launch.xml
```

通常は `jetpilot_system_launch` から起動します。初期 mode は `initial_mode` parameter で指定し、既定値は `STOP` です。
