# jetpilot_operation

## Purpose

JetPilot の operation mode と command mux を担当する package です。自律、手動、プロポ直結の各指令を直接 vehicle driver に流さず、mode と timeout を見て1本の `/vehicle/control_cmd` に絞ります。

## Nodes

| Node | 役割 |
| --- | --- |
| `operation_mode_manager_node` | `/operation_mode/request` を受け、現在 mode を `/operation_mode/state` として reliable + transient-local で配信する |
| `command_mux_node` | mode に応じて `/auto/control_cmd`、`/teleop/control_cmd`、`/propo/control_cmd` から1つを選び `/vehicle/control_cmd` へ publish する |

## Inputs / Outputs

### Input topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `operation_mode_manager_node` | `/operation_mode/request` | `jetpilot_msgs/msg/OperationModeRequest` | Reliable / Volatile | AUTO/MANUAL/STOP/PROPOの切替要求 |
| `command_mux_node` | `/operation_mode/state` | `jetpilot_msgs/msg/OperationModeState` | Reliable / Transient Local | 現在mode |
| `command_mux_node` | `/auto/control_cmd` | `jetpilot_msgs/msg/ControlCommand` | Best Effort / Volatile | controllerまたはE2Eの自律指令 |
| `command_mux_node` | `/teleop/control_cmd` | `jetpilot_msgs/msg/ControlCommand` | Best Effort / Volatile | joystick手動指令 |
| `command_mux_node` | `/propo/control_cmd` | `jetpilot_msgs/msg/ControlCommand` | Best Effort / Volatile | RC receiver由来の指令 |

### Output topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `operation_mode_manager_node` | `/operation_mode/state` | `jetpilot_msgs/msg/OperationModeState` | Reliable / Transient Local | 現在modeと変更元 |
| `command_mux_node` | `/vehicle/control_cmd` | `jetpilot_msgs/msg/ControlCommand` | Best Effort / Volatile | vehicle interfaceへ渡す選択済み指令 |

## Parameters

| Node | Main parameters | Configuration |
| --- | --- | --- |
| `operation_mode_manager_node` | `initial_mode` | [`config/operation.param.yaml`](config/operation.param.yaml) |
| `command_mux_node` | `publish_rate_hz`, `command_timeout_s`, `control_authority` | [`config/operation.param.yaml`](config/operation.param.yaml) |

## Assumptions / Known limits

- command muxは1つだけ起動します。複数起動すると`/vehicle/control_cmd`が競合します。
- `hardware_mux`構成のPROPOではJetson側から指令を出さず、hardware safety層へ制御権を残します。

control command 系 topic は `KeepLast(1)` の best-effort QoS です。最新値だけを使い、古い指令を queue しません。operation state は reliable + transient-local で、起動順に依存しないようにしています。

## Mux algorithm

`command_mux_node` は周期的に次の優先で出力を決めます。

1. mode が `AUTO` で、`/auto/control_cmd` が `command_timeout_s` 以内なら採用
2. mode が `MANUAL` で、`/teleop/control_cmd` が `command_timeout_s` 以内なら採用
3. `control_authority=jetson_mux` かつ mode が `PROPO` で、`/propo/control_cmd` が fresh なら採用
4. それ以外は steering/throttle/brake/reverse をすべて0にした停止指令

標準設定では `control_authority=hardware_mux` です。この場合 `PROPO` は Jetson の mux では出力せず、実車側の hardware mux に権限を残す想定です。`command_timeout_s` は通信途絶時の最終防衛線であり、vehicle driver 側にも独立した timeout を置きます。

## How to launch

```bash
ros2 launch jetpilot_operation jetpilot_operation.launch.xml
```

通常は `jetpilot_system_launch` から起動します。初期 mode は `initial_mode` parameter で指定し、既定値は `STOP` です。
