# jetpilot_msgs

JetPilot 内で共有する message 定義です。operation、teleop、controller、bag manager、vehicle interface が同じ型を使うことで、実機向け driver を差し替えても上位の topic 契約を保てるようにしています。

## Messages

| Message | 主な topic | 用途 |
| --- | --- | --- |
| `ControlCommand` | `/auto/control_cmd`, `/teleop/control_cmd`, `/propo/control_cmd`, `/vehicle/control_cmd`, `/control_cmd` | steering/throttle/brake/reverse を正規化値で運ぶ |
| `OperationModeRequest` | `/operation_mode/request` | joystick や UI から AUTO/MANUAL/STOP/PROPO を要求する |
| `OperationModeState` | `/operation_mode/state` | 現在の operation mode と変更元を latched state として配信する |
| `BagRequest` | `/bag/request` | rosbag record の START/STOP/SPLIT/MARK を要求する |
| `BagStatus` | `/bag/status` | record 中か、出力先 URI、最後の event を通知する |

## ControlCommand の扱い

`ControlCommand` は車両非依存の正規化指令です。`steering` は左/右を `-1.0` から `1.0` の範囲で表し、`throttle`、`brake`、`reverse` は `0.0` から `1.0` の範囲で表します。各 vehicle interface はこの値を servo PWM、VESC eRPM、brake current などの実機値へ変換します。

上位 node は原則として `header.stamp` を現在時刻に更新します。mux や driver は受信時刻で timeout を見るため、古い指令を再利用せず、停止時もゼロ指令を周期 publish します。

## Operation mode

mode は `AUTO`、`MANUAL`、`STOP`、`PROPO` の4種類です。`OperationModeRequest.source` には `joy_auto_hold` や `initial` のような発生元を入れ、状態表示と解析に使います。不正な mode 値は `jetpilot_operation` 側で無視されます。

## Bag request

`BagRequest.START` は `label` を session 名の一部として使います。`STOP` は現在の recording を終了します。`SPLIT` と `MARK` は将来拡張用の command として定義済みですが、現行の bag manager では `SPLIT` は未実装、`MARK` は event log として扱います。
