# Control QoS Notes

High-rate vehicle control topics should prefer latest-value delivery over
reliable delivery of older samples.

Use this profile for `jetpilot_msgs/msg/ControlCommand` publishers and
subscriptions:

```cpp
rclcpp::QoS(rclcpp::KeepLast(1)).best_effort()
```

Current control command topics:

| Topic | Role |
| --- | --- |
| `/auto/control_cmd` | Autonomous controller output |
| `/teleop/control_cmd` | Joystick teleop output |
| `/propo/control_cmd` | RC receiver serial output |
| `/vehicle/control_cmd` | Command mux output to the vehicle interface |

Keep mode-change topics reliable. `/operation_mode/state` should remain
`transient_local`, `reliable`, and `KeepLast(1)` so late-joining nodes receive
the current mode.
