# jetpilot_localization_manager

`jetpilot_localization_manager` coordinates saved-map localization between Isaac ROS Visual
Global Localization (VGL), Isaac ROS Visual SLAM (VSLAM), RViz, joystick input, and the web UI.

## Flow

1. A request comes from `autostart`, `/localization/trigger`, or
   `/localization/relocalize`.
2. The manager waits for the VGL Trigger service and, by default, both a subscriber on the VSLAM
   pose-hint topic and the first VSLAM localization diagnostic. These readiness checks prevent
   one-shot startup messages from being lost or sent before VSLAM initialization.
3. The manager calls VGL and validates the resulting pose.
4. The pose is held until VSLAM subscribes, then forwarded to `/localization/pose_hint`.
5. If VSLAM publishes `/visual_slam/trigger_hint`, VGL is retried with bounded backoff.
6. After timeouts or exhausted attempts, the manager waits for a valid `/initialpose` message.

When `vslam_save_map_folder_path` is set, the system launch treats the run as mapping: it does not
load an existing cuVSLAM map and does not start this manager. With a saved cuVSLAM map but no VGL
map, autostart instead enters the manual `/initialpose` fallback immediately.

Timeout and backoff deadlines use a steady clock, so they do not stall while simulated time is
waiting for `/clock`.

## External API

| Type | Name | Message/service | Purpose |
|---|---|---|---|
| Subscribe | `/localization/trigger` | `std_msgs/msg/Bool` | A `true` value starts/restarts localization (joystick). |
| Service | `/localization/relocalize` | `std_srvs/srv/Trigger` | Starts/restarts localization (web UI/CLI). |
| Subscribe | `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Manual fallback. |
| Publish | `/localization/pose_hint` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Validated hint for VSLAM. |
| Publish | `/localization/pose_hint_required` | `std_msgs/msg/Bool` | Latched UI indicator. |
| Publish | `/localization/pose_hint_state` | `std_msgs/msg/String` | Latched JSON state for UI/diagnostics. |

All names are parameters. Defaults remain compatible with the previous Python
`localization_manager` implementation.

Example request:

```bash
ros2 service call /localization/relocalize std_srvs/srv/Trigger '{}'
```

## Status semantics

Publishing a pose hint is **not** reported as successful localization. The manager enters
`awaiting_vslam`. It may enter `localized` when VSLAM diagnostics provide a conservative
`localized_in_exist_map` `No -> Yes` transition. cuVSLAM can retain an earlier `Yes` while a new
request is running, so an isolated `Yes` is deliberately not accepted as proof of the new request.
VSLAM's `trigger_hint` remains the authoritative retry signal.

An `awaiting_vslam` deadline prevents the UI from hanging forever. If VSLAM still reports `Yes`
without a request-specific `No -> Yes` transition, the state becomes `localized_unconfirmed` rather
than claiming a confirmed success. A missing/negative result retries VGL or falls back to manual
pose input. A late valid VGL pose is still accepted while manual fallback is waiting, because the
upstream VGL service has no cancellation API.

The JSON status contains `state`, `pose_hint_required`, request source, readiness, attempt counts,
last hint source, and the latest reason. The status publishers use reliable transient-local QoS.

## Build and test

```bash
colcon build --packages-select jetpilot_localization_manager
colcon test --packages-select jetpilot_localization_manager
colcon test-result --verbose
```

The unit tests cover frame, finite-number, quaternion, covariance, and timestamp pose validation.

## Topic handling notes

`/localization/pose_hint_required` と `/localization/pose_hint_state` は reliable transient-local QoS です。UI、controller、後から起動した diagnostic node が最新状態をすぐ読めるようにしています。

`/localization/trigger` は joystick 由来の edge trigger、`/localization/relocalize` は UI/CLI 由来の service trigger として扱います。どちらも同じ state machine に入り、進行中の request がある場合は新しい request として再開します。
