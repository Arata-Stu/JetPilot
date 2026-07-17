# vslam_map_tools

## VSLAM reference snapshot

`record_vslam_reference_snapshot.py` records VSLAM path, odometry, and optional
landmarks into a JSON snapshot. It periodically replaces the output atomically
and performs a final write during shutdown.

Every received odometry message is retained in the top-level
`odometry_samples` array for offline drive analysis. Each sample contains the
original header timestamp as a nanosecond string, the recorder's simulated
`received_timestamp_ns`, `frame_id`, `child_frame_id`, `pose`, and `twist`.
A zero header timestamp is intentionally stored as `"0"` and consumers can
fall back to the received timestamp for synchronization;
the string representation prevents JavaScript clients from losing integer
precision. The legacy `full_vslam_path.frame_id` and
`full_vslam_path.poses` fields remain available for existing map tools.

```json
{
  "odometry_samples": [
    {
      "timestamp_ns": "1721185200123456789",
      "frame_id": "map",
      "child_frame_id": "base_link",
      "pose": {
        "position": {"x": 1.2, "y": 0.4, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.1, "w": 0.995}
      },
      "twist": {
        "linear": {"x": 2.1, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.2}
      }
    }
  ]
}
```

```bash
ros2 run vslam_map_tools record_vslam_reference_snapshot.py \
  --path-topic /visual_slam/tracking/slam_path \
  --odom-topic /visual_slam/tracking/odometry \
  --landmarks-topic /visual_slam/vis/landmarks_cloud \
  --output /workspaces/map/course_a/vslam_reference_snapshot.json \
  --write-interval-sec 5.0
```

Offline drive analysis adds `--require-localized-map true`. In that mode the
recorder listens to `/localization/pose_hint_state` and live `/tf`, rejects all
Odometry before the manager confirms `localized`, and stores the accepted
trajectory in `map` frame. If localization never confirms or `map→odom` is
missing, no valid snapshot is produced and the analysis job fails with an
actionable status instead of displaying an odom-frame path as Map-aligned.

Convert the snapshot into an HD map editor background:

```bash
ros2 run vslam_map_tools export_aligned_landmarks_offline.py \
  --snapshot /workspaces/map/course_a/vslam_reference_snapshot.json \
  --output-image /workspaces/map/course_a/vslam_landmarks.png \
  --output-yaml /workspaces/map/course_a/vslam_landmarks.yaml \
  --require-landmarks
```

`diagnose_vgl_bag.py` uses `/visual_slam/tracking/odometry` as its default
`--pose-topic`, matching the recorder and JetPilot launch configuration.
