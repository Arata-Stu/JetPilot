# vslam_map_tools

## VSLAM reference snapshot

`record_vslam_reference_snapshot.py` records VSLAM path, odometry, and optional
landmarks into a JSON snapshot. It periodically replaces the output atomically
and performs a final write during shutdown.

```bash
ros2 run vslam_map_tools record_vslam_reference_snapshot.py \
  --path-topic /visual_slam/tracking/slam_path \
  --odom-topic /visual_slam/tracking/odometry \
  --landmarks-topic /visual_slam/vis/landmarks_cloud \
  --output /workspaces/map/course_a/vslam_reference_snapshot.json \
  --write-interval-sec 5.0
```

Convert the snapshot into an HD map editor background:

```bash
ros2 run vslam_map_tools export_aligned_landmarks_offline.py \
  --snapshot /workspaces/map/course_a/vslam_reference_snapshot.json \
  --output-image /workspaces/map/course_a/vslam_landmarks.png \
  --output-yaml /workspaces/map/course_a/vslam_landmarks.yaml \
  --require-landmarks
```
