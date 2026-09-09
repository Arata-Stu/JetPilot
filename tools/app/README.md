# JetPilot Console Design

JetPilot Console is a local browser app for managing the repeated, file-heavy
workflows around RC car autonomy. It is not the first home for live vehicle
runtime control; Jetson-side tmux launch and real-time operation can remain a
later phase. The first goal is to make the notebook-side utility workflows
visible, repeatable, cancellable, and easy to inspect.

## Current MVP

### 作業目的から始めるUI

上部メニューとホームは「走らせる」「地図を作る・直す」「走行を振り返る」
「モデルを育てる」の4つを入口にしています。各作業内のメニューから従来の画面を開けます。
タスク履歴・コマンド、ログ表示、キャッシュ消去は「補助ツール」にあります。
ホームには実行中の処理を簡潔に表示し、コマンドの詳細はログ・履歴から確認します。

E2Eモデルは「評価・走行結果」と「学習・配備」を切り替えます。
学習済みモデルの「Use in offline eval」は評価画面へ移動します。

地図を開いたら「実車調整へ」で自己位置・走行ライン・速度の確認に切り替えます。
実車調整中は編集操作を無効にし、生成・編集パネルを表示しません。
未保存の編集は保存してから切り替えてください。
形状・区間の編集は「接続を終了して地図編集へ」から戻ります。
生成・転送コマンドは地図編集の「生成・転送・その他」にまとめています。

The first implementation is intentionally dependency-light:

- Python standard-library HTTP backend.
- Static HTML/CSS/JS frontend.
- No npm install, no FastAPI install, and no build step required.

Start it from the repository root:

```bash
tools/app/scripts/start.sh --host 127.0.0.1 --port 8765
```

When running inside the JetPilot Docker environment, use the project-root mount:

```bash
/workspaces/tools/app/scripts/start.sh --host 127.0.0.1 --port 8765
```

The Docker launcher mounts the complete JetPilot project root at `/workspaces`,
so `tools` remains available at `/workspaces/tools` and future top-level
directories require no additional mount configuration.
The JetPilot container uses host networking, so the loopback bind is reachable
from the Linux host without exposing the Console to the LAN.

Then open:

```text
http://127.0.0.1:8765
```

## Local security defaults

The Console is an operator tool and does not provide user authentication. Its
safe defaults are therefore:

- listen on `127.0.0.1` only;
- reject cross-origin, non-JSON, and JSON bodies larger than 1 MiB for every
  POST endpoint;
- accept only literal loopback IPs or `localhost` in the Host header during
  loopback operation, preventing DNS-rebinding access;
- disable the arbitrary command endpoint and its browser form;
- save only the five generated Joy YAML filenames under
  `${ROS2_WS}/joy_profiles`, using atomic file replacement and refusing
  symlink outputs;
- constrain rosbag/map task paths to `RECORD_ROOT` or `MAP_ROOT` and validate
  SSH targets before starting transfers.

The normal map and transfer actions remain available. If arbitrary command
execution is deliberately needed on a trusted local machine, opt in for that
process only:

```bash
JETPILOT_CONSOLE_ENABLE_CUSTOM_COMMANDS=true \
  tools/app/scripts/start.sh --host 127.0.0.1 --port 8765
```

A non-loopback bind additionally requires `--allow-remote`, and cannot be
combined with custom command execution. It still exposes an unauthenticated
operator service and is not recommended; prefer Docker host networking plus
the loopback address.

Implemented in this MVP:

- Local rosbag scan from `RECORD_ROOT`.
- Rosbag analysis preprocessing under `ANALYSIS_ROOT` (defaults to
  `RECORD_ROOT/.jetpilot_analysis`).
- Synchronized drive viewer for image frames, operation mode, applied control,
  speed, and localized trajectory.
- Embedded notebook-side RTP viewer in the FPV tab. H.264 can be passed through
  GStreamer WebRTC without decoding or JPEG conversion for the low-latency
  path. The existing MJPEG browser relay remains available for H.265, MJPEG,
  raw RTP, and compatibility testing.
- Optional offline VGL/VSLAM replay when a recorded localization trajectory is
  unavailable, using an isolated configurable ROS domain.
- Analysis preflight checks for required topics, selected Map artifacts, VGL
  assets, and camera inputs, with visible job stage/progress and missing data.
- Local map scan from `MAP_ROOT`.
- Dashboard focused on status, recent assets, and running tasks.
- Dedicated Map Builder tab for VGL/VSLAM build entry.
- Clear Jetson tab split into connection, remote state, and transfers.
- Task runner with PID/process-group tracking.
- Task stop button using process-group termination.
- Per-task log files under `tools/app/.state/tasks/`.
- Live log terminal using server-sent events.
- Large console dialog for reading and copying task logs.
- Copy buttons for commands, logs, and paths.
- Jetson SSH inspection endpoint.
- `rsync` transfer tasks for Jetson to notebook and notebook to Jetson.
- Staged map workflow task entry points:
  - VGL/VSLAM build
  - HD raster preparation
  - browser HD map editing and save
  - raceline generation
  - preview generation
- Shared task preflight checks for the executable map stages. The Console shows
  required inputs, warnings, and concrete remediation before enabling an action,
  then repeats the same checks in the backend immediately before a task starts.
- Console map-generation tasks take an exclusive lock per map folder, so double
  clicks and overlapping Console writes cannot target one bundle concurrently.
- Offline localization analysis tasks take an exclusive lock on
  `JETPILOT_ANALYSIS_ROS_DOMAIN_ID` (default `92`) so two replay graphs cannot
  contaminate one another.
- Analysis also locks the selected Map folder, preventing a Console Map build
  or edit from changing the Map while the job is reading it.
- The Map Builder UI is tuned for the current VSLAM/VGL workflow. Occupancy-map
  specific controls such as FoundationStereo model resolution are intentionally
  hidden from the main form.

## E2E training and deployment pipeline

Open **E2E Analysis** to run the notebook-side model workflow before using the
existing offline evaluator:

1. Select a rosbag, image topic, and teacher-control topic, then create a named
   dataset under `python_ws/jetpilot_e2e_training/datasets`.
2. Select the dataset and tune the experiment, epochs, learning rates, batch
   size, validation split, data fraction, worker count, and device.
3. Export a completed run's `best.pt` checkpoint to `model.onnx` and
   `metadata.json`, then select it directly for Offline teacher comparison.
4. Transfer the ONNX model to a configured Jetson profile. Engine generation is
   enabled by default and runs `trtexec` remotely to create `model.plan` before
   the Isaac ROS TensorRT launch consumes it.

Every stage is a cancellable Console task with its command, log, output
artifacts, and an exclusive lock for the selected dataset/run/deployment target.
The UI deliberately targets the Isaac ROS TensorRT runtime pipeline; the
non-Isaac ROS PyTorch inference node remains a dependency-failure fallback and
is not exposed as a deployment choice.

## Object-detection training and deployment

Open **Object Detection** on the training Notebook to manage the 224x224 YOLOv8
workflow. The Console discovers Roboflow YOLOv8 `data.yaml` files under
`python_ws/jetpilot_object_detection_training/datasets`, checks the fixed
`vehicle, barrier` class contract, and exposes full label validation as a
cancellable task. New training, fine-tuning from a selected `best.pt`, and
resume from `last.pt` share the same run catalog and guarded GPU resources.

Completed runs expose the latest loss/mAP row, checkpoints, `model.onnx`, and
`metadata.json`. An exported run can be selected directly in **Bag Analysis**
for offline detector evaluation. Deployment transfers the selected export over
SSH and builds the FP16 TensorRT engine on the Jetson in a staging directory;
the previous model remains active if transfer or engine generation fails.
Connection profiles contain only user/host/path information. Passwords and
private-key contents are not stored in Console task state, so key-based SSH is
required.

## Goals

- Manage rosbag, map, HD map, raceline, preview, and Jetson transfer workflows
  from one GUI.
- Keep every long-running command visible as a task with PID/process-group,
  live logs, copy buttons, stop controls, status, and artifacts.
- Replace the monolithic `create_map.sh` experience with staged operations that
  can be run, retried, or skipped independently.
- Move the HD map editing workflow into a browser canvas UI and let centerline,
  raceline, and preview generation happen from the same workspace.
- Preserve shell commands as transparent implementation details: the GUI should
  show exactly what it ran and make it easy to copy.

## Rosbag drive analysis

Open **Rosbags → Analyze** or the **Bag Analysis** tab. Select the image and
telemetry topics, the Map used for the run, and a trajectory source:

- **Auto** uses recorded `/visual_slam/tracking/odometry` when present and
  otherwise runs offline VGL/VSLAM with the selected Map.
- **Recorded** never runs localization and requires a recorded odometry topic.
  When the bag contains `map→odom` on `/tf`, the preprocessor applies it and
  excludes odometry recorded before the Map transform became available.
- **Offline** replays the bag through the selected localization method. **Auto**
  first tries VGL + VSLAM and, if startup or confirmed localization fails,
  restarts the bag from the beginning with VGL disabled. **VGL + VSLAM** disables
  that fallback. **VSLAM only** loads `cuvslam_map/` and sends an identity
  `map` pose through the localization manager; the bag therefore needs to start
  near the saved Map origin. Auto also selects this path immediately when VGL
  map/model assets are unavailable. Recorded `/tf` is isolated from the new
  VSLAM graph. Replay starts paused, publishes `/clock`, and resumes only after
  the required nodes, snapshot recorder, and rosbag resume service are visible.
  Every method keeps the strict contract: the result is accepted only after the
  localization manager reports confirmed `localized` state. After replay ends,
  the graph gets a short drain interval before graceful shutdown. Live
  `map→odom` is applied before the snapshot is stored. The chosen/fallback method
  is retained in `localization/method.txt`, the timeline, manifest, and warnings.
- **None** produces image/command telemetry without a Map trajectory.

The preprocessing task writes `manifest.json`, `status.json`, `timeline.json`,
and a rate-limited JPEG sequence. The browser reads these normalized artifacts;
it does not repeatedly seek or decode the rosbag itself. Source nanosecond
timestamps are retained as decimal strings while playback uses relative
seconds, avoiding JavaScript integer precision loss.

The UI checks that the built Linux/Docker ROS workspace and analysis Python are
available before enabling Start. Corrupt or unsupported individual image frames
are skipped and reported. Long bags are automatically sampled to at most 50,000
frames so a completed timeline remains browser-readable.

The Map is pinned in the result with a fingerprint. Post-processing reports how
much of a `map`-frame trajectory falls inside its raster bounds. This is a
useful mismatch warning, not proof that the Map is correct. Results in another
frame remain viewable as a standalone trajectory but are not silently overlaid
on the Map.

## E2E bag analysis

Open **E2E Analysis** and choose one of three workflows:

- **Offline teacher comparison** extracts synchronized images, runs the selected
  `model.onnx`, aligns each prediction to the nearest manual control command,
  and reports steering/throttle MAE, RMSE, per-frame errors, preprocessing and
  inference latency, and deadline misses. `metadata.json` next to the model is
  used for input normalization, shape, and output field order.
- **Driven bag + offline localization** reads the recorded E2E command and
  replays the bag through VGL/VSLAM. Select an existing Map for map-relative
  evaluation, or choose scratch VSLAM. The resulting trajectory is projected to
  the HD Map centerline and summarized by section, lap, cross-track error,
  speed, latency, and control samples.
- **Recorded online E2E + VSLAM** uses pose and diagnostics already present in
  the bag. It avoids a second localization pass and is intended for runs where
  E2E inference and VSLAM were active together.

Every mode uses the same synchronized video viewer, play/seek/rate controls,
frame stepping, timeline cursor, and map overlay as Bag Analysis. E2E results
add GT/prediction/error/latency tracks, summary cards, worst-frame shortcuts,
and clickable section metrics. Supervised control error is only meaningful when
a teacher command exists; autonomous runs without a teacher are evaluated using
trajectory, smoothness, latency, deadline misses, and section context instead.

Teacher-free evaluation is calculated for every E2E result, including bags
without manual commands. It reports absolute steering and steering/throttle
rate, steering oscillations, control saturation, speed, longitudinal
acceleration, jerk, yaw rate, lateral acceleration, and cross-track error. A
0–100 **Aggressiveness Score** is the p95 weighted combination of those
available signals. Default normalization thresholds are 1.5 steering units/s,
1.5 throttle units/s, 2.5 m/s² longitudinal acceleration, 6.0 m/s³ jerk, and
3.0 m/s² lateral acceleration; missing signals are removed from the weight
normalization rather than treated as zero. The score is shown with its component
metrics and must not be interpreted as a standalone safety certification.

Samples scoring 60 or above are grouped into high-aggressiveness events. The UI
links each event to synchronized video, plots aggressiveness/acceleration/jerk
over time, summarizes the same metrics per Map section, and colors the Map
trajectory from calm to aggressive.

The E2E decoder publishes `/e2e/diagnostics` with capture-to-command latency,
decoder callback time, output interval, and deadline status. The default bag
configuration records this topic together with `/jetson/diagnostics`, allowing
GPU, CPU, memory, thermal, and power series to be inspected alongside the run.
Exported ONNX models are discovered under the repository/Python workspace
`outputs` folders. Additional roots can be added with
`JETPILOT_E2E_MODEL_ROOTS` (colon-separated on Linux).

Environment overrides:

```bash
ANALYSIS_ROOT=/workspaces/record/.jetpilot_analysis \
JETPILOT_ANALYSIS_ROS_DOMAIN_ID=92 \
tools/app/scripts/start.sh --host 127.0.0.1 --port 8765
```

## Live RTP in the FPV tab

The browser cannot open an RTP/UDP socket directly. **FPV → Start WebRTC**
therefore starts a fixed, validated GStreamer pipeline in the Console backend
on the notebook. For H.264, it depayloads and repayloads the encoded stream into
WebRTC without decoding, resizing, or JPEG conversion. This removes the main
extra buffering and image-conversion stages of the compatibility relay. The
**MJPEG** browser transport still decodes `h264`, `h265`, `mjpeg`, or `raw` RTP,
keeps only the newest frame, limits it to 1280x720 at 30 FPS, and serves it to
the page. Neither path requires enabling arbitrary custom commands.

Set the codec, source width/height, FPS, UDP port, and payload to match the
Jetson sender, then select the notebook IP used by the Jetson command. For
MJPEG, RTP payload type 26 is selected automatically. Keep the external viewer
stopped while the embedded viewer owns the same UDP port.

The receiver stops when you leave the FPV tab, close the page, press **Stop**,
or the browser heartbeat disappears. Missing GStreamer elements and UDP port
conflicts are shown in the FPV status panel. The Console host/container must
provide the GStreamer tools and plugins listed in
`tools/rtp_video_experiment/README.md`; the repository's `additional_setting`
Docker layer installs most of those plugin sets. WebRTC additionally requires
the distro packages `gstreamer1.0-nice` and
`gir1.2-gst-plugins-bad-1.0` and GStreamer 1.20 or newer. Start the Console on
loopback as documented above; the unauthenticated remote-bind mode intentionally
disables WebRTC signaling.

When the Console runs in Linux Docker, use host networking and keep the Console
bound to `127.0.0.1`. This first local-only WebRTC path uses host ICE candidates
without STUN/TURN; Docker bridge/NAT mode is not supported yet.

The first WebRTC implementation supports H.264 only. If a browser or sender
profile is incompatible, select **MJPEG (compatibility)** and compare behavior.
The page reports browser-side decoded frames, received/lost packets, and RTP
stall state so WebRTC and the external native viewer can be measured side by
side.

## Non-Goals For MVP

- Full Jetson tmux runtime orchestration.
- Replacing ROS launch files.
- Running live autonomy from the browser.
- Cloud deployment or multi-user collaboration.

## Proposed Repository Layout

```text
tools/app/
  README.md
  backend/
    jetpilot_console/
      __init__.py
      main.py
      config.py
      tasks.py
      process_runner.py
      rosbag_index.py
      map_index.py
      jetson.py
      map_pipeline.py
      hd_map_pipeline.py
      schemas.py
    tests/
  frontend/
    package.json
    index.html
    src/
      App.tsx
      api.ts
      components/
      pages/
      terminal/
      hd-map-editor/
  scripts/
    dev.sh
    start.sh
```

The backend should be Python/FastAPI so it can reuse existing Python map tools,
launch existing shell commands, and run naturally inside the current workspace
or container. The frontend should be TypeScript, React, and canvas-based for the
editor.

## Main Screens

### Dashboard

- High-level counts for running tasks, rosbags, and runtime-ready maps.
- Running task list with status, elapsed time, command, stop button, and log
  button.
- Recent rosbags and recent maps.
- Warnings for incomplete map bundles.

### Rosbags

- Notebook rosbag list scanned from `RECORD_ROOT`.
- Jetson rosbag list scanned over SSH.
- Topic summary, size, modified time, metadata path, and copy buttons.
- Transfer action: Jetson to notebook.

### Map Builder

- Select a rosbag, output base directory, and map name. The base directory is
  prefilled from `MAP_ROOT`.
- Predict the camera topic config from `ros2_ws/src/launch/jetpilot_system_launch/config/localization/`
  while still allowing a manual override inside that directory. Model and local
  input/output paths are likewise restricted to their configured workspace roots.
- Run VGL/VSLAM map build as a task.
- Show generated artifacts:
  - `cuvgl_map/`
  - `cuvslam_map/`
  - `vslam_reference_snapshot.json`
  - `vslam_landmarks.yaml`
  - `vslam_landmarks.png`
- Provide retry and open-log actions for each stage.

### HD Map Workspace

- Browser canvas editor for the landmark raster.
- Four focused modes keep the inspector compact:
  - Geometry for physical bounds and centerlines
  - Topology for Section Gates and Junctions
  - Driving Lines for Raceline and named Custom Lines
  - Review for readiness, versions, and final overlays
- Current editor scope:
  - preserve every lane while selecting one lane to edit
  - left/right bound point add, move, and delete
  - closed/open loop toggle
  - fit/zoom controls with scroll-based pan
  - centerline generation from bounds
  - save `<map_name>_hd_map.yaml`
  - save `<map_name>_hd_map_centerline.csv`
- Layers:
  - landmark raster
  - left bound
  - right bound
  - generated centerline
  - generated raceline
  - every named custom line, with the selected line colored by point speed
  - a `START` tangent arrow on the active centerline/raceline/custom line
  - section gates
  - junction markers; only the selected junction shows activation/release gates and Left/Straight/Right branch arrows
- Right-side inspector:
  - closed/open loop
  - artifact status
  - lane and section summaries
  - Junction ID, Signal ID, map position, required activation/release Sections, and three branch routes
- Junction route choices are validated against map lanes, Raceline, and named
  Custom Lines. Invalid or stale Custom Lines cannot be saved as branches.
  Review separately checks those authoring IDs against the optional map-local
  `competition_route.param.yaml`, including the manager-facing `/planning/route/*`
  topics, diagnostics, heartbeat, and watchdog settings. A route can exist in
  the map while still being clearly marked as not registered for driving.
- Review can generate and atomically save `competition_route.param.yaml` from
  the Junction branch IDs. Primary, Raceline, and the first Custom Line receive
  known topic defaults; routes without a runtime publisher remain incomplete
  until the operator assigns the topic that is actually published.
- Layer switches are grouped under an Advanced disclosure, Simulation is closed
  by default, and map-list readiness details are collapsed until requested.
- The workspace stacks the canvas and inspector on narrower windows and keeps
  the Map navigation usable on phone-sized viewports. Canvas labels, junction
  diamonds, and direction arrows keep a readable on-screen size when scaled.
- Unsaved Geometry, Section, Junction, and Driving Line edits survive mode
  switches. Reload, map changes, version activation, and browser close warn
  before discarding them; editor and canvas scroll positions are preserved.
- Actions:
  - prepare landmark raster
  - edit and save HD map YAML/centerline CSV
  - generate raceline with selectable forward/reverse lap direction, vehicle width, and per-side boundary margin
  - clone Centerline or Raceline into multiple named Custom Lines
  - edit Custom Line points, open/closed state, and per-section target speeds in the browser
  - select one validated Custom Line as the default for the next drive/transfer
  - generate line preview
  - copy paths and commands

### Maps

- Notebook map bundle list scanned from `MAP_ROOT`.
- Bundle completeness indicators:
  - `cuvgl_map/`
  - `cuvslam_map/`
  - `<map_name>_hd_map.yaml`
  - `<map_name>_hd_map_centerline.csv`
  - `<map_name>_raceline.csv`
  - `<map_name>_custom_line.csv` plus matching `.meta.json` when a Custom Line is selected
  - `<map_name>_line_preview.png`
- Transfer action: notebook to Jetson.
- Jetson map root browser with size, modified time, and `latest` symlink.

### Jetson

- Connection target: host, user, remote map root, remote rosbag root.
- Remote state: SSH result, latest map, map count, rosbag count, disk output.
- Pull one rosbag sequence from Jetson to notebook. The sequence picker is built
  from remote `metadata.yaml` discovery so the entire record root does not need
  to be transferred.
- Push map bundle: notebook to Jetson transfer.

### Terminal Panel

Use a VS Code-style bottom panel as the default terminal/log surface. Also allow
opening a task log in a modal/drawer from any `View Log` button.

Required terminal features:

- Live log streaming.
- Auto-scroll only while the log view is already at the bottom, so older logs can
  be inspected without being pulled back to the newest line.
- Per-task tabs.
- Copy command button.
- Copy task log button.
- Copy visible log button.
- Copy full log button.
- Download log button.
- Stop task button.
- Show PID and process group.

## Task Model

Every long-running operation is represented as a task:

```json
{
  "task_id": "20260710-153000-build-map-a1b2",
  "kind": "map_build",
  "title": "Build VGL/VSLAM map",
  "command": ["scripts/create_map.sh", "--stage", "build-map"],
  "cwd": "/workspaces",
  "status": "running",
  "pid": 12345,
  "pgid": 12345,
  "started_at": "2026-07-10T15:30:00+09:00",
  "ended_at": null,
  "exit_code": null,
  "log_path": ".../tasks/20260710-153000-build-map-a1b2/output.log",
  "artifacts": []
}
```

The backend must start commands in a new process group. Stop handling should
target the process group, not only the parent PID:

1. Send `SIGTERM` to the process group.
2. Wait a short grace period.
3. Send `SIGKILL` if anything remains.
4. Mark the task as `stopped`.

Task state should be persisted so a browser refresh does not lose task history.
Default state directory:

```text
${JETPILOT_CONSOLE_STATE_DIR:-${XDG_STATE_HOME:-~/.local/state}/jetpilot-console}
```

## Backend API Sketch

```text
GET  /api/health
GET  /api/config

GET  /api/tasks
GET  /api/tasks/{task_id}
POST /api/tasks/{task_id}/stop
GET  /api/tasks/{task_id}/log?tail=400
SSE  /api/tasks/{task_id}/stream

GET  /api/rosbags/local
POST /api/transfers/jetson-to-local
POST /api/jetson/inspect

GET  /api/maps/local
GET  /api/maps/detail?path=/workspaces/map/course_a
GET  /api/map-builder/camera-topic-configs
POST /api/preflight
POST /api/maps/build-vgl-vslam
POST /api/maps/prepare-hd-raster
POST /api/maps/save-hd-map
POST /api/maps/save-section-gates
POST /api/maps/save-junctions
POST /api/maps/generate-raceline
POST /api/maps/custom-lines/create
POST /api/maps/custom-lines/update
POST /api/maps/custom-lines/activate
POST /api/maps/custom-lines/delete
POST /api/maps/generate-preview
POST /api/transfers/local-to-jetson
```

`POST /api/preflight` accepts the same task fields plus an `action` selected
from `map-build`, `prepare-hd-raster`, `generate-raceline`, or
`generate-preview`. A valid inspection always returns `200`, including when the
result is blocked. The response includes an overall `ready` value and individual
`pass`, `warning`, or `blocked` checks with an operator-facing remediation.

The execution endpoints do not trust the browser result. They inspect the
current files again and return `409` with the fresh preflight report when a
required input has disappeared or changed. Warnings remain executable; blocked
checks never start a task. Unsafe symlinked map inputs/outputs are blocked, and
an overlapping writer for the same map folder also returns `409` with the
active task details.

`POST /api/maps/generate-raceline` accepts the following JSON body. Both width
values are metres and must be finite and non-negative:

```json
{
  "map_dir": "/workspaces/map/course_a",
  "direction": "reverse",
  "vehicle_width_m": 0.25,
  "safety_margin_m": 0.05
}
```

`direction` accepts `forward` or `reverse` and defaults to `forward`.
The effective optimizer envelope is `vehicle_width_m + 2 * safety_margin_m`.
The API defaults preserve the existing `0.25 m` vehicle and `0.05 m` per-side
margin behavior. Generation keeps the existing raceline CSV layout and writes
the selected values to `<map_name>_raceline.meta.json` for reproducibility.

Named Custom Lines live under `custom_lines/<line_id>/`. `custom_line.json` is
the editable source and `trajectory.csv` is its compiled seven-column runtime
artifact. A line can be cloned from the current Centerline or Raceline, renamed,
and edited without changing either source. Speed is authored as one Whole-line
target plus optional overrides keyed by the primary lane's Section IDs; it is
not entered waypoint by waypoint. Section gates are intersected with the edited
line so a changed line length does not move the speed boundaries. The compiler
inserts boundary samples and recalculates station, heading, curvature, speed,
and acceleration. Section targets are upper limits: curvature and forward/backward
acceleration passes can lower the compiled speed. Geometry outside the primary
lane, ambiguous gate intersections, unknown/duplicate Section IDs, non-positive
targets, and unsafe profiles are rejected.

Activation writes `<map_name>_custom_line.csv` and SHA-256-bound metadata for
both the trajectory and the HD map/Section layout. Editing the HD map or gates
recompiles the active line; if the new layout cannot be mapped unambiguously,
the stale canonical pair is removed instead of remaining drive-ready. That
canonical pair is the next-drive/transfer default; activating it in the notebook
UI does not hot-switch a vehicle process already running. A
Custom-Line-only runtime bundle is complete when both canonical files exist;
a generated Raceline remains the alternative driving-line artifact.

## Pipeline Stages

The GUI should not treat map creation as one giant script. Split it into
restartable stages:

```text
1. Discover/transfer rosbag
2. Build VGL map and compute VSLAM map/snapshot
3. Prepare landmark raster for editing
4. Edit HD map in browser
5. Define Sections, then save HD map YAML and centerline CSV
6. Generate raceline and/or create named Custom Lines
7. Select the driving-line default and generate a preview image
8. Transfer the complete bundle to Jetson
```

`create_map.sh` can remain as a legacy wrapper, but the long-term direction
should be a non-interactive CLI that the GUI can call with explicit arguments.

Example future commands:

```bash
jetpilot_map build-vgl-vslam --rosbag /workspaces/record/bag --map-dir /workspaces/map/course_a
jetpilot_map prepare-hd-raster --map-dir /workspaces/map/course_a
jetpilot_map generate-raceline --map-dir /workspaces/map/course_a
jetpilot_map generate-preview --map-dir /workspaces/map/course_a
jetpilot_map deploy --map-dir /workspaces/map/course_a --jetson 10.42.0.1
```

## Transfer Design

Use `rsync` rather than raw `scp` for large artifacts because it gives better
progress, restart behavior, and selective transfer.

Jetson inspection should collect:

- SSH reachability.
- `df -h` for relevant roots.
- rosbag root listing.
- map root listing.
- `latest` symlink target.
- bundle completeness for each map.

Transfer tasks should stream the original command output and expose progress in
the task list when parseable.

For Map pushes, the notebook source field offers the Map directories already
discovered under the local Map root as type-ahead suggestions. The Jetson
destination field remains the configured remote Map root unless the operator
edits it; transfer creates `<remote Map root>/<relative local Map path>/`, copies
the bundle there, and updates the adjacent `latest` symlink after rsync succeeds.

## HD Map Editor Design

Keep the HD map file format compatible with the current Python tools:

- `format: tamiya_local_hd_map_v1`
- `source_raster`
- `lanes`
- `left_bound`
- `right_bound`
- `centerline`
- `exports.primary_centerline_csv`

The browser editor currently implements:

- point add/move/delete
- fit/zoom controls with scroll-based pan
- left/right bound editing
- centerline generation from bounds
- save/load existing HD map YAML
- primary centerline CSV export
- all-lane preservation with an active-lane selector
- undo/redo and local curve smoothing assistance
- Section Gate and Junction editing
- preservation of `section_gates`, `sections`, and `junctions` across geometry saves

Then add:

- lane add/duplicate/delete operations
- direct lane naming and primary-lane reassignment

## MVP Build Order

1. Backend task runner with process group stop, persisted task state, log files,
   and live log streaming.
2. Frontend shell with Dashboard, Terminal Panel, and task list.
3. Local rosbag/map scanners.
4. Jetson SSH directory inspection.
5. Transfer tasks with rsync progress logs.
6. Map build task wrapper.
7. HD map browser editor MVP.
8. Raceline and preview actions inside the HD Map Workspace.

This order makes the terminal/task foundation solid before adding the richer map
editing surface.

## ペアレーンと手動ラインの編集

**Maps → 対象Map → Geometry → Edit** で編集します。

右パネル上部の **編集 / レイヤー / 表示・カメラ** で操作項目を切り替えます。
Fine tune layersは **レイヤー** から直接開けます。点群・投影動画・Raster設定は
**表示・カメラ** にあります。切り替えても未保存の編集は保持します。
編集パネル内をスクロールしても、Saveと保存状態は上部に残ります。

1. **レーン幅 (m)** と **走行可能の余裕／片側 (m)** を指定して **新規レーンを描く** を押します。
   余裕の初期値は片側0.1mです。青い走行可能境界は実際の壁・障害物に合わせて確認してください。
   **角を丸める補助（任意）** は初期状態でオフです。オフではクリック位置をそのまま使います。
   オンでは収まる角を円弧で丸めますが、半径が小さすぎる角は元のクリック位置を保持し、
   描画を止めません。描画途中でも切り替えでき、Undoで補助の切り替えも戻せます。
   急なヘアピンもまず描いてから左右境界を調整できます。保存時の形状チェックは引き続き行います。
   この生成処理は新規描画に適用し、既存Laneの形状は自動変更しません。
   カーソルを動かすと、次にクリックした場合の左右境界・中心線を破線、レーン範囲を
   薄い塗りでプレビューします。曲がる際に直前の境界が変わる様子も確認できます。
   最初の点は仮の向きで表示し、次の点で進行方向が決まります。
   プレビューはデータやUndo履歴に含まれず、クリック時に確定します。
   進行方向に中心をクリックすると左右の境界を同時に作成します。
   2点以上を置いて **描画完了**（または右クリック）を押します。
   周回コースはその後 **Closed loop** を選択します。
2. **Left boundary / Right boundary** を選び、点をドラッグして幅や形を
   調整します。**点を追加** に切り替えてクリックで追加し、削除は点を選択して **Delete Pt**。
   ペアレーンでは反対側にも対応する点の追加・削除が反映され、横線で
   対応関係を確認できます。centerlineは各ペアの中点です。
3. 分割する境界点を選んで **選択点で分割**。結合は現在レーンの終点と
   結合先の始点を左右とも25cm以内に合わせ、結合先を選んで
   **終点→始点を結合**。閉じたレーンの分割・再結合も可能です。
   これらは自動centerlineを使うペアレーン向けです。
   対応点数が揃っていれば、広げた走行可能境界も分割・結合時に保持します。
   既存の独立した境界のcenterline生成は
   従来の自動マッチングを維持します。Section / Junctionの参照がある場合は
   分割・結合を止め、参照を解消してから編集するよう案内します。
4. 新規レーンを走行出力にする場合は **このレーンを走行出力に使用** を押します。
   Sectionが未定義なら、Save付近の **全コースを1 Sectionにする**、または
   TopologyのSection Gatesで定義します。保存できない理由はSaveの直下に表示します。
   **Save** でHD Mapに保存します。Undo / Redoで保存前の操作を戻せます。
   保存エラーにLane・境界名・点番号が含まれる場合は対象Laneと境界を選択し、
   問題の点を赤く大きく表示します。`segment[i]` の場合は両端の点と区間を赤く表示します。
   点番号はエラーと同じ0始まりです。形状を編集すると古い強調を消し、再保存時に再確認します。

**HD Mapのみ削除** はSave付近にあります。確認後、現在のHD Map（Lane・境界・障害物・
Section・Junction）、生成済みCenterline／Raceline、走行用Custom lineと各有効化情報を
リセットします。未保存の編集も破棄します。点群・Raster・VSLAM/VGL地図、保存済みHD Map
バージョンとCustom line原本は保持します。削除対象ファイルは地図フォルダ内の
`.deleted-hd-map-*` に相対パスを保って退避します。地図を使用する処理の実行中は削除を拒否します。
   走行用centerline CSVはprimaryレーンだけを出力する既存仕様です。
   描画中のcenterlineは未保存として表示し、CSV出力はSave後に行われます。
   保存可能な未保存データがあるとSaveを強調表示します。EditをOFFにしても保存できます。
   コースを分割して調整した場合は、結合してから保存・Raceline生成します。

**Centerline** を選択すれば直接点を編集できます。編集すると自動的に
**Centerlineの手修正を保持** が有効になり、境界編集による上書きを防ぎます。
この設定は保存後も維持されます。**Auto Center** または保持チェックの解除で
境界からの自動生成に戻ります。ペア境界の部分平滑化は対応関係を崩すため
無効です。境界点のドラッグ、または手動centerlineの平滑化で調整してください。

境界・Centerlineの操作は **点を移動** ／ **点を追加** で切り替えます。
既定と新規レーンの描画完了後は **点を移動** です。Left / Rightを選んで既存の点を
ドラッグすると、反対側の境界を動かさずに幅や形を調整できます。空白のクリックでは
追加されません。**点を追加** ではクリックで追加し、既存の点は動きません。
ペア境界への追加時は、反対側にも対応点を補間して左右の点数を維持します。
削除は点を選択して **Delete Pt** を押します。境界・Centerlineの右クリックや
ダブルクリックでは削除しません。Undo / Redoと保存形式は従来どおりです。

走行用の別ラインは **Driving Lines → Custom Lines** で作成できます。
名前と **Centerline / Raceline** のコピー元を指定して **Clone as Custom**、
続いて **Edit shape** を押します。クリックで追加、ドラッグで移動、右クリックで
削除し、**Save** で保存します。**Use for drive** で次の走行・転送用に選択します。
元のCenterline / Racelineは変更されません。既存のコース内判定と速度検証は
引き続き適用されます。

Dependency-free editor checks:

Mapの **高さ付き点群を表示** を有効にすると、既定で **密度で濃淡** を表示します。
選択したZ範囲の点を地図上の5cm区画に集計し、疎い点は薄く、密集した輪郭は明るくします。
**集計幅**（2〜20cm）と **コントラスト**（大きいほど疎い点を抑える）を調整でき、
**従来の点表示** にも戻せます。ズームは密度計算を変えません。
表示用の点群が間引かれている場合、濃淡もその点群に基づきます。
これは編集背景の表示設定で、PNG・HD Mapの形状・座標・保存データは変更しません。

```sh
node --test tools/app/frontend/tests/lane_geometry.test.js
node --test tools/app/frontend/tests/point_cloud_ui.test.cjs
PYTHONPATH=tools/app/backend python3 -S -m unittest discover -s tools/app/backend/tests -p 'test_map_detail.py'
```


## Section未定義の保存防止と1 Sectionコース

HD Map YAMLを保存するには、走行出力（primary）レーンのSection定義が必要です。
GeometryまたはTopologyの **全コースを1 Sectionにする** を押すと、周回コースには
Gateを1つ、開いたコースには始点・終点のGateを2つ作成します。勝手には定義しません。
複数に分ける場合は **Topology → Section Gates → Edit** でcenterline上をクリックします。
新規Mapでも、保存前のcenterline上にGateを配置して、形状とSectionをまとめて保存できます。

未定義、開いたコースのGateが1つだけ、Gate位置の重複は保存できません。
画面のSaveを無効にし、保存API側でも書き込み前に検証します。
既存YAMLを更新する際のエラーでは、以前のYAMLを保持します。

周回コースの1 Gateは、同じGateから次の周の同じGateまでを1 Sectionとします。
YAMLには `end_s_m = start_s_m + lane_length_m` と `wrap: true` を出力します。
以前のUIで1 Gateだけを保存して `sections` が空になったMapは、修正版で対象Mapを開き、
**Topology → Section Gates → Edit → Save** で再保存してください。
実機で使う際は、修正版のHD map publisher/localizerと再保存したMapを反映してください。

## カメラ画像へのHD Map投影

Bag Analysisの動画に左境界（緑）、右境界（桃）、centerline（黄）、
生成済みRaceline（水色）を重ねて表示します。**HD Mapを画像に投影** で切り替えます。
複数カメラ表示でも、各カメラの撮影時刻に対応した位置・姿勢で投影します。

MapsでMapを開くと、**カメラ画像で配置を確認** パネルから同じMapで解析した動画を
選択できます。カメラ選択、再生・一時停止、コマ送り、シークが可能です。
動画を止めて境界やcenterlineを編集すると、保存前の形状をその場で画像に反映します。
Custom Lineの編集中の形状は橙で表示します。Racelineは最後に生成したものを表示するため、
境界編集後は必要に応じて再生成してください。

投影には、bag内の画像・対応する `sensor_msgs/CameraInfo`・`/tf`・`/tf_static` と、
撮影時刻のMap座標での自己位置が必要です。オフラインVSLAM解析では解析結果の3D姿勢を使い、
記録済みのmap変換は混在させません。カメラの外部校正はoptical frameまでのTFから取得します。
現在の設定ファイルの校正値で過去のbagを補完することはありません。
以前に生成した解析結果には投影情報がないため、bagを再解析してください。

Raw画像はCameraInfoのK/D、`/image_rect` を含むトピックはR/Pを使います。
対応する歪みモデルはplumb_bob、rational_polynomial、equidistantです。
独自名の補正済み画像トピックは、この命名に合わせてから解析してください。
校正・撮影時刻・TFが不足する場合、動的TFが150msより古い場合、または自己位置推定Mapが
解析時から変更されている場合は、投影を止めて理由を表示します。
HD Mapの線だけを編集しても、自己位置推定Mapとの対応は失われません。

2Dの線の初期高さは、解析時の `map → base_footprint` TFから求めます。
水平な路面を前提に、画像撮影時刻で得た路面Zの中央値をBag AnalysisとMapsの両方へ設定します。
路面がMapのXY平面から2度より傾く場合、または高さ変動が5cmを超える場合は自動設定しません。
路面基準TFがない解析や旧解析では0 mを仮の初期値とし、その旨を表示します。
**投影高さ（Map Z / m）** で手動調整できます。同じ解析の再描画で手動値を上書きしません。
カメラの取付高さはすでにTFに含まれており、この欄には路面のMap座標Zを指定します。
TT-02では車軸高さを実測してbase_footprint構成で記録し、bagを再解析すると自動設定できます。
配置ずれを見つけるための表示であり、
壁との衝突や遮蔽は判定しません。精度は内部・外部校正、時刻同期、自己位置精度に依存します。

投影計算・抽出処理の依存追加なしの確認:

```sh
node --test tools/app/frontend/tests/camera_projection.test.js
PYTHONPATH=tools/app/backend python3 -S -m unittest discover -s tools/app/backend/tests -p 'test_camera_projection.py'
```

TT-02の55 mm支柱構成では、路面基準TFがない場合に記録済みの`tt02_ground_estimate`を
代わりに使えます。支柱55 mmは確認済み、ロウワデッキ取付面の地上高25 mmとプレート下面から
CAD原点までの差0 mmは仮定です。プレートのMap Zが0なら初期値は−0.080 mとなります。
UIはこの値を「推定路面TF」と表示します。現行の取付TFで新しく記録したbagを再解析してください。

## 実車調整モード

**Maps → Mapを開く → 実車調整モード** に、画像転送を伴わない自己位置表示、
センター／レース／Custom Lineの選択、全体・区間速度のプレビューと停車中の適用を追加しました。
HD Map・センターライン・Custom Line・Section Gateの編集は既存エディターを共用します。
Jetsonには専用の調整bridge/controllerを起動します。起動方法、通信断時の扱い、
通常の競技planningとの違いは [実車調整の手順](runtime/README.md) を参照してください。

## 走行可能境界・経路生成境界・固定障害物

Maps → Geometry → Editで、2種類の左右境界を個別に編集できます。

- **経路生成：左／右**（緑・桃）：Centerlineの自動生成とRacelineの最適化に使う帯。
- **走行可能：左／右**（青）：壁などの物理的な走行可能領域。ここを動かしてもCenterlineは変わりません。

旧Mapは従来の境界を両方の初期値として読み込みます。その後は個別に編集・保存できます。
経路生成境界は走行可能境界の内側に置いてください。
「現在の経路生成境界を走行可能境界へコピー」で物理境界を置き換えられます。Undoで戻せます。
独立して形状を変更した走行可能境界を持つレーンの分割・結合は、対応点を安全に決められないため未対応です。

段ボールなどの配置手順：

1. **固定障害物 → 障害物を描く**を押す。
2. 地図上で底面の角を順にクリックし、3点以上で**障害物の描画完了**を押す。
3. 名前・高さ・余裕を設定する。高さはカメラ投影・ROS表示用、余裕は障害物から離す距離です。
4. 頂点のドラッグで形を変え、内部のドラッグで全体を移動できます。「点を追加」、選択点の削除、障害物全体の削除、Undo / Redoも使えます。
5. **HD MapのSave**で境界・障害物・Sectionをまとめて保存する。

障害物はHD Map YAML直下の`obstacles`に、`id`・`name`・`polygon`・`height_m`・`margin_m`で保存します。
従来の`left_bound` / `right_bound`は経路生成用として維持し、物理境界はレーン内の
`drivable_left_bound` / `drivable_right_bound`に保存します。
Centerline CSVの左右幅は経路生成境界から出力します。

Custom Lineは走行可能境界と障害物を検証します。経路生成帯の外でも物理領域内なら編集できます。
障害物は点だけでなく線分全体との交差・余裕も確認し、点間の薄い障害物も検出します。
Custom Lineでは車幅を自動加算しないため、必要な距離を障害物の余裕で指定してください。
Raceline生成では、同じフォルダのHD Map（または`--hd-map`指定）を読み、車幅の半分と安全余裕も含めて
物理境界・障害物を検証し、干渉する出力の書き込みを拒否します。

障害物を追加した地図自体は、既存Centerlineに重なっていても保存できます。その場合は保存済みラインの警告を表示します。
障害物の配置だけでは迂回経路や実車の回避制御は自動生成されません。
経路生成境界を迂回させたい側へ調整し、Centerline / Racelineを再生成してください。
ROSのHD Map表示には物理境界と障害物の高さ付き輪郭を出力します。

### Plannerの走行領域チェック

通常・競技planningの起動時に`drivable_guard`が起動し、最終経路（復帰用の後退経路も含む）を確認します。
HD Map publisherの`/hd_map/drivable_area`から物理境界と固定障害物を受け取り、
`/planning/safety_status`に走行許可・緊急・理由をまとめて出します。
`/planning/ready`は従来どおり経路選択の準備状態です。controllerはそれに加えて安全判定の許可を必須にします。

- **緊急**：現在の車体が走行不可領域に触れている、または現在の速度・旋回から予測した停止までの動きが境界や障害物に触れる。
- **通常の停止**：選択経路が走行不可領域を通る、地図・速度・自己位置・経路が不明／古い、地図更新に失敗した。
- 経路生成境界から外れただけでは緊急にしません。危険判定を実衝突用の`/safety/collision_detected`へ送らず、誤って後退復帰を開始させません。

controllerは安全判定が0.3秒届かなくなっても停止します。判定周期と受信期限には、シミュレーション時間が
止まっても進む時計を使います。緊急はcontroller診断でERROR、通常の待機はWARNとして区別します。
地図の欠落・更新失敗時は表示用に旧地図を保持しても走行許可には使いません。
再生bagの安全判定・走行領域は隔離し、現在の処理で判定し直します。

車体は`base_link`を中心に全体を囲む円と余裕で扱い、停止までその円が通る範囲を検証します。
停止距離は「速度 × 反応時間 + 速度² ÷ (2 × 減速度)」です。
旋回を続ける場合と、停止指令で操舵を中央へ戻す場合の両方を確認します。前進・後退・横方向速度に対応します。
経路は停止距離以上の前方を確認し、型付き経路の速度と周回設定も反映します。
障害物は底面を走行不可とする平面判定で、高さによるくぐり抜けは扱いません。

設定は`jetpilot_hdmap_publisher/config/drivable_guard.param.yaml`です。
`front_m` / `rear_m`は座標原点から車体端まで、`width_m`は車幅、`margin_m`は追加余裕です。
`reaction_s`と`braking_mps2`は**実測済みではない仮値**です。controllerの`safety_brake_command`は
従来の値を使い、標準値0ではスロットルを切る停止指令になります。実車で停止距離を測り、実際に得られる減速度以下に設定してください。
車体を囲む円を使うため、車体が通れる狭い部分でも停止することがあります。また、レーンの境界をまたぐ場合は
各短区間の車体円がいずれか一つのレーンに収まることを要求するため、重なりのない接続部では保守的に停止します。
地図と自己位置の誤差、路面の変化まで含めた停止保証や、自動迂回を提供するものではありません。

新しいメッセージ・HD Map publisher・planning・controllerをROS実行環境で再ビルドし、同時に再起動してください。
旧publisherだけを残した場合、新しいcontrollerは安全判定を受け取れず停止します。


## Simulationで追従3方式を比較

Maps → 対象Map → Simulationで経路・車両設定と**Controller**を選び、**Run**を押します。
Pure Pursuit・Map Pursuit・Kinematic MPCを単独で動かせます。**3方式を同時に再生**または**3台を同時にRun**では3台が同じ時刻で動きます。
**Pause**で状態を保持して一時停止し、**Run**で再開、**Step**で0.1秒進めます。**Reset**、controller・経路・条件の変更では初期状態からやり直します。
Pure Pursuit・Map Pursuit・Kinematic MPCが、同じ初期位置・速度0・同じ比較時間で走ります。
実行時間（1〜120秒）、初期横ずれ、初期向きのずれを指定できます。
青・橙・紫の軌跡を重ね、追従誤差RMS、最大誤差、単位時間あたりの操舵変化量、走行距離、経過時間、終了理由を表示します。
誤差は経路の点ではなく線分までの距離です。終点到達・追従不能では途中終了するため、誤差だけでなく経過時間も比較してください。
Customは各地点の速度profile、それ以外は画面の目標速度を使用します。逐次再生と最終比較は同じ計算処理を使います。

比較はブラウザー内で実行され、実車の設定は変更しません。計算は別処理で行い、再生中も画面を操作できます。経路・設定を変更すると古い結果を破棄します。
操舵の計算は実車のC++実装に対応し、Map Pursuitの横誤差補正・速度による操舵低減、
MPCの操舵候補ごとの予測・コスト評価を再現しています。選択したcontrollerに応じた設定欄で方式固有の値を調整できます。3方式同時再生では両方の欄を表示します。
- Map Pursuit：横誤差補正、操舵低減の開始／終了速度、最大低減率。
- Kinematic MPC：予測ステップ数・時間刻み（合計予測時間も表示）、操舵候補数、予測最低速度、経路・向き・操舵・終端の重み。

方式ごとに「標準値に戻す」が使えます。変更すると再生は初期状態へ戻り、次のRunから新しい値を使います。
調整値は画面内で保持し、controllerを切り替えても維持しますが、ページ再読み込みでは標準値に戻ります。実車設定には書き込みません。
MPCは現在の実車実装と同じ、予測区間内では一定の操舵を使う候補探索です。
車両運動・加減速は3方式共通の簡易モデルで、実車の操舵応答、ROS通信、制動、走行領域の安全判定は再現しません。

### Optunaで軽く自動調整

Simulationの**Controllerを1方式選択 → Optunaで軽く自動調整 → 自動調整を開始**で、
現在値を基準に追従誤差RMSを最小化します。候補数は5〜50、標準20です（現在値の評価は別に1回）。
経路・速度・車体寸法・初期横ずれ／向き・実行時間を固定し、ブラウザーのSimulation Workerで候補を評価します。
追従不能、サンプルなし、ほぼ停止、時間終了なのに評価時間未満、基準から大きく走行距離が減る候補は採用しません。
途中終了して誤差が小さくなる候補を避けるため、基準が終点に到達した場合は候補にも到達を要求します。

探索対象と範囲：

| 方式 | 探索対象 |
| --- | --- |
| Pure Pursuit | 最小Lookahead 0.15〜1.2 m、最大1.2〜3.5 m、速度ゲイン0〜0.8 s |
| Map Pursuit | 上記＋横誤差補正0〜1.5、操舵低減の開始0.5〜2.5 m/s・終了2.5〜5 m/s・率0〜0.5 |
| Kinematic MPC | 予測ステップ6〜24、経路重み0.5〜12、向き0.05〜3、操舵0.01〜1、終端0.1〜8 |

MPCの時間刻み・候補数・予測最低速度は画面の設定で固定します。
Optunaの[Ask-and-Tell](https://optuna.readthedocs.io/en/stable/tutorial/20_recipes/009_ask_and_tell.html)で
候補の提案とブラウザー評価を分離しています。TPE、seed 42、最初の5候補を初期探索に使います。
追加のNode.js環境やROS環境は不要です。

**Consoleを動かしているPython環境**にのみ、追加依存が必要です。

```sh
python3 -m pip install -r tools/app/requirements-simulation-optuna.txt
```

未導入の場合は自動調整欄に案内を表示し、通常のSimulationには影響しません。
この変更のローカル検証ではOptunaは導入・実行しておらず、連携部分を代替した標準ライブラリのテストを行っています。

探索中は現在の設定を書き換えません。最良候補が現在値を改善したら、
**候補をSimulationに適用 → Run**で動きを確認してください。基準評価が失敗した場合でも、有効な候補が見つかれば適用できます。
途中中止までに得られた候補も確認できます。設定・経路を変更すると古い候補を破棄します。
1候補の評価は60秒で打ち切り、長い経路などで超えた場合は案内を表示します。
探索セッションと結果はメモリー上のみで保持し、ブラウザー再読み込み／Console再起動では引き継ぎません。
放置したサーバー側セッションは30分で期限切れとなり、同時に8件までです。

評価対象はその経路・初期条件での追従性能です。全コースへの最適性、操舵の滑らかさ、実車の安全性は保証しません。
物理境界・固定障害物の安全判定、実車の遅延・制動はこの探索に含まず、実車設定にも書き込みません。

## Sections / Junctionsの配置と使い方

Topology → Sections → Editでは、センターライン付近にカーソルを当てると、
クリックで置かれるゲート線と吸着先の点・ライン上の距離 `s` を表示します。
吸着先は既存の頂点だけでなく、センターラインの線分上の位置です。
既存ゲートに重なる場合はそのゲートの選択を予告し、新しいゲートは増えません。
ラインから離れた位置では配置プレビューを出さず、クリックしても配置しません。
カーソルがキャンバス外へ出るとプレビューは消えます。

Junctionは「交差点の点」だけでなく、**信号の指示を受け付ける区間・指示ごとの走行ルート・選択を解除する区間**をまとめた設定です。
菱形の `Map position` は表示上の目印で、センターラインへの吸着や、実行時の分岐開始位置を指定するものではありません。
New / Placeでは配置予定の菱形がカーソルに追従し、クリックした地図座標に配置されます。
Snapは最初に指定したActivation区間の終了ゲートへ目印を移します。

設定手順：

1. **Topology → Sections → Edit**で分岐手前・通過後の区間を用意し、保存します。ゲート間がSectionになります。
2. 分岐後に走るラインを先に用意します。Junctionへの割り当てでラインが自動生成されるわけではありません。
3. **Topology → Junctions → Edit → New**を押し、Junction IDとSignal IDを設定します。Signal IDは検出側が出すIDに合わせます。
4. **Activation sections**に分岐手前の区間を指定します。Newの初期選択が意図した区間か確認してください。ここに入ると有効な信号指示を受けるまで停止します。
5. **Left / Straight / Right**に、各指示で使う既存のルートを割り当てます。現在の保存仕様では3方向すべてが必須です。
6. **Release sections**に分岐通過後の区間を指定します。ここに入ると分岐ルートの選択が解除されます。Activationと同じ区間は指定できません。
7. **Place**で目印を置くか**Snap**でActivationの終了ゲートへ寄せ、**Save**します。画面内の「Junctionの設定手順」でも確認できます。
8. **Review**で分岐先ルートの実行時登録とpublisherの配信先を確認します。地図への保存だけでは実行時の経路配信は追加されません。

例えば `手前のSection → 分岐内 → 通過後のSection` と分け、手前をActivation、通過後をReleaseにします。
手前で「左」を受信するとLeftのルートを選択し、通過後のSectionに入るまで選択を保持します。

### 分岐・合流するLaneネットワーク

Map UIのLane編集に「分岐・合流の接続」を追加しました。Laneの重なりは許可しますが、
重なりや近さから接続を推測しません。接続元・接続先のIDを明示的に保存します。

1. 分岐点・合流点を端点に持つ、開いたLaneを作成します。途中から分岐する場合は、その位置で分割します。
2. 接続元Laneを選択し、「接続先Lane」を選びます。
3. 端点が一致していれば「端点を直接接続」。離れている場合は「直線Laneで接続」または「カーブLaneで接続」を使います。
   後者は両端の幅・向きを使った独立した接続Laneを作成します。作成後に形状を確認・編集でき、Undoも使えます。
   進行方向の延長線が前方で交わる通常のコーナーは、一定半径の円弧と直線で接続します。
   入口・出口から交点までの距離が異なる場合も、円弧を引き伸ばさず長い側に直線を残します。
   左右境界と走行可能境界は中心線に沿った幅から生成し、両端の幅が異なる場合は徐々に変化させます。
   内側が潰れる半径では作成を拒否します。平行区間や折り返しなど、この円弧を構成できない
   配置は接線方向を使った曲線で接続するため、作成後の確認が必要です。保存済み接続は自動変更しません。
4. 同じ接続元から別の接続先へも接続すれば分岐です。複数の接続元から同じ接続先へ接続すれば合流です。
5. 「通常選ぶ接続先」を指定します。接続先が1つなら自動。複数ある場合はデフォルト分岐が必須です。最初に接続したLaneを初期値にし、変更できます。未指定の地図は保存・実行前に拒否します。
6. 各LaneのCenterlineを確認して保存します。「保存して各LaneのRaceline候補を生成」で、共通区間・各分岐・接続区間の候補をまとめて生成できます。

Centerlineは既存のAuto Center、接続Laneは選択した直線またはカーブで生成します。
ネットワークのRaceline候補は境界内で曲がり方を滑らかにする標準ライブラリの反復計算です。
最初と最後の2サンプルを固定し、接続位置と方向を維持します。全周の最短時間を求める既存の
閉ループ用最適化とは別方式です。共通区間を複製せずLaneごとに保持し、分岐の組み合わせを全列挙しません。

生成結果は対象地図の `*_hd_map.yaml` の各Laneに `network_raceline` として保存し、橙線で表示します。
各Laneの `successor_ids` に接続先、`default_successor_id` に通常の選択先を保存します。
既存のprimary用Centerline CSVは引き続きprimary単体です。ネットワークの全経路CSVではありません。
地図の形状・接続・障害物が変わると候補を無効化するため、再生成してください。
旧Pythonエディタは接続情報を更新できないため、ネットワーク地図の上書きを拒否します。

生成時の物理的な余裕は標準の安全判定と同じ、車体を囲む円の半径（約0.323m）です。
開始・終了地点も車体全体が入るよう、物理的な境界はCenterlineの端点より先まで確保してください。
実車の寸法や安全判定の設定を変更した場合は、この生成条件との整合も確認してください。

#### 実行側の選択と現在Lane

ROS側はビルド後、通常のbringupで以下を指定して有効化します。
既存の単一ルート走行の挙動を変更しないため、ネットワークモードは初期状態では無効です。

```text
enable_planning:=true
enable_lane_network:=true
hd_map_yaml_path:=/absolute/path/to/course_hd_map.yaml
network_initial_lane_id:=lane_001
network_line_mode:=centerline
```

`network_line_mode:=raceline` で生成済み候補を使用します。初期Laneを省略するとprimaryを使用します。
最も近いLaneを自動選択しないため、重なっている場所から開始する場合も初期Laneを明示できます。

- `/planning/current_lane`：現在追従中のLane ID。Sectionの判定もこのLane内だけで行います。
- `/planning/network_branch_choices`：JSON形式の選択 `{ "分岐元Lane ID": "接続先Lane ID" }` を受信します。
  接続されていないIDは拒否します。通常はデフォルト分岐を使用します。終端の1m手前で確定し、確定済みの接続先は次のLaneへ入るまで切り替えません。
- `/planning/network_status`：待機・停止・追従状態の理由を通知します。
- 出力は既存の `/planning/trajectory`・速度付き `/planning/trajectory_profile`・`/planning/ready` に接続します。
  ネットワークモードでは従来のroute selectorを起動しないため、出力は競合しません。

現在Laneの終端を進行方向へ通過し、選択した接続先の入口に到達したときだけLaneを更新します。
自己位置が飛んだ場合も、近い別Laneへの自動復帰は行いません。地図が実行中に変更された場合は停止し、
開始Laneを確認してplannerを再起動します。終端には減速する速度profileを出します。信号による指定がなくても、分岐では保存されたデフォルト接続先を使用します。

物理的な安全判定は全Lane領域の和集合の外周と障害物を使用します。重複部分や共有する継ぎ目を壁扱いしません。
一方、経路選択は明示された接続だけを使用します。重なっている別Laneへ横移動してよい、という意味ではありません。

現段階のネットワークplannerは通常planning専用です。既存の信号Junction／competition managerの
ルート切替とはまだ統合しておらず、同時有効化は拒否します。ブラウザSimulationでは、以下の手動信号付きネットワーク走行を利用できます。
ROSの実行環境でのビルド・走行確認は別途必要です。


#### ブラウザでのネットワークSimulationと手動信号

SimulationのPath sourceで「Laneネットワーク · Centerline」または「Laneネットワーク · Raceline候補」を選択します。
開始Laneを指定し、Controllerを選んでRunします。「3台を同時にRun」でも同じネットワークと信号を使用します。
表には各車両の現在Lane・次のLane・分岐が確定済みかどうかを表示します。

「Simulationの信号」で、実行中やPause中に次の操作ができます。信号を変更しても時刻・位置はリセットしません。

- 通常：そのLaneに保存したデフォルト分岐へ進みます。
- 停止：停止位置の手前で減速・待機します。方向または通常へ戻すと再発進します。
- 左・直進・右：既存Junctionのbranchesを接続先へ対応付けます。接続Laneを経由する場合も、途中に別の分岐がなければ対応付けできます。
- Junctionがない分岐：接続先のLane IDを直接選択できます。

既存JunctionはActivation Sectionの開始から信号を評価し、その区間終端を停止位置に使います。
同じLaneに異なるJunctionが複数ある場合は、信号ごとにLaneを分割してください。
方向を確定するのは通常Lane終端の1m手前です。Activationがそれより後ならActivationから、
停止線がそれより前なら停止線の通過時に確定します。確定後の方向変更では走行中の枝を変更しません。
停止は停止線を通過するまで有効で、通過後に停止へ変えた場合は次回の進入から適用します。
遅すぎる停止指示で停止線を越えた車両は、その理由を表示して当該車両のSimulationを終了します。

Centerline/Racelineとも、現在Lane内で進行位置を追跡します。別Laneの線が近い・重なっているだけでは切り替えません。
各Controllerの車両が個別に接続先へ遷移するため、追従の違いで分岐到達時刻が変わることも確認できます。
地図・接続・Junctionを変更した場合は、以前の手動信号をクリアします。Resetでは同じ信号設定で開始位置へ戻ります。

手動信号付きネットワークSimulationではOptunaを無効にしています。既存の単一ライン評価では引き続き使用できます。
このSimulationは簡易車両モデルで、実車の信号認識、通信遅延、物理的な走行可能領域・障害物の安全判定は再現しません。
