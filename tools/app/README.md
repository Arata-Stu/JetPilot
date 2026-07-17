# JetPilot Console Design

JetPilot Console is a local browser app for managing the repeated, file-heavy
workflows around RC car autonomy. It is not the first home for live vehicle
runtime control; Jetson-side tmux launch and real-time operation can remain a
later phase. The first goal is to make the notebook-side utility workflows
visible, repeatable, cancellable, and easy to inspect.

## Current MVP

The first implementation is intentionally dependency-light:

- Python standard-library HTTP backend.
- Static HTML/CSS/JS frontend.
- No npm install, no FastAPI install, and no build step required.

Start it from the repository root:

```bash
tools/app/scripts/start.sh --host 127.0.0.1 --port 8765
```

When running inside the JetPilot Docker environment, use the `tools` mount:

```bash
/workspaces/tools/app/scripts/start.sh --host 127.0.0.1 --port 8765
```

The Docker launcher mounts the top-level `tools` directory to
`/workspaces/tools` alongside `ros2_ws`, `python_ws`, `record`, and `map`.
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

Environment overrides:

```bash
ANALYSIS_ROOT=/workspaces/record/.jetpilot_analysis \
JETPILOT_ANALYSIS_ROS_DOMAIN_ID=92 \
tools/app/scripts/start.sh --host 127.0.0.1 --port 8765
```

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
- Current editor scope:
  - one primary lane
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
  - section gates
- Right-side inspector:
  - closed/open loop
  - artifact status
  - lane and section summaries
- Actions:
  - prepare landmark raster
  - edit and save HD map YAML/centerline CSV
  - generate raceline with adjustable vehicle width and per-side boundary margin
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
  "cwd": "/workspaces/JetPilot",
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
POST /api/maps/generate-raceline
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
  "vehicle_width_m": 0.25,
  "safety_margin_m": 0.05
}
```

The effective optimizer envelope is `vehicle_width_m + 2 * safety_margin_m`.
The API defaults preserve the existing `0.25 m` vehicle and `0.05 m` per-side
margin behavior. Generation keeps the existing raceline CSV layout and writes
the selected values to `<map_name>_raceline.meta.json` for reproducibility.

## Pipeline Stages

The GUI should not treat map creation as one giant script. Split it into
restartable stages:

```text
1. Discover/transfer rosbag
2. Build VGL map and compute VSLAM map/snapshot
3. Prepare landmark raster for editing
4. Edit HD map in browser
5. Save HD map YAML and centerline CSV
6. Generate raceline
7. Generate preview image
8. Transfer complete bundle to Jetson
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
- preservation of existing `section_gates` and `sections` blocks on save

Then add:

- lane selection
- undo/redo
- curve assist
- VSLAM path overlay
- raceline overlay
- section gates

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
