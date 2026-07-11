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
/workspaces/tools/app/scripts/start.sh --host 0.0.0.0 --port 8765
```

The Docker launcher mounts the top-level `tools` directory to
`/workspaces/tools` alongside `ros2_ws`, `python_ws`, `record`, and `map`.

Then open:

```text
http://127.0.0.1:8765
```

Implemented in this MVP:

- Local rosbag scan from `RECORD_ROOT`.
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
  - raceline generation
  - preview generation
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
  while still allowing a manual override.
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
- Layers:
  - landmark raster
  - VSLAM path
  - left bound
  - right bound
  - generated centerline
  - generated raceline
- Right-side inspector:
  - active lane
  - closed/open loop
  - smoothing settings
  - centerline spacing
  - raceline parameters
  - artifact status
- Actions:
  - save HD map YAML
  - generate centerline CSV
  - generate raceline
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
WS   /api/tasks/{task_id}/stream

GET  /api/rosbags/local
GET  /api/rosbags/jetson
POST /api/transfers/jetson-to-local

GET  /api/maps/local
GET  /api/maps/jetson
GET  /api/map-builder/camera-topic-configs
POST /api/maps/build-vgl-vslam
POST /api/maps/{map_id}/prepare-hd-raster
POST /api/maps/{map_id}/generate-raceline
POST /api/maps/{map_id}/generate-preview
POST /api/transfers/local-to-jetson

GET  /api/hd-map/{map_id}
PUT  /api/hd-map/{map_id}
```

## Pipeline Stages

The GUI should not treat map creation as one giant script. Split it into
restartable stages:

```text
1. Discover/transfer rosbag
2. Build VGL map and compute VSLAM map/snapshot
3. Prepare landmark raster for editing
4. Edit HD map in browser
5. Generate centerline CSV
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

The first browser editor should implement:

- pan and zoom
- point add/move/delete
- lane selection
- left/right bound editing
- centerline generation from bounds
- save/load existing HD map YAML
- undo/redo
- copy path buttons

Then add:

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
