# map_tools

Utilities for creating and checking local HD map artifacts.

## Browser HD map editor

The JetPilot Console Maps workspace can edit the primary HD map lane directly
on the landmark raster. It writes the same `<map_name>_hd_map.yaml` and
`<map_name>_hd_map_centerline.csv` files as the Python editor, so the Raceline
and Preview stages can run immediately after saving.

## HD map editor

`hd_map_editor.py` opens a raster map YAML as the background and writes an
editable HD map YAML. By default, you draw only `left_bound` and `right_bound`;
the lane `centerline` is generated from those bounds.

```bash
python3 python_ws/map_tools/hd_map_editor.py \
  --map-yaml /path/to/map.yaml \
  --output /path/to/course_hd_map.yaml
```

Controls:

- `2`: edit `left_bound`
- `3`: edit `right_bound`
- left click / drag: add or move a boundary point
- `d`: delete the nearest boundary point, or the last point if none is near
- `u`: remove the last point on the active boundary
- `c`: smooth the active curve with curve assist
- `a`: regenerate centerlines from bounds
- `n`: create a lane
- `x`: delete the active lane; the final lane is cleared instead
- `z` / `y`: undo / redo edits, including curve assist and lane deletion
- `v`: toggle the optional VSLAM path overlay
- `m`: toggle generated centerline overlays (hidden initially in auto mode)
- `s`: save the HD map YAML and primary centerline CSV
- `o`: toggle closed/open loop for the active lane

Curve assist applies a light smoothing pass to the active polyline and resamples
it at about `0.10 m` spacing. Press `c` more than once for a stronger effect, or
tune one press with the options below. Press `z` to restore the pre-smoothing
point sequence.

```bash
python3 python_ws/map_tools/hd_map_editor.py \
  --map-yaml /path/to/map.yaml \
  --output /path/to/course_hd_map.yaml \
  --curve-assist-iterations 2 \
  --curve-assist-spacing 0.05
```

Automatic centerlines are resampled at about `0.10 m` spacing by default. Tune
that density with:

```bash
python3 python_ws/map_tools/hd_map_editor.py \
  --map-yaml /path/to/map.yaml \
  --output /path/to/course_hd_map.yaml \
  --auto-centerline-spacing 0.05
```

If you need the old workflow, add `--manual-centerline` and edit `1:center`
manually.

The primary lane CSV can also be regenerated without opening the GUI:

```bash
python3 python_ws/map_tools/hd_map_editor.py \
  --map-yaml /path/to/map.yaml \
  --output /path/to/course_hd_map.yaml \
  --export-only
```

After raceline generation, `scripts/create_map.sh` runs
`visualize_race_lines.py` and writes `<map_name>_line_preview.png` with the HD
lane bounds, centerline, and raceline overlaid on the landmark raster.

## Raceline vehicle clearance

`generate_raceline.py` reserves the physical vehicle width plus an additional
safety margin on each side. For example, a `0.25 m` vehicle with a `0.05 m`
margin uses a `0.35 m` optimizer envelope:

```bash
python3 python_ws/map_tools/generate_raceline.py \
  --centerline /path/to/course_hd_map_centerline.csv \
  --output /path/to/course_raceline.csv \
  --vehicle-width-m 0.25 \
  --safety-margin-m 0.05 \
  --max-speed 3.0 \
  --min-speed 0.8 \
  --lateral-accel-limit 2.5 \
  --accel-limit 1.5 \
  --decel-limit 2.5
```

The defaults remain `0.25 m` and `0.05 m` for compatibility. The older
`--vehicle-width` and `--safety-margin` names are still accepted.

The centerline CSV widths already describe the distance from the reference line
to the right and left boundaries. Those widths are passed to
`global_racetrajectory_optimization` unchanged; its `w_veh` constraint applies
half of the effective envelope to each boundary. This avoids subtracting the
vehicle width twice. If any part of the track is narrower than the requested
envelope, generation stops with the point index and measured width instead of
silently expanding the map bounds.

The CSV stays in the existing seven-column format. Generation also writes a
non-breaking JSON sidecar next to it (for example,
`course_raceline.meta.json`). The sidecar records the source centerline,
direction, optimizer mode, physical width, per-side margin, effective envelope,
minimum available track width, speed profile limits, and result size/length. A
`both` direction run writes separate metadata for the forward and reverse
outputs.

Use `--direction forward` or `--direction reverse` to select the lap direction.
The JetPilot Console exposes the same choice in the Raceline generation panel.
When using `scripts/create_map.sh`, the workflow asks for the direction before
generation; `RACELINE_DIRECTION=reverse` changes its default choice.

## Section gate editor

After the HD map YAML has a centerline, use `hd_map_section_gate_editor.py` to
place section gates. It computes each gate's `s_m` by projecting onto the lane
centerline and regenerates the `sections` block on save.
