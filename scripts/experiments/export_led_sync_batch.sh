#!/usr/bin/env bash
set -euo pipefail

CONFIG="/workspaces/ros2_ws/src/tool/multi_sensor_calibration/config/rc_popout_evs_rgb.yaml"
RGB_ROI="0,0,848,480"
EVS_ROI="0,0,640,480"
BIN_MS="1"
PREVIEW_FPS="60"
PREVIEW_WINDOW_S="12"
ROI_TILE_SIZE="16"
FORCE=false
SELECTED_SESSION=""
CALIBRATION_SESSION="20260921_185407_rcp_calibration_checkerboard_20260921_185405"
ROOTS=(
  "/workspaces/record/evs-popup-v1"
  "/workspaces/record/evs-popup-v2"
)

usage() {
  cat <<'EOF'
Usage: scripts/experiments/export_led_sync_batch.sh [options]

Options:
  --rgb-roi X,Y,W,H   RGB LED ROI (default: full 848x480 frame)
  --evs-roi X,Y,W,H   EVS LED ROI (default: full 640x480 frame)
  --bin-ms MS          EVS bin width (default: 1)
  --preview-fps FPS    RGB/EVS preview rate (default: 60)
  --preview-window-s S Export this many seconds at start and end (default: 12)
  --roi-tile-size PX   Spatial tile size for UI ROI recalculation (default: 16)
  --no-preview         Do not export visual preview images
  --config PATH        Calibration config path
  --session DIR        Process only this recording session directory
  --force              Reprocess sessions with an existing JSON output
  -h, --help           Show this help

The calibration checkerboard session is skipped. Outputs are written below:
  evs-popup-vN/analysis/led_sync/<session>/
EOF
}

while (($#)); do
  case "$1" in
    --rgb-roi)
      RGB_ROI="${2:?--rgb-roi requires X,Y,W,H}"
      shift 2
      ;;
    --evs-roi)
      EVS_ROI="${2:?--evs-roi requires X,Y,W,H}"
      shift 2
      ;;
    --bin-ms)
      BIN_MS="${2:?--bin-ms requires a value}"
      shift 2
      ;;
    --preview-fps)
      PREVIEW_FPS="${2:?--preview-fps requires a value}"
      shift 2
      ;;
    --preview-window-s)
      PREVIEW_WINDOW_S="${2:?--preview-window-s requires a value}"
      shift 2
      ;;
    --roi-tile-size)
      ROI_TILE_SIZE="${2:?--roi-tile-size requires a value}"
      shift 2
      ;;
    --no-preview)
      PREVIEW_FPS="0"
      shift
      ;;
    --config)
      CONFIG="${2:?--config requires a path}"
      shift 2
      ;;
    --session)
      SELECTED_SESSION="${2:?--session requires a directory}"
      shift 2
      ;;
    --force)
      FORCE=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ! -f "$CONFIG" ]]; then
  echo "Config not found: $CONFIG" >&2
  exit 1
fi

if [[ -f /workspaces/ros2_ws/install/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /workspaces/ros2_ws/install/setup.bash
  set -u
else
  echo "ROS workspace is not built: /workspaces/ros2_ws/install/setup.bash" >&2
  exit 1
fi

if [[ -n "${MULTI_SENSOR_CALIBRATION_VENV:-}" && -f "$MULTI_SENSOR_CALIBRATION_VENV/bin/activate" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "$MULTI_SENSOR_CALIBRATION_VENV/bin/activate"
  set -u
elif [[ -f /opt/multi_sensor_calibration_env/bin/activate ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/multi_sensor_calibration_env/bin/activate
  set -u
elif [[ -f /workspaces/.venvs/multi_sensor_calibration/bin/activate ]]; then
  set +u
  # shellcheck disable=SC1091
  source /workspaces/.venvs/multi_sensor_calibration/bin/activate
  set -u
fi

sessions=()
if [[ -n "$SELECTED_SESSION" ]]; then
  if [[ ! -d "$SELECTED_SESSION" ]]; then
    echo "Session directory not found: $SELECTED_SESSION" >&2
    exit 1
  fi
  selected_name="$(basename "$SELECTED_SESSION")"
  if [[ "$selected_name" == "analysis" || "$selected_name" == "$CALIBRATION_SESSION" || "$selected_name" == *"_calibration_"* ]]; then
    echo "--session must select a non-calibration recording: $SELECTED_SESSION" >&2
    exit 1
  fi
  sessions+=("$(cd -- "$SELECTED_SESSION" && pwd)")
else
  for root in "${ROOTS[@]}"; do
    if [[ ! -d "$root" ]]; then
      echo "[skip root] not found: $root"
      continue
    fi
    while IFS= read -r -d '' session; do
      name="$(basename "$session")"
      if [[ "$name" == "analysis" || "$name" == "$CALIBRATION_SESSION" || "$name" == *"_calibration_"* ]]; then
        continue
      fi
      sessions+=("$session")
    done < <(find "$root" -mindepth 1 -maxdepth 1 -type d -print0)
  done
fi

total=${#sessions[@]}
if ((total == 0)); then
  echo "No popup recording sessions were found." >&2
  exit 1
fi

echo "LED sync batch"
echo "  sessions : $total"
echo "  RGB ROI  : $RGB_ROI"
echo "  EVS ROI  : $EVS_ROI"
echo "  bin       : ${BIN_MS} ms"
echo "  preview   : ${PREVIEW_FPS} fps, first/last ${PREVIEW_WINDOW_S} s"
echo "  ROI tiles : ${ROI_TILE_SIZE} px"
echo "  calibration session skipped: $CALIBRATION_SESSION"

succeeded=0
skipped=0
failed=0
index=0

for session in "${sessions[@]}"; do
  index=$((index + 1))
  name="$(basename "$session")"
  root="$(dirname "$session")"
  output_dir="$root/analysis/led_sync/$name"
  output_json="$output_dir/led_sync_data.json"

  if [[ -s "$output_json" && "$FORCE" != true ]]; then
    echo "[$index/$total] skip existing: $name"
    skipped=$((skipped + 1))
    continue
  fi

  raw_file=""
  while IFS= read -r candidate; do
    raw_file="$candidate"
    break
  done < <(find "$session" -maxdepth 1 -type f -name '*.raw' | sort)

  if [[ -z "$raw_file" ]]; then
    echo "[$index/$total] FAILED (RAW not found): $name" >&2
    failed=$((failed + 1))
    continue
  fi
  if [[ ! -f "$raw_file.metadata.yaml" ]]; then
    echo "[$index/$total] FAILED (RAW metadata not found): $name" >&2
    failed=$((failed + 1))
    continue
  fi
  if ! find "$session" -maxdepth 1 -type f -name '*.mcap' -print -quit | grep -q .; then
    echo "[$index/$total] FAILED (MCAP not found): $name" >&2
    failed=$((failed + 1))
    continue
  fi

  echo "[$index/$total] processing: $name"
  mkdir -p "$output_dir"
  if ros2 run multi_sensor_calibration multi-sensor-calibration led-sync-export \
    --config "$CONFIG" \
    --bag "$session" \
    --evs-source metavision_file \
    --event-file "$raw_file" \
    --rgb-roi "$RGB_ROI" \
    --evs-roi "$EVS_ROI" \
    --bin-ms "$BIN_MS" \
    --preview-fps "$PREVIEW_FPS" \
    --preview-window-s "$PREVIEW_WINDOW_S" \
    --roi-tile-size "$ROI_TILE_SIZE" \
    --session-name "$name" \
    --output-dir "$output_dir" \
    >"$output_dir/export.log" 2>&1; then
    echo "[$index/$total] complete: $output_json"
    succeeded=$((succeeded + 1))
  else
    echo "[$index/$total] FAILED: $name (see $output_dir/export.log)" >&2
    failed=$((failed + 1))
  fi
done

echo
echo "Batch finished: succeeded=$succeeded skipped=$skipped failed=$failed total=$total"
if ((failed > 0)); then
  exit 1
fi
