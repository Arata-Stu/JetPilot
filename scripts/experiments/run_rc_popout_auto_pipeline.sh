#!/usr/bin/env bash
set -euo pipefail

CONFIG="/workspaces/ros2_ws/src/tool/multi_sensor_calibration/config/rc_popout_evs_rgb.yaml"
CAMCHAIN="/workspaces/ros2_ws/src/tool/multi_sensor_calibration/config/calibrations/rc_popout_default/kalibr-camchain.yaml"
RGB_ROI="0,0,848,480"
EVS_ROI="0,0,640,480"
BIN_MS="1"
PREVIEW_FPS="60"
PREVIEW_WINDOW_S="12"
ROI_TILE_SIZE="16"
EVENT_WINDOW_MS="10"
EVENT_DILATE_PX="2"
SELECTED_SESSION=""
FORCE_EXPORT=false
FORCE_SYNC=false
FORCE_VIDEO=false
NO_VIDEO=false
CALIBRATION_SESSION="20260921_185407_rcp_calibration_checkerboard_20260921_185405"
ROOTS=(
  "/workspaces/record/evs-popup-v1"
  "/workspaces/record/evs-popup-v2"
)
SUMMARY_DIR="/workspaces/record/evs-popup-analysis/auto_pipeline"

usage() {
  cat <<'EOF'
Usage: scripts/experiments/run_rc_popout_auto_pipeline.sh [options]

Runs the complete unattended pipeline for every non-calibration recording:
  1. export start/end spatial LED data when missing
  2. automatically find RGB/EVS LED ROIs and estimate time synchronization
  3. render the full DSEC-style RGB/event comparison videos
  4. create ROI debug sheets and a portable visual review page

Options:
  --session DIR          Process one recording instead of all v1/v2 sessions
  --no-video             Stop after automatic ROI and time synchronization
  --force-export         Recreate led_sync_data.json and roi_data
  --force-sync           Recompute automatic ROI and time synchronization
  --force-video          Preserve the old video directory as a timestamped backup,
                         then regenerate it
  --event-window-ms MS   Event accumulation for videos (default: 10)
  --event-dilate-px PX   Event point thickness (default: 2)
  --summary-dir DIR      Batch TSV/log destination
  --config PATH          LED export configuration
  --camchain PATH        Default EVS/RGB Kalibr camchain
  -h, --help             Show this help

The script is resumable: completed stages are skipped unless their force option
is supplied. Low-confidence ROI results are still rendered and marked REVIEW in
the summary so they can be checked visually in the UI.
EOF
}

while (($#)); do
  case "$1" in
    --session) SELECTED_SESSION="${2:?--session requires a directory}"; shift 2 ;;
    --no-video) NO_VIDEO=true; shift ;;
    --force-export) FORCE_EXPORT=true; shift ;;
    --force-sync) FORCE_SYNC=true; shift ;;
    --force-video) FORCE_VIDEO=true; shift ;;
    --event-window-ms) EVENT_WINDOW_MS="${2:?--event-window-ms requires a value}"; shift 2 ;;
    --event-dilate-px) EVENT_DILATE_PX="${2:?--event-dilate-px requires a value}"; shift 2 ;;
    --summary-dir) SUMMARY_DIR="${2:?--summary-dir requires a directory}"; shift 2 ;;
    --config) CONFIG="${2:?--config requires a path}"; shift 2 ;;
    --camchain) CAMCHAIN="${2:?--camchain requires a path}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

for required in "$CONFIG" "$CAMCHAIN"; do
  if [[ ! -f "$required" ]]; then
    echo "Required file not found: $required" >&2
    exit 1
  fi
done

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
  if [[ "$selected_name" == "$CALIBRATION_SESSION" || "$selected_name" == *"_calibration_"* ]]; then
    echo "--session must select a non-calibration recording: $SELECTED_SESSION" >&2
    exit 1
  fi
  sessions+=("$(cd -- "$SELECTED_SESSION" && pwd)")
else
  for root in "${ROOTS[@]}"; do
    [[ -d "$root" ]] || continue
    while IFS= read -r -d '' session; do
      name="$(basename "$session")"
      if [[ "$name" == "analysis" || "$name" == "$CALIBRATION_SESSION" || "$name" == *"_calibration_"* ]]; then
        continue
      fi
      sessions+=("$session")
    done < <(find "$root" -mindepth 1 -maxdepth 1 -type d -print0 | sort -z)
  done
fi

if ((${#sessions[@]} == 0)); then
  echo "No popup recording sessions were found." >&2
  exit 1
fi

mkdir -p "$SUMMARY_DIR"
SUMMARY_TSV="$SUMMARY_DIR/summary.tsv"
printf 'session\tstatus\treview\tconfidence\tmatched_edges\toffset_ms\tdrift_ppm\trms_ms\tauto_result\ttime_sync\tvideo\treview_page\tmessage\n' >"$SUMMARY_TSV"

record_summary() {
  local session_name="$1" status="$2" review="$3" confidence="$4" matched="$5"
  local offset_ms="$6" drift_ppm="$7" rms_ms="$8" result_path="$9"
  local sync_path="${10}" video_path="${11}" review_path="${12}" message="${13}"
  message="${message//$'\t'/ }"
  message="${message//$'\n'/ }"
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$session_name" "$status" "$review" "$confidence" "$matched" \
    "$offset_ms" "$drift_ppm" "$rms_ms" "$result_path" "$sync_path" \
    "$video_path" "$review_path" "$message" >>"$SUMMARY_TSV"
}

echo "RC popout automatic pipeline"
echo "  sessions : ${#sessions[@]}"
echo "  summary  : $SUMMARY_TSV"
echo "  video    : $([[ "$NO_VIDEO" == true ]] && echo disabled || echo enabled)"

succeeded=0
failed=0
review_count=0
index=0

for session in "${sessions[@]}"; do
  index=$((index + 1))
  name="$(basename "$session")"
  root="$(dirname "$session")"
  sync_dir="$root/analysis/led_sync/$name"
  data_json="$sync_dir/led_sync_data.json"
  auto_json="$sync_dir/auto_led_sync_result.json"
  time_sync="$sync_dir/time_sync_led_auto.yaml"
  video_dir="$root/analysis/scenario_overlay/$name/rotation_only_auto"
  video_file="$video_dir/rgb_vs_overlay.mp4"
  review_dir="$sync_dir/review"
  review_html="$review_dir/index.html"
  raw_file=""
  while IFS= read -r candidate; do raw_file="$candidate"; break; done < <(find "$session" -maxdepth 1 -type f -name '*.raw' | sort)

  echo "[$index/${#sessions[@]}] $name"
  if [[ -z "$raw_file" || ! -f "$raw_file.metadata.yaml" ]]; then
    echo "  FAILED: RAW or sidecar metadata not found" >&2
    record_summary "$name" FAILED yes unknown 0 0 0 0 "$auto_json" "$time_sync" "$video_file" "$review_html" "RAW or sidecar metadata not found"
    failed=$((failed + 1))
    continue
  fi
  if ! find "$session" -maxdepth 1 -type f -name '*.mcap' -print -quit | grep -q .; then
    echo "  FAILED: MCAP not found" >&2
    record_summary "$name" FAILED yes unknown 0 0 0 0 "$auto_json" "$time_sync" "$video_file" "$review_html" "MCAP not found"
    failed=$((failed + 1))
    continue
  fi

  needs_export=false
  if [[ "$FORCE_EXPORT" == true || ! -s "$data_json" ]]; then
    needs_export=true
  elif ! grep -q '"roi_data"' "$data_json"; then
    needs_export=true
  fi
  if [[ "$needs_export" == true ]]; then
    echo "  1/4 exporting LED previews and spatial tiles"
    mkdir -p "$sync_dir"
    if ! ros2 run multi_sensor_calibration multi-sensor-calibration led-sync-export \
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
      --output-dir "$sync_dir" >"$sync_dir/export.log" 2>&1; then
      echo "  FAILED: LED export (see $sync_dir/export.log)" >&2
      record_summary "$name" FAILED yes unknown 0 0 0 0 "$auto_json" "$time_sync" "$video_file" "$review_html" "LED export failed"
      failed=$((failed + 1))
      continue
    fi
  else
    echo "  1/4 spatial LED data already exists"
  fi

  if [[ "$FORCE_SYNC" == true || ! -s "$auto_json" || ! -s "$time_sync" || "$data_json" -nt "$auto_json" || "$data_json" -nt "$time_sync" ]]; then
    echo "  2/4 detecting ROIs and estimating LED time synchronization"
    if ! ros2 run multi_sensor_calibration multi-sensor-calibration auto-led-sync \
      --data-json "$data_json" \
      --output-yaml "$time_sync" \
      --output-json "$auto_json" >"$sync_dir/auto_led_sync.log" 2>&1; then
      echo "  FAILED: automatic LED sync (see $sync_dir/auto_led_sync.log)" >&2
      record_summary "$name" FAILED yes unknown 0 0 0 0 "$auto_json" "$time_sync" "$video_file" "$review_html" "automatic LED sync failed"
      failed=$((failed + 1))
      continue
    fi
  else
    echo "  2/4 automatic LED sync already exists"
  fi

  result_fields="$(python3 - "$auto_json" <<'PY'
import json
import sys
value = json.load(open(sys.argv[1], encoding="utf-8"))
clock = value["clock"]
print(
    value.get("overall_confidence", "unknown"),
    value.get("overall_localization_confidence", "unknown"),
    clock.get("matched_edges", 0),
    clock.get("offset_at_anchor_s", 0.0) * 1000.0,
    clock.get("drift_ppm", 0.0),
    clock.get("residual_rms_s", 0.0) * 1000.0,
)
PY
)"
  read -r confidence localization_confidence matched offset_ms drift_ppm rms_ms <<<"$result_fields"
  review=no
  if [[ "$confidence" != high || "$localization_confidence" != high ]]; then
    review=yes
    review_count=$((review_count + 1))
  fi

  needs_video=false
  if [[ "$FORCE_VIDEO" == true || ! -s "$video_file" || "$time_sync" -nt "$video_file" ]]; then
    needs_video=true
  fi
  if [[ "$NO_VIDEO" == true ]]; then
    echo "  3/4 video disabled"
  elif [[ "$needs_video" != true ]]; then
    echo "  3/4 video already exists"
  else
    if [[ -d "$video_dir" ]]; then
      backup="${video_dir}.before_$(date +%Y%m%d_%H%M%S)"
      mv -- "$video_dir" "$backup"
      echo "  previous video preserved: $backup"
    fi
    echo "  3/4 rendering full RGB/event videos"
    mkdir -p "$(dirname "$video_dir")"
    if ! ros2 run multi_sensor_calibration multi-sensor-calibration scenario-overlay \
      --bag "$session" \
      --event-file "$raw_file" \
      --time-sync "$time_sync" \
      --camchain "$CAMCHAIN" \
      --projection rotation-only \
      --event-window-ms "$EVENT_WINDOW_MS" \
      --event-dilate-px "$EVENT_DILATE_PX" \
      --alpha 0.85 \
      --output-dir "$video_dir" >"$(dirname "$video_dir")/rotation_only_auto.log" 2>&1; then
      echo "  FAILED: video rendering" >&2
      record_summary "$name" FAILED "$review" "$confidence" "$matched" "$offset_ms" "$drift_ppm" "$rms_ms" "$auto_json" "$time_sync" "$video_file" "$review_html" "video rendering failed"
      failed=$((failed + 1))
      continue
    fi
  fi

  needs_review=false
  if [[ ! -s "$review_html" || "$auto_json" -nt "$review_html" || "$data_json" -nt "$review_html" ]]; then
    needs_review=true
  elif [[ -s "$video_file" && "$video_file" -nt "$review_html" ]]; then
    needs_review=true
  fi
  if [[ "$needs_review" == true ]]; then
    echo "  4/4 creating ROI debug sheets and visual review page"
    review_command=(
      ros2 run multi_sensor_calibration multi-sensor-calibration led-sync-review
      --data-json "$data_json"
      --auto-result "$auto_json"
      --output-dir "$review_dir"
    )
    if [[ -d "$video_dir" ]]; then
      review_command+=(--video-dir "$video_dir")
    fi
    if ! "${review_command[@]}" >"$sync_dir/review.log" 2>&1; then
      echo "  FAILED: visual review artifacts (see $sync_dir/review.log)" >&2
      record_summary "$name" FAILED "$review" "$confidence" "$matched" "$offset_ms" "$drift_ppm" "$rms_ms" "$auto_json" "$time_sync" "$video_file" "$review_html" "visual review artifact generation failed"
      failed=$((failed + 1))
      continue
    fi
  else
    echo "  4/4 visual review artifacts already exist"
  fi

  echo "  complete: confidence=$confidence offset=${offset_ms}ms rms=${rms_ms}ms review=$review"
  record_summary "$name" COMPLETE "$review" "$confidence" "$matched" "$offset_ms" "$drift_ppm" "$rms_ms" "$auto_json" "$time_sync" "$video_file" "$review_html" ""
  succeeded=$((succeeded + 1))
done

echo
echo "Finished: complete=$succeeded failed=$failed review=$review_count total=${#sessions[@]}"
echo "Summary: $SUMMARY_TSV"
if ((failed > 0)); then
  exit 1
fi
