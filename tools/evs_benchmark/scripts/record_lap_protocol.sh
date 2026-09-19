#!/usr/bin/env bash
set -euo pipefail

if (($# > 6)); then
  echo "Usage: $0 [LAPS=3] [REPEATS=3] [LABEL_PREFIX=course] [NORMAL_THROTTLE LOW_THROTTLE HIGH_THROTTLE]" >&2
  exit 2
fi

laps="${1:-3}"
repeats="${2:-3}"
label_prefix="${3:-course}"
normal_throttle="${4:-}"
low_throttle="${5:-}"
high_throttle="${6:-}"

[[ "$laps" =~ ^[1-9][0-9]*$ ]] || {
  echo "LAPS must be a positive integer" >&2
  exit 2
}
[[ "$repeats" =~ ^[1-9][0-9]*$ ]] || {
  echo "REPEATS must be a positive integer" >&2
  exit 2
}
[[ "$label_prefix" =~ ^[A-Za-z0-9_-]+$ ]] || {
  echo "LABEL_PREFIX may contain only letters, digits, underscore, and hyphen" >&2
  exit 2
}

throttle_count=0
[[ -z "$normal_throttle" ]] || throttle_count=$((throttle_count + 1))
[[ -z "$low_throttle" ]] || throttle_count=$((throttle_count + 1))
[[ -z "$high_throttle" ]] || throttle_count=$((throttle_count + 1))
if ((throttle_count != 0 && throttle_count != 3)); then
  echo "Specify all three throttle values, or omit all three" >&2
  exit 2
fi
for throttle in "$normal_throttle" "$low_throttle" "$high_throttle"; do
  [[ -z "$throttle" || "$throttle" =~ ^(0([.][0-9]+)?|1([.]0+)?)$ ]] || {
    echo "Throttle values must be decimal numbers within [0, 1]" >&2
    exit 2
  }
done
command -v ros2 >/dev/null 2>&1 || {
  echo "ros2 is not available; source the JetPilot ROS workspace first" >&2
  exit 1
}

record_root="${EVS_RECORD_ROOT:-/workspaces/record}"
[[ -d "$record_root" && -w "$record_root" ]] || {
  echo "Record root is not a writable directory: $record_root" >&2
  exit 1
}

conditions=(normal low high)
total_runs=$((${#conditions[@]} * repeats))
protocol_timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
protocol_csv="${record_root}/${protocol_timestamp}_${label_prefix}_lap_protocol.csv"
printf '%s\n' \
  'run_index,condition,repetition,laps,fixed_throttle,label,start_utc,end_utc,duration_s' > "$protocol_csv"

if ((throttle_count == 3)); then
  fixed_mode="$(ros2 param get /teleop_cmd_node fixed_throttle_mode 2>/dev/null)" || {
    echo "Cannot read /teleop_cmd_node; launch the fixed-throttle drive stack first" >&2
    exit 1
  }
  case "$fixed_mode" in
    *[Tt][Rr][Uu][Ee]*) ;;
    *)
      echo "/teleop_cmd_node fixed_throttle_mode is not true" >&2
      exit 1
      ;;
  esac
fi

recording=false
active_label=""

stop_recording() {
  if [[ "$recording" == true ]]; then
    ros2 topic pub --once /bag/request jetpilot_msgs/msg/BagRequest \
      "{command: 2, label: ${active_label}}" >/dev/null
    recording=false
  fi
}

handle_signal() {
  stop_recording
  echo
  echo "Protocol interrupted. Completed-run log: ${protocol_csv}" >&2
  exit 130
}

trap stop_recording EXIT
trap handle_signal INT TERM HUP

echo "Lap protocol: ${#conditions[@]} conditions x ${repeats} repeats = ${total_runs} sessions"
echo "Each session: ${laps} laps"
echo "Protocol log: ${protocol_csv}"
if ((throttle_count == 3)); then
  echo "Fixed throttle: normal=${normal_throttle}, low=${low_throttle}, high=${high_throttle}"
fi

run_index=0
for ((repetition = 1; repetition <= repeats; ++repetition)); do
  # Rotate condition order between repeats so battery, temperature, and time-of-day
  # drift do not always coincide with the same speed condition.
  for ((condition_offset = 0; condition_offset < ${#conditions[@]}; ++condition_offset)); do
    condition_index=$(((repetition - 1 + condition_offset) % ${#conditions[@]}))
    condition="${conditions[$condition_index]}"
    fixed_throttle=""
    case "$condition" in
      normal) fixed_throttle="$normal_throttle" ;;
      low) fixed_throttle="$low_throttle" ;;
      high) fixed_throttle="$high_throttle" ;;
    esac
    run_index=$((run_index + 1))
    repetition_label="$(printf '%02d' "$repetition")"
    active_label="${label_prefix}_${condition}_${laps}lap_${repetition_label}"

    echo
    echo "[${run_index}/${total_runs}] condition=${condition}, repetition=${repetition}/${repeats}"
    echo "Label: ${active_label}"
    if [[ -n "$fixed_throttle" ]]; then
      read -r -p "Confirm the vehicle is in STOP mode, then press Enter to apply fixed throttle ${fixed_throttle}... "
      ros2 param set /teleop_cmd_node fixed_throttle "$fixed_throttle" >/dev/null
      echo "Applied fixed throttle: $(ros2 param get /teleop_cmd_node fixed_throttle)"
    fi
    read -r -p "Place the car at the start position, then press Enter to start recording... "

    start_utc="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    start_epoch="$(date +%s)"
    ros2 topic pub --once /bag/request jetpilot_msgs/msg/BagRequest \
      "{command: 1, label: ${active_label}}" >/dev/null
    recording=true

    echo "Recording. Complete exactly ${laps} laps, stop the car safely, then return here."
    read -r -p "Press Enter to stop recording... "
    stop_recording

    end_epoch="$(date +%s)"
    end_utc="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    duration_s=$((end_epoch - start_epoch))
    printf '%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
      "$run_index" "$condition" "$repetition" "$laps" "$fixed_throttle" "$active_label" \
      "$start_utc" "$end_utc" "$duration_s" >> "$protocol_csv"
    echo "Finished ${active_label}: ${duration_s} s"
done
done

trap - EXIT INT TERM HUP
echo
echo "All ${total_runs} sessions finished."
echo "Session root: ${record_root}"
echo "Protocol log: ${protocol_csv}"
