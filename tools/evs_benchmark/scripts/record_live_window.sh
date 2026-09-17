#!/usr/bin/env bash
set -euo pipefail

if (($# < 2 || $# > 2)); then
  echo "Usage: $0 DURATION_SECONDS LABEL" >&2
  exit 2
fi

duration_s="$1"
label="$2"

[[ "$duration_s" =~ ^[1-9][0-9]*$ ]] || {
  echo "DURATION_SECONDS must be a positive integer" >&2
  exit 2
}
[[ "$label" =~ ^[A-Za-z0-9_-]+$ ]] || {
  echo "LABEL may contain only letters, digits, underscore, and hyphen" >&2
  exit 2
}
command -v ros2 >/dev/null 2>&1 || {
  echo "ros2 is not available; source the JetPilot ROS workspace first" >&2
  exit 1
}

recording=false

stop_recording() {
  if [[ "$recording" == true ]]; then
    ros2 topic pub --once /bag/request jetpilot_msgs/msg/BagRequest \
      "{command: 2, label: ${label}}" >/dev/null
    recording=false
  fi
}

trap stop_recording EXIT INT TERM

ros2 topic pub --once /bag/request jetpilot_msgs/msg/BagRequest \
  "{command: 1, label: ${label}}" >/dev/null
recording=true
echo "Recording ${label} for ${duration_s} s..."
sleep "$duration_s"
stop_recording
trap - EXIT INT TERM
echo "Finished. Session root: /workspaces/record"
