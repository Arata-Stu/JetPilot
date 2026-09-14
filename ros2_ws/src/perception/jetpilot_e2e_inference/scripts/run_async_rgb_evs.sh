#!/usr/bin/env bash
set -euo pipefail

MODEL_ROOT="${1:-}"
if [[ -z "$MODEL_ROOT" || ! -f "${MODEL_ROOT}/metadata.json" ]]; then
  echo "Usage: run_async_rgb_evs.sh MODEL_ROOT [additional ros2 launch arguments...]" >&2
  exit 1
fi
shift

mapfile -t MODEL_CONFIG < <(python3 - "$MODEL_ROOT/metadata.json" <<'PY'
import json
import sys

metadata = json.load(open(sys.argv[1], encoding="utf-8"))
if metadata.get("modality") != "rgb_event_async":
    raise SystemExit("metadata modality must be rgb_event_async")
event = metadata.get("event_representation", {})
inputs = metadata.get("inputs", [])
rgb_input = next((item for item in inputs if item.get("name") == "rgb"), {})
event_input = next((item for item in inputs if item.get("name") == "event_tensors"), {})
config = metadata.get("config", {}).get("data", {})
window_ms = float(event.get("window_ms", 40.0))
stride_ms = float(event.get("stride_ms", 4.0))
bins = int(event.get("bins", 10))
interpolation = str(event.get("temporal_interpolation", "none"))
window_us = round(window_ms * 1000.0)
stride_us = round(stride_ms * 1000.0)
bin_width_us = window_us // bins if bins > 0 and window_us % bins == 0 else 0
cuda_compatible = (
    interpolation == "none"
    and bin_width_us > 0
    and stride_us <= window_us
    and stride_us % bin_width_us == 0
)
values = (
    metadata.get("image_topic", "/realsense/color/image_raw"),
    event.get("event_topic", "/event_camera/events"),
    str(bins),
    str(window_ms),
    str(stride_ms),
    event.get("polarity_mode", "separate"),
    event.get("polarity_layout", "polarity_major"),
    interpolation,
    json.dumps(event_input.get("mean", config.get("event_mean", [0.0])), separators=(",", ":")),
    json.dumps(event_input.get("std", config.get("event_std", [1.0])), separators=(",", ":")),
    json.dumps(rgb_input.get("mean", config.get("mean", [0.485, 0.456, 0.406])), separators=(",", ":")),
    json.dumps(rgb_input.get("std", config.get("std", [0.229, 0.224, 0.225])), separators=(",", ":")),
    str(config.get("input_width", 212)),
    str(config.get("input_height", 120)),
    str(config.get("event_sample_hz", 100.0)),
    "cuda" if cuda_compatible else "cpu",
)
print("\n".join(str(value) for value in values))
PY
)

if [[ "${#MODEL_CONFIG[@]}" -ne 16 ]]; then
  echo "Failed to read async RGB-EVS metadata" >&2
  exit 1
fi

exec ros2 launch jetpilot_e2e_inference async_rgb_evs_latent.launch.py \
  "model_root:=${MODEL_ROOT}" \
  "rgb_image_topic:=${MODEL_CONFIG[0]}" \
  "event_topic:=${MODEL_CONFIG[1]}" \
  "event_bins:=${MODEL_CONFIG[2]}" \
  "event_window_ms:=${MODEL_CONFIG[3]}" \
  "event_stride_ms:=${MODEL_CONFIG[4]}" \
  "event_polarity_mode:=${MODEL_CONFIG[5]}" \
  "event_polarity_layout:=${MODEL_CONFIG[6]}" \
  "event_temporal_interpolation:=${MODEL_CONFIG[7]}" \
  "event_mean:=${MODEL_CONFIG[8]}" \
  "event_stddev:=${MODEL_CONFIG[9]}" \
  "rgb_mean:=${MODEL_CONFIG[10]}" \
  "rgb_stddev:=${MODEL_CONFIG[11]}" \
  "network_width:=${MODEL_CONFIG[12]}" \
  "network_height:=${MODEL_CONFIG[13]}" \
  "event_output_rate_hz:=${MODEL_CONFIG[14]}" \
  "event_representation_backend:=${MODEL_CONFIG[15]}" \
  "$@"
