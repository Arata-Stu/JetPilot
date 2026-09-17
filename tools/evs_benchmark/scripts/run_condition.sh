#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 5 ]]; then
  echo "Usage: $0 EVS_BENCH INPUT.evbin OUTPUT_DIR BACKEND ALGORITHM [extra evs_bench args...]" >&2
  exit 2
fi

bench_bin=$1
input_file=$2
output_dir=$3
backend=$4
algorithm=$5
shift 5

mkdir -p "$output_dir"
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
"$script_dir/collect_system_info.sh" "$output_dir/system_info.txt"
if command -v sha256sum >/dev/null 2>&1; then
  sha256sum "$input_file" >"$output_dir/input.sha256"
elif command -v shasum >/dev/null 2>&1; then
  shasum -a 256 "$input_file" >"$output_dir/input.sha256"
fi
telemetry_pid=""
stop_telemetry() {
  if [[ -n "$telemetry_pid" ]] && kill -0 "$telemetry_pid" 2>/dev/null; then
    kill -TERM "$telemetry_pid" 2>/dev/null || true
    wait "$telemetry_pid" 2>/dev/null || true
  fi
}
trap stop_telemetry EXIT INT TERM

if command -v tegrastats >/dev/null 2>&1; then
  tegrastats --interval 100 --logfile "$output_dir/tegrastats.log" &
  telemetry_pid=$!
elif command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi \
    --query-gpu=timestamp,utilization.gpu,utilization.memory,power.draw,clocks.sm,temperature.gpu \
    --format=csv --loop-ms=100 >"$output_dir/nvidia_smi.csv" &
  telemetry_pid=$!
fi

printf '%q ' "$bench_bin" --input "$input_file" --backend "$backend" \
  --algorithm "$algorithm" --output "$output_dir/results.csv" \
  --metadata "$output_dir/metadata.json" "$@" >"$output_dir/command.txt"
printf '\n' >>"$output_dir/command.txt"

if command -v /usr/bin/time >/dev/null 2>&1; then
  /usr/bin/time -v -o "$output_dir/resource_usage.txt" \
    "$bench_bin" --input "$input_file" --backend "$backend" \
    --algorithm "$algorithm" --output "$output_dir/results.csv" \
    --metadata "$output_dir/metadata.json" "$@" \
    2>"$output_dir/stderr.log" | tee "$output_dir/stdout.log"
else
  "$bench_bin" --input "$input_file" --backend "$backend" \
    --algorithm "$algorithm" --output "$output_dir/results.csv" \
    --metadata "$output_dir/metadata.json" "$@" \
    2>"$output_dir/stderr.log" | tee "$output_dir/stdout.log"
fi

stop_telemetry
if [[ -s "$output_dir/tegrastats.log" ]]; then
  python3 "$script_dir/parse_tegrastats.py" "$output_dir/tegrastats.log" \
    --timeline "$output_dir/tegrastats_timeline.csv" \
    --summary "$output_dir/tegrastats_summary.json" \
    --interval-ms 100 \
    --metadata "$output_dir/metadata.json" \
    --results "$output_dir/results.csv"
elif [[ -s "$output_dir/nvidia_smi.csv" ]]; then
  python3 "$script_dir/summarize_nvidia_smi.py" "$output_dir/nvidia_smi.csv" \
    --summary "$output_dir/nvidia_smi_summary.json" \
    --interval-ms 100 \
    --metadata "$output_dir/metadata.json" \
    --results "$output_dir/results.csv"
fi
