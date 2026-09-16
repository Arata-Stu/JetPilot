#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 3 ]]; then
  echo "Usage: $0 RGB_BENCH INPUT.rgbbin RESULTS_ROOT [extra rgb_bench args...]" >&2
  exit 2
fi
bench_bin=$1
input_file=$2
results_root=$3
shift 3
timestamp=$(date -u +%Y%m%dT%H%M%SZ)
run_root="$results_root/$timestamp"
mkdir -p "$run_root"
telemetry_pid=""
cleanup() {
  if [[ -n "$telemetry_pid" ]] && kill -0 "$telemetry_pid" 2>/dev/null; then
    kill -TERM "$telemetry_pid" 2>/dev/null || true
    wait "$telemetry_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

run_one() {
  local backend=$1
  shift
  local output_dir="$run_root/$backend"
  mkdir -p "$output_dir"
  telemetry_pid=""
  if command -v tegrastats >/dev/null 2>&1; then
    tegrastats --interval 100 --logfile "$output_dir/tegrastats.log" &
    telemetry_pid=$!
  elif command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi --query-gpu=timestamp,utilization.gpu,utilization.memory,power.draw,clocks.sm,temperature.gpu \
      --format=csv --loop-ms=100 >"$output_dir/nvidia_smi.csv" &
    telemetry_pid=$!
  fi
  printf '%q ' "$bench_bin" --input "$input_file" --backend "$backend" \
    --output "$output_dir/results.csv" "$@" >"$output_dir/command.txt"
  printf '\n' >>"$output_dir/command.txt"
  if command -v /usr/bin/time >/dev/null 2>&1; then
    /usr/bin/time -v -o "$output_dir/resource_usage.txt" "$bench_bin" \
      --input "$input_file" --backend "$backend" --output "$output_dir/results.csv" "$@" \
      2>"$output_dir/stderr.log" | tee "$output_dir/stdout.log"
  else
    "$bench_bin" --input "$input_file" --backend "$backend" \
      --output "$output_dir/results.csv" "$@" 2>"$output_dir/stderr.log" | tee "$output_dir/stdout.log"
  fi
  if [[ -n "$telemetry_pid" ]] && kill -0 "$telemetry_pid" 2>/dev/null; then
    kill -TERM "$telemetry_pid" 2>/dev/null || true
    wait "$telemetry_pid" 2>/dev/null || true
  fi
  telemetry_pid=""
  if command -v sha256sum >/dev/null 2>&1; then sha256sum "$input_file" >"$output_dir/input.sha256"; fi
  "$(dirname "$0")/../../evs_benchmark/scripts/collect_system_info.sh" "$output_dir/system_info.txt"
}

run_one cpu "$@"
if "$bench_bin" --check-cuda >/dev/null 2>&1; then run_one cuda "$@"; fi
echo "$run_root"
