#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 3 ]]; then
  echo "Usage: $0 EVS_BENCH INPUT.evbin OUTPUT_DIR [extra evs_bench args...]" >&2
  exit 2
fi

bench_bin=$1
input_file=$2
output_dir=$3
shift 3
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
mkdir -p "$output_dir"

"$bench_bin" --input "$input_file" --backend cpu --algorithm incremental \
  "$@" --warmup 2 --trace-only \
  --trace-output "$output_dir/cpu_incremental_trace.csv" \
  --metadata "$output_dir/cpu_incremental_metadata.json"

trace_inputs=("$output_dir/cpu_incremental_trace.csv")
if "$bench_bin" --check-cuda >/dev/null 2>&1; then
  "$bench_bin" --input "$input_file" --backend cuda --algorithm rolling \
    "$@" --warmup 2 --trace-only \
    --trace-output "$output_dir/cuda_rolling_trace.csv" \
    --metadata "$output_dir/cuda_rolling_metadata.json"
  trace_inputs+=("$output_dir/cuda_rolling_trace.csv")
fi

python3 "$script_dir/summarize_trace.py" "${trace_inputs[@]}" \
  --summary "$output_dir/trace_summary.csv" \
  --svg "$output_dir/trace_timeline.svg" \
  --deadline-ms 4.0

echo "$output_dir"
