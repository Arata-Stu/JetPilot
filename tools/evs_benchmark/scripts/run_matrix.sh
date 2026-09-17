#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 3 ]]; then
  echo "Usage: $0 EVS_BENCH INPUT.evbin RESULTS_ROOT [extra evs_bench args...]" >&2
  exit 2
fi

bench_bin=$1
input_file=$2
results_root=$3
shift 3
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
timestamp=$(date -u +%Y%m%dT%H%M%SZ)
run_root="$results_root/$timestamp"
mkdir -p "$run_root"

"$script_dir/run_condition.sh" "$bench_bin" "$input_file" \
  "$run_root/cpu_full" cpu full "$@"
"$script_dir/run_condition.sh" "$bench_bin" "$input_file" \
  "$run_root/cpu_incremental" cpu incremental "$@"
if "$bench_bin" --check-cuda >/dev/null 2>&1; then
  has_cuda=true
  "$script_dir/run_condition.sh" "$bench_bin" "$input_file" \
    "$run_root/cuda_rolling" cuda rolling "$@"
else
  has_cuda=false
  echo "CUDA backend unavailable; cuda_rolling skipped" | tee "$run_root/cuda_skipped.txt"
fi

python3 "$script_dir/summarize.py" "$run_root"/*/results.csv \
  --output "$run_root/summary.csv"

verify_backend=cpu
if [[ "$has_cuda" == true ]]; then
  verify_backend=all
fi
"$bench_bin" --input "$input_file" --backend "$verify_backend" --algorithm all \
  "$@" --verify-sequence-only \
  --correctness-output "$run_root/correctness.csv"

"$bench_bin" --input "$input_file" --backend cpu --algorithm incremental \
  "$@" --warmup 2 --trace-only \
  --trace-output "$run_root/cpu_incremental/trace.csv" \
  --metadata "$run_root/cpu_incremental/trace_metadata.json"
if [[ "$has_cuda" == true ]]; then
  "$bench_bin" --input "$input_file" --backend cuda --algorithm rolling \
    "$@" --warmup 2 --trace-only \
    --trace-output "$run_root/cuda_rolling/trace.csv" \
    --metadata "$run_root/cuda_rolling/trace_metadata.json"
fi
trace_inputs=("$run_root/cpu_incremental/trace.csv")
if [[ "$has_cuda" == true ]]; then
  trace_inputs+=("$run_root/cuda_rolling/trace.csv")
fi
python3 "$script_dir/summarize_trace.py" "${trace_inputs[@]}" \
  --summary "$run_root/trace_summary.csv" \
  --svg "$run_root/trace_timeline.svg" \
  --deadline-ms 4.0
echo "$run_root"
