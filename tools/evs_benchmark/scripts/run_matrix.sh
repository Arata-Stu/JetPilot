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
  "$script_dir/run_condition.sh" "$bench_bin" "$input_file" \
    "$run_root/cuda_rolling" cuda rolling "$@"
else
  echo "CUDA backend unavailable; cuda_rolling skipped" | tee "$run_root/cuda_skipped.txt"
fi

python3 "$script_dir/summarize.py" "$run_root"/*/results.csv \
  --output "$run_root/summary.csv"
echo "$run_root"
