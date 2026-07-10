#!/usr/bin/env bash
set -euo pipefail

SAMPLE_SECONDS="${SAMPLE_SECONDS:-3}"
INCLUDE_HIDDEN="${INCLUDE_HIDDEN:-0}"
OUTPUT_FILE="${OUTPUT_FILE:-}"

usage() {
  cat <<'EOF'
Usage:
  scripts/profile_topic_bw.sh [options]

Options:
  -s, --sample-seconds SEC   Seconds to sample each topic with ros2 topic bw.
                             Default: SAMPLE_SECONDS or 3.
  -o, --output FILE          Write CSV output to FILE.
  --include-hidden           Include hidden ROS topics.
  -h, --help                 Show this help.

Environment:
  SAMPLE_SECONDS
  INCLUDE_HIDDEN=1
  OUTPUT_FILE

Output columns:
  bytes_per_sec,topic,type,human_bandwidth
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -s|--sample-seconds)
      SAMPLE_SECONDS="$2"
      shift 2
      ;;
    -o|--output)
      OUTPUT_FILE="$2"
      shift 2
      ;;
    --include-hidden)
      INCLUDE_HIDDEN=1
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

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 command not found. Source your ROS 2 environment first." >&2
  exit 1
fi

if ! command -v timeout >/dev/null 2>&1; then
  echo "timeout command not found. Install GNU coreutils or run inside the ROS container." >&2
  exit 1
fi

topic_list_args=()
if [[ "${INCLUDE_HIDDEN}" == "1" ]]; then
  topic_list_args+=(--include-hidden-topics)
fi

tmp_dir="$(mktemp -d)"
trap 'rm -rf "${tmp_dir}"' EXIT
raw_csv="${tmp_dir}/topic_bw.csv"
printf 'bytes_per_sec,topic,type,human_bandwidth\n' > "${raw_csv}"

mapfile -t topics < <(ros2 topic list "${topic_list_args[@]}" | sort)

if [[ ${#topics[@]} -eq 0 ]]; then
  echo "No topics found." >&2
  exit 0
fi

echo "Profiling ${#topics[@]} topics. sample=${SAMPLE_SECONDS}s/topic" >&2

for topic in "${topics[@]}"; do
  topic_type="$(ros2 topic info "${topic}" 2>/dev/null | awk -F': ' '/Type:/ {print $2; exit}')"
  topic_type="${topic_type:-unknown}"

  echo "Measuring ${topic} (${topic_type})" >&2
  output="$(
    timeout "${SAMPLE_SECONDS}" ros2 topic bw "${topic}" 2>/dev/null || true
  )"
  avg_line="$(printf '%s\n' "${output}" | awk '/average:/ {line=$0} END {print line}')"

  if [[ -z "${avg_line}" ]]; then
    bytes_per_sec=0
    human_bandwidth="no_data"
  else
    human_bandwidth="$(printf '%s\n' "${avg_line}" | sed -E 's/.*average:[[:space:]]*//')"
    bytes_per_sec="$(
      printf '%s\n' "${human_bandwidth}" |
        awk '
          {
            value=$1
            unit=$2
            gsub(/[^A-Za-z]/, "", unit)
            mult=1
            if (unit ~ /^KB/) mult=1024
            else if (unit ~ /^MB/) mult=1024*1024
            else if (unit ~ /^GB/) mult=1024*1024*1024
            printf "%.0f", value * mult
          }'
    )"
  fi

  printf '%s,"%s","%s","%s"\n' \
    "${bytes_per_sec}" "${topic}" "${topic_type}" "${human_bandwidth}" >> "${raw_csv}"
done

sorted_csv="${tmp_dir}/topic_bw_sorted.csv"
{
  head -n 1 "${raw_csv}"
  tail -n +2 "${raw_csv}" | sort -t, -k1,1nr
} > "${sorted_csv}"

if [[ -n "${OUTPUT_FILE}" ]]; then
  cp "${sorted_csv}" "${OUTPUT_FILE}"
  echo "Wrote ${OUTPUT_FILE}" >&2
fi

cat "${sorted_csv}"
