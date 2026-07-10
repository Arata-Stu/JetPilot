#!/usr/bin/env bash
set -euo pipefail

OUT_DIR="${OUT_DIR:-record/rtp_video/stats_$(date +%Y%m%d_%H%M%S)}"
IFACE="${IFACE:-}"
INTERVAL="${INTERVAL:-1}"
DURATION="${DURATION:-60}"

mkdir -p "$OUT_DIR"

detect_iface() {
  ip route get 1.1.1.1 2>/dev/null | awk '{for (i=1; i<=NF; ++i) if ($i=="dev") {print $(i+1); exit}}'
}

IFACE="${IFACE:-$(detect_iface)}"
[[ -n "$IFACE" ]] || {
  echo "error: IFACE is empty and auto-detection failed" >&2
  exit 1
}

system_csv="${OUT_DIR}/system.csv"
net_csv="${OUT_DIR}/netdev_${IFACE}.csv"
sync_log="${OUT_DIR}/time_sync.txt"

{
  date --iso-8601=ns || date
  if command -v chronyc >/dev/null 2>&1; then
    chronyc tracking || true
    chronyc sources -v || true
  fi
} > "$sync_log" 2>&1

if command -v tegrastats >/dev/null 2>&1; then
  tegrastats --interval "$((INTERVAL * 1000))" > "${OUT_DIR}/tegrastats.log" &
  tegra_pid=$!
else
  tegra_pid=""
fi

if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi \
    --query-gpu=timestamp,utilization.gpu,utilization.memory,memory.used,memory.total,power.draw \
    --format=csv \
    -l "$INTERVAL" > "${OUT_DIR}/nvidia_smi.csv" &
  nvidia_smi_pid=$!
else
  nvidia_smi_pid=""
fi

cleanup() {
  [[ -n "${tegra_pid:-}" ]] && kill "$tegra_pid" >/dev/null 2>&1 || true
  [[ -n "${nvidia_smi_pid:-}" ]] && kill "$nvidia_smi_pid" >/dev/null 2>&1 || true
}
trap cleanup EXIT

read_cpu() {
  awk '/^cpu / {print $2, $3, $4, $5, $6, $7, $8}' /proc/stat
}

cpu_busy_percent() {
  local prev=($1)
  local curr=($2)
  local prev_idle=$((prev[3] + prev[4]))
  local curr_idle=$((curr[3] + curr[4]))
  local prev_total=0
  local curr_total=0
  local value

  for value in "${prev[@]}"; do
    prev_total=$((prev_total + value))
  done
  for value in "${curr[@]}"; do
    curr_total=$((curr_total + value))
  done

  local total_delta=$((curr_total - prev_total))
  local idle_delta=$((curr_idle - prev_idle))
  if (( total_delta <= 0 )); then
    printf '0.00'
  else
    awk -v total="$total_delta" -v idle="$idle_delta" 'BEGIN {printf "%.2f", 100.0 * (total - idle) / total}'
  fi
}

mem_available_kb() {
  awk '/MemAvailable:/ {print $2}' /proc/meminfo
}

mem_total_kb() {
  awk '/MemTotal:/ {print $2}' /proc/meminfo
}

net_value() {
  local name="$1"
  cat "/sys/class/net/${IFACE}/statistics/${name}"
}

echo "wall_ns,mono_ns,cpu_busy_percent,mem_available_kb,mem_total_kb,load1,load5,load15" > "$system_csv"
echo "wall_ns,mono_ns,rx_bytes,tx_bytes,rx_packets,tx_packets,rx_dropped,tx_dropped,rx_errors,tx_errors" > "$net_csv"

prev_cpu="$(read_cpu)"
end_time=$((SECONDS + DURATION))

while (( SECONDS < end_time )); do
  sleep "$INTERVAL"
  curr_cpu="$(read_cpu)"
  wall_ns="$(date +%s%N)"
  mono_ns="$(python3 - <<'PY'
import time
print(time.monotonic_ns())
PY
)"
  read -r load1 load5 load15 _ < /proc/loadavg

  echo "${wall_ns},${mono_ns},$(cpu_busy_percent "$prev_cpu" "$curr_cpu"),$(mem_available_kb),$(mem_total_kb),${load1},${load5},${load15}" >> "$system_csv"
  echo "${wall_ns},${mono_ns},$(net_value rx_bytes),$(net_value tx_bytes),$(net_value rx_packets),$(net_value tx_packets),$(net_value rx_dropped),$(net_value tx_dropped),$(net_value rx_errors),$(net_value tx_errors)" >> "$net_csv"

  prev_cpu="$curr_cpu"
done

echo "Wrote stats to $OUT_DIR"

