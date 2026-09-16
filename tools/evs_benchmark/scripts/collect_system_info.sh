#!/usr/bin/env bash
set -u

output=${1:-system_info.txt}
{
  echo "utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "kernel=$(uname -a)"
  echo "git_commit=$(git rev-parse HEAD 2>/dev/null || echo unavailable)"
  echo "git_dirty_files=$(git status --porcelain 2>/dev/null | wc -l | tr -d ' ')"
  echo "compiler=$(${CXX:-c++} --version 2>/dev/null | head -1 || echo unavailable)"
  echo "cmake=$(cmake --version 2>/dev/null | head -1 || echo unavailable)"
  echo "nvcc=$(nvcc --version 2>/dev/null | tail -1 || echo unavailable)"
  echo
  echo "[lscpu]"
  lscpu 2>/dev/null || true
  echo
  echo "[memory]"
  free -h 2>/dev/null || true
  echo
  echo "[nvidia-smi]"
  nvidia-smi -q 2>/dev/null || true
  echo
  echo "[jetson]"
  cat /etc/nv_tegra_release 2>/dev/null || true
  nvpmodel -q 2>/dev/null || true
  jetson_clocks --show 2>/dev/null || true
} >"$output"
