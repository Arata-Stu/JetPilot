#!/usr/bin/env bash
set -euo pipefail

readonly EXPECTED_POWER_MODE=2

usage() {
  cat <<'EOF'
Usage:
  ./scripts/jetson_max_performance.sh
  ./scripts/jetson_max_performance.sh --check

MAXN_SUPER (mode 2) が選択済みであることを確認し、CPU・GPU・EMCを
最大クロックへ固定してファンを最大化します。

Options:
  --check  設定を変更せず、現在の電力モード、クロック、ファンを検証します。
  -h, --help
           このヘルプを表示します。

このスクリプトはnvpmodelのモードを変更しません。mode 2でない場合は、先に
`sudo nvpmodel -m 2`を実行し、要求された場合は再起動してください。
EOF
}

die() {
  echo "Error: $*" >&2
  exit 1
}

run_as_root() {
  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  else
    command -v sudo >/dev/null 2>&1 || die "sudoが見つかりません。rootで実行してください。"
    sudo "$@"
  fi
}

mode=apply
if (($# > 1)); then
  usage >&2
  exit 2
fi
if (($# == 1)); then
  case "$1" in
    --check) mode=check ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; die "不明なオプションです: $1" ;;
  esac
fi

command -v nvpmodel >/dev/null 2>&1 || die "nvpmodelが見つかりません。Jetson上で実行してください。"
command -v jetson_clocks >/dev/null 2>&1 || die "jetson_clocksが見つかりません。Jetson上で実行してください。"

power_status="$(run_as_root nvpmodel -q 2>&1)"
echo "$power_status"
power_mode="$(awk '/^[[:space:]]*[0-9]+[[:space:]]*$/ {gsub(/[[:space:]]/, ""); print; exit}' <<< "$power_status")"
[[ "$power_mode" == "$EXPECTED_POWER_MODE" ]] || {
  echo >&2
  echo "MAXN_SUPER mode 2が選択されていません。" >&2
  echo "先に次を実行し、要求された場合は再起動してください:" >&2
  echo "  sudo nvpmodel -m ${EXPECTED_POWER_MODE}" >&2
  exit 1
}

if [[ "$mode" == apply ]]; then
  echo "MAXN_SUPER mode 2を確認しました。最大クロックと最大ファンを適用します。"
  run_as_root jetson_clocks --fan
  sleep 2
fi

clock_status="$(run_as_root jetson_clocks --show 2>&1)"
echo "$clock_status"

clock_domains=0
clock_failures=0
while IFS= read -r line; do
  case "$line" in
    cpu[0-9]*:*|GPU\ MinFreq=*|EMC\ MinFreq=*)
      if [[ "$line" =~ MinFreq=([0-9]+).*MaxFreq=([0-9]+) ]]; then
        clock_domains=$((clock_domains + 1))
        if [[ "${BASH_REMATCH[1]}" != "${BASH_REMATCH[2]}" ]]; then
          echo "Clock is not fixed: $line" >&2
          clock_failures=$((clock_failures + 1))
        fi
      fi
      ;;
  esac
done <<< "$clock_status"

((clock_domains >= 3)) || die "CPU・GPU・EMCのクロック情報を十分に取得できませんでした。"
((clock_failures == 0)) || die "最大クロックに固定されていないdomainがあります。"

grep -q 'FAN Dynamic Speed Control=disabled.*pwm1=255' <<< "$clock_status" ||
  die "ファンが最大PWM 255に固定されていません。"

if [[ "$mode" == check ]]; then
  echo "OK: MAXN_SUPER mode 2、最大クロック、最大ファンを確認しました。"
else
  echo "OK: MAXN_SUPER mode 2で最大クロックと最大ファンを適用・確認しました。"
fi
