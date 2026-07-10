#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  netem.sh apply IFACE DELAY JITTER LOSS
  netem.sh clear IFACE
  netem.sh show IFACE

Examples:
  sudo ./netem.sh apply eth0 20ms 5ms 1%
  sudo ./netem.sh clear eth0
EOF
}

[[ $# -ge 2 ]] || {
  usage >&2
  exit 2
}

mode="$1"
iface="$2"

case "$mode" in
  apply)
    [[ $# -eq 5 ]] || {
      usage >&2
      exit 2
    }
    delay="$3"
    jitter="$4"
    loss="$5"
    tc qdisc replace dev "$iface" root netem delay "$delay" "$jitter" loss "$loss"
    tc qdisc show dev "$iface"
    ;;
  clear)
    tc qdisc del dev "$iface" root >/dev/null 2>&1 || true
    tc qdisc show dev "$iface"
    ;;
  show)
    tc qdisc show dev "$iface"
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac

