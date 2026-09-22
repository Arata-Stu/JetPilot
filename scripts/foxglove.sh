#!/usr/bin/env bash
set -euo pipefail

case "${1:-}" in
  -h|--help)
    printf 'Usage: %s\n接続先IPを選択し、Foxgloveを開きます（ポート8767）。\n' "$(basename -- "$0")"
    exit 0
    ;;
  '') ;;
  *) printf '引数は不要です。接続先は対話形式で選択します。\n' >&2; exit 1 ;;
esac

valid_ipv4() {
  local ip="$1" part
  local parts=()
  [[ "$ip" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]] || return 1
  IFS=. read -r -a parts <<< "$ip"
  for part in "${parts[@]}"; do
    [[ ${#part} -le 3 ]] || return 1
    ((10#$part <= 255)) || return 1
  done
}

printf '%s\n' \
  'Foxglove 接続先' \
  '  1) 192.168.11.190' \
  '  2) 192.168.55.1' \
  '  3) 10.42.0.1' \
  '  4) 192.168.11.11' \
  '  5) manual（手入力）' \
  '  q) 中止'

while true; do
  read -r -p '番号 [1]: ' choice || exit 0
  case "${choice:-1}" in
    1) ip=192.168.11.190; break ;;
    2) ip=192.168.55.1; break ;;
    3) ip=10.42.0.1; break ;;
    4) ip=192.168.11.11; break ;;
    5|manual)
      while true; do
        read -r -p 'IPv4アドレス（qで中止）: ' ip || exit 0
        [[ "$ip" != q && "$ip" != Q ]] || exit 0
        if valid_ipv4 "$ip"; then
          break 2
        fi
        printf 'IPv4アドレスを入力してください（例: 192.168.11.11）。\n' >&2
      done
      ;;
    q|Q) exit 0 ;;
    *) printf '1〜5、またはqを入力してください。\n' >&2 ;;
  esac
done

url="foxglove://open?ds=foxglove-websocket&ds.url=ws://${ip}:8767/"
case "$(uname -s)" in
  Darwin) launcher=open ;;
  Linux) launcher=xdg-open ;;
  *) printf 'macOSまたはLinuxのデスクトップで実行してください。\n' >&2; exit 1 ;;
esac
command -v "$launcher" >/dev/null 2>&1 || {
  printf '%s が見つかりません。Foxgloveをインストールしたデスクトップで実行してください。\n' "$launcher" >&2
  exit 1
}
printf 'Foxgloveを起動: ws://%s:8767/\n' "$ip"
exec "$launcher" "$url"
