#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="${ROS2_WS:-/workspaces/ros2_ws}"
PROJECT_ROOT="${JETPILOT_PROJECT_ROOT:-$(dirname -- "$SCRIPT_DIR")}"
if [[ -z "${JETPILOT_PROJECT_ROOT:-}" \
      && ! -d "${PROJECT_ROOT}/ros2_ws" \
      && -d "$ROS2_WS" ]]; then
  PROJECT_ROOT="$(dirname -- "$ROS2_WS")"
fi
ROS2_SETUP_FILE="${ROS2_SETUP_FILE:-${ROS2_WS}/install/setup.bash}"
MAP_ROOT="${MAP_ROOT:-/workspaces/map}"
RECORD_ROOT="${RECORD_ROOT:-/workspaces/record}"

PROFILE=''
SESSION_NAME=''
RECREATE=false
CREATED_SESSION=false

die() {
  printf 'error: %s\n' "$*" >&2
  exit 1
}

print_usage() {
  cat <<EOF
Usage:
  $(basename "$0") [run|map|dev] [options]

Profiles:
  run   実機実行用: bringup、node monitor、topic monitor
  map   map作成用: create_map、map artifacts、rosbag一覧
  dev   開発用: build、git status、ROS package確認

Options:
  -s, --session NAME  tmux session名を指定
  --recreate          同名sessionを削除して作り直す
  --list              tmux session一覧を表示
  -h, --help          このヘルプを表示

各paneにはコマンドだけを入力し、実行はしません。
session接続後、実行したいpaneでEnterを押してください。
EOF
}

while (($# > 0)); do
  case "$1" in
    run|map|dev)
      [[ -z "$PROFILE" ]] || die "profile is already specified: $PROFILE"
      PROFILE="$1"
      shift
      ;;
    -s|--session)
      (($# >= 2)) || die "$1 requires a session name"
      SESSION_NAME="$2"
      shift 2
      ;;
    --recreate)
      RECREATE=true
      shift
      ;;
    --list)
      command -v tmux >/dev/null 2>&1 || die "tmux command was not found"
      tmux list-sessions 2>/dev/null || printf 'tmux sessionはありません。\n'
      exit 0
      ;;
    -h|--help)
      print_usage
      exit 0
      ;;
    *)
      die "unknown option or profile: $1"
      ;;
  esac
done

command -v tmux >/dev/null 2>&1 || die "tmux command was not found"

choose_profile() {
  local choice

  printf 'tmux session profile:\n'
  printf '  [1] run  - 実機実行 / bringup\n'
  printf '  [2] map  - map作成 / rosbag評価\n'
  printf '  [3] dev  - build / 開発\n'
  printf '  [q] cancel\n\n'

  while true; do
    read -r -p 'Select profile [1]: ' choice
    case "${choice:-1}" in
      1|run)
        PROFILE='run'
        return
        ;;
      2|map)
        PROFILE='map'
        return
        ;;
      3|dev)
        PROFILE='dev'
        return
        ;;
      q|Q)
        exit 0
        ;;
      *)
        printf '1, 2, 3, q のいずれかを入力してください。\n' >&2
        ;;
    esac
  done
}

if [[ -z "$PROFILE" ]]; then
  [[ -t 0 && -t 1 ]] || die "profileを引数で指定してください: run, map, dev"
  choose_profile
fi

if [[ -z "$SESSION_NAME" ]]; then
  SESSION_NAME="rc-${PROFILE}"
fi
[[ "$SESSION_NAME" =~ ^[A-Za-z0-9_.-]+$ ]] \
  || die "session nameには英数字、_, ., - のみ使用できます: $SESSION_NAME"

attach_session() {
  local session="$1"

  if [[ -n "${TMUX:-}" ]]; then
    tmux switch-client -t "$session"
  else
    [[ -t 0 && -t 1 ]] || die "tmux sessionへの接続にはinteractive terminalが必要です"
    exec tmux attach-session -t "$session"
  fi
}

if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
  if [[ "$RECREATE" == 'true' ]]; then
    tmux kill-session -t "$SESSION_NAME"
  else
    printf '既存のtmux sessionへ接続します: %s\n' "$SESSION_NAME"
    attach_session "$SESSION_NAME"
    exit 0
  fi
fi

cleanup_failed_session() {
  local status=$?

  if ((status != 0)) && [[ "$CREATED_SESSION" == 'true' ]]; then
    tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true
  fi
}
trap cleanup_failed_session EXIT

printf -v PROJECT_ROOT_Q '%q' "$PROJECT_ROOT"
printf -v SCRIPT_DIR_Q '%q' "$SCRIPT_DIR"
printf -v ROS2_SETUP_FILE_Q '%q' "$ROS2_SETUP_FILE"
printf -v MAP_ROOT_Q '%q' "$MAP_ROOT"
printf -v RECORD_ROOT_Q '%q' "$RECORD_ROOT"

prepare_pane() {
  local pane="$1"
  local title="$2"
  local command="$3"

  tmux select-pane -t "$pane" -T "$title"
  tmux send-keys -t "$pane" -l "$command"
}

configure_session() {
  tmux set-option -t "$SESSION_NAME" mouse on
  tmux set-option -t "$SESSION_NAME" destroy-unattached off
  tmux set-option -t "$SESSION_NAME" history-limit 50000
  tmux set-window-option -t "${SESSION_NAME}:" pane-border-status top
  tmux set-window-option -t "${SESSION_NAME}:" pane-border-format ' #{pane_title} '
  tmux set-option -t "$SESSION_NAME" status-left "#[bold] ${SESSION_NAME} "
}

create_run_session() {
  local main_pane
  local node_pane
  local topic_pane

  tmux new-session -d -s "$SESSION_NAME" -n run -c "$PROJECT_ROOT"
  CREATED_SESSION=true
  main_pane="$(tmux display-message -p -t "${SESSION_NAME}:run.0" '#{pane_id}')"
  node_pane="$(
    tmux split-window -d -h -p 38 -t "$main_pane" -c "$PROJECT_ROOT" \
      -P -F '#{pane_id}'
  )"
  topic_pane="$(
    tmux split-window -d -v -p 50 -t "$node_pane" -c "$PROJECT_ROOT" \
      -P -F '#{pane_id}'
  )"

  prepare_pane \
    "$main_pane" \
    'bringup' \
    "cd ${PROJECT_ROOT_Q} && ${SCRIPT_DIR_Q}/bringup.sh"
  prepare_pane \
    "$node_pane" \
    'ROS nodes' \
    "source ${ROS2_SETUP_FILE_Q} && watch -n 2 'ros2 node list'"
  prepare_pane \
    "$topic_pane" \
    'ROS topics' \
    "source ${ROS2_SETUP_FILE_Q} && watch -n 2 'ros2 topic list --show-types'"

  tmux select-pane -t "$main_pane"
}

create_map_session() {
  local main_pane
  local map_pane
  local rosbag_pane

  tmux new-session -d -s "$SESSION_NAME" -n map -c "$PROJECT_ROOT"
  CREATED_SESSION=true
  main_pane="$(tmux display-message -p -t "${SESSION_NAME}:map.0" '#{pane_id}')"
  map_pane="$(
    tmux split-window -d -h -p 40 -t "$main_pane" -c "$PROJECT_ROOT" \
      -P -F '#{pane_id}'
  )"
  rosbag_pane="$(
    tmux split-window -d -v -p 50 -t "$map_pane" -c "$PROJECT_ROOT" \
      -P -F '#{pane_id}'
  )"

  prepare_pane \
    "$main_pane" \
    'create map' \
    "cd ${PROJECT_ROOT_Q} && ${SCRIPT_DIR_Q}/create_map.sh"
  prepare_pane \
    "$map_pane" \
    'map artifacts' \
    "watch -n 2 'find ${MAP_ROOT_Q} -maxdepth 4 -type f | sort | tail -n 30'"
  prepare_pane \
    "$rosbag_pane" \
    'rosbags' \
    "find ${RECORD_ROOT_Q} -type f -name metadata.yaml | sort -r | head -n 30"

  tmux select-pane -t "$main_pane"
}

create_dev_session() {
  local build_pane
  local git_pane
  local ros_pane

  tmux new-session -d -s "$SESSION_NAME" -n dev -c "$PROJECT_ROOT"
  CREATED_SESSION=true
  build_pane="$(tmux display-message -p -t "${SESSION_NAME}:dev.0" '#{pane_id}')"
  git_pane="$(
    tmux split-window -d -h -p 40 -t "$build_pane" -c "$PROJECT_ROOT" \
      -P -F '#{pane_id}'
  )"
  ros_pane="$(
    tmux split-window -d -v -p 50 -t "$git_pane" -c "$PROJECT_ROOT" \
      -P -F '#{pane_id}'
  )"

  prepare_pane \
    "$build_pane" \
    'build' \
    "cd ${PROJECT_ROOT_Q} && ${SCRIPT_DIR_Q}/build.sh"
  prepare_pane \
    "$git_pane" \
    'git status' \
    "cd ${PROJECT_ROOT_Q} && git status --short --branch"
  prepare_pane \
    "$ros_pane" \
    'ROS package' \
    "source ${ROS2_SETUP_FILE_Q} && ros2 pkg prefix system_launch"

  tmux select-pane -t "$build_pane"
}

case "$PROFILE" in
  run) create_run_session ;;
  map) create_map_session ;;
  dev) create_dev_session ;;
esac

configure_session
CREATED_SESSION=false
printf 'tmux sessionを作成しました: %s (%s)\n' "$SESSION_NAME" "$PROFILE"
attach_session "$SESSION_NAME"
