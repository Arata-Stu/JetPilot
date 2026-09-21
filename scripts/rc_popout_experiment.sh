#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="${ROS2_WS:-/workspaces/ros2_ws}"
ROS2_SETUP_FILE="${ROS2_SETUP_FILE:-${ROS2_WS}/install/setup.bash}"
RECORD_ROOT="${RECORD_ROOT:-/workspaces/record}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
PLAN_HELPER="${SCRIPT_DIR}/rc_popout_plan.py"
PLAN_DIR="${RC_POPOUT_PLAN_DIR:-${RECORD_ROOT}/rc_popout_experiment}"
PLAN_FILE=''
VEHICLE=''
NEW_PLAN=false
STATUS_ONLY=false
DRY_RUN=false
BRINGUP_PID=''
RECORDING=false

die() {
  printf 'error: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Usage:
  rc_popout_experiment.sh [--plan-dir DIR] [--new-plan] [--vehicle PROFILE]
  rc_popout_experiment.sh [--plan-dir DIR] --status
  rc_popout_experiment.sh [--plan-dir DIR] --dry-run

初回に実験因子を入力して試行表を作り、以後は未完了の次条件から再開します。
各試行のSTART/STOPはBag Managerへ送られ、RGB MCAPとEVS native RAWが同じsessionに保存されます。
EOF
}

prompt_default() {
  local label="$1"
  local default="$2"
  local value
  read -r -p "${label} [${default}]: " value
  printf '%s' "${value:-$default}"
}

json_field() {
  "$PYTHON_BIN" -c \
    'import json,sys; print(json.loads(sys.argv[1])[sys.argv[2]])' "$1" "$2"
}

initialize_plan() {
  local speeds directions obstacle_positions camera_positions repetitions seed
  local init_args=()
  printf '\nRCカー飛び出し実験計画を作成します。値はカンマ区切りで指定してください。\n'
  printf '速度は実測m/s、固定スロットル値、slow/medium/fastなど、実験中に一貫する表記を使います。\n\n'
  speeds="$(prompt_default '飛び出し速度' 'slow,medium,fast')"
  directions="$(prompt_default '飛び出し方向' 'left,right')"
  obstacle_positions="$(prompt_default '障害物位置' 'near,far')"
  camera_positions="$(prompt_default 'カメラ位置' 'center')"
  repetitions="$(prompt_default '各条件の反復回数' '3')"
  seed="$(prompt_default '条件順序の乱数seed' "$(date +%Y%m%d)")"
  mkdir -p "$PLAN_DIR"
  init_args=(
    --plan "$PLAN_FILE"
    --speeds "$speeds"
    --directions "$directions"
    --obstacle-positions "$obstacle_positions"
    --camera-positions "$camera_positions"
    --repetitions "$repetitions"
    --seed "$seed"
  )
  [[ "$NEW_PLAN" == true ]] && init_args+=(--force)
  "$PYTHON_BIN" "$PLAN_HELPER" init "${init_args[@]}"
}

publish_bag_request() {
  local command="$1"
  local label="$2"
  ros2 topic pub --once /bag/request jetpilot_msgs/msg/BagRequest \
    "{command: ${command}, label: '${label}'}" >/dev/null
}

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  if [[ "$RECORDING" == true ]] && command -v ros2 >/dev/null 2>&1; then
    publish_bag_request 2 interrupted || true
  fi
  if [[ -n "$BRINGUP_PID" ]] && kill -0 "$BRINGUP_PID" 2>/dev/null; then
    kill -INT "$BRINGUP_PID" 2>/dev/null || true
    wait "$BRINGUP_PID" 2>/dev/null || true
  fi
  exit "$exit_code"
}

wait_for_bag_manager() {
  local attempt
  for attempt in $(seq 1 30); do
    if ! kill -0 "$BRINGUP_PID" 2>/dev/null; then
      die "bringupが終了しました。ログを確認してください: ${PLAN_DIR}/bringup.log"
    fi
    if ros2 topic list 2>/dev/null | grep -Fxq /bag/request; then
      return 0
    fi
    sleep 1
  done
  die '30秒以内にBag Managerが起動しませんでした'
}

show_run() {
  local record="$1"
  printf '\n============================================================\n'
  printf '次の試行: %s  (block %s)\n' \
    "$(json_field "$record" run_id)" "$(json_field "$record" block_id)"
  printf '  飛び出し速度 : %s\n' "$(json_field "$record" speed)"
  printf '  飛び出し方向 : %s\n' "$(json_field "$record" direction)"
  printf '  障害物位置   : %s\n' "$(json_field "$record" obstacle_position)"
  printf '  カメラ位置   : %s\n' "$(json_field "$record" camera_position)"
  printf '  反復番号     : %s\n' "$(json_field "$record" repetition)"
  printf '============================================================\n'
}

while (($# > 0)); do
  case "$1" in
    --plan-dir) (($# >= 2)) || die '--plan-dir requires a directory'; PLAN_DIR="$2"; shift 2 ;;
    --new-plan) NEW_PLAN=true; shift ;;
    --vehicle) (($# >= 2)) || die '--vehicle requires a profile'; VEHICLE="$2"; shift 2 ;;
    --status) STATUS_ONLY=true; shift ;;
    --dry-run) DRY_RUN=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown option: $1" ;;
  esac
done

PLAN_FILE="${PLAN_DIR}/plan.tsv"
[[ -f "$PLAN_HELPER" ]] || die "plan helper was not found: $PLAN_HELPER"
command -v "$PYTHON_BIN" >/dev/null 2>&1 || die "Python command was not found: $PYTHON_BIN"

if [[ "$NEW_PLAN" == true && -f "$PLAN_FILE" ]]; then
  backup_stamp="$(date +%Y%m%d_%H%M%S)"
  backup="${PLAN_FILE}.${backup_stamp}.bak"
  cp -p "$PLAN_FILE" "$backup"
  printf '既存planを退避しました: %s\n' "$backup"
  if [[ -f "${PLAN_DIR}/experiment_metadata.json" ]]; then
    cp -p "${PLAN_DIR}/experiment_metadata.json" \
      "${PLAN_DIR}/experiment_metadata.json.${backup_stamp}.bak"
  fi
fi
if [[ ! -f "$PLAN_FILE" || "$NEW_PLAN" == true ]]; then
  [[ -t 0 && -t 1 ]] || die 'plan作成には対話型terminalが必要です'
  initialize_plan
fi

if [[ "$STATUS_ONLY" == true ]]; then
  "$PYTHON_BIN" "$PLAN_HELPER" status --plan "$PLAN_FILE"
  exit 0
fi

next_record="$($PYTHON_BIN "$PLAN_HELPER" next --plan "$PLAN_FILE")" || {
  result=$?
  [[ "$result" == 3 ]] && { printf 'すべての試行が完了しました。\n'; exit 0; }
  exit "$result"
}
show_run "$next_record"

if [[ "$DRY_RUN" == true ]]; then
  printf '\nDry-run: センサ起動と記録は行いません。\n'
  "$PYTHON_BIN" "$PLAN_HELPER" status --plan "$PLAN_FILE"
  exit 0
fi

[[ -t 0 && -t 1 ]] || die 'experiment runnerには対話型terminalが必要です'
[[ -f "$ROS2_SETUP_FILE" ]] || die "ROS 2 environment is unavailable: $ROS2_SETUP_FILE"
set +u
# shellcheck disable=SC1090
source "$ROS2_SETUP_FILE"
set -u
mkdir -p "$PLAN_DIR"
if command -v rs-enumerate-devices >/dev/null 2>&1; then
  device_info="${PLAN_DIR}/realsense-device-info-$(date +%Y%m%d_%H%M%S).txt"
  rs-enumerate-devices -c >"$device_info" 2>&1 || true
fi

bringup_command=("${SCRIPT_DIR}/bringup.sh" rc-popout -y)
[[ -z "$VEHICLE" ]] || bringup_command+=(--vehicle "$VEHICLE")
"${bringup_command[@]}" >"${PLAN_DIR}/bringup.log" 2>&1 &
BRINGUP_PID=$!
trap cleanup EXIT INT TERM
wait_for_bag_manager
printf '\nセンサとBag Managerが起動しました。ログ: %s\n' "${PLAN_DIR}/bringup.log"

while next_record="$($PYTHON_BIN "$PLAN_HELPER" next --plan "$PLAN_FILE")"; do
  show_run "$next_record"
  run_id="$(json_field "$next_record" run_id)"
  label="rcp_${run_id}_spd-$(json_field "$next_record" speed)_dir-$(json_field "$next_record" direction)_obs-$(json_field "$next_record" obstacle_position)_cam-$(json_field "$next_record" camera_position)_rep-$(json_field "$next_record" repetition)"
  label="$(printf '%s' "$label" | tr -c '[:alnum:]_-' '_')"

  read -r -p '配置を確認してください。[Enter]=記録開始 / s=skip / q=終了: ' action
  case "$action" in
    q|Q) break ;;
    s|S)
      "$PYTHON_BIN" "$PLAN_HELPER" update --plan "$PLAN_FILE" \
        --run-id "$run_id" --status skipped --note 'operator skipped before recording'
      continue
      ;;
    '') ;;
    *) printf '入力を認識できません。試行を開始しません。\n'; continue ;;
  esac

  publish_bag_request 1 "$label"
  RECORDING=true
  printf '記録中: %s\n' "$label"
  read -r -p '飛び出し試行が終了したらEnterで記録停止: ' _
  publish_bag_request 2 "$label"
  RECORDING=false

  read -r -p '[Enter]=成功 / r=やり直す / s=除外: ' result
  case "$result" in
    '') status=completed; note='' ;;
    r|R) status=retry; note='operator requested retry' ;;
    s|S) status=skipped; note='operator excluded after recording' ;;
    *) status=retry; note="unrecognized result: $result" ;;
  esac
  "$PYTHON_BIN" "$PLAN_HELPER" update --plan "$PLAN_FILE" \
    --run-id "$run_id" --status "$status" --note "$note"
  "$PYTHON_BIN" "$PLAN_HELPER" status --plan "$PLAN_FILE"
done

printf '\n実験を終了します。現在の進捗:\n'
"$PYTHON_BIN" "$PLAN_HELPER" status --plan "$PLAN_FILE"
