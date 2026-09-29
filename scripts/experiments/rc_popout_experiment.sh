#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "${SCRIPT_DIR}/lib/project_env.sh"
ROS2_WS="${ROS2_WS:-/workspaces/ros2_ws}"
ROS2_SETUP_FILE="${ROS2_SETUP_FILE:-${ROS2_WS}/install/setup.bash}"
RECORD_ROOT="${RECORD_ROOT:-/workspaces/record}"
EXP_DIR="${RC_POPOUT_EXP_DIR:-${RECORD_ROOT}/rc_popout_experiment}"
PLAN_FILE=''
COMPLETED_FILE=''
SKIPPED_FILE=''
DRY_RUN=false
NEW_PLAN=false
STATUS_ONLY=false
RECORDING=false
EGO_REQUESTED=false
EGO_THROTTLE=''
EGO_DURATION=''
EGO_BRAKE=''
EGO_BRAKE_DURATION=1.0
EGO_STEERING=0.0
DRIVE_PID=''
DRIVE_HELPER="${SCRIPT_DIR}/experiments/rc_popout_timed_drive.py"

die() {
  printf 'error: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Usage:
  scripts/experiments/rc_popout_experiment.sh [--new] [--experiment-dir DIR]
  scripts/experiments/rc_popout_experiment.sh --status [--experiment-dir DIR]
  scripts/experiments/rc_popout_experiment.sh --dry-run [--experiment-dir DIR]

Terminal 1で `scripts/bringup.sh rc-popout` を起動してから、Terminal 2で実行します。
このスクリプトはセンサを起動せず、既存のBag Managerへ記録START/STOPを送ります。
自車走行時は bringup.sh rc-popout --vehicle jpbb を使用し、以下を追加します:
  --ego-throttle VALUE       自車の正規化スロットル [0,1]（実速度ではない）
  --ego-duration SEC         発進指令から制動までの秒数 (0,60]
  --ego-brake VALUE          制動指令 [0,1]（0=中立）
  --ego-brake-duration SEC   制動指令の保持秒数 (0,10]、既定1秒
  --ego-steering VALUE       固定操舵 [-1,1]、既定0
走行条件ごとに新しい --experiment-dir を指定してください。
EOF
}

prompt_default() {
  local label="$1"
  local default="$2"
  local value
  read -r -p "${label} [${default}]: " value
  printf '%s' "${value:-$default}"
}

trim() {
  local value="$1"
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%"${value##*[![:space:]]}"}"
  printf '%s' "$value"
}

sanitize_label() {
  printf '%s' "$1" | tr -c '[:alnum:]_-' '_'
}

is_finished() {
  local run_id="$1"
  grep -Fxq "$run_id" "$COMPLETED_FILE" 2>/dev/null \
    || grep -Fxq "$run_id" "$SKIPPED_FILE" 2>/dev/null
}

count_lines() {
  local path="$1"
  [[ -f "$path" ]] || { printf '0'; return; }
  awk 'END { print NR + 0 }' "$path"
}

show_status() {
  local total completed skipped remaining
  total="$(awk 'NR > 1 { count++ } END { print count + 0 }' "$PLAN_FILE")"
  completed="$(count_lines "$COMPLETED_FILE")"
  skipped="$(count_lines "$SKIPPED_FILE")"
  remaining=$((total - completed - skipped))
  printf '進捗: 完了=%s / 除外=%s / 未完了=%s / 合計=%s\n' \
    "$completed" "$skipped" "$remaining" "$total"
}

create_plan() {
  local camera_wall_distance obstacle_wall_distance gaps lightings directions throttles repetitions answer
  local -a gap_values lighting_values direction_values throttle_values
  local gap lighting direction throttle repetition run_number

  printf '\n飛び出し実験の条件を登録します。値はカンマ区切りで入力してください。\n'
  printf '今回の初期値は、壁基準でカメラ200 cm・段ボール100 cmの配置です。\n\n'
  camera_wall_distance="$(prompt_default '壁からカメラまでの距離' '200cm')"
  obstacle_wall_distance="$(prompt_default '壁から段ボールまでの距離' '100cm')"
  gaps="$(prompt_default '2つの段ボール間の開口幅' '150cm')"
  lightings="$(prompt_default '部屋の照明条件' 'normal')"
  directions="$(prompt_default '条件（left/right/none）' 'left,right,none')"
  throttles="$(prompt_default 'プロポのアクセル上限設定（実速度ではありません）' '20pct')"
  repetitions="$(prompt_default '各条件の反復回数' '3')"
  [[ "$repetitions" =~ ^[1-9][0-9]*$ ]] || die '反復回数は1以上の整数にしてください'

  camera_wall_distance="$(trim "$camera_wall_distance")"
  obstacle_wall_distance="$(trim "$obstacle_wall_distance")"
  [[ -n "$camera_wall_distance" ]] || die 'カメラ位置が空です'
  [[ -n "$obstacle_wall_distance" ]] || die '段ボール位置が空です'
  IFS=',' read -r -a gap_values <<< "$gaps"
  IFS=',' read -r -a lighting_values <<< "$lightings"
  IFS=',' read -r -a direction_values <<< "$directions"
  IFS=',' read -r -a throttle_values <<< "$throttles"

  mkdir -p "$EXP_DIR"
  if [[ -f "$PLAN_FILE" ]]; then
    backup_stamp="$(date +%Y%m%d_%H%M%S)"
    backup="${PLAN_FILE}.${backup_stamp}.bak"
    cp -p "$PLAN_FILE" "$backup"
    printf '既存の試行表を退避しました: %s\n' "$backup"
    [[ ! -f "$COMPLETED_FILE" ]] \
      || cp -p "$COMPLETED_FILE" "${COMPLETED_FILE}.${backup_stamp}.bak"
    [[ ! -f "$SKIPPED_FILE" ]] \
      || cp -p "$SKIPPED_FILE" "${SKIPPED_FILE}.${backup_stamp}.bak"
  fi

  printf 'run_id\tcamera_wall_distance\tobstacle_wall_distance\tgap_width\tlighting\tdirection\tthrottle\trepetition\n' >"$PLAN_FILE"
  run_number=1
  for gap in "${gap_values[@]}"; do
    gap="$(trim "$gap")"; [[ -n "$gap" ]] || die '開口幅に空の値があります'
    for lighting in "${lighting_values[@]}"; do
      lighting="$(trim "$lighting")"; [[ -n "$lighting" ]] || die '照明条件に空の値があります'
      for direction in "${direction_values[@]}"; do
        direction="$(trim "$direction")"; [[ -n "$direction" ]] || die '方向に空の値があります'
        case "$direction" in left|right|none) ;; *) die '条件はleft/right/noneを指定してください' ;; esac
        for throttle in "${throttle_values[@]}"; do
          throttle="$(trim "$throttle")"; [[ -n "$throttle" ]] || die 'スロットルに空の値があります'
          # 出現なしはアクセル条件を持たず、上限設定数だけ重複させない。
          if [[ "$direction" == none ]]; then throttle=na; fi
          for ((repetition=1; repetition<=repetitions; repetition++)); do
            printf 'r%03d\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
              "$run_number" "$camera_wall_distance" "$obstacle_wall_distance" \
              "$gap" "$lighting" "$direction" "$throttle" "$repetition" >>"$PLAN_FILE"
            run_number=$((run_number + 1))
          done
          if [[ "$direction" == none ]]; then break; fi
        done
      done
    done
  done

  : >"$COMPLETED_FILE"
  : >"$SKIPPED_FILE"
  cat >"${EXP_DIR}/experiment_conditions.txt" <<EOF
created_at=$(date '+%Y-%m-%dT%H:%M:%S%z')
camera_wall_distance=${camera_wall_distance}
obstacle_wall_distance=${obstacle_wall_distance}
gap_width=${gaps}
lighting=${lightings}
direction=${directions}
throttle=${throttles}
repetitions=${repetitions}
bringup=rc-popout
rgb=848x480@60
evs=native_RAW
ego_motion=$([[ -n "$EGO_THROTTLE" ]] && printf fixed_throttle || printf stationary)
target_control=propo_limit_full_trigger
throttle_semantics=propo_setting_not_measured_speed
EOF
  printf '\n'
  show_status
  read -r -p 'この試行表を使用しますか？ [Y/n]: ' answer
  case "$answer" in
    n|N) die '試行表の作成を中止しました。--newで作り直せます' ;;
  esac
}

publish_request() {
  local command="$1"
  local label="$2"
  if [[ "$DRY_RUN" == true ]]; then
    printf '[dry-run] BagRequest command=%s label=%s\n' "$command" "$label"
    return 0
  fi
  ros2 topic pub --once /bag/request jetpilot_msgs/msg/BagRequest \
    "{command: ${command}, label: '${label}'}" >/dev/null
}

log_attempt() {
  local outcome="$1" note="$2"
  [[ "$DRY_RUN" != true ]] || return 0
  python3 - "$EXP_DIR/attempts.jsonl" "$run_id" "$label" "$outcome" "$note" <<'PY'
import datetime
import json
import sys
path, trial, label, outcome, note = sys.argv[1:]
with open(path, 'a', encoding='utf-8') as stream:
    stream.write(json.dumps(dict(
        logged_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        trial_id=trial, recording_label=label, outcome=outcome, note=note,
        clock_semantics='operator_log_wall_time_not_sensor_or_reaction_time',
    ), ensure_ascii=False) + '\n')
PY
}

save_result() {
  local path="$1"
  local run_id="$2"
  if [[ "$DRY_RUN" == true ]]; then
    printf '[dry-run] 進捗は更新しません: %s\n' "$run_id"
    return 0
  fi
  printf '%s\n' "$run_id" >>"$path"
}

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  if [[ -n "$DRIVE_PID" ]]; then
    kill -TERM "$DRIVE_PID" 2>/dev/null || true
    wait "$DRIVE_PID" 2>/dev/null || true
  fi
  if [[ "$RECORDING" == true ]]; then
    publish_request 2 interrupted || true
  fi
  exit "$exit_code"
}

check_bag_manager() {
  [[ "$DRY_RUN" == true ]] && return 0
  command -v ros2 >/dev/null 2>&1 || {
    [[ -f "$ROS2_SETUP_FILE" ]] || die "ROS 2環境が見つかりません: $ROS2_SETUP_FILE"
    set +u
    # shellcheck disable=SC1090
    source "$ROS2_SETUP_FILE"
    set -u
  }
  ros2 topic list 2>/dev/null | grep -Fxq /bag/request \
    || die 'Bag Managerが見つかりません。Terminal 1で bringup.sh rc-popout を先に起動してください'
}

record_calibration() {
  local answer label
  read -r -p '最初にチェッカーボード校正を記録しますか？ [y/N]: ' answer
  case "$answer" in
    y|Y) ;;
    *) return 0 ;;
  esac

  printf '\n【チェッカーボード校正】\n'
  printf '1. RGBとEVSの両方にチェッカーボードが見える位置へ置いてください。\n'
  printf '2. 必要ならevent imageで、画角とピントを確認してください。\n'
  printf '3. LED同期基板も両方の視野に入ることを確認してください。\n'
  read -r -p '準備ができたらEnterで校正記録を開始: ' _
  label="rcp_calibration_checkerboard_$(date +%Y%m%d_%H%M%S)"
  publish_request 1 "$label"
  RECORDING=true
  printf '\n記録中です。LEDパターンを映し、チェッカーボードを複数の位置・角度で撮影してください。\n'
  read -r -p '校正撮影が終わったらEnterで記録終了: ' _
  publish_request 2 "$label"
  RECORDING=false
  printf '校正記録を終了しました。\n'
}

next_trial() {
  local run_id camera_wall_distance obstacle_wall_distance gap lighting direction throttle repetition
  while IFS=$'\t' read -r run_id camera_wall_distance obstacle_wall_distance gap lighting direction throttle repetition; do
    [[ "$run_id" != run_id ]] || continue
    is_finished "$run_id" && continue
    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
      "$run_id" "$camera_wall_distance" "$obstacle_wall_distance" \
      "$gap" "$lighting" "$direction" "$throttle" "$repetition"
    return 0
  done <"$PLAN_FILE"
  return 1
}

while (($# > 0)); do
  case "$1" in
    --ego-throttle|--ego-duration|--ego-brake|--ego-brake-duration|--ego-steering)
      EGO_REQUESTED=true
      (($# >= 2)) || die "$1 には値が必要です"
      case "$1" in
        --ego-throttle) EGO_THROTTLE="$2" ;;
        --ego-duration) EGO_DURATION="$2" ;;
        --ego-brake) EGO_BRAKE="$2" ;;
        --ego-brake-duration) EGO_BRAKE_DURATION="$2" ;;
        --ego-steering) EGO_STEERING="$2" ;;
      esac
      shift 2 ;;
    --new) NEW_PLAN=true; shift ;;
    --experiment-dir) (($# >= 2)) || die '--experiment-dirにはpathが必要です'; EXP_DIR="$2"; shift 2 ;;
    --status) STATUS_ONLY=true; shift ;;
    --dry-run) DRY_RUN=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "不明なoptionです: $1" ;;
  esac
done

DRIVE_ARGS=()
if [[ "$EGO_REQUESTED" == true ]]; then
  [[ -n "$EGO_THROTTLE" && -n "$EGO_DURATION" && -n "$EGO_BRAKE" ]] \
    || die '--ego-throttle / --ego-duration / --ego-brake をすべて指定してください'
  DRIVE_ARGS=(--throttle "$EGO_THROTTLE" --duration "$EGO_DURATION"
    --brake "$EGO_BRAKE" --brake-duration "$EGO_BRAKE_DURATION" --steering "$EGO_STEERING")
  python3 "$DRIVE_HELPER" "${DRIVE_ARGS[@]}" --validate
fi
# Prevent accidental mixing of stationary and moving runs, or changing ego settings on resume.
if [[ "$STATUS_ONLY" != true ]]; then
  python3 - "$EXP_DIR" "$EGO_THROTTLE" "$EGO_DURATION" "$EGO_BRAKE" "$EGO_BRAKE_DURATION" "$EGO_STEERING" "$DRY_RUN" <<'PYCONFIG'
import json, sys
from pathlib import Path
root = Path(sys.argv[1])
values = sys.argv[2:7]
config = dict(ego_motion='stationary')
if values[0]:
    config = dict(ego_motion='fixed_throttle', **dict(zip(
        ['throttle', 'duration_s', 'brake', 'brake_duration_s', 'steering'], map(float, values))))
path = root / 'ego_motion.json'
if path.exists():
    if json.loads(path.read_text()) != config:
        raise SystemExit('自車条件が保存済み条件と異なります。別の --experiment-dir を使用してください')
elif (root / 'plan.tsv').exists() and values[0]:
    raise SystemExit('静止実験の試行表へ走行条件を追加できません。別の --experiment-dir を使用してください')
elif sys.argv[7] != 'true':
    root.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(config, ensure_ascii=False, indent=2) + '\n')
PYCONFIG
fi

PLAN_FILE="${EXP_DIR}/plan.tsv"
COMPLETED_FILE="${EXP_DIR}/completed.txt"
SKIPPED_FILE="${EXP_DIR}/skipped.txt"

if [[ ! -f "$PLAN_FILE" || "$NEW_PLAN" == true ]]; then
  [[ -t 0 && -t 1 ]] || die '新しい試行表の作成には対話型terminalが必要です'
  create_plan
fi
touch "$COMPLETED_FILE" "$SKIPPED_FILE"

if [[ "$STATUS_ONLY" == true ]]; then
  show_status
  exit 0
fi

if [[ "$DRY_RUN" != true ]]; then
  [[ -t 0 && -t 1 ]] || die '実験進行には対話型terminalが必要です'
fi
check_bag_manager
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
record_calibration

while trial="$(next_trial)"; do
  IFS=$'\t' read -r run_id camera_wall_distance obstacle_wall_distance gap lighting direction throttle repetition <<< "$trial"
  printf '\n============================================================\n'
  printf '次の本番試行: %s\n' "$run_id"
  printf '  壁－カメラ距離     : %s\n' "$camera_wall_distance"
  printf '  壁－段ボール距離   : %s\n' "$obstacle_wall_distance"
  printf '  段ボール間の開口幅 : %s\n' "$gap"
  printf '  部屋の照明         : %s\n' "$lighting"
  printf '  飛び出し方向       : %s\n' "$direction"
  printf '  プロポ上限設定     : %s（実速度ではありません）\n' "$throttle"
  printf '  反復番号           : %s\n' "$repetition"
  printf '============================================================\n'
  printf '開口幅は段ボールの内側端面どうしで測り、床の基準テープに合わせてください。\n'
  if [[ "$direction" == none ]]; then
    printf '出現なし試行です。相手車を出現させず、飛び出し試行と同じ長さを記録してください。\n'
  else
    printf 'RCカーを指定方向側へ隠し、開始位置・向き・助走距離を揃えてください。\n'
    printf 'プロポ上限設定を確認し、走行区間はフルトリガを保持してください。\n'
  fi
  if [[ -n "$EGO_THROTTLE" ]]; then
    printf '自車: throttle=%s / 走行=%ss / brake=%sを%ss / steering=%s\n' \
      "$EGO_THROTTLE" "$EGO_DURATION" "$EGO_BRAKE" "$EGO_BRAKE_DURATION" "$EGO_STEERING"
    printf '自車を開始位置へ戻し、CH3をHOST許可側、操作モードをSTOPにしてください。\n'
  else
    printf '自車・カメラは静止させます。設定画面・配置・バッテリー情報を台帳へ残してください。\n'
  fi
  printf '上記の条件になるよう、カメラ・障害物・照明・RCカーを正しい場所にセットしてください。\n'
  printf '開始同期用LEDをRGBとEVSの両方に見える位置へ配置してください。\n'
  if [[ -n "$EGO_THROTTLE" ]]; then
    printf 'ここでは記録だけを開始します。自車は後の発進用Enterまで動かしません。\n'
  fi
  read -r -p '[Enter]=記録開始 / s=この条件を除外 / q=中断: ' action
  case "$action" in
    q|Q) break ;;
    s|S) save_result "$SKIPPED_FILE" "$run_id"; show_status; continue ;;
    '') ;;
    *) printf '入力を認識できないため開始しません。\n'; continue ;;
  esac

  attempt_id="$(python3 -c 'import uuid; print(uuid.uuid4().hex[:12])')"
  label="rcp_${run_id}_cam-$(sanitize_label "$camera_wall_distance")_obs-$(sanitize_label "$obstacle_wall_distance")_gap-$(sanitize_label "$gap")_lux-$(sanitize_label "$lighting")_dir-$(sanitize_label "$direction")_thr-$(sanitize_label "$throttle")_rep-${repetition}_attempt-${attempt_id}"
  log_attempt start_requested ''
  publish_request 1 "$label"
  RECORDING=true
  printf '\n記録中です。まずLED同期パターンをRGBとEVSの画面内に映してください。\n'
  printf 'LED板を外して1秒以上待ち、出現前の映像を十分に残してください。\n'
  if [[ "$direction" == none ]]; then
    printf '相手車は動かさず、通常試行と同じ観測時間を確保してください。\n'
  else
    printf '指定方向から、プロポ上限設定＋フルトリガで飛び出しを行ってください。\n'
  fi
  drive_ok=true
  if [[ -n "$EGO_THROTTLE" ]]; then
    printf '記録を継続したまま発進操作を待っています。開始LEDの撮影・撤去を済ませてください。\n'
    read -r -p '開始LEDを外して1秒以上待ち、走行準備ができたらEnterで自車発進 / q=試行中断: ' action
    if [[ -z "$action" ]]; then
      log_attempt ego_start_requested ''
      if [[ "$DRY_RUN" == true ]]; then
        python3 "$DRIVE_HELPER" "${DRIVE_ARGS[@]}" --dry-run
      else
        python3 "$DRIVE_HELPER" "${DRIVE_ARGS[@]}" --label "$label" --log "$EXP_DIR/ego_drive.jsonl" &
        DRIVE_PID=$!
        if wait "$DRIVE_PID"; then
          log_attempt ego_finished ''
        else
          drive_ok=false
          log_attempt ego_failed '走行中断または事前確認失敗'
        fi
        DRIVE_PID=''
      fi
    else
      drive_ok=false
      log_attempt ego_cancelled ''
    fi
  fi
  printf '記録は継続中です。車両の停止を確認し、LEDをRGBとEVSの両方に見える位置へ再配置してください。\n'
  printf '終了側のLED同期パターンを撮影してから記録を終了します。開始表示だけでは保存成功は保証されません。\n'
  read -r -p '飛び出しと終了側のLED撮影が終わったらEnterで記録終了: ' _
  publish_request 2 "$label"
  RECORDING=false
  log_attempt stop_requested ''

  if [[ "$drive_ok" != true ]]; then
    log_attempt retry '自車走行が正常終了しなかったため未完了のまま保持'
    printf '自車走行が完了していないため採用せず、同じ条件を再提示します。\n'
    [[ "$DRY_RUN" != true ]] || break
    continue
  fi
  read -r -p '[Enter]=採用 / r=同じ条件をやり直す / s=除外: ' result
  case "$result" in
    '') log_attempt accepted ''; save_result "$COMPLETED_FILE" "$run_id" ;;
    r|R) log_attempt retry ''; printf 'この条件を未完了のまま残します。\n' ;;
    s|S) log_attempt excluded ''; save_result "$SKIPPED_FILE" "$run_id" ;;
    *) log_attempt unresolved ''; printf '入力を認識できないため、同じ条件をやり直します。\n' ;;
  esac
  show_status
  if [[ "$DRY_RUN" == true ]]; then
    printf 'Dry-runでは最初の未完了試行だけを確認して終了します。\n'
    break
  fi
done

printf '\n実験進行を終了します。\n'
show_status
