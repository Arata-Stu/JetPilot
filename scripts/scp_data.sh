#!/bin/bash
set -euo pipefail

echo "=== rosbag 受信スクリプト ==="

DEFAULT_REMOTE_USER="tamiya"
IP_CANDIDATES=("10.42.0.1" "192.168.55.1" "192.168.11.190")

REMOTE_BASE_DIR_DEFAULT="/home/tamiya/workspaces/JetPilot/record/"
LOCAL_DEST_DIR_DEFAULT="/home/arata-22/workspaces/JetPilot/record/"
REMOTE_LIST_MAX_DEPTH="${REMOTE_LIST_MAX_DEPTH:-4}"

has_fzf() {
    command -v fzf >/dev/null 2>&1
}

choose_one() {
    local prompt="$1"
    shift

    if has_fzf; then
        printf '%s\n' "$@" | fzf \
            --prompt="${prompt} > " \
            --height=40% \
            --border \
            --reverse
    else
        echo "$prompt" >&2
        local i
        for i in "${!@}"; do :; done

        local options=("$@")
        for i in "${!options[@]}"; do
            echo "  $((i + 1))) ${options[$i]}" >&2
        done

        local choice
        read -rp "番号を入力: " choice >&2

        if [[ "$choice" =~ ^[0-9]+$ ]] &&
           (( choice >= 1 && choice <= ${#options[@]} )); then
            echo "${options[$((choice - 1))]}"
        else
            echo "${options[0]}"
        fi
    fi
}

relative_path() {
    local path="$1"
    local base="${REMOTE_BASE_DIR%/}"

    if [[ "$path" == "$base"/* ]]; then
        echo "${path#"$base"/}"
    else
        echo "$path"
    fi
}

choose_dirs() {
    local prompt="$1"
    shift
    local dirs=("$@")

    if has_fzf; then
        for d in "${dirs[@]}"; do
            printf '%s\t%s\n' "$(relative_path "$d")" "$d"
        done | fzf \
            --multi \
            --bind 'space:toggle' \
            --prompt="${prompt} > " \
            --height=80% \
            --border \
            --reverse \
            --delimiter=$'\t' \
            --with-nth=1 \
            --header="Space: 選択/解除  Enter: 決定" \
            | cut -f2-
    else
        echo "" >&2
        echo "取得対象を選択してください。例: 1 3" >&2

        local i
        for i in "${!dirs[@]}"; do
            printf "  %2d) %s\n" "$((i + 1))" "$(relative_path "${dirs[$i]}")" >&2
        done

        local choices
        read -rp "番号を入力: " choices >&2

        local choice
        for choice in $choices; do
            if [[ "$choice" =~ ^[0-9]+$ ]] &&
               (( choice >= 1 && choice <= ${#dirs[@]} )); then
                echo "${dirs[$((choice - 1))]}"
            fi
        done
    fi
}

rsync_source() {
    local path="$1"
    local base="${REMOTE_BASE_DIR%/}"

    if [[ "$path" == "$base"/* ]]; then
        local rel="${path#"$base"/}"
        echo "${REMOTE_USER}@${REMOTE_IP}:${base}/./${rel}"
    else
        echo "${REMOTE_USER}@${REMOTE_IP}:${path}"
    fi
}

is_under_remote_base() {
    local path="$1"
    local base="${REMOTE_BASE_DIR%/}"

    [[ "$path" == "$base"/* ]]
}

read -rp "相手のユーザー名 (Enterで '${DEFAULT_REMOTE_USER}'): " REMOTE_USER
REMOTE_USER="${REMOTE_USER:-$DEFAULT_REMOTE_USER}"

echo ""

IP_SELECTED="$(choose_one "接続先IPを選択" "${IP_CANDIDATES[@]}" "手動入力")"

if [[ "$IP_SELECTED" == "手動入力" ]]; then
    read -rp "IPアドレスを入力: " REMOTE_IP
else
    REMOTE_IP="$IP_SELECTED"
fi

read -rp "リモートのベースディレクトリ (Enterで '${REMOTE_BASE_DIR_DEFAULT}'): " REMOTE_BASE_DIR
REMOTE_BASE_DIR="${REMOTE_BASE_DIR:-$REMOTE_BASE_DIR_DEFAULT}"
REMOTE_BASE_DIR="${REMOTE_BASE_DIR%/}"

MODE="$(choose_one "指定方法を選択" "リモートから一覧を取得して選択" "ディレクトリ名を直接入力")"

SELECTED_DIRS=()

if [[ "$MODE" == "リモートから一覧を取得して選択" ]]; then
    echo ""
    echo "リモート (${REMOTE_IP}) からディレクトリ一覧を取得中..."

    REMOTE_FIND_CMD=$(printf 'find %q -mindepth 1 -maxdepth %q -type d -print | sort' \
        "$REMOTE_BASE_DIR" "$REMOTE_LIST_MAX_DEPTH")

    mapfile -t DIRS < <(
        ssh -n -o ConnectTimeout=5 "${REMOTE_USER}@${REMOTE_IP}" \
            "$REMOTE_FIND_CMD" \
            2>/dev/null
    )

    if [[ "${#DIRS[@]}" -eq 0 ]]; then
        echo "エラー: ディレクトリが見つからないか、接続に失敗しました。"
        exit 1
    fi

    mapfile -t SELECTED_DIRS < <(
        choose_dirs "取得対象を選択" "${DIRS[@]}"
    )
else
    echo ""
    echo "ベースディレクトリ: ${REMOTE_BASE_DIR}"
    read -rp "転送したいディレクトリ名を入力してください。スペース区切り可: " MANUAL_INPUT

    for name in $MANUAL_INPUT; do
        if [[ "$name" = /* ]]; then
            SELECTED_DIRS+=("$name")
        else
            SELECTED_DIRS+=("${REMOTE_BASE_DIR}/${name}")
        fi
    done
fi

if [[ "${#SELECTED_DIRS[@]}" -eq 0 ]]; then
    echo "エラー: 対象が選択されていません。"
    exit 1
fi

echo ""
read -rp "ローカル保存先 (Enterで '${LOCAL_DEST_DIR_DEFAULT}'): " LOCAL_DEST_DIR
LOCAL_DEST_DIR="${LOCAL_DEST_DIR:-$LOCAL_DEST_DIR_DEFAULT}"

mkdir -p "$LOCAL_DEST_DIR"

echo ""
echo "================ 転送内容 ================"
echo "リモート: ${REMOTE_USER}@${REMOTE_IP}"
for dir in "${SELECTED_DIRS[@]}"; do
    echo "  - $(relative_path "$dir")"
done
echo "保存先  : ${LOCAL_DEST_DIR}"
echo "=========================================="

read -rp "rsyncを開始しますか？ (Y/n): " CONFIRM

if [[ "${CONFIRM:-y}" =~ ^[Yy]$ ]]; then
    for dir in "${SELECTED_DIRS[@]}"; do
        echo ">>> Transferring: $(relative_path "$dir")"

        if is_under_remote_base "$dir"; then
            rsync -avP -R "$(rsync_source "$dir")" "${LOCAL_DEST_DIR}/"
        else
            rsync -avP "$(rsync_source "$dir")" "${LOCAL_DEST_DIR}/"
        fi
    done

    echo "✅ 完了しました。"
else
    echo "キャンセルしました。"
fi