#!/bin/bash
set -euo pipefail

WS_DIR="/workspaces/ros2_ws"
SRC_DIR="${WS_DIR}/src"

CLEAN_MODE=false

for arg in "$@"; do
  case "$arg" in
    -c|--clean)
      CLEAN_MODE=true
      ;;
  esac
done

cd "$WS_DIR" || {
  echo "Directory not found: $WS_DIR"
  exit 1
}

has_fzf() {
  command -v fzf >/dev/null 2>&1
}

choose_one() {
  local prompt="$1"
  shift
  local options=("$@")

  if has_fzf; then
    local selected
    selected=$(
      printf '%s\n' "${options[@]}" | fzf \
        --prompt="${prompt} > " \
        --height=40% \
        --border \
        --reverse \
        || true
    )

    if [[ -n "$selected" ]]; then
      echo "$selected"
    else
      echo "${options[0]}"
    fi
  else
    echo "$prompt"
    local i
    for i in "${!options[@]}"; do
      echo "  $((i + 1))) ${options[$i]}"
    done

    local choice
    read -rp "番号を入力: " choice

    if [[ "$choice" =~ ^[0-9]+$ ]] &&
       (( choice >= 1 && choice <= ${#options[@]} )); then
      echo "${options[$((choice - 1))]}"
    else
      echo "${options[0]}"
    fi
  fi
}

extract_pkg_name() {
  local package_xml="$1"

  awk '
    BEGIN { RS="</name>" }
    /<name>/ {
      sub(/.*<name>[[:space:]]*/, "")
      gsub(/[[:space:]]+/, "")
      print
      exit
    }
  ' "$package_xml"
}

relative_to_src() {
  local path="$1"

  if [[ "$path" == "$SRC_DIR"/* ]]; then
    echo "${path#"$SRC_DIR"/}"
  else
    echo "$path"
  fi
}

find_build_candidates() {
  if [[ ! -d "$SRC_DIR" ]]; then
    echo "src directory not found: $SRC_DIR" >&2
    exit 1
  fi

  declare -A seen_dirs=()

  # package.xml があるディレクトリを候補にする
  while IFS= read -r -d '' package_xml; do
    local dir
    dir="$(dirname "$package_xml")"

    if [[ -n "${seen_dirs[$dir]:-}" ]]; then
      continue
    fi

    local pkg_name
    pkg_name="$(extract_pkg_name "$package_xml")"

    if [[ -z "$pkg_name" ]]; then
      pkg_name="$(basename "$dir")"
    fi

    seen_dirs["$dir"]=1
    printf '%s\t%s\n' "$pkg_name" "$(relative_to_src "$dir")"
  done < <(find "$SRC_DIR" -type f -name package.xml -print0 | sort -z)

  # CMakeLists.txt だけがあるディレクトリも候補にする
  while IFS= read -r -d '' cmake_file; do
    local dir
    dir="$(dirname "$cmake_file")"

    if [[ -n "${seen_dirs[$dir]:-}" ]]; then
      continue
    fi

    local pkg_name
    pkg_name="$(basename "$dir")"

    seen_dirs["$dir"]=1
    printf '%s\t%s\n' "$pkg_name" "$(relative_to_src "$dir")"
  done < <(find "$SRC_DIR" -type f -name CMakeLists.txt -print0 | sort -z)
}

select_packages() {
  local candidates=("$@")

  if [[ "${#candidates[@]}" -eq 0 ]]; then
    echo "ビルド候補が見つかりませんでした。" >&2
    exit 1
  fi

  if has_fzf; then
    printf '%s\n' "${candidates[@]}" | fzf \
      --multi \
      --bind 'space:toggle' \
      --prompt="build package > " \
      --height=80% \
      --border \
      --reverse \
      --delimiter=$'\t' \
      --with-nth=1,2 \
      --header="Space: 選択/解除  Enter: 決定" \
      | cut -f1
  else
    echo ""
    echo "ビルドするパッケージを選択してください。例: 1 3"
    local i
    for i in "${!candidates[@]}"; do
      local pkg
      local path

      pkg="$(echo "${candidates[$i]}" | cut -f1)"
      path="$(echo "${candidates[$i]}" | cut -f2-)"

      printf "  %2d) %-30s %s\n" "$((i + 1))" "$pkg" "$path"
    done

    local choices
    read -rp "番号を入力: " choices

    local choice
    for choice in $choices; do
      if [[ "$choice" =~ ^[0-9]+$ ]] &&
         (( choice >= 1 && choice <= ${#candidates[@]} )); then
        echo "${candidates[$((choice - 1))]}" | cut -f1
      fi
    done
  fi
}

echo "=== ROS 2 colcon build script ==="
echo ""

BUILD_MODE="$(choose_one "ビルド方法を選択" "all" "select")"

SELECTED_PACKAGES=()

if [[ "$BUILD_MODE" == "select" ]]; then
  echo ""
  echo "ビルド候補を探索中: $SRC_DIR"

  mapfile -t CANDIDATES < <(find_build_candidates)

  mapfile -t SELECTED_PACKAGES < <(
    select_packages "${CANDIDATES[@]}"
  )

  if [[ "${#SELECTED_PACKAGES[@]}" -eq 0 ]]; then
    echo "エラー: パッケージが選択されていません。"
    exit 1
  fi
fi

if [[ "$CLEAN_MODE" == true ]]; then
  echo ""
  echo "Cleaning up build, install, and log directories..."
  rm -rf build/ install/ log/
fi

echo ""
echo "================ ビルド内容 ================"
echo "workspace : $WS_DIR"
echo "mode      : $BUILD_MODE"

if [[ "$BUILD_MODE" == "select" ]]; then
  echo "packages  :"
  for pkg in "${SELECTED_PACKAGES[@]}"; do
    echo "  - $pkg"
  done
else
  echo "packages  : all"
fi

echo "clean     : $CLEAN_MODE"
echo "============================================"

echo ""
echo "Starting colcon build with --symlink-install..."

if [[ "$BUILD_MODE" == "select" ]]; then
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select "${SELECTED_PACKAGES[@]}"
else
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
fi
