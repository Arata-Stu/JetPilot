#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
REQUESTED_ROOT="${JETPILOT_WORKSPACE_ROOT:-$DEFAULT_REPO_ROOT}"
CHECK_ONLY=false
QUIET=false

usage() {
  printf '%s\n' \
    "Usage: scripts/prepare_workspace_dirs.sh [--check] [--quiet]" \
    "" \
    "Prepare writable directories used through the JetPilot project-root mount." \
    "" \
    "Options:" \
    "  --check  Do not create missing directories; only verify them" \
    "  --quiet  Print only errors" \
    "  -h, --help"
}

while (($# > 0)); do
  case "$1" in
    --check) CHECK_ONLY=true; shift ;;
    --quiet) QUIET=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "error: unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ ! -d "$REQUESTED_ROOT" ]]; then
  echo "error: JetPilot workspace root does not exist: $REQUESTED_ROOT" >&2
  exit 1
fi
REPO_ROOT="$(cd -- "$REQUESTED_ROOT" && pwd)"

WORKSPACE_DIRS=(
  "record"
  "map"
  "ros2_ws/models/e2e"
  "python_ws/jetpilot_e2e_training/datasets"
  "python_ws/jetpilot_e2e_training/outputs/e2e"
)

errors=0
for relative_path in "${WORKSPACE_DIRS[@]}"; do
  target="${REPO_ROOT}/${relative_path}"

  if [[ -L "$target" ]]; then
    echo "error: workspace directory must not be a symlink: $target" >&2
    errors=$((errors + 1))
    continue
  fi

  if [[ ! -d "$target" ]]; then
    if [[ "$CHECK_ONLY" == true ]]; then
      echo "error: workspace directory is missing: $target" >&2
      errors=$((errors + 1))
      continue
    fi
    if ! mkdir -p -- "$target"; then
      echo "error: could not create workspace directory: $target" >&2
      errors=$((errors + 1))
      continue
    fi
  fi

  probe="${target}/.jetpilot-write-test.$$"
  if mkdir -- "$probe" 2>/dev/null; then
    rmdir -- "$probe"
    if [[ "$QUIET" == false ]]; then
      printf 'ok: %s\n' "$target"
    fi
  else
    echo "error: workspace directory is not writable: $target" >&2
    if command -v id >/dev/null 2>&1; then
      printf '  owner repair example: sudo chown -R %q:%q %q\n' \
        "$(id -un)" "$(id -gn)" "$target" >&2
    fi
    errors=$((errors + 1))
  fi
done

if ((errors > 0)); then
  echo "JetPilot workspace preparation failed with ${errors} error(s)." >&2
  exit 1
fi

if [[ "$QUIET" == false ]]; then
  echo "JetPilot workspace directories are ready."
fi
