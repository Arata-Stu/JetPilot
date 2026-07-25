#!/usr/bin/env bash
set -euo pipefail

readonly DEFAULT_KEEP_IMAGE="cached_isaac_run_dev_image_local:latest"
readonly DEFAULT_CACHE_UNTIL="168h"
readonly DEFAULT_CACHE_KEEP="50GB"
readonly DEFAULT_CACHE_BUILDER="default"

keep_image="${DEFAULT_KEEP_IMAGE}"
cache_until="${DEFAULT_CACHE_UNTIL}"
cache_keep="${DEFAULT_CACHE_KEEP}"
cache_builder="${DEFAULT_CACHE_BUILDER}"
execute=false
assume_yes=false
prune_cache=true
prune_all_cache=false

usage() {
  cat <<'EOF'
Usage:
  ./scripts/cleanup_isaac_ros_docker.sh [options]

Keeps the current Isaac ROS image and removes older hashed final images from the
same image family. Containers and images referenced by containers are never
removed. Build cache is limited separately by age and retained size.

The default mode is a dry run. No Docker data is changed unless --execute is
specified.

Options:
  --execute                 Perform the displayed cleanup.
  --yes                     Do not ask for confirmation (requires --execute).
  --keep-image IMAGE        Image that defines the generation to keep.
                             Default: cached_isaac_run_dev_image_local:latest
  --cache-until DURATION    Remove eligible cache older than this duration.
                             Default: 168h
  --cache-keep SIZE         Maximum cache storage after pruning.
                             Default: 50GB
  --cache-builder NAME      Buildx builder whose cache should be pruned.
                             Default: default
  --all-cache               Include all unused build cache, not only dangling
                             cache. Age and storage limits still apply.
  --skip-cache              Remove old image tags only; do not prune cache.
  -h, --help                Show this help.

Examples:
  # Preview only
  ./scripts/cleanup_isaac_ros_docker.sh

  # Keep the current image, remove old generations, and limit week-old cache
  ./scripts/cleanup_isaac_ros_docker.sh --execute

  # Non-interactive maintenance
  ./scripts/cleanup_isaac_ros_docker.sh \
    --execute --yes --cache-until 336h --cache-keep 80GB
EOF
}

die() {
  echo "Error: $*" >&2
  exit 1
}

capture_docker_storage() {
  docker system df \
    --format '{{.Type}}{{"\t"}}{{.Size}}{{"\t"}}{{.Reclaimable}}'
}

print_storage_snapshot() {
  local title="$1"
  local report="$2"

  echo "${title}"
  awk -F '\t' '
    function to_bytes(value, number, unit) {
      gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
      number = value
      sub(/[[:alpha:]]+$/, "", number)
      unit = tolower(value)
      sub(/^[0-9.]+/, "", unit)

      if (unit == "kb") return number * 1000
      if (unit == "mb") return number * 1000 * 1000
      if (unit == "gb") return number * 1000 * 1000 * 1000
      if (unit == "tb") return number * 1000 * 1000 * 1000 * 1000
      if (unit == "pb") return number * 1000 * 1000 * 1000 * 1000 * 1000
      return number
    }

    {
      total += to_bytes($2)
      printf "  %-18s %12s  reclaimable: %s\n", $1, $2, $3
    }

    END {
      printf "  %-18s %9.2f GB\n", "Reported total", total / 1000000000
    }
  ' <<< "${report}"
}

print_storage_comparison() {
  local before_report="$1"
  local after_report="$2"

  {
    while IFS=$'\t' read -r type size reclaimable; do
      [[ -n "${type}" ]] || continue
      printf 'before\t%s\t%s\t%s\n' "${type}" "${size}" "${reclaimable}"
    done <<< "${before_report}"

    while IFS=$'\t' read -r type size reclaimable; do
      [[ -n "${type}" ]] || continue
      printf 'after\t%s\t%s\t%s\n' "${type}" "${size}" "${reclaimable}"
    done <<< "${after_report}"
  } | awk -F '\t' '
    function to_bytes(value, number, unit) {
      gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
      number = value
      sub(/[[:alpha:]]+$/, "", number)
      unit = tolower(value)
      sub(/^[0-9.]+/, "", unit)

      if (unit == "kb") return number * 1000
      if (unit == "mb") return number * 1000 * 1000
      if (unit == "gb") return number * 1000 * 1000 * 1000
      if (unit == "tb") return number * 1000 * 1000 * 1000 * 1000
      if (unit == "pb") return number * 1000 * 1000 * 1000 * 1000 * 1000
      return number
    }

    BEGIN {
      order[1] = "Images"
      order[2] = "Containers"
      order[3] = "Local Volumes"
      order[4] = "Build Cache"
    }

    $1 == "before" {
      before_raw[$2] = $3
      before_bytes[$2] = to_bytes($3)
      before_total += before_bytes[$2]
      next
    }

    $1 == "after" {
      after_raw[$2] = $3
      after_bytes[$2] = to_bytes($3)
      after_total += after_bytes[$2]
      next
    }

    END {
      print "Docker storage comparison (reported by docker system df):"
      printf "  %-18s %12s %12s %14s\n", "Category", "Before", "After", "Freed"

      for (i = 1; i <= 4; i++) {
        category = order[i]
        before = (category in before_raw) ? before_raw[category] : "0B"
        after = (category in after_raw) ? after_raw[category] : "0B"
        freed = before_bytes[category] - after_bytes[category]
        printf "  %-18s %12s %12s %11.2f GB\n",
          category, before, after, freed / 1000000000
      }

      total_freed = before_total - after_total
      printf "  %-18s %9.2f GB %9.2f GB %11.2f GB\n",
        "Reported total",
        before_total / 1000000000,
        after_total / 1000000000,
        total_freed / 1000000000

      if (total_freed >= 0) {
        printf "\nDocker storage: %.2f GB -> %.2f GB (%.2f GB freed)\n",
          before_total / 1000000000,
          after_total / 1000000000,
          total_freed / 1000000000
      } else {
        printf "\nDocker storage: %.2f GB -> %.2f GB (%.2f GB increased)\n",
          before_total / 1000000000,
          after_total / 1000000000,
          -total_freed / 1000000000
      }
    }
  '
}

require_value() {
  local option="$1"
  local value="${2:-}"
  [[ -n "${value}" && "${value}" != --* ]] \
    || die "${option} requires a value."
}

while (($# > 0)); do
  case "$1" in
    --execute)
      execute=true
      shift
      ;;
    --yes)
      assume_yes=true
      shift
      ;;
    --keep-image)
      require_value "$1" "${2:-}"
      keep_image="$2"
      shift 2
      ;;
    --cache-until)
      require_value "$1" "${2:-}"
      cache_until="$2"
      shift 2
      ;;
    --cache-keep)
      require_value "$1" "${2:-}"
      cache_keep="$2"
      shift 2
      ;;
    --cache-builder)
      require_value "$1" "${2:-}"
      cache_builder="$2"
      shift 2
      ;;
    --all-cache)
      prune_all_cache=true
      shift
      ;;
    --skip-cache)
      prune_cache=false
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "Unknown option: $1"
      ;;
  esac
done

if "${assume_yes}" && ! "${execute}"; then
  die "--yes may only be used with --execute."
fi

command -v docker >/dev/null 2>&1 || die "docker was not found."
docker info >/dev/null 2>&1 || die "Docker is not running or is not accessible."

storage_before="$(capture_docker_storage)" \
  || die "Could not read Docker storage usage."

current_image_id="$(
  docker image inspect --format '{{.Id}}' "${keep_image}" 2>/dev/null
)" || die "Keep image not found: ${keep_image}"

current_refs=()
while IFS= read -r ref; do
  [[ -n "${ref}" ]] || continue
  current_refs+=("${ref}")
done < <(
  docker image inspect \
    --format '{{range .RepoTags}}{{println .}}{{end}}' \
    "${keep_image}"
)

# A build produces both slash-style and registry-tag-style references:
#   nvcr.io/.../additional_setting_<md5>-amd64:latest
#   nvcr.io/...:...-additional_setting_<md5>-amd64
# Derive both exact families from references attached to the current image.
declare -a family_prefixes=()
declare -a family_suffixes=()

for ref in "${current_refs[@]}"; do
  if [[ "${ref}" =~ ^(.*additional_setting_)([0-9a-f]{32})(-(amd64|arm64-[a-z0-9_-]+))(:latest)?$ ]]; then
    family_prefixes+=("${BASH_REMATCH[1]}")
    family_suffixes+=("${BASH_REMATCH[3]}${BASH_REMATCH[5]}")
  fi
done

((${#family_prefixes[@]} > 0)) || die \
  "Could not derive an additional_setting image family from ${keep_image}."

is_family_ref() {
  local ref="$1"
  local prefix suffix hash_part
  local index

  for index in "${!family_prefixes[@]}"; do
    prefix="${family_prefixes[$index]}"
    suffix="${family_suffixes[$index]}"
    [[ "${ref}" == "${prefix}"*"${suffix}" ]] || continue

    hash_part="${ref#"${prefix}"}"
    hash_part="${hash_part%"${suffix}"}"
    if [[ "${hash_part}" =~ ^[0-9a-f]{32}$ ]]; then
      return 0
    fi
  done
  return 1
}

# Keep an empty sentinel because macOS Bash 3.2 treats expansion of a truly
# empty array as an unbound variable when `set -u` is enabled.
container_image_ids=("")
while IFS= read -r container_id; do
  [[ -n "${container_id}" ]] || continue
  container_image_id="$(
    docker container inspect --format '{{.Image}}' "${container_id}"
  )"
  container_image_ids+=("${container_image_id}")
done < <(docker container ls --all --quiet --no-trunc)

is_container_image_id() {
  local candidate_id="$1"
  local used_id

  for used_id in "${container_image_ids[@]}"; do
    if [[ "${candidate_id}" == "${used_id}" ]]; then
      return 0
    fi
  done
  return 1
}

declare -a removable_refs=()
declare -a skipped_refs=()

while IFS=$'\t' read -r ref image_id; do
  [[ -n "${ref}" && "${ref}" != "<none>:<none>" ]] || continue
  is_family_ref "${ref}" || continue

  if [[ "${image_id}" == "${current_image_id}" ]]; then
    continue
  fi
  if is_container_image_id "${image_id}"; then
    skipped_refs+=("${ref} (${image_id}; referenced by a container)")
    continue
  fi
  removable_refs+=("${ref}")
done < <(
  docker image ls \
    --no-trunc \
    --format '{{.Repository}}:{{.Tag}}{{"\t"}}{{.ID}}'
)

echo "Keep image:"
echo "  ${keep_image}"
echo "  ${current_image_id}"
echo
echo "Current generation references:"
for ref in "${current_refs[@]}"; do
  echo "  ${ref}"
done
echo

if ((${#removable_refs[@]} > 0)); then
  echo "Old image references selected for removal:"
  for ref in "${removable_refs[@]}"; do
    echo "  ${ref}"
  done
else
  echo "Old image references selected for removal: none"
fi

if ((${#skipped_refs[@]} > 0)); then
  echo
  echo "Skipped because a container references the image:"
  for ref in "${skipped_refs[@]}"; do
    echo "  ${ref}"
  done
fi

echo
if "${prune_cache}"; then
  echo "Build cache policy:"
  echo "  builder:    ${cache_builder}"
  echo "  older than: ${cache_until}"
  echo "  max size:   ${cache_keep}"
  echo "  scope:      $("${prune_all_cache}" && echo "all unused cache" || echo "dangling cache")"
else
  echo "Build cache policy: skipped"
fi

echo
print_storage_snapshot "Current Docker storage (reported by docker system df):" \
  "${storage_before}"

if ! "${execute}"; then
  echo
  echo "Dry run only. Re-run with --execute to perform this cleanup."
  exit 0
fi

if ! "${assume_yes}"; then
  [[ -t 0 ]] || die "Interactive confirmation is unavailable; use --execute --yes."
  echo
  read -r -p "Proceed with the cleanup shown above? [y/N] " answer
  case "${answer}" in
    y|Y|yes|YES)
      ;;
    *)
      echo "Canceled."
      exit 0
      ;;
  esac
fi

removal_failures=0
for ref in "${removable_refs[@]}"; do
  if ! docker image rm "${ref}"; then
    echo "Warning: failed to remove ${ref}" >&2
    removal_failures=$((removal_failures + 1))
  fi
done

if "${prune_cache}"; then
  builder_prune_help="$(docker builder prune --help)"

  prune_args=(
    builder prune
    --force
    --filter "until=${cache_until}"
  )
  if [[ "${builder_prune_help}" == *"--builder string"* ]]; then
    prune_args+=(--builder "${cache_builder}")
  fi
  if [[ "${builder_prune_help}" == *"--max-used-space"* ]]; then
    prune_args+=(--max-used-space "${cache_keep}")
  elif [[ "${builder_prune_help}" == *"--keep-storage"* ]]; then
    prune_args+=(--keep-storage "${cache_keep}")
  else
    die "This Docker version cannot enforce the requested cache size limit."
  fi
  if "${prune_all_cache}"; then
    prune_args+=(--all)
  fi
  docker "${prune_args[@]}"
fi

echo
storage_after="$(capture_docker_storage)" \
  || die "Cleanup ran, but Docker storage usage could not be read afterward."
print_storage_comparison "${storage_before}" "${storage_after}"

if ((removal_failures > 0)); then
  die "${removal_failures} image reference(s) could not be removed."
fi

echo
echo "Cleanup completed. The current image and container-referenced images were kept."
