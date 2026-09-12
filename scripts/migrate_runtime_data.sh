#!/usr/bin/env bash

set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXECUTE=0

if [[ "${1:-}" == "--execute" ]]; then
  EXECUTE=1
elif [[ $# -gt 0 ]]; then
  echo "Usage: $0 [--execute]" >&2
  exit 2
fi

move_children() {
  local source_dir="$1"
  local destination_dir="$2"
  [[ -d "${source_dir}" ]] || return 0
  mkdir -p "${destination_dir}"

  local source
  local destination
  while IFS= read -r -d '' source; do
    destination="${destination_dir}/$(basename "${source}")"
    if [[ -e "${destination}" ]]; then
      echo "REFUSE collision: ${destination}" >&2
      continue
    fi
    if [[ "${EXECUTE}" -eq 1 ]]; then
      echo "MOVE ${source} -> ${destination}"
      mv "${source}" "${destination}"
    else
      echo "WOULD MOVE ${source} -> ${destination}"
    fi
  done < <(find "${source_dir}" -mindepth 1 -maxdepth 1 -print0)
}

move_children "${ROOT_DIR}/ros2_ws/bags" "${ROOT_DIR}/data/bags"
move_children "${ROOT_DIR}/ros2_ws/trajectories" "${ROOT_DIR}/data/trajectories"
move_children "${ROOT_DIR}/ros2_ws/processed_data" "${ROOT_DIR}/data/processed"
move_children "${ROOT_DIR}/ros2_ws/output" "${ROOT_DIR}/runtime/orbslam"

if [[ "${EXECUTE}" -eq 0 ]]; then
  echo "Dry run only. Review collisions, then rerun with --execute."
fi
