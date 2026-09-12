#!/usr/bin/env bash

set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
BUILD=0

case "${1:-}" in
  "") ;;
  --build) BUILD=1 ;;
  -h|--help)
    echo "Usage: $0 [--build]"
    echo "Without --build, only checks prerequisites and creates runtime directories."
    exit 0
    ;;
  *) echo "Unknown option: $1" >&2; exit 2 ;;
esac

require_path() {
  [[ -e "$1" ]] || { echo "Missing: $1" >&2; exit 1; }
}

require_path "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
require_path "${ROOT_DIR}/core_ws/src/ORB_SLAM3"
require_path "${ROOT_DIR}/core_ws/src/Pangolin"
require_path "${ROOT_DIR}/ros2_ws/src/ORB_SLAM3_ROS2"
require_path "${ROOT_DIR}/ros2_ws/src/umi_dataset_tools"

mkdir -p \
  "${ROOT_DIR}/data/bags" \
  "${ROOT_DIR}/data/trajectories" \
  "${ROOT_DIR}/data/processed" \
  "${ROOT_DIR}/runtime/orbslam"

if [[ "${BUILD}" -eq 0 ]]; then
  echo "Prerequisites and directory layout look valid."
  echo "Run '$0 --build' to compile the project."
  exit 0
fi

source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"

cmake -S "${ROOT_DIR}/core_ws/src/Pangolin" \
  -B "${ROOT_DIR}/core_ws/src/Pangolin/build" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "${ROOT_DIR}/core_ws/src/Pangolin/build" --parallel "$(nproc)"

if [[ ! -f "${ROOT_DIR}/core_ws/src/ORB_SLAM3/lib/libORB_SLAM3.so" ]]; then
  (cd "${ROOT_DIR}/core_ws/src/ORB_SLAM3" && ./build.sh)
fi

export ORB_SLAM3_ROOT_DIR="${ROOT_DIR}/core_ws/src/ORB_SLAM3"
cd "${ROOT_DIR}/ros2_ws"
colcon build --symlink-install --cmake-args \
  "-DPangolin_DIR=${ROOT_DIR}/core_ws/src/Pangolin/build"

echo "Build complete. Run: source ${ROOT_DIR}/ros2_ws/install/setup.bash"
