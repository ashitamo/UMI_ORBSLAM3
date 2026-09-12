#!/usr/bin/env bash

set -Ee -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="${ROOT_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
ROS2_WS="${ROS2_WS:-${ROOT_DIR}/ros2_ws}"
CORE_WS="${CORE_WS:-${ROOT_DIR}/core_ws}"
DATA_DIR="${DATA_DIR:-${ROOT_DIR}/data}"
RUNTIME_DIR="${RUNTIME_DIR:-${ROOT_DIR}/runtime}"
BAGS_DIR="${BAGS_DIR:-${DATA_DIR}/bags}"
OUTPUT_DIR="${OUTPUT_DIR:-${RUNTIME_DIR}/orbslam}"
TRAJECTORY_DIR="${TRAJECTORY_DIR:-${DATA_DIR}/trajectories}"
PROCESSED_DIR="${PROCESSED_DIR:-${DATA_DIR}/processed}"
LEGACY_ROS2_WS="${LEGACY_ROS2_WS:-}"
CALIBRATION="${CALIBRATION:-${ROS2_WS}/src/umi_dataset_tools/aruco_calibration.yaml}"
VOCAB="${VOCAB:-${CORE_WS}/src/ORB_SLAM3/Vocabulary/ORBvoc.txt}"
SETTINGS="${SETTINGS:-${CORE_WS}/src/ORB_SLAM3/Examples/RGB-D-Inertial/D405_rgbd_inertial_locating_200_60.yaml}"
BAG_RATE="${BAG_RATE:-0.5}"
ORB_WARMUP_SEC="${ORB_WARMUP_SEC:-1}"
ORB_READY_TIMEOUT_SEC="${ORB_READY_TIMEOUT_SEC:-120}"
ORB_STOP_TIMEOUT_SEC="${ORB_STOP_TIMEOUT_SEC:-300}"
STARTUP_CHECK_SEC="${STARTUP_CHECK_SEC:-6.0}"
STARTUP_CHECK_SCRIPT="${ROS2_WS}/src/umi_dataset_tools/umi_dataset_tools/check_bag_startup_imu.py"
POINTCLOUD_STRIDE="${POINTCLOUD_STRIDE:-1}"
PIXEL_STRIDE="${PIXEL_STRIDE:-4}"
ORB_VIEWER="${ORB_VIEWER:-true}"
PIPELINE_DOMAIN_ID=""
RUN_ORBSLAM=1
RUN_EXPORT=1
RUN_STARTUP_CHECK=1
SKIP_HIGH_RISK_STARTUP=0
ALLOW_LIVE_PUBLISHERS=0

ORB_PID=""
BAG_PID=""

usage() {
  cat <<'EOF'
Usage:
  ros2_ws/scripts/run_orbslam_and_process_bags.sh [options] [bag ...]

Default bags:
  data0812_01 data0812_02 data0812_03

Options:
  --bag-rate VALUE            ros2 bag play rate, default: 0.5
  --orb-warmup-sec VALUE      settle time after ORB subscriptions are ready, default: 1
  --orb-ready-timeout-sec VALUE maximum seconds to wait for ORB subscriptions, default: 120
  --orb-stop-timeout-sec VALUE seconds to wait for ORB-SLAM3 shutdown, default: 300
  --startup-check-sec VALUE   bag-start IMU interval to inspect, default: 6
  --bags-dir PATH             input bag root, default: <project>/data/bags
  --trajectory-dir PATH       per-bag trajectory output directory
  --processed-dir PATH        final processed output directory
  --runtime-dir PATH          temporary ORB-SLAM3 output directory
  --settings PATH             ORB-SLAM3 yaml path
  --vocab PATH                ORB-SLAM3 vocabulary path
  --calibration PATH          ArUco calibration yaml path
  --pointcloud-stride VALUE   use every Nth depth frame, default: 1
  --pixel-stride VALUE        use every Nth depth pixel in x/y, default: 4
  --no-viewer                 disable the ORB-SLAM3 Pangolin viewer
  --viewer                    enable the ORB-SLAM3 Pangolin viewer (default)
  --domain-id VALUE           isolate bag playback and ORB-SLAM3 in this ROS domain
  --skip-orbslam              only run process_rgbd_bags with existing trajectories
  --skip-export               only run ORB-SLAM3 and save trajectories
  --skip-startup-check        do not inspect startup IMU stability
  --skip-high-risk-startup    skip bags with no valid startup static-IMU window
  --allow-live-publishers     allow existing RGB-D/IMU publishers (unsafe for replay)
  -h, --help                  show this help

Examples:
  ros2_ws/scripts/run_orbslam_and_process_bags.sh

  ros2_ws/scripts/run_orbslam_and_process_bags.sh \
    --bag-rate 1.0 \
    --pointcloud-stride 30 \
    data0812_01 data0812_02 data0812_03
EOF
}

cleanup() {
  if [[ -n "${BAG_PID}" ]] && kill -0 "${BAG_PID}" 2>/dev/null; then
    kill -INT "${BAG_PID}" 2>/dev/null || true
    wait "${BAG_PID}" 2>/dev/null || true
  fi
  BAG_PID=""
  if [[ -n "${ORB_PID}" ]] && kill -0 "${ORB_PID}" 2>/dev/null; then
    kill -INT "-${ORB_PID}" 2>/dev/null || kill -INT "${ORB_PID}" 2>/dev/null || true
    wait "${ORB_PID}" 2>/dev/null || true
  fi
  ORB_PID=""
}

source_if_exists() {
  local setup_file="$1"
  if [[ -f "${setup_file}" ]]; then
    # shellcheck source=/dev/null
    source "${setup_file}"
  fi
}

source_environment() {
  source_if_exists "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
  if [[ -n "${LEGACY_ROS2_WS}" ]]; then
    source_if_exists "${LEGACY_ROS2_WS}/install/setup.bash"
  fi
  source_if_exists "${ROS2_WS}/install/setup.bash"
}

verify_orb_inputs() {
  if [[ ! -f "${VOCAB}" ]]; then
    echo "ORB vocabulary not found: ${VOCAB}" >&2
    exit 1
  fi
  if [[ ! -f "${SETTINGS}" ]]; then
    echo "ORB settings yaml not found: ${SETTINGS}" >&2
    exit 1
  fi

  local load_atlas
  load_atlas="$(sed -n 's/^[[:space:]]*System.LoadAtlasFromFile:[[:space:]]*"\([^"]*\)".*/\1/p' "${SETTINGS}" | head -n 1)"
  if [[ -n "${load_atlas}" ]]; then
    local atlas_path="${load_atlas}"
    if [[ "${atlas_path}" != /* ]]; then
      atlas_path="${ROS2_WS}/${atlas_path}"
    fi
    if [[ ! -f "${atlas_path}" && ! -f "${atlas_path}.osa" ]]; then
      echo "ORB atlas requested by settings but not found: ${load_atlas}" >&2
      echo "Looked for: ${atlas_path} and ${atlas_path}.osa" >&2
      echo "Run mapping first, or use --settings with a yaml that does not load an atlas." >&2
      exit 1
    fi
  fi
}

abs_existing_path() {
  local input="$1"
  local candidate=""

  if [[ "${input}" == *"{"* || "${input}" == *"}"* || "${input}" == *"..."* ]]; then
    echo "Unexpanded bag expression: ${input}" >&2
    echo "Use Bash brace syntax with two dots, for example: data0814_{01..20}" >&2
    return 1
  fi

  if [[ "${input}" != */* && -e "${BAGS_DIR}/${input}" ]]; then
    candidate="${BAGS_DIR}/${input}"
  elif [[ -e "${input}" ]]; then
    candidate="${input}"
  else
    echo "Bag not found: ${input}" >&2
    return 1
  fi
  local directory
  directory="$(cd "$(dirname "${candidate}")" && pwd)"
  printf '%s/%s\n' "${directory}" "$(basename "${candidate}")"
}

bag_label() {
  local path="$1"
  local name
  name="$(basename "${path}")"
  name="${name%.*}"
  if [[ "${name}" == *_0 ]]; then
    name="${name%_0}"
  fi
  printf '%s\n' "${name}"
}

bag_storage_id() {
  local bag_path="$1"
  local storage_id=""

  if [[ -d "${bag_path}" ]]; then
    local metadata_path="${bag_path}/metadata.yaml"
    if [[ ! -f "${metadata_path}" ]]; then
      echo "Bag metadata not found: ${metadata_path}" >&2
      return 1
    fi
    storage_id="$(sed -n 's/^[[:space:]]*storage_identifier:[[:space:]]*\([^[:space:]#]*\).*/\1/p' "${metadata_path}" | head -n 1)"
  elif [[ "${bag_path}" == *.mcap ]]; then
    storage_id="mcap"
  elif [[ "${bag_path}" == *.db3 ]]; then
    storage_id="sqlite3"
  fi

  if [[ -z "${storage_id}" ]]; then
    echo "Unable to determine rosbag storage id: ${bag_path}" >&2
    return 1
  fi
  printf '%s\n' "${storage_id}"
}

verify_bag() {
  local bag_path="$1"
  local label="$2"
  local storage_id="$3"
  local bag_info=""
  local required_topics=(
    "/camera/camera/infra1/image_rect_raw"
    "/camera/camera/depth/image_rect_raw"
    "/imu/data"
  )

  if ! bag_info="$(ros2 bag info "${bag_path}" --storage "${storage_id}" 2>&1)"; then
    echo "[${label}] bag validation failed:" >&2
    echo "${bag_info}" >&2
    return 1
  fi

  local topic
  for topic in "${required_topics[@]}"; do
    if ! grep -Fq "Topic: ${topic} |" <<<"${bag_info}"; then
      echo "[${label}] required topic missing: ${topic}" >&2
      return 1
    fi
  done
  echo "[${label}] bag validation passed (${storage_id})"
}

check_bag_startup() {
  local bag_path="$1"
  local label="$2"
  local storage_id="$3"
  local report_dir="${TRAJECTORY_DIR}/${label}"
  local report_path="${report_dir}/startup_check.txt"
  local output=""
  local status=0

  if [[ "${RUN_STARTUP_CHECK}" -ne 1 ]]; then
    return 0
  fi
  if [[ ! -f "${STARTUP_CHECK_SCRIPT}" ]]; then
    echo "[${label}] startup checker not found: ${STARTUP_CHECK_SCRIPT}" >&2
    return 1
  fi

  mkdir -p "${report_dir}"
  set +e
  output="$(python3 -u "${STARTUP_CHECK_SCRIPT}" \
    "${bag_path}" \
    --storage-id "${storage_id}" \
    --startup-sec "${STARTUP_CHECK_SEC}" 2>&1)"
  status=$?
  set -e
  printf '%s\n' "${output}" | tee "${report_path}"

  if [[ "${status}" -eq 1 ]]; then
    echo "[${label}] invalid bag startup; skipping" >&2
    return 1
  fi
  if [[ "${status}" -eq 2 ]]; then
    echo "[${label}] WARNING: startup IMU is high risk for Atlas relocalization" >&2
    if [[ "${SKIP_HIGH_RISK_STARTUP}" -eq 1 ]]; then
      echo "[${label}] skipping high-risk startup as requested" >&2
      return 1
    fi
  fi
  return 0
}

topic_subscription_count() {
  local topic="$1"
  local output=""
  output="$(timeout 5 ros2 topic info "${topic}" 2>/dev/null || true)"
  sed -n 's/^Subscription count:[[:space:]]*\([0-9][0-9]*\).*/\1/p' <<<"${output}" | head -n 1
}

topic_publisher_count() {
  local topic="$1"
  local output=""
  output="$(timeout 5 ros2 topic info "${topic}" 2>/dev/null || true)"
  sed -n 's/^Publisher count:[[:space:]]*\([0-9][0-9]*\).*/\1/p' <<<"${output}" | head -n 1
}

verify_no_live_replay_publishers() {
  local topics=(
    "/camera/camera/infra1/image_rect_raw"
    "/camera/camera/depth/image_rect_raw"
    "/imu/data"
  )
  local conflicts=()
  local topic
  local count

  for topic in "${topics[@]}"; do
    count="$(topic_publisher_count "${topic}")"
    count="${count:-0}"
    if (( count > 0 )); then
      conflicts+=("${topic} (${count})")
    fi
  done

  if [[ ${#conflicts[@]} -eq 0 ]]; then
    return 0
  fi
  echo "Existing publishers would mix live timestamps with rosbag replay:" >&2
  printf '  %s\n' "${conflicts[@]}" >&2
  if [[ "${ALLOW_LIVE_PUBLISHERS}" -eq 1 ]]; then
    echo "WARNING: continuing because --allow-live-publishers was specified" >&2
    return 0
  fi
  echo "Stop the RealSense/Xsens launch processes and run this script again." >&2
  echo "Use --allow-live-publishers only if these publishers are intentionally isolated." >&2
  return 1
}

wait_for_orbslam_ready() {
  local label="$1"
  local waited=0
  local rgb_count=0
  local depth_count=0
  local imu_count=0

  echo "[${label}] waiting for ORB RGB/Depth/IMU subscriptions"
  while (( waited < ORB_READY_TIMEOUT_SEC )); do
    if ! kill -0 "${ORB_PID}" 2>/dev/null; then
      echo "[${label}] ORB-SLAM3 exited before becoming ready" >&2
      return 1
    fi
    if ! kill -0 "${BAG_PID}" 2>/dev/null; then
      echo "[${label}] paused rosbag player exited before ORB became ready" >&2
      return 1
    fi

    rgb_count="$(topic_subscription_count "/camera/camera/infra1/image_rect_raw")"
    depth_count="$(topic_subscription_count "/camera/camera/depth/image_rect_raw")"
    imu_count="$(topic_subscription_count "/imu/data")"
    rgb_count="${rgb_count:-0}"
    depth_count="${depth_count:-0}"
    imu_count="${imu_count:-0}"
    if (( rgb_count > 0 && depth_count > 0 && imu_count > 0 )); then
      echo "[${label}] ORB subscriptions ready: rgb=${rgb_count} depth=${depth_count} imu=${imu_count}"
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
    if (( waited % 10 == 0 )); then
      echo "[${label}] still waiting for ORB subscriptions... ${waited}s"
    fi
  done
  echo "[${label}] ORB subscriptions not ready after ${ORB_READY_TIMEOUT_SEC}s" >&2
  return 1
}

resume_bag_playback() {
  local label="$1"
  if ! timeout 15 ros2 service call \
    /rosbag2_player/resume \
    rosbag2_interfaces/srv/Resume \
    "{}" >/dev/null; then
    echo "[${label}] failed to resume paused rosbag player" >&2
    return 1
  fi
  echo "[${label}] rosbag playback resumed"
}

wait_for_bag_playback() {
  local label="$1"
  local first_status=0

  set +e
  wait -n "${BAG_PID}" "${ORB_PID}"
  first_status=$?
  set -e

  if kill -0 "${BAG_PID}" 2>/dev/null; then
    echo "[${label}] ORB-SLAM3 exited before bag playback finished" >&2
    kill -INT "${BAG_PID}" 2>/dev/null || true
    wait "${BAG_PID}" 2>/dev/null || true
    BAG_PID=""
    wait "${ORB_PID}" 2>/dev/null || true
    ORB_PID=""
    if [[ "${first_status}" -eq 0 ]]; then
      return 1
    fi
    return "${first_status}"
  fi

  BAG_PID=""
  if [[ "${first_status}" -ne 0 ]]; then
    echo "[${label}] ros2 bag play exited with status ${first_status}" >&2
    kill -INT "-${ORB_PID}" 2>/dev/null || kill -INT "${ORB_PID}" 2>/dev/null || true
    wait "${ORB_PID}" 2>/dev/null || true
    ORB_PID=""
    return "${first_status}"
  fi
  return 0
}

stop_orbslam() {
  local label="$1"
  local waited=0

  echo "[${label}] stopping ORB-SLAM3 to save trajectories"
  kill -INT "-${ORB_PID}" 2>/dev/null || kill -INT "${ORB_PID}" 2>/dev/null || true

  while kill -0 "${ORB_PID}" 2>/dev/null; do
    sleep 1
    waited=$((waited + 1))
    if (( waited % 10 == 0 )); then
      echo "[${label}] waiting for ORB-SLAM3 shutdown/save... ${waited}s"
    fi
    if (( waited >= ORB_STOP_TIMEOUT_SEC )); then
      echo "[${label}] ORB-SLAM3 did not stop after ${ORB_STOP_TIMEOUT_SEC}s" >&2
      echo "[${label}] sending SIGTERM; trajectory may be missing or incomplete" >&2
      kill -TERM "-${ORB_PID}" 2>/dev/null || kill -TERM "${ORB_PID}" 2>/dev/null || true
      wait "${ORB_PID}" 2>/dev/null || true
      ORB_PID=""
      return 1
    fi
  done

  wait "${ORB_PID}" 2>/dev/null || true
  ORB_PID=""
  return 0
}

run_orbslam_for_bag() {
  local bag_path="$1"
  local label="$2"
  local storage_id
  if ! storage_id="$(bag_storage_id "${bag_path}")"; then
    return 1
  fi
  local camera_trajectory="${OUTPUT_DIR}/CameraTrajectory.txt"
  local keyframe_trajectory="${OUTPUT_DIR}/KeyFrameTrajectory.txt"

  mkdir -p "${OUTPUT_DIR}" "${TRAJECTORY_DIR}/${label}"
  if ! verify_bag "${bag_path}" "${label}" "${storage_id}"; then
    return 1
  fi
  if ! check_bag_startup "${bag_path}" "${label}" "${storage_id}"; then
    return 1
  fi

  rm -f \
    "${camera_trajectory}" \
    "${keyframe_trajectory}" \
    "${TRAJECTORY_DIR}/${label}/CameraTrajectory.txt" \
    "${TRAJECTORY_DIR}/${label}/KeyFrameTrajectory.txt"

  echo "[${label}] starting ORB-SLAM3"
  (
    cd "${ROS2_WS}"
    exec setsid ros2 run orbslam3 rgbd-inertial \
      "${VOCAB}" \
      "${SETTINGS}" \
      "${ORB_VIEWER}" \
      "${camera_trajectory}" \
      "${keyframe_trajectory}" \
      --ros-args \
      -p imu_time_offset_sec:=0.0 \
      -p publish_pose:=false \
      -p publish_odometry:=false \
      -p publish_tf:=false \
      -p map_frame_id:=map \
      -p camera_frame_id:=camera_infra1_optical_frame \
      -r camera/rgb:=/camera/camera/infra1/image_rect_raw \
      -r camera/depth:=/camera/camera/depth/image_rect_raw
  ) &
  ORB_PID=$!

  echo "[${label}] starting paused ${storage_id} bag at rate ${BAG_RATE}"
  ros2 bag play \
    "${bag_path}" \
    --storage "${storage_id}" \
    --rate "${BAG_RATE}" \
    --start-paused \
    --disable-keyboard-controls &
  BAG_PID=$!

  if ! wait_for_orbslam_ready "${label}"; then
    cleanup
    return 1
  fi
  sleep "${ORB_WARMUP_SEC}"
  if ! resume_bag_playback "${label}"; then
    cleanup
    return 1
  fi
  if ! wait_for_bag_playback "${label}"; then
    return 1
  fi

  if ! stop_orbslam "${label}"; then
    return 1
  fi

  if ! awk 'NF >= 8 && $1 !~ /^#/ { found=1; exit } END { exit !found }' "${camera_trajectory}" 2>/dev/null; then
    echo "[${label}] missing or empty CameraTrajectory.txt" >&2
    return 1
  fi

  cp "${camera_trajectory}" "${TRAJECTORY_DIR}/${label}/CameraTrajectory.txt"
  if [[ -s "${keyframe_trajectory}" ]]; then
    cp "${keyframe_trajectory}" "${TRAJECTORY_DIR}/${label}/KeyFrameTrajectory.txt"
  fi
  echo "[${label}] saved trajectory to ${TRAJECTORY_DIR}/${label}"
  return 0
}

run_export_for_bag() {
  local bag_path="$1"
  local label="$2"
  echo "[${label}] exporting RGB images, point clouds, trajectory and gripper widths"
  ros2 run umi_dataset_tools process_rgbd_bags \
    "${bag_path}" \
    --trajectory "${TRAJECTORY_DIR}" \
    --calibration "${CALIBRATION}" \
    --output "${PROCESSED_DIR}" \
    --pointcloud-stride "${POINTCLOUD_STRIDE}" \
    --pixel-stride "${PIXEL_STRIDE}"
}

BAG_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag-rate)
      BAG_RATE="$2"
      shift 2
      ;;
    --orb-warmup-sec)
      ORB_WARMUP_SEC="$2"
      shift 2
      ;;
    --orb-ready-timeout-sec)
      ORB_READY_TIMEOUT_SEC="$2"
      shift 2
      ;;
    --orb-stop-timeout-sec)
      ORB_STOP_TIMEOUT_SEC="$2"
      shift 2
      ;;
    --startup-check-sec)
      STARTUP_CHECK_SEC="$2"
      shift 2
      ;;
    --bags-dir)
      BAGS_DIR="$2"
      shift 2
      ;;
    --trajectory-dir)
      TRAJECTORY_DIR="$2"
      shift 2
      ;;
    --processed-dir)
      PROCESSED_DIR="$2"
      shift 2
      ;;
    --runtime-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --settings)
      SETTINGS="$2"
      shift 2
      ;;
    --vocab)
      VOCAB="$2"
      shift 2
      ;;
    --calibration)
      CALIBRATION="$2"
      shift 2
      ;;
    --pointcloud-stride)
      POINTCLOUD_STRIDE="$2"
      shift 2
      ;;
    --pixel-stride)
      PIXEL_STRIDE="$2"
      shift 2
      ;;
    --no-viewer)
      ORB_VIEWER=false
      shift
      ;;
    --viewer)
      ORB_VIEWER=true
      shift
      ;;
    --domain-id)
      PIPELINE_DOMAIN_ID="$2"
      shift 2
      ;;
    --skip-orbslam)
      RUN_ORBSLAM=0
      shift
      ;;
    --skip-export)
      RUN_EXPORT=0
      shift
      ;;
    --skip-startup-check)
      RUN_STARTUP_CHECK=0
      shift
      ;;
    --skip-high-risk-startup)
      SKIP_HIGH_RISK_STARTUP=1
      shift
      ;;
    --allow-live-publishers)
      ALLOW_LIVE_PUBLISHERS=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      while [[ $# -gt 0 ]]; do
        BAG_ARGS+=("$1")
        shift
      done
      ;;
    -*)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
    *)
      BAG_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ ${#BAG_ARGS[@]} -eq 0 ]]; then
  BAG_ARGS=(data0812_01 data0812_02 data0812_03)
fi

trap cleanup EXIT INT TERM

if [[ -n "${PIPELINE_DOMAIN_ID}" ]]; then
  if [[ ! "${PIPELINE_DOMAIN_ID}" =~ ^[0-9]+$ ]] \
      || (( PIPELINE_DOMAIN_ID < 0 || PIPELINE_DOMAIN_ID > 232 )); then
    echo "Invalid --domain-id: ${PIPELINE_DOMAIN_ID}; expected an integer from 0 to 232" >&2
    exit 1
  fi
  export ROS_DOMAIN_ID="${PIPELINE_DOMAIN_ID}"
  echo "Using isolated ROS_DOMAIN_ID=${ROS_DOMAIN_ID} for bag playback and ORB-SLAM3"
fi

source_environment

BAG_PATHS=()
PATH_RESOLUTION_FAILURES=()
for bag_arg in "${BAG_ARGS[@]}"; do
  if bag_path="$(abs_existing_path "${bag_arg}")"; then
    BAG_PATHS+=("${bag_path}")
  else
    PATH_RESOLUTION_FAILURES+=("${bag_arg}")
  fi
done

SUCCESSFUL_BAG_PATHS=()
ORB_FAILURES=()
if [[ "${RUN_ORBSLAM}" -eq 1 ]]; then
  verify_orb_inputs
  if ! verify_no_live_replay_publishers; then
    exit 1
  fi
  for bag_path in "${BAG_PATHS[@]}"; do
    label="$(bag_label "${bag_path}")"
    if run_orbslam_for_bag "${bag_path}" "${label}"; then
      SUCCESSFUL_BAG_PATHS+=("${bag_path}")
    else
      cleanup
      ORB_FAILURES+=("${label}")
      echo "[${label}] SKIPPED: ORB-SLAM3 did not produce a valid camera trajectory" >&2
    fi
  done
else
  for bag_path in "${BAG_PATHS[@]}"; do
    label="$(bag_label "${bag_path}")"
    trajectory_path="${TRAJECTORY_DIR}/${label}/CameraTrajectory.txt"
    if awk 'NF >= 8 && $1 !~ /^#/ { found=1; exit } END { exit !found }' "${trajectory_path}" 2>/dev/null; then
      SUCCESSFUL_BAG_PATHS+=("${bag_path}")
    else
      ORB_FAILURES+=("${label}")
      echo "[${label}] SKIPPED: existing camera trajectory is missing or empty" >&2
    fi
  done
fi

EXPORT_FAILURES=()
if [[ "${RUN_EXPORT}" -eq 1 ]]; then
  for bag_path in "${SUCCESSFUL_BAG_PATHS[@]}"; do
    label="$(bag_label "${bag_path}")"
    if ! run_export_for_bag "${bag_path}" "${label}"; then
      EXPORT_FAILURES+=("${label}")
      echo "[${label}] SKIPPED: export failed" >&2
    fi
  done
fi

echo "============================================================"
echo "Pipeline summary"
echo "  requested: ${#BAG_ARGS[@]}"
echo "  valid trajectories: ${#SUCCESSFUL_BAG_PATHS[@]}"
echo "  path failures: ${PATH_RESOLUTION_FAILURES[*]:-none}"
echo "  ORB/precheck failures: ${ORB_FAILURES[*]:-none}"
echo "  export failures: ${EXPORT_FAILURES[*]:-none}"
echo "============================================================"
echo "done"
