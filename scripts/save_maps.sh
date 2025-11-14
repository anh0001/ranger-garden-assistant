#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
MAP_ROOT="${ROOT_DIR}/maps"
FASTLIO2_DIR="${MAP_ROOT}/fastlio2"
PROJECTED_DIR="${MAP_ROOT}/projected"
TIMESTAMP="${1:-$(date +%Y%m%d_%H%M%S)}"
FASTLIO2_OUTPUT_DIR="${FASTLIO2_DIR}/map_${TIMESTAMP}"

# Ensure ROS environment is available when called outside VSCode tasks
# Set trace variables to avoid 'unbound variable' errors with bash set -u
export COLCON_TRACE="${COLCON_TRACE:-}"
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-$(command -v python3)}"
export COLCON_PYTHON_EXECUTABLE="${COLCON_PYTHON_EXECUTABLE:-$(command -v python3)}"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  source /opt/ros/humble/setup.bash
fi
if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${ROOT_DIR}/install/setup.bash"
fi

mkdir -p "${FASTLIO2_OUTPUT_DIR}" "${PROJECTED_DIR}"

PROJECTED_PREFIX="${PROJECTED_DIR}/projected_${TIMESTAMP}"

cat <<INFO
============================================
Saving FASTLIO2 maps (timestamp: ${TIMESTAMP})
Output root: ${MAP_ROOT}
--------------------------------------------
INFO

# Save FASTLIO2 global map and patches via PGO service
printf '1) Saving FASTLIO2 global map (.pcd + patches) -> %s\n' "${FASTLIO2_OUTPUT_DIR}"
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: '${FASTLIO2_OUTPUT_DIR}', save_patches: true}"

# Save Octomap 2D projection for Nav2
printf '2) Saving projected 2D occupancy grid (/projected_map) -> %s.yaml/.pgm\n' "${PROJECTED_PREFIX}"
if ros2 topic info /projected_map &>/dev/null; then
  # Check if topic is actively publishing
  printf '   Waiting for /projected_map message...\n'
  if timeout 5s ros2 topic echo /projected_map --once &>/dev/null; then
    # Topic is publishing, now save the map
    # Match QoS: octomap_server publishes with VOLATILE durability by default
    timeout 15s ros2 run nav2_map_server map_saver_cli -t /projected_map -f "${PROJECTED_PREFIX}" \
      --ros-args \
      -p map_subscribe_transient_local:=false \
      -p save_map_timeout:=10000.0 || {
      echo "Warning: Failed to save projected map. Check that octomap_server is publishing regularly."
    }
  else
    echo "Warning: /projected_map is not publishing. Skipping projected map save."
  fi
else
  echo "Warning: /projected_map topic not available. Skipping projected map save."
fi

cat <<INFO
Done!
Saved files:
  - FASTLIO2 map directory: ${FASTLIO2_OUTPUT_DIR}/ (map.pcd, poses.txt, patches/)
  - Nav2 projected map: ${PROJECTED_PREFIX}.yaml/.pgm
INFO
