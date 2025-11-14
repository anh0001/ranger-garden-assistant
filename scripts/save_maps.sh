#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
MAP_ROOT="${ROOT_DIR}/maps"
FASTLIO2_DIR="${MAP_ROOT}/fastlio2"
PROJECTED_DIR="${MAP_ROOT}/projected"
PATCH_DIR="${FASTLIO2_DIR}/patches"
TIMESTAMP="${1:-$(date +%Y%m%d_%H%M%S)}"

# Ensure ROS environment is available when called outside VSCode tasks
if [[ -z "${ROS_DISTRO:-}" ]]; then
  source /opt/ros/humble/setup.bash
fi
if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${ROOT_DIR}/install/setup.bash"
fi

mkdir -p "${FASTLIO2_DIR}" "${PROJECTED_DIR}" "${PATCH_DIR}"

FASTLIO2_PREFIX="${FASTLIO2_DIR}/map_${TIMESTAMP}"
PROJECTED_PREFIX="${PROJECTED_DIR}/projected_${TIMESTAMP}"

cat <<INFO
============================================
Saving FASTLIO2 maps (timestamp: ${TIMESTAMP})
Output root: ${MAP_ROOT}
--------------------------------------------
INFO

# Save FASTLIO2 global map and patches via PGO service
printf '1) Saving FASTLIO2 global map (.pcd + patches) -> %s[.pcd]\n' "${FASTLIO2_PREFIX}"
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: '${FASTLIO2_PREFIX}', save_patches: true}"

# Save Octomap 2D projection for Nav2
printf '2) Saving projected 2D occupancy grid (/projected_map) -> %s.yaml/.pgm\n' "${PROJECTED_PREFIX}"
ros2 run nav2_map_server map_saver_cli -t /projected_map -f "${PROJECTED_PREFIX}" --ros-args -p map_subscribe_transient_local:=true

cat <<INFO
Done!
Saved files:
  - FASTLIO2 map: ${FASTLIO2_PREFIX}.pcd (+ patches directory)
  - Nav2 projected map: ${PROJECTED_PREFIX}.yaml/.pgm
INFO
