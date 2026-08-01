#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly ROS_SETUP="/opt/ros/jazzy/setup.bash"

if [[ ! -r "${ROS_SETUP}" ]]; then
  echo "Khong tim thay ROS 2 Jazzy tai ${ROS_SETUP}." >&2
  echo "Hay cai ROS 2 Jazzy Desktop, Nav2 va ros_gz truoc khi build." >&2
  exit 1
fi

# ROS setup files probe optional environment variables and are not compatible
# with nounset while they are being sourced.
set +u
# shellcheck disable=SC1091
source "${ROS_SETUP}"
set -u
cd "${WORKSPACE_ROOT}"

if command -v rosdep >/dev/null 2>&1; then
  # ament_python is the colcon build type used by the benchmark package. On
  # Jazzy it has no standalone rosdep rule, while the required tooling is
  # already supplied by ros-dev-tools / colcon.
  rosdep install \
    --from-paths src \
    --ignore-src \
    --rosdistro jazzy \
    --skip-keys ament_python \
    -r -y
else
  echo "Canh bao: khong co rosdep; bo qua buoc cai dependency." >&2
fi

colcon build --symlink-install

echo
echo "Build hoan tat. Trong terminal nay, chay:"
echo "  source \"${WORKSPACE_ROOT}/install/setup.bash\""
echo "  ros2 launch vacuum_robot_gazebo switchable_simulation.launch.py gui:=true"
