#!/usr/bin/env bash
set -e
package_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if ! command -v ros2 >/dev/null 2>&1; then
  ros_setup="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
  if [[ ! -f "${ros_setup}" ]]; then
    echo "未找到 ROS 2。请先安装并 source 对应发行版，或设置 ROS_DISTRO。" >&2
    exit 1
  fi
  source "${ros_setup}"
fi
install_setup="${package_root}/install/setup.bash"
if [[ ! -f "${install_setup}" ]]; then
  echo "尚未编译，请先运行 ${package_root}/launch/build.sh" >&2
  exit 1
fi
source "${install_setup}"
cd "${package_root}"
export ROS_LOG_DIR="${package_root}/log/ros"
mkdir -p "${ROS_LOG_DIR}"
exec ros2 launch danmucamera calibration.launch.py
