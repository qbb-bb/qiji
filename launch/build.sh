#!/usr/bin/env bash
set +u
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
cd "${package_root}"
colcon --log-base "${package_root}/log" build \
  --base-paths "${package_root}" \
  --build-base "${package_root}/build" \
  --install-base "${package_root}/install" \
  --symlink-install --packages-select danmucamera "$@"
