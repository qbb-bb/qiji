#!/usr/bin/env bash
# Ubuntu 22.04: install this project's system dependencies, build, and launch.
set -eo pipefail

script_path="${BASH_SOURCE[0]}"
[[ "${script_path}" == */* ]] || script_path="$(command -v "${script_path}")"
source "${script_path%/*}/clean_snap_env.sh"
package_root="$(cd "${script_path%/*}/.." && pwd)"
source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 22.04 ]]; then
  echo "此脚本仅支持 Ubuntu 22.04；当前系统为 ${PRETTY_NAME:-unknown}。" >&2
  exit 1
fi
case "$(dpkg --print-architecture)" in
  amd64|arm64) ;;
  *) echo "ROS 2 Humble 的此安装流程仅支持 amd64/arm64。" >&2; exit 1 ;;
esac
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "未检测到图形桌面会话。请在新电脑的桌面终端运行此脚本。" >&2
  exit 1
fi

is_installed() {
  [[ "$(dpkg-query -W -f='${Status}' "$1" 2>/dev/null || true)" == "install ok installed" ]]
}

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "安装 ROS 2 Humble 软件源…"
  if ! is_installed ros2-apt-source; then
    sudo apt-get update
    sudo apt-get install -y software-properties-common curl ca-certificates
    sudo add-apt-repository -y universe
    ros_source_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | /usr/bin/python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
    ros_source_deb="$(mktemp --suffix=.deb)"
    trap 'rm -f -- "${ros_source_deb}"' EXIT
    curl -fL --retry 3 -o "${ros_source_deb}" \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_source_version}/ros2-apt-source_${ros_source_version}.jammy_all.deb"
    sudo dpkg -i "${ros_source_deb}"
    rm -f -- "${ros_source_deb}"
    trap - EXIT
  fi
  sudo apt-get update
fi

packages=(
  build-essential cmake python3-colcon-common-extensions
  python3-catkin-pkg python3-ament-package
  ros-humble-ros-base ros-humble-ament-cmake
  ros-humble-rclcpp ros-humble-rcl-interfaces
  ros-humble-sensor-msgs ros-humble-std-msgs ros-humble-std-srvs
  ros-humble-launch ros-humble-launch-ros
  qt6-base-dev libopencv-dev libopencv-contrib-dev v4l-utils
)
missing=()
for package in "${packages[@]}"; do
  is_installed "${package}" || missing+=("${package}")
done
if (( ${#missing[@]} )); then
  echo "安装缺少的依赖：${missing[*]}"
  sudo apt-get update
  sudo apt-get install -y "${missing[@]}"
fi

source /opt/ros/humble/setup.bash
"${package_root}/launch/build.sh"
echo "编译成功，正在启动标定程序…"
exec "${package_root}/launch/run.sh"
