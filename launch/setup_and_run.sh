#!/usr/bin/env bash
# Add only missing dependencies, then build and launch in this process.
# Existing package versions, shell startup files and user SDK settings stay intact.
set -eo pipefail

script_path="${BASH_SOURCE[0]}"
[[ "${script_path}" == */* ]] || script_path="$(command -v "${script_path}")"
package_root="$(cd "${script_path%/*}/.." && pwd)"

check_only=false
camera_mode=auto
requested_ros=auto
while (( $# )); do
  case "$1" in
    --check) check_only=true ;;
    --ros=auto|--ros=humble|--ros=jazzy) requested_ros="${1#--ros=}" ;;
    --camera=auto|--camera=orbbec|--camera=realsense|--camera=all|--camera=none)
      camera_mode="${1#--camera=}" ;;
    *) echo "未知参数：$1（支持 --check、--ros=auto|humble|jazzy、--camera=auto|orbbec|realsense|all|none）" >&2; exit 2 ;;
  esac
  shift
done

source /etc/os-release
if [[ "${ID:-}" != ubuntu ]]; then
  echo "此安装流程支持 Ubuntu 22.04/24.04；当前为 ${PRETTY_NAME:-unknown}。未改动系统。" >&2
  exit 1
fi
case "${VERSION_ID:-}" in
  22.04) default_ros=humble; ubuntu_codename=jammy ;;
  24.04) default_ros=jazzy; ubuntu_codename=noble ;;
  *) echo "当前 Ubuntu ${VERSION_ID:-unknown} 没有此项目验证过的 ROS 2 安装组合；未改动系统。" >&2; exit 1 ;;
esac
installed_ros=()
for setup_file in /opt/ros/*/setup.bash; do
  [[ -f "${setup_file}" ]] || continue
  installed_name="${setup_file%/setup.bash}"
  installed_ros+=("${installed_name##*/}")
done
if [[ "${requested_ros}" != auto ]]; then
  ros_distro="${requested_ros}"
elif [[ -n "${QIJI_ROS_DISTRO:-}" && -f "/opt/ros/${QIJI_ROS_DISTRO}/setup.bash" ]]; then
  ros_distro="${QIJI_ROS_DISTRO}"
elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  ros_distro="${ROS_DISTRO}"
elif (( ${#installed_ros[@]} == 1 )); then
  ros_distro="${installed_ros[0]}"
elif (( ${#installed_ros[@]} > 1 )); then
  if [[ -f "/opt/ros/${default_ros}/setup.bash" ]]; then
    ros_distro="${default_ros}"
  else
    echo "发现多个 ROS 2 安装：${installed_ros[*]}。请用 --ros=humble 或 --ros=jazzy 选择，未改动系统。" >&2
    exit 1
  fi
else
  ros_distro="${default_ros}"
fi
if [[ "${ros_distro}" != humble && "${ros_distro}" != jazzy ]]; then
  echo "现有 ROS ${ros_distro} 未经过此项目验证；请选择 Humble/Jazzy，未改动系统。" >&2
  exit 1
fi
if (( ${#installed_ros[@]} > 0 )) && [[ ! -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
  echo "已安装 ROS ${installed_ros[*]}；不会额外安装 ${ros_distro}。请选择现有发行版。" >&2
  exit 1
fi
ros_existing=false
[[ -f "/opt/ros/${ros_distro}/setup.bash" ]] && ros_existing=true
export QIJI_ROS_DISTRO="${ros_distro}"
source "${script_path%/*}/clean_snap_env.sh"
case "$(dpkg --print-architecture)" in
  amd64|arm64) ;;
  *) echo "此安装流程需要 amd64 或 arm64；未改动系统。" >&2; exit 1 ;;
esac

if [[ "${check_only}" == false && -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "未检测到图形桌面。请在登录桌面后的终端运行；未改动系统。" >&2
  exit 1
fi
if [[ "${check_only}" == false && "${EUID}" == 0 ]]; then
  echo "请以普通桌面用户运行脚本；需要安装时脚本会单独调用 sudo。" >&2
  exit 1
fi

is_installed() {
  [[ "$(dpkg-query -W -f='${Status}' "$1" 2>/dev/null || true)" == 'install ok installed' ]]
}
apt_has() { apt-cache show "$1" >/dev/null 2>&1; }
apt_refreshed=false
refresh_apt() {
  if [[ "${apt_refreshed}" == false ]]; then
    sudo apt-get update
    apt_refreshed=true
  fi
}
install_missing() {
  local package
  local missing=()
  for package in "$@"; do
    is_installed "${package}" || missing+=("${package}")
  done
  (( ${#missing[@]} )) || return 0
  echo "缺少软件包：${missing[*]}"
  if [[ "${check_only}" == true ]]; then return 0; fi
  refresh_apt
  for package in "${missing[@]}"; do
    if ! apt_has "${package}"; then
      echo "软件源中找不到 ${package}；保留现有系统配置，请检查网络和软件源。" >&2
      exit 1
    fi
  done
  # Installing new packages is allowed; changing or removing installed ones is not.
  simulation="$(apt-get -s --no-upgrade --no-remove install "${missing[@]}")"
  while read -r action candidate rest; do
    if [[ "${action}" == Inst ]] && is_installed "${candidate}"; then
      echo "补装依赖需要升级已安装软件 ${candidate}；为保留现有环境，安装已停止。" >&2
      exit 1
    fi
  done <<< "${simulation}"
  sudo apt-get --no-upgrade --no-remove -y install "${missing[@]}"
}

has_orbbec_sdk() {
  local prefix libdir
  local prefixes=("/opt/ros/${ros_distro}" /usr/local /usr)
  if [[ -n "${CMAKE_PREFIX_PATH:-}" ]]; then
    IFS=: read -r -a extra_prefixes <<< "${CMAKE_PREFIX_PATH}"
    prefixes+=("${extra_prefixes[@]}")
  fi
  for prefix in "${prefixes[@]}"; do
    [[ -f "${prefix}/include/libobsensor/ObSensor.hpp" ]] || continue
    for libdir in "${prefix}/lib" "${prefix}/lib64" "${prefix}/lib/$(gcc -dumpmachine 2>/dev/null || true)"; do
      [[ -e "${libdir}/libOrbbecSDK.so" ]] && return 0
    done
  done
  return 1
}
has_realsense_sdk() {
  is_installed librealsense2-dev || {
    [[ -f /usr/include/librealsense2/rs.hpp || -f /usr/local/include/librealsense2/rs.hpp ]] &&
      ldconfig -p 2>/dev/null | grep -q 'librealsense2.so'
  }
}

orbbec_detected=false
realsense_detected=false
for vendor_file in /sys/bus/usb/devices/*/idVendor; do
  [[ -f "${vendor_file}" ]] || continue
  read -r vendor < "${vendor_file}" || continue
  case "${vendor,,}" in
    2bc5) orbbec_detected=true ;;
    8086)
      product_file="${vendor_file%/*}/product"
      if [[ -r "${product_file}" ]]; then
        read -r product < "${product_file}" || true
        [[ "${product,,}" == *realsense* ]] && realsense_detected=true
      fi ;;
  esac
done
case "${camera_mode}" in
  orbbec) orbbec_detected=true ;;
  realsense) realsense_detected=true ;;
  all) orbbec_detected=true; realsense_detected=true ;;
  none) orbbec_detected=false; realsense_detected=false ;;
esac
if [[ "${ros_existing}" == true ]]; then ros_status=现有; else ros_status=待安装; fi
echo "系统：${PRETTY_NAME}；ROS 2：${ros_distro}（${ros_status}）；Orbbec：${orbbec_detected}；RealSense：${realsense_detected}"

packages=(build-essential cmake python3-colcon-common-extensions
  qt6-base-dev qt6-qpa-plugins libopencv-dev libopencv-contrib-dev v4l-utils)
if [[ "${ros_existing}" == false ]] || is_installed "ros-${ros_distro}-ros-base" || is_installed "ros-${ros_distro}-rclcpp"; then
  packages+=("ros-${ros_distro}-ros-base" "ros-${ros_distro}-ament-cmake"
    "ros-${ros_distro}-rclcpp" "ros-${ros_distro}-rcl-interfaces"
    "ros-${ros_distro}-sensor-msgs" "ros-${ros_distro}-std-msgs" "ros-${ros_distro}-std-srvs"
    "ros-${ros_distro}-launch" "ros-${ros_distro}-launch-ros")
else
  echo "检测到自定义安装的 ROS ${ros_distro}；保留它并在编译时验证依赖。"
fi
/usr/bin/python3 -c 'import catkin_pkg' >/dev/null 2>&1 || packages+=(python3-catkin-pkg)
/usr/bin/python3 -c 'import ament_package' >/dev/null 2>&1 || packages+=(python3-ament-package)
[[ -n "${WAYLAND_DISPLAY:-}" ]] && packages+=(qt6-wayland)

if [[ "${check_only}" == true ]]; then
  install_missing "${packages[@]}"
  if [[ "${orbbec_detected}" == true ]] && ! has_orbbec_sdk; then echo "缺少 Orbbec SDK v2"; fi
  if [[ "${realsense_detected}" == true ]] && ! has_realsense_sdk; then echo "缺少 librealsense2 开发包"; fi
  echo "检查完成；没有安装软件、编译或启动。"
  exit 0
fi

if [[ "${ros_existing}" == false ]] && ! apt_has "ros-${ros_distro}-ros-base"; then
  refresh_apt
  if ! apt_has "ros-${ros_distro}-ros-base"; then
    if is_installed ros2-apt-source; then
      echo "已安装 ros2-apt-source，但找不到 ROS ${ros_distro} 软件包；未修改现有软件源。" >&2
      exit 1
    fi
    install_missing curl ca-certificates software-properties-common
    if ! apt_has qt6-base-dev; then
      sudo add-apt-repository -y universe
      sudo apt-get update
    fi
    temp_dir="$(mktemp -d)"
    trap 'rm -rf -- "${temp_dir}"' EXIT
    source_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | /usr/bin/python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
    curl -fL --retry 3 -o "${temp_dir}/ros2-apt-source.deb" \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${source_version}/ros2-apt-source_${source_version}.${ubuntu_codename}_all.deb"
    sudo dpkg -i "${temp_dir}/ros2-apt-source.deb"
    sudo apt-get update
    rm -rf -- "${temp_dir}"
    trap - EXIT
    apt_refreshed=true
  fi
fi
if ! apt_has qt6-base-dev || ! apt_has libopencv-contrib-dev; then
  refresh_apt
  if ! apt_has qt6-base-dev || ! apt_has libopencv-contrib-dev; then
    install_missing software-properties-common
    sudo add-apt-repository -y universe
    sudo apt-get update
    apt_refreshed=true
  fi
fi
install_missing "${packages[@]}"

if [[ "${orbbec_detected}" == true ]] && ! has_orbbec_sdk; then
  install_missing "ros-${ros_distro}-orbbec-camera"
fi
if [[ "${orbbec_detected}" == true ]]; then
  rule_source="/opt/ros/${ros_distro}/share/orbbec_camera/udev/99-obsensor-libusb.rules"
  rule_target=/etc/udev/rules.d/99-obsensor-libusb.rules
  if [[ ! -e "${rule_target}" && ! -e /usr/lib/udev/rules.d/99-obsensor-libusb.rules ]]; then
    downloaded_rule=false
    if [[ ! -f "${rule_source}" ]]; then
      install_missing curl ca-certificates
      rule_source="$(mktemp)"
      downloaded_rule=true
      trap 'rm -f -- "${rule_source}"' EXIT
      curl -fsSL -o "${rule_source}" https://raw.githubusercontent.com/orbbec/OrbbecSDK_ROS2/v2-main/orbbec_camera/udev/99-obsensor-libusb.rules
    fi
    sudo install -m 0644 "${rule_source}" "${rule_target}"
    if [[ "${downloaded_rule}" == true ]]; then
      rm -f -- "${rule_source}"
      trap - EXIT
    fi
    sudo udevadm control --reload-rules
    sudo udevadm trigger --subsystem-match=usb --attr-match=idVendor=2bc5 || echo "请重新插拔 Orbbec 相机以加载新权限规则。"
  fi
fi

if [[ "${realsense_detected}" == true ]] && ! has_realsense_sdk; then
  if ! apt_has librealsense2-dev; then
    refresh_apt
  fi
  if ! apt_has librealsense2-dev; then
    if [[ -e /etc/apt/sources.list.d/librealsense.list ]]; then
      echo "已有 RealSense 软件源配置，但找不到 librealsense2-dev；未改动现有配置。" >&2
      exit 1
    fi
    install_missing curl ca-certificates gnupg
    temp_dir="$(mktemp -d)"
    trap 'rm -rf -- "${temp_dir}"' EXIT
    curl -fsSL -o "${temp_dir}/realsense.asc" https://librealsense.realsenseai.com/Debian/librealsenseai.asc
    gpg --dearmor --output "${temp_dir}/librealsenseai.gpg" "${temp_dir}/realsense.asc"
    sudo install -d -m 0755 /etc/apt/keyrings
    if [[ ! -e /etc/apt/keyrings/librealsenseai.gpg ]]; then
      sudo install -m 0644 "${temp_dir}/librealsenseai.gpg" /etc/apt/keyrings/librealsenseai.gpg
    fi
    printf 'deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo %s main\n' "${ubuntu_codename}" |
      sudo tee /etc/apt/sources.list.d/librealsense.list >/dev/null
    sudo apt-get update
    rm -rf -- "${temp_dir}"
    trap - EXIT
    apt_refreshed=true
  fi
  install_missing librealsense2-dev librealsense2-udev-rules
fi
if [[ "${realsense_detected}" == true && ! -e /etc/udev/rules.d/99-realsense-libusb.rules && ! -e /usr/lib/udev/rules.d/99-realsense-libusb.rules ]]; then
  install_missing curl ca-certificates
  rule_source="$(mktemp)"
  trap 'rm -f -- "${rule_source}"' EXIT
  curl -fsSL -o "${rule_source}" https://raw.githubusercontent.com/realsenseai/librealsense/master/config/99-realsense-libusb.rules
  sudo install -m 0644 "${rule_source}" /etc/udev/rules.d/99-realsense-libusb.rules
  rm -f -- "${rule_source}"
  trap - EXIT
  sudo udevadm control --reload-rules
  sudo udevadm trigger --subsystem-match=usb --attr-match=idVendor=8086 || echo "请重新插拔 RealSense 相机以加载新权限规则。"
fi

unset ROS_DISTRO
source "/opt/ros/${ros_distro}/setup.bash"
source "${script_path%/*}/clean_snap_env.sh"
/usr/bin/python3 -c 'import catkin_pkg, ament_package, launch, launch_ros'
"${package_root}/launch/build.sh"
"${package_root}/launch/create_shortcut.sh"
echo "编译成功，正在启动标定程序…"
exec "${package_root}/launch/run.sh"
