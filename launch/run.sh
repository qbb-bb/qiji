#!/usr/bin/env bash
set -e
script_path="${BASH_SOURCE[0]}"
[[ "${script_path}" == */* ]] || script_path="$(command -v "${script_path}")"
package_root="$(cd "${script_path%/*}/.." && pwd)"
source "${script_path%/*}/select_ros.sh"
source "${script_path%/*}/clean_snap_env.sh"
unset ROS_DISTRO
source "/opt/ros/${QIJI_ROS_DISTRO}/setup.bash"
install_setup="${package_root}/install/setup.bash"
if [[ ! -f "${install_setup}" ]]; then
  echo "尚未编译，请先运行 ${package_root}/launch/build.sh" >&2
  exit 1
fi
source "${install_setup}"
# ROS/colcon setup files can re-add paths inherited when the workspace was built
# inside a Snap editor. Clean once more after all setup scripts have run.
source "${script_path%/*}/clean_snap_env.sh"
cd "${package_root}"
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "未检测到图形桌面会话；请在登录桌面后的终端运行。" >&2
  exit 1
fi
if [[ -z "${QT_QPA_PLATFORM:-}" && -n "${WAYLAND_DISPLAY:-}" ]] &&
   [[ "$(dpkg-query -W -f='${Status}' qt6-wayland 2>/dev/null || true)" == 'install ok installed' ]]; then
  export QT_QPA_PLATFORM=wayland
fi
ui_binary="${package_root}/install/danmucamera/lib/danmucamera/calibration_ui"
if [[ ! -x "${ui_binary}" ]]; then
  echo "未找到标定界面程序，请先运行 ${package_root}/launch/build.sh。" >&2
  exit 1
fi
if command -v ldd >/dev/null 2>&1; then
  libraries="$(ldd "${ui_binary}")"
  if [[ "${libraries}" == *'not found'* ]]; then
    echo "界面缺少动态库：" >&2
    printf '%s\n' "${libraries}" | grep 'not found' >&2
    echo "请运行 ${package_root}/launch/setup_and_run.sh 补齐依赖。" >&2
    exit 1
  fi
  if [[ "${libraries}" == *'/snap/'* ]]; then
    echo "界面仍链接到 Snap 自带动态库；请运行 ${package_root}/launch/build.sh 重建。" >&2
    exit 1
  fi
fi
qt_plugin_dir="$(find /usr/lib -type d -path '*/qt6/plugins/platforms' -print -quit 2>/dev/null)"
if [[ -z "${qt_plugin_dir}" ]]; then
  echo "未找到 Qt 6 平台插件；请运行 ${package_root}/launch/setup_and_run.sh。" >&2
  exit 1
fi
case "${QT_QPA_PLATFORM:-xcb}" in
  wayland*) qt_plugin="$(find "${qt_plugin_dir}" -maxdepth 1 -name 'libqwayland*.so' -print -quit)" ;;
  xcb) qt_plugin="${qt_plugin_dir}/libqxcb.so" ;;
  *) qt_plugin="" ;;
esac
if [[ -n "${qt_plugin}" ]]; then
  if [[ ! -f "${qt_plugin}" ]]; then
    echo "缺少 Qt 平台插件 ${QT_QPA_PLATFORM:-xcb}；请运行 ${package_root}/launch/setup_and_run.sh。" >&2
    exit 1
  fi
  if command -v ldd >/dev/null 2>&1; then
    plugin_libraries="$(ldd "${qt_plugin}")"
    if [[ "${plugin_libraries}" == *'not found'* ]]; then
      echo "Qt 平台插件缺少动态库：" >&2
      printf '%s\n' "${plugin_libraries}" | grep 'not found' >&2
      echo "请运行 ${package_root}/launch/setup_and_run.sh 补齐依赖。" >&2
      exit 1
    fi
  fi
fi
export ROS_LOG_DIR="${package_root}/log/ros"
mkdir -p "${ROS_LOG_DIR}"
exec ros2 launch danmucamera calibration.launch.py
