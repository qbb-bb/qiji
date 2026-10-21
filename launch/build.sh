#!/usr/bin/env bash
set +u
set -e
script_path="${BASH_SOURCE[0]}"
[[ "${script_path}" == */* ]] || script_path="$(command -v "${script_path}")"
package_root="$(cd "${script_path%/*}/.." && pwd)"
source "${script_path%/*}/select_ros.sh"
source "${script_path%/*}/clean_snap_env.sh"
unset ROS_DISTRO
source "/opt/ros/${QIJI_ROS_DISTRO}/setup.bash"
source "${script_path%/*}/clean_snap_env.sh"
cd "${package_root}"
# CMake can retain a Snap runtime path from a previous build in its cache.
cache="${package_root}/build/danmucamera/CMakeCache.txt"
ui_binary="${package_root}/build/danmucamera/calibration_ui"
cache_home=""
if [[ -f "${cache}" ]]; then
  cache_home="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "${cache}")"
fi
if { [[ -n "${cache_home}" && "${cache_home}" != "${package_root}" ]]; } || \
   { [[ -f "${cache}" ]] && grep -Fq '/snap/' "${cache}"; } || \
   { [[ -f "${cache}" && -n "${QIJI_ROS_DISTRO:-}" ]] && grep -E '/opt/ros/[^/]+' "${cache}" | grep -Fvq "/opt/ros/${QIJI_ROS_DISTRO}/"; } || \
   { [[ -f "${ui_binary}" ]] && command -v readelf >/dev/null 2>&1 && readelf -d "${ui_binary}" | grep -Fq '/snap/'; }; then
  rm -rf -- "${package_root}/build/danmucamera"
fi
# Keep Conda packages out of the system ROS/OpenCV dependency graph.
# PATH is also a CMake package search source, and an old cache can retain
# Conda's fmt/spdlog even after the environment has been cleaned.
if [[ -n "${CONDA_PREFIX:-}" ]]; then
  conda_prefix="${CONDA_PREFIX}"
  if [[ -f "${cache}" ]] && grep -Fq "${conda_prefix}/" "${cache}"; then
    rm -rf -- "${package_root}/build/danmucamera"
  fi
  clean_path=()
  IFS=: read -r -a path_entries <<< "${PATH}"
  for entry in "${path_entries[@]}"; do
    [[ "${entry}" == "${conda_prefix}" || "${entry}" == "${conda_prefix}/"* ]] || clean_path+=("${entry}")
  done
  PATH="$(IFS=:; echo "${clean_path[*]}")"
  export PATH
  unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_EXE CONDA_PYTHON_EXE CONDA_PROMPT_MODIFIER
fi
# ROS 2 installed from OS packages uses the system Python. An active Conda
# environment can otherwise make CMake cache a Python without ROS modules.
ros_python="${ROS_PYTHON_EXECUTABLE:-/usr/bin/python3}"
if [[ ! -x "${ros_python}" ]] || ! "${ros_python}" -c 'import catkin_pkg, ament_package' >/dev/null 2>&1; then
  echo "${ros_python} 缺少 ROS 2 构建所需的 catkin_pkg/ament_package；请设置 ROS_PYTHON_EXECUTABLE。" >&2
  exit 1
fi
colcon --log-base "${package_root}/log" build \
  --base-paths "${package_root}" \
  --build-base "${package_root}/build" \
  --install-base "${package_root}/install" \
  --symlink-install --packages-select danmucamera \
  --cmake-args "-DPython3_EXECUTABLE=${ros_python}" "$@"
