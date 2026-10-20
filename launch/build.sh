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
# Keep Conda packages out of the system ROS/OpenCV dependency graph.
# PATH is also a CMake package search source, and an old cache can retain
# Conda's fmt/spdlog even after the environment has been cleaned.
if [[ -n "${CONDA_PREFIX:-}" ]]; then
  conda_prefix="${CONDA_PREFIX}"
  cache="${package_root}/build/danmucamera/CMakeCache.txt"
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
