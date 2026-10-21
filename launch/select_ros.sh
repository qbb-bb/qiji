#!/usr/bin/env bash
# Select an installed ROS 2 distro without changing the user's login shell.
if [[ -n "${QIJI_ROS_DISTRO:-}" ]]; then
  selected_ros="${QIJI_ROS_DISTRO}"
elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  selected_ros="${ROS_DISTRO}"
else
  available_ros=()
  for ros_file in /opt/ros/*/setup.bash; do
    [[ -f "${ros_file}" ]] || continue
    ros_name="${ros_file%/setup.bash}"
    available_ros+=("${ros_name##*/}")
  done
  if (( ${#available_ros[@]} == 1 )); then
    selected_ros="${available_ros[0]}"
  elif (( ${#available_ros[@]} > 1 )); then
    source /etc/os-release
    case "${VERSION_ID:-}" in
      22.04) preferred_ros=humble ;;
      24.04) preferred_ros=jazzy ;;
      *) preferred_ros= ;;
    esac
    if [[ -n "${preferred_ros}" && -f "/opt/ros/${preferred_ros}/setup.bash" ]]; then
      selected_ros="${preferred_ros}"
    else
      echo "发现多个 ROS 2 安装：${available_ros[*]}。请用 QIJI_ROS_DISTRO 选择。" >&2
      return 1
    fi
  else
    echo "未找到 ROS 2。请先运行 ./launch/setup_and_run.sh。" >&2
    return 1
  fi
fi
if [[ ! -f "/opt/ros/${selected_ros}/setup.bash" ]]; then
  echo "未找到 /opt/ros/${selected_ros}/setup.bash，请检查 QIJI_ROS_DISTRO。" >&2
  return 1
fi
export QIJI_ROS_DISTRO="${selected_ros}"
unset selected_ros available_ros ros_file ros_name preferred_ros
