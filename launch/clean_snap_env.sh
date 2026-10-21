#!/usr/bin/env bash
# Remove paths that can inject libraries from a Snap editor, Conda/venv, or
# another ROS distro. This runs only in the script process, not the login shell.
for variable in PATH LD_LIBRARY_PATH LIBRARY_PATH CMAKE_PREFIX_PATH PKG_CONFIG_PATH QT_PLUGIN_PATH PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH; do
  value="${!variable:-}"
  [[ -n "${value}" ]] || continue
  clean_entries=()
  IFS=: read -r -a entries <<< "${value}"
  for entry in "${entries[@]}"; do
    case "${entry}" in
      /snap/*|/var/lib/snapd/snap/*) continue ;;
    esac
    if [[ -n "${CONDA_PREFIX:-}" && ( "${entry}" == "${CONDA_PREFIX}" || "${entry}" == "${CONDA_PREFIX}/"* ) ]]; then
      continue
    fi
    if [[ -n "${VIRTUAL_ENV:-}" && ( "${entry}" == "${VIRTUAL_ENV}" || "${entry}" == "${VIRTUAL_ENV}/"* ) ]]; then
      continue
    fi
    if [[ -n "${QIJI_ROS_DISTRO:-}" && "${entry}" == /opt/ros/* && "${entry}" != "/opt/ros/${QIJI_ROS_DISTRO}" && "${entry}" != "/opt/ros/${QIJI_ROS_DISTRO}/"* ]]; then
      continue
    fi
    clean_entries+=("${entry}")
  done
  printf -v "${variable}" '%s' "$(IFS=:; echo "${clean_entries[*]}")"
  export "${variable}"
done
if [[ "${QT_QPA_PLATFORM_PLUGIN_PATH:-}" == /snap/* || "${QT_QPA_PLATFORM_PLUGIN_PATH:-}" == /var/lib/snapd/snap/* || ( -n "${CONDA_PREFIX:-}" && "${QT_QPA_PLATFORM_PLUGIN_PATH:-}" == "${CONDA_PREFIX}/"* ) ]]; then
  unset QT_QPA_PLATFORM_PLUGIN_PATH
fi
if [[ "${LD_PRELOAD:-}" == *'/snap/'* || ( -n "${CONDA_PREFIX:-}" && "${LD_PRELOAD:-}" == *"${CONDA_PREFIX}"* ) ]]; then
  unset LD_PRELOAD
fi
unset variable value clean_entries entries entry
