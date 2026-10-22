#!/usr/bin/env bash
# Replace old qiji launchers with one shortcut for the current project path.
set -e
script_path="${BASH_SOURCE[0]}"
[[ "${script_path}" == */* ]] || script_path="$(command -v "${script_path}")"
package_root="$(cd "${script_path%/*}/.." && pwd)"
shortcut_name="qiji-calibration.desktop"
applications_dir="${HOME}/.local/share/applications"
desktop_dir=""
if command -v xdg-user-dir >/dev/null 2>&1; then
  desktop_dir="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
fi
if [[ -z "${desktop_dir}" || "${desktop_dir}" == "${HOME}" || ! -d "${desktop_dir}" ]]; then
  desktop_dir=""
  for candidate in "${HOME}/桌面" "${HOME}/Desktop"; do
    if [[ -d "${candidate}" ]]; then desktop_dir="${candidate}"; break; fi
  done
fi

# Old versions wrote one shortcut to the menu and another to the desktop.
# Remove only qiji launchers from those locations before creating the new one.
shopt -s nullglob
for location in "${applications_dir}" "${HOME}/桌面" "${HOME}/Desktop" "${desktop_dir}"; do
  [[ -d "${location}" ]] || continue
  for old_shortcut in "${location}"/qiji-calibration*.desktop; do
    rm -- "${old_shortcut}"
    echo "已删除旧启动图标：${old_shortcut}"
  done
done

escape_exec() {
  local value="$1"
  value="${value//\\/\\\\}"
  value="${value//\"/\\\"}"
  value="${value//\$/\\\$}"
  value="${value//\`/\\\`}"
  printf '%s' "${value}"
}
shortcut_body="[Desktop Entry]
Type=Application
Name=相机标定工作台
Comment=启动 qiji ROS 2 相机标定界面
Exec=/bin/bash \"$(escape_exec "${package_root}/launch/desktop_start.sh")\"
Path=${package_root}
Icon=camera-photo
Terminal=true
Categories=Graphics;
StartupNotify=true"

if [[ -n "${desktop_dir}" ]]; then
  shortcut_dir="${desktop_dir}"
else
  shortcut_dir="${applications_dir}"
  mkdir -p "${shortcut_dir}"
fi
shortcut_path="${shortcut_dir}/${shortcut_name}"
printf '%s\n' "${shortcut_body}" > "${shortcut_path}"
chmod 755 "${shortcut_path}"
echo "已创建启动图标：${shortcut_path}"
if [[ "${shortcut_dir}" == "${desktop_dir}" ]]; then
  if command -v gio >/dev/null 2>&1; then
    gio set "${shortcut_path}" metadata::trusted true >/dev/null 2>&1 || true
  fi
fi
