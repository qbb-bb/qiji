#!/usr/bin/env bash
# Create a per-project desktop launcher without replacing existing shortcuts.
set -e
script_path="${BASH_SOURCE[0]}"
[[ "${script_path}" == */* ]] || script_path="$(command -v "${script_path}")"
package_root="$(cd "${script_path%/*}/.." && pwd)"
shortcut_id="$(printf '%s' "${package_root}" | sha256sum | cut -c1-12)"
shortcut_name="qiji-calibration-${shortcut_id}.desktop"
applications_dir="${HOME}/.local/share/applications"
mkdir -p "${applications_dir}"

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

write_shortcut() {
  local target="$1"
  if [[ -e "${target}" ]]; then
    echo "保留已有启动图标：${target}"
    return
  fi
  printf '%s\n' "${shortcut_body}" > "${target}"
  chmod 755 "${target}"
  echo "已创建启动图标：${target}"
}
write_shortcut "${applications_dir}/${shortcut_name}"

desktop_dir=""
if command -v xdg-user-dir >/dev/null 2>&1; then
  desktop_dir="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
fi
if [[ -z "${desktop_dir}" || "${desktop_dir}" == "${HOME}" ]]; then
  for candidate in "${HOME}/桌面" "${HOME}/Desktop"; do
    if [[ -d "${candidate}" ]]; then desktop_dir="${candidate}"; break; fi
  done
fi
if [[ -d "${desktop_dir}" && "${desktop_dir}" != "${HOME}" ]]; then
  write_shortcut "${desktop_dir}/${shortcut_name}"
  if command -v gio >/dev/null 2>&1; then
    gio set "${desktop_dir}/${shortcut_name}" metadata::trusted true >/dev/null 2>&1 || true
  fi
fi
