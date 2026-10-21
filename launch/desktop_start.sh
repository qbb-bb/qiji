#!/usr/bin/env bash
set -e
script_path="${BASH_SOURCE[0]}"
package_root="$(cd "${script_path%/*}/.." && pwd)"
cd "${package_root}"
set +e
"${package_root}/launch/run.sh"
status=$?
set -e
if (( status != 0 )); then
  printf '\n启动失败（退出码 %s）。按 Enter 键关闭窗口。\n' "${status}"
  read -r _
fi
