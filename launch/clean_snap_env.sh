#!/usr/bin/env bash
# Snap-launched terminals may export their private glibc/Qt paths. Those
# libraries are incompatible with binaries built against the host Ubuntu.
for variable in PATH LD_LIBRARY_PATH LIBRARY_PATH CMAKE_PREFIX_PATH PKG_CONFIG_PATH QT_PLUGIN_PATH; do
  value="${!variable:-}"
  [[ -n "${value}" ]] || continue
  clean_entries=()
  IFS=: read -r -a entries <<< "${value}"
  for entry in "${entries[@]}"; do
    case "${entry}" in
      /snap/*|/var/lib/snapd/snap/*) ;;
      *) clean_entries+=("${entry}") ;;
    esac
  done
  printf -v "${variable}" '%s' "$(IFS=:; echo "${clean_entries[*]}")"
  export "${variable}"
done
if [[ "${QT_QPA_PLATFORM_PLUGIN_PATH:-}" == /snap/* || "${QT_QPA_PLATFORM_PLUGIN_PATH:-}" == /var/lib/snapd/snap/* ]]; then
  unset QT_QPA_PLATFORM_PLUGIN_PATH
fi
if [[ "${LD_PRELOAD:-}" == *'/snap/'* ]]; then
  unset LD_PRELOAD
fi
unset variable value clean_entries entries entry
