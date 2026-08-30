#!/usr/bin/env bash
# ==============================================================================
# ReloBot CPU & Memory Profiler Launcher
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROFILER_SCRIPT="${SCRIPT_DIR}/helpers/cpu_profiler/cpu_profiler.py"

if [ ! -f "${PROFILER_SCRIPT}" ]; then
  echo "Error: CPU profiler script not found at ${PROFILER_SCRIPT}"
  exit 1
fi

python3 "${PROFILER_SCRIPT}" "$@"
