#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

CMD="${1:-build}"
if [[ $# -gt 0 ]]; then shift; fi

TARGET="knives"
if [[ $# -gt 0 && ! "$1" =~ ^- ]]; then
  TARGET="$1"
  shift
fi

exec "$ROOT_DIR/run_firmware.sh" "$CMD" "$TARGET" "$@"
