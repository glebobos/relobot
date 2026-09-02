#!/usr/bin/env bash
#
# Convenience script to start ReloBot Gazebo simulation
#
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Default to "up" command if none provided
if [ $# -eq 0 ]; then
    exec "${SCRIPT_DIR}/start_robot.sh" up --sim
elif [ "$1" == "down" ] || [ "$1" == "logs" ] || [ "$1" == "build" ] || [ "$1" == "ps" ]; then
    CMD="$1"
    shift
    exec "${SCRIPT_DIR}/start_robot.sh" "$CMD" --sim "$@"
else
    exec "${SCRIPT_DIR}/start_robot.sh" --sim "$@"
fi
