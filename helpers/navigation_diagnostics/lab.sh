#!/usr/bin/env bash
set -euo pipefail

LAB_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${1:-}" == "_container" ]]; then
    shift
    set +u
    source /opt/ros/humble/setup.bash
    source /lab/install/setup.bash
    source /opt/nav2_mppi_install/setup.bash
    export PYTHONPATH="/repo/ros2_ws/src/nav2:${PYTHONPATH:-}"
    set -u
    exec "$@"
fi

COMMAND="${1:-help}"
if [[ $# -gt 0 ]]; then
    shift
fi
case "$COMMAND" in
    build)
        docker compose -f "$LAB_DIR/compose.yml" build "$@"
        ;;
    test)
        docker compose -f "$LAB_DIR/compose.yml" run --rm --no-deps -T lab \
            python3 -m pytest -q -s --tb=short -p no:cacheprovider \
            /repo/ros2_ws/src/nav2/test "$@"
        ;;
    analyze)
        if [[ $# != 1 || ! "$1" =~ ^[a-zA-Z0-9_-]+$ ]]; then
            printf 'Usage: lab.sh analyze RUN_ID\n' >&2
            exit 2
        fi
        docker compose -f "$LAB_DIR/compose.yml" run --rm --no-deps -T lab \
            python3 /repo/helpers/navigation_diagnostics/analyze.py \
            "/repo/ros2_ws/log/navigation/$1"
        ;;
    help|--help|-h)
        printf '%s\n' \
            'Navigation lab (no robot services or devices)' \
            '  bash helpers/navigation_diagnostics/lab.sh build' \
            '  bash helpers/navigation_diagnostics/lab.sh test [-k expression]' \
            '  bash helpers/navigation_diagnostics/lab.sh analyze RUN_ID' \
            'Build the production Nav2 base image with ./start_robot.sh build ros2_nav2 --sim --headless first.'
        ;;
    *)
        printf 'Unknown lab command: %s\n' "$COMMAND" >&2
        exit 2
        ;;
esac