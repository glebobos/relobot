#!/usr/bin/env bash
set -eo pipefail

case "${PICO_KNIVES_ENABLED:-true}" in
    true)
        ZENOH_CONFIG_OVERRIDE="$(python3 "$(dirname "${BASH_SOURCE[0]}")/zenoh_devices.py" \
            --config "${1:-/etc/relobot/zenoh_devices.json}" router-override)"
        export ZENOH_CONFIG_OVERRIDE
        ;;
    false)
        unset ZENOH_CONFIG_OVERRIDE
        ;;
    *)
        echo "Error: PICO_KNIVES_ENABLED must be true or false" >&2
        exit 1
        ;;
esac

source /opt/ros/lyrical/setup.bash
exec /opt/ros/lyrical/lib/rmw_zenoh_cpp/rmw_zenohd
