#!/usr/bin/env bash
set -eo pipefail

SHM_OVERRIDE="${ZENOH_ROUTER_SHM_OVERRIDE:-transport/shared_memory/enabled=true;transport/shared_memory/mode=\"lazy\";transport/shared_memory/transport_optimization/enabled=true;transport/shared_memory/transport_optimization/pool_size=${ZENOH_SHM_ALLOC_SIZE:-16777216};transport/shared_memory/transport_optimization/message_size_threshold=${ZENOH_SHM_MESSAGE_SIZE_THRESHOLD:-512}}"

ROUTER_ENDPOINTS_OVERRIDE="$(python3 "$(dirname "${BASH_SOURCE[0]}")/zenoh_devices.py" \
    --config "${1:-/etc/relobot/zenoh_devices.json}" router-override)"
ZENOH_CONFIG_OVERRIDE="${ROUTER_ENDPOINTS_OVERRIDE};${SHM_OVERRIDE}"
export ZENOH_CONFIG_OVERRIDE

source /opt/ros/lyrical/setup.bash
exec /opt/ros/lyrical/lib/rmw_zenoh_cpp/rmw_zenohd
