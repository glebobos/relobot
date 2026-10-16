#!/usr/bin/env bash
set -euo pipefail

if [[ ! -f /.dockerenv || $# != 1 || -e "$1" ]]; then
    echo "Usage inside an isolated Docker container: bash $0 <fresh-evidence-dir>" >&2
    exit 2
fi

evidence="$1"
mkdir -p "$evidence"
evidence="$(cd "$evidence" && pwd)"
router_pid=""
observer_pid=""

cleanup() {
    for process in "$observer_pid" "$router_pid"; do
        if [[ -n "$process" ]]; then
            kill "$process" 2>/dev/null || true
            wait "$process" 2>/dev/null || true
        fi
    done
}
trap cleanup EXIT

set +u
source /opt/ros/lyrical/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_zenoh_cpp RUST_LOG=warn
export ZENOH_SESSION_CONFIG_URI="$evidence/session.json"

printf '%s\n' '{"mode":"router","listen":{"endpoints":["tcp/127.0.0.1:17447"]},"scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}}}' \
    > "$evidence/router.json"
printf '%s\n' '{"mode":"client","connect":{"endpoints":["tcp/127.0.0.1:17447"]},"scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}}}' \
    > "$evidence/session.json"

ZENOH_ROUTER_CONFIG_URI="$evidence/router.json" \
    timeout --kill-after=3s 45s /opt/ros/lyrical/lib/rmw_zenoh_cpp/rmw_zenohd \
    > "$evidence/router.log" 2>&1 &
router_pid=$!
sleep 1

timeout --kill-after=3s 20s ros2 topic echo /host_serial_patch_smoke \
    std_msgs/msg/Float32 --once --no-lost-messages --filter 'm.data == 37.0' \
    --qos-reliability reliable --qos-durability volatile \
    > "$evidence/subscriber.log" 2>&1 &
observer_pid=$!

timeout --kill-after=3s 20s ros2 topic pub /host_serial_patch_smoke \
    std_msgs/msg/Float32 '{data: 37.0}' --once \
    --wait-matching-subscriptions 1 --max-wait-time-secs 10 \
    --qos-reliability reliable --qos-durability volatile \
    > "$evidence/publisher.log" 2>&1
wait "$observer_pid"
observer_pid=""
grep -Fx 'data: 37.0' "$evidence/subscriber.log"
echo "PASS: stock ROS CLI typed pub/echo over isolated TCP loopback"
echo "This does not test a serial Zenoh session or physical USB recovery."
