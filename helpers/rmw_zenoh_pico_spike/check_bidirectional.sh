#!/usr/bin/env bash
set -euo pipefail

if [[ $# != 1 || ! "$1" =~ ^[1-9][0-9]{0,7}$ ]] || (( $1 > 16777215 )); then
    echo "Usage: bash $0 <new-integer-sequence:1..16777215>" >&2
    exit 2
fi

sequence="$1"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
compose=(docker compose -f "${script_dir}/../../ros2_ws/docker-compose.yml")
observer_pid=""

cleanup() {
    if [[ -n "${observer_pid}" ]]; then
        kill "${observer_pid}" 2>/dev/null || true
        wait "${observer_pid}" 2>/dev/null || true
    fi
}
trap cleanup EXIT

ros() {
    "${compose[@]}" exec -T ros2_zenoh_router bash -c '
        source /opt/ros/lyrical/setup.bash
        export RMW_IMPLEMENTATION=rmw_zenoh_cpp RUST_LOG=warn
        timeout --kill-after=3s 20s ros2 topic "$@"
    ' bash "$@"
}

sample() {
    local output
    output="$(ros echo "/picoros_usb_spike/$1" std_msgs/msg/Float32 \
        --once --field data --no-lost-messages \
        --qos-reliability reliable --qos-durability volatile)"
    printf '%s\n' "${output}" | sed -n '1p'
}

expect() {
    ros echo "/picoros_usb_spike/$1" std_msgs/msg/Float32 \
        --once --no-lost-messages --filter "$2" \
        --qos-reliability reliable --qos-durability volatile
}

send() {
    ros pub /picoros_usb_spike/command std_msgs/msg/Float32 "{data: $1}" \
        --once --wait-matching-subscriptions 1 --max-wait-time-secs 10 \
        --qos-reliability reliable --qos-durability volatile --qos-depth 1
}

date -Is
ls -l /dev/serial/by-id/usb-Raspberry_Pi_Pico_E464C498C3380C35-if00
ack="$(sample ack)"
info="$(ros info /picoros_usb_spike/command --verbose --no-daemon)"
printf '%s\n' "${info}"
if ! grep -Fqx 'Subscription count: 1' <<<"${info}"; then
    echo "FAIL: expected exactly one command subscriber; check graph/board identity." >&2
    exit 1
fi

if ! [[ "${ack}" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
    ! awk -v ack="${ack}" -v sequence="${sequence}" 'BEGIN {exit !(sequence > ack)}'; then
    echo "FAIL: choose a sequence greater than the current ack (${ack})." >&2
    exit 1
fi
rejected="$(sample rejected)"
invalid="$(sample invalid)"
timeouts="$(sample timeouts)"

expect active 'm.data == 1.0' &
observer_pid=$!
sleep 2
send "${sequence}"
wait "${observer_pid}"
observer_pid=""
expect ack "m.data == ${sequence}.0"
sleep 2
expect active 'm.data == 0.0'
expect timeouts "m.data > ${timeouts}"

send "${sequence}"
expect rejected "m.data > ${rejected}"
expect ack "m.data == ${sequence}.0"
expect active 'm.data == 0.0'
if (( sequence > 1 )); then
    send "$((sequence - 1))"
    expect rejected "m.data >= ${rejected} + 2"
    expect ack "m.data == ${sequence}.0"
    expect active 'm.data == 0.0'
fi

send 0
expect invalid "m.data > ${invalid}"
expect ack "m.data == ${sequence}.0"
expect active 'm.data == 0.0'
date -Is
echo "PASS: typed roundtrip, timeout, duplicate/older sequence and invalid-value checks."
echo "This does NOT prove command age, reboot replay protection, or actuator safety."
