#!/usr/bin/env bash
set -euo pipefail

if [[ $# -gt 3 || ! "${1:-20}" =~ ^[1-9][0-9]?$ ||
      ! "${2:-1001}" =~ ^[1-9][0-9]{0,7}$ ]] ||
    (( ${2:-1001} + ${1:-20} - 1 > 16777215 )); then
    echo "Usage: bash $0 [cycles:1..99] [first-sequence] [evidence-directory]" >&2
    exit 2
fi

cycles="${1:-20}"
first_sequence="${2:-1001}"
evidence_dir="${3:-/tmp/picoros-router-matrix-$(date -u +%Y%m%dT%H%M%SZ)}"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
compose=(docker compose -f "${script_dir}/../../ros2_ws/docker-compose.yml")
device=/dev/serial/by-id/usb-Raspberry_Pi_Pico_E464C498C3380C35-if00
mkdir -p "${evidence_dir}"
if [[ -e "${evidence_dir}/results.tsv" ]]; then
    echo "Refusing to overwrite an earlier matrix: ${evidence_dir}" >&2
    exit 2
fi
printf 'cycle\tstarted_utc\tfinished_utc\tdevice_before\tdevice_after\tsequence\tresult\n' \
    > "${evidence_dir}/results.tsv"

ros() {
    "${compose[@]}" exec -T ros2_zenoh_router bash -c '
        source /opt/ros/lyrical/setup.bash
        export RMW_IMPLEMENTATION=rmw_zenoh_cpp RUST_LOG=warn
        timeout --kill-after=3s 30s ros2 topic "$@"
    ' bash "$@"
}

expect() {
    ros echo "/picoros_usb_spike/$1" std_msgs/msg/Float32 --once \
        --filter "$2" --qos-reliability reliable --qos-durability volatile
}

send() {
    ros pub /picoros_usb_spike/command std_msgs/msg/Float32 "{data: $1}" \
        --once --max-wait-time-secs 15 --qos-reliability reliable \
        --qos-durability volatile --qos-depth 1
}

if [[ ! -e "${device}" ]]; then
    echo "Spare KAmod is not attached to WSL: ${device}" >&2
    exit 1
fi
prior_ack="$(ros echo /picoros_usb_spike/ack std_msgs/msg/Float32 \
    --once --field data --qos-reliability reliable | sed -n '1p')"
if ! [[ "${prior_ack}" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
    ! awk -v ack="${prior_ack}" -v sequence="${first_sequence}" \
        'BEGIN {exit !(sequence > ack)}'; then
    echo "Choose a first sequence greater than the current ack (${prior_ack})." >&2
    exit 2
fi

echo "Restarting ONLY the experimental ros2_zenoh_router; evidence: ${evidence_dir}"
for ((cycle = 1; cycle <= cycles; cycle++)); do
    sequence=$((first_sequence + cycle - 1))
    started="$(date -u +%FT%TZ)"
    before="$(readlink -f "${device}")"
    log="${evidence_dir}/cycle-${cycle}.log"
    printf '%s cycle=%s sequence=%s\n' "${started}" "${cycle}" "${sequence}"
    result=PASS
    if ! (
        "${compose[@]}" restart ros2_zenoh_router &&
        expect ack "m.data == ${prior_ack}" &&
        expect active 'm.data == 0.0' &&
        send "${sequence}" &&
        expect ack "m.data == ${sequence}.0" &&
        sleep 2 &&
        send "${prior_ack}" &&
        expect ack "m.data == ${sequence}.0" &&
        expect active 'm.data == 0.0' &&
        ros info /picoros_usb_spike/ack --verbose --no-daemon > "${log}.publisher" &&
        grep -Fqx 'Publisher count: 1' "${log}.publisher" &&
        ros info /picoros_usb_spike/command --verbose --no-daemon > "${log}.subscriber" &&
        grep -Fqx 'Subscription count: 1' "${log}.subscriber"
    ) > "${log}" 2>&1; then
        result=FAIL
    fi
    after="$(readlink -f "${device}" || true)"
    finished="$(date -u +%FT%TZ)"
    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
        "${cycle}" "${started}" "${finished}" "${before}" "${after}" \
        "${sequence}" "${result}" | tee -a "${evidence_dir}/results.tsv"
    "${compose[@]}" logs --since="${started}" --no-color ros2_zenoh_router \
        > "${evidence_dir}/cycle-${cycle}.router.log" 2>&1
    if [[ "${result}" != PASS ]]; then
        tail -35 "${log}"
        echo "Stopped at failed cycle ${cycle}; remaining cycles are NOT TESTED." >&2
        exit 1
    fi
    prior_ack="${sequence}.0"
done
echo "PASS: ${cycles} router cycles; no MCU reset/USB reattachment performed by this script."
echo "Timing includes CLI overhead; <=5 s recovery is not established by these results."
