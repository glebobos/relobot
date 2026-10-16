#!/usr/bin/env bash
set -euo pipefail

readonly PICOROS_REPOSITORY="https://github.com/Pico-ROS/Pico-ROS-software.git"
readonly PICOROS_COMMIT="b739fc713da2ee1caa3d1c278c5ed9cc1702eaa2"
readonly BUILDER_IMAGE="relobot-picoros-poc-builder"
readonly TALKER_CONTAINER="relobot-picoros-poc-talker"
readonly ROUTER_CONTAINER="ros2_zenoh_router"
readonly EXPECTED_MESSAGE="data: Hello from Pico-ROS!"

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
work_dir="${PICOROS_POC_WORK_DIR:-/tmp/relobot-picoros-poc}"

cleanup() {
    docker rm -f "${TALKER_CONTAINER}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

if ! docker inspect "${ROUTER_CONTAINER}" \
    --format '{{.State.Running}}' 2>/dev/null | grep -qx true; then
    echo "Error: ${ROUTER_CONTAINER} is not running." >&2
    echo "Start it from the repository root with:" >&2
    echo "  ./start_robot.sh up ros2_zenoh_router --detach" >&2
    exit 1
fi

if [[ ! -d "${work_dir}/.git" ]]; then
    git -c url.https://github.com/.insteadOf=git@github.com: \
        clone --recursive "${PICOROS_REPOSITORY}" "${work_dir}"
fi

if [[ "$(git -C "${work_dir}" remote get-url origin)" != "${PICOROS_REPOSITORY}" ]]; then
    echo "Error: ${work_dir} is not a clone of ${PICOROS_REPOSITORY}." >&2
    exit 1
fi

git -C "${work_dir}" fetch --depth 1 origin "${PICOROS_COMMIT}"
git -C "${work_dir}" checkout --detach "${PICOROS_COMMIT}"
git -C "${work_dir}" \
    -c url.https://github.com/.insteadOf=git@github.com: \
    submodule update --init --recursive

docker build -t "${BUILDER_IMAGE}" "${script_dir}"
docker run --rm \
    -v "${work_dir}:/src" \
    -w /src \
    "${BUILDER_IMAGE}" \
    bash -lc 'cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DPICOROS_BUILD_TESTS=ON &&
              cmake --build build -j2 &&
              ./build/test_examples_types_serdes'

docker rm -f "${TALKER_CONTAINER}" >/dev/null 2>&1 || true
docker run -d --rm \
    --name "${TALKER_CONTAINER}" \
    --network host \
    -v "${work_dir}:/src:ro" \
    -w /src \
    "${BUILDER_IMAGE}" \
    ./build/talker -m client -a tcp/127.0.0.1:7447 \
    >/dev/null

topic_found=false
for _ in $(seq 1 20); do
    if docker exec "${ROUTER_CONTAINER}" bash -lc \
        'source /opt/ros/lyrical/setup.bash &&
         RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 topic list -t' \
        | grep -Fqx '/picoros/chatter [std_msgs/msg/String]'; then
        topic_found=true
        break
    fi
    sleep 1
done

if [[ "${topic_found}" != true ]]; then
    echo "Error: ROS 2 did not discover the typed Pico-ROS topic within 20 seconds." >&2
    exit 1
fi

echo_output="$(
    docker exec "${ROUTER_CONTAINER}" bash -lc \
        'source /opt/ros/lyrical/setup.bash &&
         export RMW_IMPLEMENTATION=rmw_zenoh_cpp &&
         timeout 15 ros2 topic echo /picoros/chatter std_msgs/msg/String --once'
)"
printf '%s\n' "${echo_output}"

if ! grep -Fq "${EXPECTED_MESSAGE}" <<<"${echo_output}"; then
    echo "Error: ROS 2 discovered the topic but did not decode the expected CDR payload." >&2
    exit 1
fi

echo "PASS: pinned Pico-ROS -> project rmw_zenoh_cpp typed interoperability"
