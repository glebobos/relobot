#!/usr/bin/env bash
set -euo pipefail

readonly BUILDER_IMAGE="microros/micro_ros_static_library_builder@sha256:f06c5d5ade3136f781404277e6e3e2ad0de8e720e0cccc0a8253bfa7ab3a80de"

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

docker run --rm \
    --user root \
    -e MICROROS_LIBRARY_FOLDER=helpers/rmw_zenoh_pico_spike \
    -v "${script_dir}/../..:/project" \
    "${BUILDER_IMAGE}"
