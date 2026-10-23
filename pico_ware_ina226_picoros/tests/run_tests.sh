#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/../.." && pwd)"

mkdir -p "${script_dir}/build"
docker run --rm --network none --user "$(id -u):$(id -g)" --entrypoint bash \
    -v "${repo_root}:/project:ro" \
    -v "${script_dir}/build:/output" \
    relobot_pico_firmware_builder:latest \
    -lc 'set -euo pipefail
         cmake -S /project/pico_ware_ina226_picoros/tests -B /output/test-tree
         cmake --build /output/test-tree -j2
         ctest --test-dir /output/test-tree --output-on-failure'
