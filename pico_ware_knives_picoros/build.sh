#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"

readonly BUILDER_IMAGE="relobot_pico_ware_wheels_microros:latest"
readonly SPIKE_BUILD="${repo_root}/helpers/rmw_zenoh_pico_spike/build"

if [[ ! -f "${SPIKE_BUILD}/libmicroros/libmicroros.a" ||
      ! -d "${SPIKE_BUILD}/FreeRTOS-Kernel/.git" ||
      ! -d "${SPIKE_BUILD}/picoros/.git" ]]; then
    echo "Dependencies missing in ${SPIKE_BUILD}. Ensure rmw_zenoh_pico_spike dependencies are built first." >&2
    exit 1
fi

echo "==> Building Knives Pico-ROS firmware..."
docker run --rm --user root --entrypoint bash \
    -v "${repo_root}:/project" \
    -e PICO_SDK_PATH=/project/helpers/rmw_zenoh_pico_spike/build/pico-sdk \
    -e FREERTOS_KERNEL_PATH=/project/helpers/rmw_zenoh_pico_spike/build/FreeRTOS-Kernel \
    "${BUILDER_IMAGE}" \
    -lc 'cmake -S /project/pico_ware_knives_picoros \
               -B /project/pico_ware_knives_picoros/build \
               -DPICOROS_SOURCE=/project/helpers/rmw_zenoh_pico_spike/build/picoros \
               -DRELOBOT_ROS_TYPES_PATH=/project/helpers/rmw_zenoh_pico_spike/build/libmicroros \
               -DRELOBOT_PICO_BOARD=xiao_rp2040 \
               -DRELOBOT_PICOROS_USB_POLL=ON \
               -DCMAKE_BUILD_TYPE=Release &&
         cmake --build /project/pico_ware_knives_picoros/build --target picoros_knives -j2'

echo "==> Copying UF2 artifact..."
if [[ -f "${script_dir}/build/picoros_knives.uf2" ]]; then
    cp "${script_dir}/build/picoros_knives.uf2" "${script_dir}/picoros_knives.uf2"
    echo "✅ Success: Created ${script_dir}/picoros_knives.uf2"
fi
