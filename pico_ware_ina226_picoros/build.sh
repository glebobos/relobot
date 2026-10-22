#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"

readonly BUILDER_IMAGE="relobot_pico_ware_wheels_microros:latest"
readonly SPIKE_BUILD="${repo_root}/helpers/rmw_zenoh_pico_spike/build"
board="${1:-kamod_rp2040}"
mode="${2:-bench}"
if [[ $# -gt 2 || ! "$board" =~ ^(kamod_rp2040|pico|xiao_rp2040)$ ||
      ! "$mode" =~ ^(bench|sensor)$ ]]; then
    echo "Usage: $0 [kamod_rp2040|pico|xiao_rp2040] [bench|sensor]" >&2
    exit 1
fi
bench=ON
if [[ "$mode" = sensor ]]; then bench=OFF; fi

if [[ ! -f "${SPIKE_BUILD}/libmicroros/libmicroros.a" ||
      ! -d "${SPIKE_BUILD}/FreeRTOS-Kernel/.git" ||
      ! -d "${SPIKE_BUILD}/picoros/.git" ]]; then
    echo "Dependencies missing in ${SPIKE_BUILD}. Ensure rmw_zenoh_pico_spike dependencies are built first." >&2
    exit 1
fi

echo "==> Building INA226 Pico-ROS firmware: ${board}, ${mode}"
mkdir -p "${script_dir}/build"
docker run --rm --network none --user "$(id -u):$(id -g)" --entrypoint bash \
    -v "${repo_root}:/project:ro" \
    -v "${script_dir}/build:/output" \
    -e INA226_BOARD="$board" -e INA226_MODE="$mode" -e INA226_BENCH="$bench" \
    -e PICO_SDK_PATH=/project/helpers/rmw_zenoh_pico_spike/build/pico-sdk \
    -e FREERTOS_KERNEL_PATH=/project/helpers/rmw_zenoh_pico_spike/build/FreeRTOS-Kernel \
    "${BUILDER_IMAGE}" \
    -lc 'set -euo pipefail
         cp -a /project/helpers/rmw_zenoh_pico_spike/build/picoros /tmp/picoros
         cmake -S /project/pico_ware_ina226_picoros \
               -B "/output/${INA226_BOARD}-${INA226_MODE}" \
               -DPICOROS_SOURCE=/tmp/picoros \
               -DRELOBOT_ROS_TYPES_PATH=/project/helpers/rmw_zenoh_pico_spike/build/libmicroros \
               -DRELOBOT_PICO_BOARD="$INA226_BOARD" \
               -DRELOBOT_INA226_BENCH="$INA226_BENCH" \
               -DRELOBOT_PICOROS_USB_POLL=ON \
               -DCMAKE_BUILD_TYPE=Release &&
         cmake --build "/output/${INA226_BOARD}-${INA226_MODE}" --target picoros_ina226 -j2'

echo "==> Copying UF2 artifact..."
artifact="${script_dir}/picoros_ina226_${board}_${mode}.uf2"
cp "${script_dir}/build/${board}-${mode}/picoros_ina226.uf2" "$artifact"
echo "==> Created ${artifact} (not flashed)"
