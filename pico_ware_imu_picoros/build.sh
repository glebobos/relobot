#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"

readonly BUILDER_IMAGE="relobot_pico_firmware_builder:latest"
readonly FIRMWARE_DEPS="${repo_root}/pico_firmware/build"
board="${1:-pico}"
mode="${2:-sensor}"
if [[ $# -gt 2 || ! "$board" =~ ^(kamod_rp2040|pico|xiao_rp2040)$ ||
      ! "$mode" =~ ^(bench|sensor)$ ]]; then
    echo "Usage: $0 [kamod_rp2040|pico|xiao_rp2040] [bench|sensor]" >&2
    exit 1
fi
bench=ON
if [[ "$mode" = sensor ]]; then bench=OFF; fi

if [[ ! -f "${FIRMWARE_DEPS}/libmicroros/libmicroros.a" ||
      ! -d "${FIRMWARE_DEPS}/FreeRTOS-Kernel/.git" ||
      ! -d "${FIRMWARE_DEPS}/picoros/.git" ]]; then
    echo "Dependencies missing in ${FIRMWARE_DEPS}. Run ./pico_firmware/setup_dependencies.sh first." >&2
    exit 1
fi

echo "==> Building IMU Pico-ROS firmware: ${board}, ${mode}"
mkdir -p "${script_dir}/build"
docker run --rm --network none --user "$(id -u):$(id -g)" --entrypoint bash \
    -v "${repo_root}:/project:ro" \
    -v "${script_dir}/build:/output" \
    -e IMU_BOARD="$board" -e IMU_MODE="$mode" -e IMU_BENCH="$bench" \
    -e PICO_SDK_PATH=/project/pico_firmware/build/pico-sdk \
    -e FREERTOS_KERNEL_PATH=/project/pico_firmware/build/FreeRTOS-Kernel \
    "${BUILDER_IMAGE}" \
    -lc 'set -euo pipefail
         cp -a /project/pico_firmware/build/picoros /tmp/picoros
         cmake -S /project/pico_ware_imu_picoros \
               -B "/output/${IMU_BOARD}-${IMU_MODE}" \
               -DPICOROS_SOURCE=/tmp/picoros \
               -DRELOBOT_ROS_TYPES_PATH=/project/pico_firmware/build/libmicroros \
               -DRELOBOT_PICO_BOARD="$IMU_BOARD" \
               -DRELOBOT_IMU_BENCH="$IMU_BENCH" \
               -DRELOBOT_PICOROS_USB_POLL=ON \
               -DCMAKE_BUILD_TYPE=Release &&
         cmake --build "/output/${IMU_BOARD}-${IMU_MODE}" --target picoros_imu -j2'

echo "==> Copying UF2 artifact..."
artifact="${script_dir}/picoros_imu_${board}_${mode}.uf2"
cp "${script_dir}/build/${board}-${mode}/picoros_imu.uf2" "$artifact"
echo "==> Created ${artifact} (not flashed)"
