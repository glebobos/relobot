#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"

readonly BUILDER_IMAGE="relobot_pico_firmware_builder:latest"
readonly FIRMWARE_DEPS="${repo_root}/pico_firmware/build"
board="${1:-kamod_rp2040}"
mode="${2:-bench}"
if [[ $# -gt 2 || ! "$board" =~ ^(kamod_rp2040|xiao_rp2040)$ ||
      ! "$mode" =~ ^(bench|motor|calibration)$ || ( "$mode" =~ ^(motor|calibration)$ && "$board" != xiao_rp2040 ) ]]; then
    echo "Usage: $0 [kamod_rp2040|xiao_rp2040] [bench|motor|calibration]; motor/calibration requires xiao_rp2040" >&2
    exit 1
fi
bench=ON
calibration=OFF
if [[ "$mode" = motor ]]; then bench=OFF; fi
if [[ "$mode" = calibration ]]; then bench=OFF; calibration=ON; fi

if [[ ! -f "${FIRMWARE_DEPS}/libmicroros/libmicroros.a" ||
      ! -d "${FIRMWARE_DEPS}/FreeRTOS-Kernel/.git" ||
      ! -d "${FIRMWARE_DEPS}/picoros/.git" ]]; then
    echo "Dependencies missing in ${FIRMWARE_DEPS}. Run ./pico_firmware/setup_dependencies.sh first." >&2
    exit 1
fi

echo "==> Building Wheels Pico-ROS firmware: ${board}, ${mode}"
mkdir -p "${script_dir}/build"
docker run --rm --network none --user "$(id -u):$(id -g)" --entrypoint bash \
    -v "${repo_root}:/project:ro" \
    -v "${script_dir}/build:/output" \
    -e WHEELS_BOARD="$board" -e WHEELS_MODE="$mode" -e WHEELS_BENCH="$bench" -e WHEELS_CALIBRATION="$calibration" \
    -e PICO_SDK_PATH=/project/pico_firmware/build/pico-sdk \
    -e FREERTOS_KERNEL_PATH=/project/pico_firmware/build/FreeRTOS-Kernel \
    "${BUILDER_IMAGE}" \
    -lc 'set -euo pipefail
         cp -a /project/pico_firmware/build/picoros /tmp/picoros
         cmake -S /project/pico_ware_wheels_picoros \
               -B "/output/${WHEELS_BOARD}-${WHEELS_MODE}" \
               -DPICOROS_SOURCE=/tmp/picoros \
               -DRELOBOT_ROS_TYPES_PATH=/project/pico_firmware/build/libmicroros \
               -DRELOBOT_PICO_BOARD="$WHEELS_BOARD" \
               -DRELOBOT_WHEELS_BENCH="$WHEELS_BENCH" \
               -DRELOBOT_WHEELS_CALIBRATION="$WHEELS_CALIBRATION" \
               -DRELOBOT_PICOROS_USB_POLL=ON \
               -DCMAKE_BUILD_TYPE=Release &&
         cmake --build "/output/${WHEELS_BOARD}-${WHEELS_MODE}" --target picoros_wheels -j2'

echo "==> Copying UF2 artifact..."
artifact="${script_dir}/picoros_wheels_${board}_${mode}.uf2"
cp "${script_dir}/build/${board}-${mode}/picoros_wheels.uf2" "$artifact"
if [[ "$mode" = "calibration" ]]; then
    cp "${script_dir}/build/${board}-${mode}/picoros_wheels.uf2" "${script_dir}/picoros_wheels_calibration.uf2"
fi
echo "==> Created ${artifact} (not flashed)"
