#!/usr/bin/env bash
set -euo pipefail

readonly ROOT="/project/helpers/rmw_zenoh_pico_spike"
readonly ZENOH_PICO_COMMIT="87ce955e651f699516b8b57b6a310f22bff527b8"
readonly FREERTOS_COMMIT="88971102bf8aa34b4fcc24f176db620dd9c854f7"

rm -rf "${ROOT}/build/zenoh-pico" "${ROOT}/build/FreeRTOS-Kernel"
git clone --filter=blob:none --no-checkout https://github.com/eclipse-zenoh/zenoh-pico.git "${ROOT}/build/zenoh-pico"
git -C "${ROOT}/build/zenoh-pico" checkout "${ZENOH_PICO_COMMIT}"
git -c safe.directory="${ROOT}/build/zenoh-pico" \
  -C "${ROOT}/build/zenoh-pico" apply "${ROOT}/patches/zenoh-pico-usb-task.patch"
git clone --filter=blob:none --no-checkout https://github.com/FreeRTOS/FreeRTOS-Kernel.git "${ROOT}/build/FreeRTOS-Kernel"
git -C "${ROOT}/build/FreeRTOS-Kernel" checkout "${FREERTOS_COMMIT}"

git -c safe.directory="${ROOT}/build/rmw_zenoh_pico" \
  -C "${ROOT}/build/rmw_zenoh_pico" reset --hard HEAD
git -c safe.directory="${ROOT}/build/rmw_zenoh_pico" \
  -C "${ROOT}/build/rmw_zenoh_pico" apply "${ROOT}/patches/rp2040-serial.patch"

export PICO_SDK_PATH=/pico-sdk
export FREERTOS_KERNEL_PATH="${ROOT}/build/FreeRTOS-Kernel"
cmake -S "${ROOT}/firmware" -B "${ROOT}/build/firmware" \
  -DPICO_BOARD=pico \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "${ROOT}/build/firmware" -j2
