#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
PATCHES_DIR="${SCRIPT_DIR}/patches"

readonly PICOROS_COMMIT="b739fc713da2ee1caa3d1c278c5ed9cc1702eaa2"
readonly PICO_SDK_COMMIT="a1438dff1d38bd9c65dbd693f0e5db4b9ae91779"
readonly FREERTOS_COMMIT="88971102bf8aa34b4fcc24f176db620dd9c854f7"

mkdir -p "${BUILD_DIR}"

# 1. Pico SDK
sdk_dir="${BUILD_DIR}/pico-sdk"
if [[ ! -d "${sdk_dir}/.git" ]]; then
    echo "==> Cloning Raspberry Pi Pico SDK..."
    git clone --filter=blob:none --no-checkout https://github.com/raspberrypi/pico-sdk.git "${sdk_dir}"
fi
git -C "${sdk_dir}" checkout --detach "${PICO_SDK_COMMIT}"
git -C "${sdk_dir}" submodule update --init --depth 1

# 2. FreeRTOS Kernel
freertos_dir="${BUILD_DIR}/FreeRTOS-Kernel"
if [[ ! -d "${freertos_dir}/.git" ]]; then
    echo "==> Cloning FreeRTOS Kernel..."
    git clone https://github.com/FreeRTOS/FreeRTOS-Kernel.git "${freertos_dir}"
fi
git -C "${freertos_dir}" checkout --detach "${FREERTOS_COMMIT}"

# 3. Pico-ROS and Zenoh-Pico
source_dir="${BUILD_DIR}/picoros"
if [[ ! -d "${source_dir}/.git" ]]; then
    echo "==> Cloning Pico-ROS software..."
    git clone https://github.com/Pico-ROS/Pico-ROS-software.git "${source_dir}"
fi
git -C "${source_dir}" fetch --depth 1 origin "${PICOROS_COMMIT}"
git -C "${source_dir}" checkout --detach "${PICOROS_COMMIT}"
git -C "${source_dir}" \
    -c url.https://github.com/.insteadOf=git@github.com: \
    submodule update --init --recursive

# 4. Apply Zenoh-Pico / Pico-ROS USB serial patches
echo "==> Applying USB serial and polling patches..."
usb_patch="${PATCHES_DIR}/zenoh-pico-usb-task.patch"
zenoh_dir="${source_dir}/thirdparty/zenoh-pico"
if git -C "${zenoh_dir}" apply --reverse --check "${usb_patch}" 2>/dev/null; then
    :
else
    git -C "${zenoh_dir}" apply "${usb_patch}"
fi

for spec in \
    "${source_dir}:${PATCHES_DIR}/picoros-init-stages.patch" \
    "${source_dir}:${PATCHES_DIR}/picoros-session-access.patch" \
    "${zenoh_dir}:${PATCHES_DIR}/picoros-usb-init-stages.patch" \
    "${zenoh_dir}:${PATCHES_DIR}/zenoh-pico-usb-disconnect.patch" \
    "${zenoh_dir}:${PATCHES_DIR}/zenoh-pico-usb-poll.patch"; do
    checkout="${spec%%:*}"
    patch="${spec#*:}"
    if git -C "${checkout}" apply --reverse --check "${patch}" 2>/dev/null; then
        :
    else
        git -C "${checkout}" apply "${patch}"
    fi
done

echo "✅ Pico-ROS dependencies successfully set up in ${BUILD_DIR}"
