#!/usr/bin/env bash
set -euo pipefail

readonly PICOROS_COMMIT="b739fc713da2ee1caa3d1c278c5ed9cc1702eaa2"
readonly PICO_SDK_COMMIT="a1438dff1d38bd9c65dbd693f0e5db4b9ae91779"
readonly BUILDER_IMAGE="relobot_pico_ware_wheels_microros:latest"

if [[ $# -gt 1 || "${1:-connectivity}" != connectivity ]]; then
    echo "Usage: $0 [connectivity]" >&2
    echo "Only the validated USB roundtrip application is retained; earlier prototype variants are retired." >&2
    exit 2
fi
target="picoros_usb_connectivity"

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
build_dir="${script_dir}/build"
source_dir="${build_dir}/picoros"
sdk_dir="${build_dir}/pico-sdk"
firmware_build=firmware_picoros_connectivity_sdk220

if [[ ! -f "${build_dir}/libmicroros/libmicroros.a" ||
      ! -d "${build_dir}/FreeRTOS-Kernel/.git" ]]; then
    echo "Missing pinned micro-ROS client library or FreeRTOS checkout; run ./helpers/rmw_zenoh_pico_spike/build.sh first." >&2
    exit 1
fi

if [[ ! -d "${source_dir}/.git" ]]; then
    git clone https://github.com/Pico-ROS/Pico-ROS-software.git "${source_dir}"
fi
if [[ "$(git -C "${source_dir}" remote get-url origin)" != "https://github.com/Pico-ROS/Pico-ROS-software.git" ]]; then
    echo "Unexpected Pico-ROS source at ${source_dir}" >&2
    exit 1
fi
git -C "${source_dir}" fetch --depth 1 origin "${PICOROS_COMMIT}"
git -C "${source_dir}" checkout --detach "${PICOROS_COMMIT}"
git -C "${source_dir}" \
    -c url.https://github.com/.insteadOf=git@github.com: \
    submodule update --init --recursive

if [[ ! -d "${sdk_dir}/.git" ]]; then
    git clone --filter=blob:none --no-checkout https://github.com/raspberrypi/pico-sdk.git "${sdk_dir}"
fi
if [[ "$(git -C "${sdk_dir}" remote get-url origin)" != "https://github.com/raspberrypi/pico-sdk.git" ]]; then
    echo "Unexpected Pico SDK source at ${sdk_dir}" >&2
    exit 1
fi
git -C "${sdk_dir}" checkout --detach "${PICO_SDK_COMMIT}"
git -C "${sdk_dir}" submodule update --init --depth 1

usb_patch="${script_dir}/patches/zenoh-pico-usb-task.patch"
zenoh_dir="${source_dir}/thirdparty/zenoh-pico"
if git -C "${zenoh_dir}" apply --reverse --check "${usb_patch}" 2>/dev/null; then
    :
else
    git -C "${zenoh_dir}" apply "${usb_patch}"
fi
for spec in \
    "${source_dir}:${script_dir}/patches/picoros-init-stages.patch" \
    "${source_dir}:${script_dir}/patches/picoros-session-access.patch" \
    "${zenoh_dir}:${script_dir}/patches/picoros-usb-init-stages.patch" \
    "${zenoh_dir}:${script_dir}/patches/zenoh-pico-usb-disconnect.patch" \
    "${zenoh_dir}:${script_dir}/patches/zenoh-pico-usb-poll.patch"; do
    checkout="${spec%%:*}"
    patch="${spec#*:}"
    if git -C "${checkout}" apply --reverse --check "${patch}" 2>/dev/null; then
        :
    else
        git -C "${checkout}" apply "${patch}"
    fi
done

docker run --rm --user root --entrypoint bash \
    -v "${script_dir}/../..:/project" \
    -e PICO_SDK_PATH=/project/helpers/rmw_zenoh_pico_spike/build/pico-sdk \
    -e FREERTOS_KERNEL_PATH=/project/helpers/rmw_zenoh_pico_spike/build/FreeRTOS-Kernel \
    -e PICOROS_BUILD_TARGET="${target}" \
    -e PICOROS_FIRMWARE_BUILD="${firmware_build}" \
    "${BUILDER_IMAGE}" \
    -lc 'cmake -S /project/helpers/rmw_zenoh_pico_spike/firmware_picoros \
                    -B "/project/helpers/rmw_zenoh_pico_spike/build/${PICOROS_FIRMWARE_BUILD}" \
                    -DPICOROS_SOURCE=/project/helpers/rmw_zenoh_pico_spike/build/picoros \
                    -DRELOBOT_PICOROS_USB_POLL=ON \
                    -DCMAKE_BUILD_TYPE=Release &&
              cmake --build "/project/helpers/rmw_zenoh_pico_spike/build/${PICOROS_FIRMWARE_BUILD}" \
                    --target "${PICOROS_BUILD_TARGET}" -j2'
