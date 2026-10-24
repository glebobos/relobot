#!/usr/bin/env bash
set -euo pipefail

readonly PICO_SDK_COMMIT="a1438dff1d38bd9c65dbd693f0e5db4b9ae91779"
readonly RMW_ZENOH_PICO_DIR="/project/helpers/rmw_zenoh_pico_spike"

apt-get update
apt-get install -y --no-install-recommends \
  gcc-arm-none-eabi \
  libnewlib-arm-none-eabi \
  libstdc++-arm-none-eabi-newlib \
  rsync

rm -rf /pico-sdk
git clone --filter=blob:none --no-checkout https://github.com/raspberrypi/pico-sdk.git /pico-sdk
git -C /pico-sdk checkout "${PICO_SDK_COMMIT}"
git -C /pico-sdk submodule update --init --depth 1

cd /uros_ws
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source install/local_setup.bash
set -u
ros2 run micro_ros_setup create_firmware_ws.sh generate_lib

pushd firmware/mcu_ws >/dev/null
mkdir -p extra_packages
pushd extra_packages >/dev/null
vcs import --input "${RMW_ZENOH_PICO_DIR}/library_generation/extra_packages/extra_packages.repos"
popd >/dev/null

# The RMW is compiled later by the Pico SDK build so it can see the RP2040,
# FreeRTOS, and TinyUSB platform libraries. Keep only its generated ROS headers
# and dependent type support in this archive generation step.
touch extra_packages/rmw_zenoh_pico/rmw_zenoh_pico/COLCON_IGNORE
popd >/dev/null

rm -rf firmware/build
export PICO_SDK_PATH=/pico-sdk
ros2 run micro_ros_setup build_firmware.sh \
  "${RMW_ZENOH_PICO_DIR}/library_generation/toolchain.cmake" \
  "${RMW_ZENOH_PICO_DIR}/library_generation/colcon.meta"

rm -rf "${RMW_ZENOH_PICO_DIR}/build"
mkdir -p "${RMW_ZENOH_PICO_DIR}/build/libmicroros/include"
find firmware/build/include -name "*.c" -delete
cp -R firmware/build/include/. "${RMW_ZENOH_PICO_DIR}/build/libmicroros/include/"
cp firmware/build/libmicroros.a "${RMW_ZENOH_PICO_DIR}/build/libmicroros/"
mapfile -t xrce_rmw_objects < <(
  arm-none-eabi-ar t "${RMW_ZENOH_PICO_DIR}/build/libmicroros/libmicroros.a" |
    grep '^librmw_microxrcedds-'
)
if [[ ${#xrce_rmw_objects[@]} -eq 0 ]]; then
  echo "Error: no rmw_microxrcedds objects found in generated archive." >&2
  exit 1
fi
arm-none-eabi-ar d \
  "${RMW_ZENOH_PICO_DIR}/build/libmicroros/libmicroros.a" \
  "${xrce_rmw_objects[@]}"

pushd firmware/mcu_ws >/dev/null
include_packages="$(colcon list | awk '{print $1}')"
popd >/dev/null
for package in ${include_packages}; do
  nested="${RMW_ZENOH_PICO_DIR}/build/libmicroros/include/${package}/${package}"
  if [[ -d "${nested}" ]]; then
    rsync -r "${nested}/" "${RMW_ZENOH_PICO_DIR}/build/libmicroros/include/${package}/"
    rm -rf "${nested}"
  fi
done

cp -R firmware/mcu_ws/extra_packages/rmw_zenoh_pico "${RMW_ZENOH_PICO_DIR}/build/rmw_zenoh_pico"

"${RMW_ZENOH_PICO_DIR}/library_generation/build_firmware.sh"
chown -R "${HOST_UID:-1000}:${HOST_GID:-1000}" "${RMW_ZENOH_PICO_DIR}/build"
