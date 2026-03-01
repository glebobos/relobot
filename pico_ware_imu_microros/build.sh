#!/bin/bash
set -e
cd /project/src && rm -rf build && mkdir build && cd build
cmake .. -DPICO_SDK_PATH=/opt/pico-sdk
make -j$(nproc)
cp /project/src/build/*.uf2 /project/src/
rm -rf /project/src/build
