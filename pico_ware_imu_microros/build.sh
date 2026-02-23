#!/bin/bash
set -e
cd /project/src && rm -rf build && mkdir build && cd build
cmake .. -DPICO_SDK_PATH=/opt/pico-sdk
make -j$(nproc)
