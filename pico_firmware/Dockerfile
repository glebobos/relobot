FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake \
    g++ \
    gcc-arm-none-eabi \
    doxygen \
    libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib \
    git \
    python3 \
    build-essential \
    libusb-1.0-0-dev \
    pkg-config \
    picocom \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

ENV PICO_SDK_PATH=/opt/pico-sdk
RUN git clone --depth=1 --branch 2.2.0 https://github.com/raspberrypi/pico-sdk.git $PICO_SDK_PATH

# Pre-build and install picotool for USB flashing and board inspection
RUN git clone --depth=1 https://github.com/raspberrypi/picotool.git /tmp/picotool && \
    cd /tmp/picotool && \
    mkdir build && \
    cd build && \
    cmake .. -DPICO_SDK_PATH=/opt/pico-sdk && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/picotool

WORKDIR /project

CMD ["/bin/bash"]
