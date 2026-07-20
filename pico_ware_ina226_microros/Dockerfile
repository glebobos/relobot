FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies needed for both Pico SDK and retrieving micro-ROS
RUN apt-get update && apt-get install -y \
    cmake \
    g++ \
    gcc-arm-none-eabi \
    doxygen \
    libnewlib-arm-none-eabi \
    git \
    python3 \
    build-essential \
    libusb-1.0-0-dev \
    pkg-config \
    picocom \
    && rm -rf /var/lib/apt/lists/*

# Install the Pico SDK
ENV PICO_SDK_PATH=/opt/pico-sdk
RUN git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git $PICO_SDK_PATH

# Pre-build and install picotool so that compilation can run offline
RUN git clone https://github.com/raspberrypi/picotool.git /tmp/picotool && \
    cd /tmp/picotool && \
    mkdir build && \
    cd build && \
    cmake .. -DPICO_SDK_PATH=/opt/pico-sdk && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/picotool

# Clone the micro-ROS RP2040 SDK (Humble branch)
# NOTE: The precompiled libmicroros.a targets Cortex-M0+ (RP2040).
# RP2350 is Cortex-M33. If linking fails with ABI errors, rebuild the library
# using: docker run -it --rm -v $(pwd):/project microros/micro_ros_static_library_builder:humble
# with PICO_BOARD=pico2 and appropriate -mcpu=cortex-m33 flags.
WORKDIR /project
RUN git clone --branch autoupdate_micro_ros_rpipico_humble https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git .

# Copy and setup the build script
COPY build.sh /usr/local/bin/build.sh
RUN chmod +x /usr/local/bin/build.sh

CMD ["/usr/local/bin/build.sh"]
