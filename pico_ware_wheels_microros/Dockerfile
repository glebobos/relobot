FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    cmake \
    g++ \
    gcc-arm-none-eabi \
    doxygen \
    libnewlib-arm-none-eabi \
    git \
    python3 \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

ENV PICO_SDK_PATH=/opt/pico-sdk
RUN git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git $PICO_SDK_PATH

WORKDIR /project
RUN git clone --branch autoupdate_micro_ros_rpipico_humble https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git .

COPY build.sh /usr/local/bin/build.sh
RUN chmod +x /usr/local/bin/build.sh

CMD ["/usr/local/bin/build.sh"]
