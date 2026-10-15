FROM ros:lyrical-ros-base-resolute@sha256:f79cd486fa218596747c45d00ca42101b0303fc6d94ed5adb72145cd6f54b86c

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-lyrical-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*

CMD ["bash", "-c", "source /opt/ros/lyrical/setup.bash && ros2 run rmw_zenoh_cpp rmw_zenohd"]
