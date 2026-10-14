FROM ros:lyrical-ros-base-resolute@sha256:f79cd486fa218596747c45d00ca42101b0303fc6d94ed5adb72145cd6f54b86c

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive RMW_IMPLEMENTATION=rmw_zenoh_cpp

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-lyrical-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/relobot/lidar
COPY src/ldlidar src/ldlidar
RUN source /opt/ros/lyrical/setup.bash && \
    colcon build --packages-select ldlidar \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
    rm -rf build log

RUN printf '%s\n' \
    '#!/bin/bash' \
  'set -eo pipefail' \
    'source /opt/ros/lyrical/setup.bash' \
    'if [[ "${DEV:-false}" == "true" ]]; then' \
    '  colcon --log-base /tmp/relobot-lidar/log build --base-paths /ros2_ws/src/ldlidar --packages-select ldlidar --build-base /tmp/relobot-lidar/build --install-base /tmp/relobot-lidar/install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF' \
    '  source /tmp/relobot-lidar/install/setup.bash' \
    'else' \
    '  source /opt/relobot/lidar/install/setup.bash' \
    'fi' \
    'exec ros2 launch ldlidar stl27l.launch.py' \
    > /start_dev.sh && chmod +x /start_dev.sh

CMD ["/start_dev.sh"]