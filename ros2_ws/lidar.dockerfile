FROM ros:lyrical-ros-base-resolute@sha256:f79cd486fa218596747c45d00ca42101b0303fc6d94ed5adb72145cd6f54b86c

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive RMW_IMPLEMENTATION=rmw_zenoh_cpp

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-lyrical-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

RUN echo "source /opt/ros/lyrical/setup.bash" >> /root/.bashrc

RUN printf '%s\n' \
    '#!/bin/bash' \
    'set -eo pipefail' \
    'source /opt/ros/lyrical/setup.bash' \
    'cd /ros2_ws' \
    'if [ "$DEV" = "true" ] || [ ! -f /ros2_ws/install/ldlidar/share/ldlidar/package.xml ]; then' \
    '  colcon build --packages-select ldlidar --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF' \
    'fi' \
    'source install/setup.bash' \
    'exec ros2 launch ldlidar stl27l.launch.py' \
    > /start_dev.sh && chmod +x /start_dev.sh

CMD ["/start_dev.sh"]