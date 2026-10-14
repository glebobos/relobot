FROM ros:lyrical-ros-base-resolute@sha256:f79cd486fa218596747c45d00ca42101b0303fc6d94ed5adb72145cd6f54b86c AS dependencies

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive RMW_IMPLEMENTATION=rmw_zenoh_cpp

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    libgdal-dev \
    python3-opencv \
    python3-pytest \
    python3-shapely \
    ros-lyrical-rmw-zenoh-cpp \
    ros-lyrical-navigation2 \
    ros-lyrical-nav2-bringup \
    ros-lyrical-fields2cover \
    ros-lyrical-ortools-vendor \
    ros-lyrical-opennav-docking \
    ros-lyrical-opennav-docking-core \
    ros-lyrical-opennav-docking-bt \
    ros-lyrical-apriltag-ros \
    ros-lyrical-slam-toolbox \
    ros-lyrical-robot-localization \
    && mkdir -p /opt/relobot \
    && dpkg-query -W -f='${Package}=${Version}\n' > /opt/relobot/navigation-packages.txt \
    && rm -rf /var/lib/apt/lists/*

FROM dependencies AS runtime

ARG OPENNAV_COVERAGE_REVISION=29c96cf0a6d2e565beaea510d4d482d9d37bc5de
COPY patches/opennav_coverage_lyrical.patch /opt/relobot/opennav_coverage_lyrical.patch
RUN git clone --no-checkout https://github.com/open-navigation/opennav_coverage.git /opt/opennav_coverage && \
    git -C /opt/opennav_coverage checkout --detach "${OPENNAV_COVERAGE_REVISION}" && \
    git -C /opt/opennav_coverage apply /opt/relobot/opennav_coverage_lyrical.patch

WORKDIR /opt/relobot/navigation
COPY src/nav2 src/nav2
COPY src/coverage_geometry src/coverage_geometry
COPY src/robot_pose_publisher src/robot_pose_publisher
COPY src/explore_lite src/explore_lite
COPY src/explore_lite_msgs src/explore_lite_msgs
RUN source /opt/ros/lyrical/setup.bash && \
    colcon build --base-paths src /opt/opennav_coverage \
      --packages-up-to nav2 robot_pose_publisher explore_lite opennav_coverage \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
    rm -rf build log

RUN printf '%s\n' \
    '#!/bin/bash' \
    'set -eo pipefail' \
    'source /opt/ros/lyrical/setup.bash' \
    'if [[ "${DEV:-false}" == "true" ]]; then' \
    '  colcon --log-base /tmp/relobot-navigation/log build --base-paths /ros2_ws/src /opt/opennav_coverage --packages-up-to nav2 robot_pose_publisher explore_lite opennav_coverage --build-base /tmp/relobot-navigation/build --install-base /tmp/relobot-navigation/install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF' \
    '  source /tmp/relobot-navigation/install/setup.bash' \
    'else' \
    '  source /opt/relobot/navigation/install/setup.bash' \
    'fi' \
    'if [[ "${USE_SIM_TIME:-false}" == "true" ]]; then' \
    '  timeout 60 ros2 topic echo /clock rosgraph_msgs/msg/Clock --once --no-daemon > /dev/null' \
    'fi' \
    'exec ros2 launch nav2 navigation_launch.py use_sim_time:="${USE_SIM_TIME:-false}"' \
    > /start_dev.sh && chmod +x /start_dev.sh

CMD ["/start_dev.sh"]