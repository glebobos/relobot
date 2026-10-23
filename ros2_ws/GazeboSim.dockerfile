FROM ros:lyrical-ros-base-resolute@sha256:f79cd486fa218596747c45d00ca42101b0303fc6d94ed5adb72145cd6f54b86c AS dependencies

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive RMW_IMPLEMENTATION=rmw_zenoh_cpp

RUN apt-get update && apt-get install -y --no-install-recommends \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1 \
    libvulkan1 \
    vulkan-tools \
    dbus-x11 \
    x11-xserver-utils \
    x11-utils \
    ros-lyrical-rmw-zenoh-cpp \
    ros-lyrical-ros-gz \
    ros-lyrical-gz-ros2-control \
    ros-lyrical-ros2-control \
    ros-lyrical-ros2-controllers \
    ros-lyrical-joint-state-publisher \
    ros-lyrical-robot-state-publisher \
    ros-lyrical-robot-localization \
    ros-lyrical-xacro \
    ros-lyrical-web-video-server \
    ros-lyrical-cv-bridge \
    ros-lyrical-image-transport \
    ros-lyrical-compressed-image-transport \
    ros-lyrical-rviz2 \
    && rm -rf /var/lib/apt/lists/*

FROM dependencies AS runtime

WORKDIR /ros2_ws

ENV QT_X11_NO_MITSHM=1
RUN echo "source /opt/ros/lyrical/setup.bash" >> /root/.bashrc

RUN printf '%s\n' \
    '#!/bin/bash' \
    'set -eo pipefail' \
    'source /opt/ros/lyrical/setup.bash' \
    'cd /ros2_ws' \
    'if [[ -d /usr/lib/wsl/lib ]]; then export LD_LIBRARY_PATH="/usr/lib/wsl/lib:${LD_LIBRARY_PATH:-}"; fi' \
    'if [[ ! -e /dev/dxg && ! -e /dev/dri && -z "${NVIDIA_VISIBLE_DEVICES:-}" ]]; then export LIBGL_ALWAYS_SOFTWARE=1; fi' \
    'if [ "$DEV" = "true" ] || [ ! -f /ros2_ws/install/relobot_gazebo/share/relobot_gazebo/package.xml ]; then' \
    '  colcon build --packages-select diff_drive_hardware relobot_gazebo --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF' \
    'fi' \
    'source install/setup.bash' \
    'exec ros2 launch relobot_gazebo gazebo_sim.launch.py world:="${SIM_WORLD:-garden_world.sdf}" gui:="${SIM_GUI:-true}" use_sim_time:=true' \
    > /start_sim.sh && chmod +x /start_sim.sh

EXPOSE 8080

CMD ["/start_sim.sh"]
