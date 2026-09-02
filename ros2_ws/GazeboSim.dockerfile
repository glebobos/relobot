FROM ros:humble

# Prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install Gazebo Sim (Fortress / ros_gz), ROS2 Control, and GUI libraries
RUN apt-get update && apt-get install -y \
    git \
    curl \
    wget \
    gnupg \
    lsb-release \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libvulkan1 \
    vulkan-tools \
    dbus-x11 \
    x11-xserver-utils \
    x11-utils \
    ros-humble-ros-gz \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-interfaces \
    ros-humble-gz-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-diff-drive-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-robot-localization \
    ros-humble-xacro \
    ros-humble-web-video-server \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Set up environment for WSLg / GPU / X11
ENV QT_X11_NO_MITSHM=1
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# Workspace setup
WORKDIR /ros2_ws

# Source ROS2 setup in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Create startup script for simulation
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
cd /ros2_ws\n\
\n\
# Configure DirectX / WSLg library path if mounted\n\
if [ -d "/usr/lib/wsl/lib" ]; then\n\
    export LD_LIBRARY_PATH="/usr/lib/wsl/lib:$LD_LIBRARY_PATH"\n\
fi\n\
\n\
# Check if hardware acceleration is unavailable, fallback to software rendering\n\
if [ ! -e "/dev/dxg" ] && [ ! -e "/dev/dri" ] && [ -z "$NVIDIA_VISIBLE_DEVICES" ]; then\n\
    export LIBGL_ALWAYS_SOFTWARE=1\n\
fi\n\
\n\
if [ "$DEV" = "true" ] || [ ! -f /ros2_ws/install/relobot_gazebo/share/relobot_gazebo/package.xml ]; then\n\
    colcon build --packages-select diff_drive_hardware relobot_gazebo --symlink-install\n\
fi\n\
\n\
source install/setup.bash\n\
\n\
WORLD=${SIM_WORLD:-garden_world.sdf}\n\
GUI=${SIM_GUI:-true}\n\
\n\
echo "Starting ReloBot Gazebo Simulation with world=$WORLD, gui=$GUI..."\n\
exec ros2 launch relobot_gazebo gazebo_sim.launch.py world:="$WORLD" gui:="$GUI" use_sim_time:=true' > /start_sim.sh && \
chmod +x /start_sim.sh

EXPOSE 8080

CMD ["/start_sim.sh"]
