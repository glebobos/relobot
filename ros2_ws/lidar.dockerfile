FROM ros:humble

# Install additional dependencies
RUN apt-get update 
# Create workspace
WORKDIR /ros2_ws

# Source the workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Create development script
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
cd /ros2_ws\n\
git clone  https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git ./src/ldlidar_stl_ros2 || true\n\
colcon build --packages-select ldlidar_stl_ros2 \n\
source install/setup.bash\n\
ros2 launch ldlidar_stl_ros2 stl27l.launch.py' > /start_dev.sh && \
chmod +x /start_dev.sh

CMD ["/start_dev.sh"]