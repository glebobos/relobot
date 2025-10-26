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
colcon build --packages-select ldlidar \n\
source install/setup.bash\n\
ros2 launch ldlidar stl27l.launch.py' > /start_dev.sh && \
chmod +x /start_dev.sh

CMD ["/start_dev.sh"]