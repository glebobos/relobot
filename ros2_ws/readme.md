docker run -ti --rm -v ${PWD}:/ros2_ws --device=/dev/ttyACM1 ros:humble bash

sudo apt-get update && sudo apt-get install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-realtime-tools \
    ros-humble-serial-driver \
    ros-humble-asio-cmake-module \
    ros-humble-io-context \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    libserial-dev
or apt update  \
 && rosdep install --from-paths src --ignore-src -r -y

    colcon build --packages-select diff_drive_hardware
source install/setup.bash

ros2 launch diff_drive_hardware diffbot.launch.py --debug

# Format: linear.x for forward/backward, angular.z for turning
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --once

# Examples:
# Move forward (0.2 m/s)
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --once

# Turn left (0.5 rad/s)
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}" --once

# Turn right (-0.5 rad/s)
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: -0.5}}" --once

# Stop
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once

ros2 topic echo /joint_states

docker run -it --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY  -e XDG_RUNTIME_DIR -e ROS_DISCOVERY_SERVER=192.168.40.120:11811 -e ROS_SUPER_CLIENT=True -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e PULSE_SERVER -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 --network host osrf/ros:jazzy-desktop rviz2

usbipd list
usbipd bind --busid 2-2
usbipd attach --wsl --busid 2-2

colcon build --packages-select serial
colcon build --packages-select icm_20948
ros2 launch icm_20948 run.launch.xml port:=/dev/ttyACM0