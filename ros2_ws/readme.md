# ros2_ws — ROS2 workspace

This workspace runs inside Docker containers (see `docker-compose.yml`).  
All services are started from the repo root via `./start_robot.sh`.

## Building locally (outside Docker)

```bash
sudo apt-get update && sudo apt-get install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-topic-based-ros2-control \
    ros-humble-realtime-tools \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-robot-localization

colcon build --packages-select diff_drive_hardware
source install/setup.bash
```

## Launching the diff-drive stack

```bash
ros2 launch diff_drive_hardware diffbot.launch.py
```

Environment variables (set before launch or in docker-compose):

| Variable | Default | Description |
| :--- | :--- | :--- |
| `WHEELS_TTY` | `/dev/ttyACM0` | Wheels Pico USB port |
| `IMU_TTY` | `/dev/ttyACM4` | IMU Pico USB port |
| `INA226_TTY` | `/dev/ttyACM2` | INA226 Pico 2 USB port |
| `KNIVES_TTY` | `/dev/ttyACM3` | Knives Pico USB port |

## Sending test commands

```bash
# Move forward 0.2 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --once

# Turn left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}" --once

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

## Monitoring joint states

```bash
ros2 topic echo /robot_joint_states
ros2 topic echo /joint_states
```

## Hardware plugin

`diff_drive_hardware` no longer contains a C++ serial plugin.  
The hardware interface is `topic_based_ros2_control/TopicBasedSystem`, configured in `src/diff_drive_hardware/config/diffbot.urdf.xacro`:

```xml
<hardware>
  <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
  <param name="joint_commands_topic">/robot_joint_commands</param>
  <param name="joint_states_topic">/robot_joint_states</param>
  <param name="trigger_joint_command_threshold">-1</param>
</hardware>
```

The wheels Pico firmware subscribes to `/robot_joint_commands` and publishes `/robot_joint_states` directly.

## RViz2

```bash
# From repo root
./rviz2.sh

# Or manually (WSL / Linux with X/Wayland)
docker run -it --rm -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg \
  -e DISPLAY -e WAYLAND_DISPLAY -e XDG_RUNTIME_DIR -e PULSE_SERVER \
  -e ROS_DISCOVERY_SERVER=192.168.40.120:11811 -e ROS_SUPER_CLIENT=True \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  --network host osrf/ros:jazzy-desktop rviz2
```