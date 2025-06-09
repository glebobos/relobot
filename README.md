# ReloBot - ROS2 Robotics Platform

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Overview

ReloBot is a comprehensive robotics platform built on ROS2 (Robot Operating System 2) that integrates multiple hardware components including differential drive motors, cutting blades (knives), IMU sensors, and ToF cameras for navigation and object detection. The system is designed with a microcontroller architecture using Raspberry Pi Pico boards for hardware control, with a central ROS2 system coordinating the operation.

## System Architecture

The ReloBot platform consists of the following major components:

1. **Differential Drive System** - Controls robot movement with two independently driven wheels
2. **Cutting Mechanism** - Motor-controlled blades/knives with RPM control and PID stabilization
3. **Sensor Suite**:
   - IMU (Inertial Measurement Unit) - ICM20948 for orientation data
   - ToF (Time of Flight) Camera - For distance measurement and object detection
4. **Web-based Control Interface** - For remote operation and monitoring
5. **SLAM** (Simultaneous Localization and Mapping) - For environment mapping and navigation

## Hardware Components

- Raspberry Pi Pico microcontrollers (for wheels and knife control)
- ICM20948 IMU sensor
- ToF Camera (B0410)
- DC Motors with encoders
- Various serial interfaces and USB connections

## Software Stack

- **ROS2 Humble** - Core robotics framework
- **FastDDS** - For distributed communications
- **Python** - For high-level control and microcontroller programming
- **Docker** - For containerization of ROS2 nodes
- **Web Server** - For remote control interface

## Project Structure

```
├── finddevice.py              # Utility to find connected serial devices
├── start_app_helper.sh        # Startup script for the application
├── B0410_Tof_Firmware/        # ToF camera firmware
├── helpers/                   # Utility scripts and tools
│   ├── joystic_test/          # Joystick testing utilities
│   └── rviz2/                 # RViz2 visualization tools
├── pico_ware_imu/             # IMU sensor code and libraries
├── pico_ware_knives/          # Blade/knife control system
│   ├── boot.py
│   ├── code.py                # Main control logic
│   └── calibration/           # Calibration tools
├── pico_ware_wheels/          # Wheel control system
│   ├── boot.py
│   ├── code.py                # Main control logic
│   └── calibration/           # Calibration tools
└── ros2_ws/                   # ROS2 workspace
    ├── docker-compose.yml     # Docker configuration for all services
    ├── *.dockerfile           # Individual service Dockerfiles
    └── src/                   # ROS2 packages
        ├── arducam_rclpy_tof_pointcloud/   # ToF camera ROS2 package
        ├── diff_drive_hardware/            # Differential drive controller
        ├── icm_20948/                      # IMU sensor package
        ├── mower_knife_controller/         # Knife motor controller
        ├── serial/                         # Serial communication library
        ├── slam_tool/                      # SLAM implementation
        └── web_server/                     # Web-based control interface
```

## Getting Started

### Prerequisites

- Docker and Docker Compose
- ROS2 Humble (if running outside of Docker)
- Python 3.x
- USB access to hardware components

### Hardware Setup

1. Connect the Raspberry Pi Pico boards running the wheel and knife control firmware
2. Connect the IMU sensor to a USB port
3. Connect the ToF camera to a USB port

### Software Setup

1. Clone this repository:
   ```bash
   git clone https://github.com/glebobos/relobot.git
   cd relobot
   ```

2. Automatic device detection:
   ```bash
   python3 finddevice.py
   ```
   This will identify and map the connected devices.

3. Start the ROS2 nodes:
   ```bash
   cd ros2_ws
   export HOST_IP=$(hostname -I | awk '{print $1}')
   docker compose up
   ```

## Docker Containers

ReloBot uses multiple Docker containers to organize its functionality:

1. **Discovery Server** (`ds`) - Manages ROS2 node discovery
2. **Differential Drive** (`ros2_diff_robot`) - Controls robot movement
3. **IMU Sensor** (`ros2_imu`) - Processes orientation data
4. **ToF Camera** (`ros2_tof_camera`) - Processes depth information
5. **Web Server** (`ros2_web_server`) - Provides web interface
6. **SLAM** (`ros2_slam`) - Handles mapping and localization

## Running RViz2 for Visualization

For visualization in Windows WSL or Linux:
```bash
docker run -it --rm -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg -e DISPLAY \
  -e WAYLAND_DISPLAY -e XDG_RUNTIME_DIR -e PULSE_SERVER \
  osrf/ros:jazzy-desktop rviz2
```

For RViz2 with DDS discovery:
```bash
docker run -it --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY \
  -e XDG_RUNTIME_DIR -e ROS_DISCOVERY_SERVER=192.168.40.120:11811 \
  -e ROS_SUPER_CLIENT=True -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e PULSE_SERVER -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  --network host osrf/ros:jazzy-desktop rviz2
```

## Robot Control

### Basic Movement Commands

Control the robot via ROS2 topics:

```bash
# Move forward (0.2 m/s)
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" --once

# Turn left (0.5 rad/s)
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}" --once

# Turn right (-0.5 rad/s)
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: -0.5}}" --once

# Stop
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

### Monitoring Robot State

```bash
# View joint state information
ros2 topic echo /joint_states
```

## Microcontroller Firmware

### Wheel Controller

The wheel controller (`pico_ware_wheels/code.py`) manages the differential drive system. The firmware:
- Controls motor speed and direction
- Reads encoder feedback for position tracking
- Provides serial interface to ROS2

### Knife Controller

The knife controller (`pico_ware_knives/code.py`) manages the cutting mechanism. The firmware:
- Controls knife motor RPM with PID stabilization
- Reads encoder feedback for speed control
- Provides serial interface to ROS2

## Development

### Building ROS2 Packages

```bash
# Build a specific package
colcon build --packages-select diff_drive_hardware

# Source the setup script
source install/setup.bash

# Launch a package
ros2 launch diff_drive_hardware diffbot.launch.py
```

### USB Connection in WSL (Windows)

For Windows users using WSL, USB devices need to be forwarded:

```bash
usbipd list
usbipd bind --busid 2-2
usbipd attach --wsl --busid 2-2
```

## Troubleshooting

### Common Issues

1. **Serial Connection Problems**
   - Check the device connections with `ls /dev/tty*`
   - Run `python3 finddevice.py` to detect connected devices

2. **Docker Communication Issues**
   - Ensure HOST_IP is correctly set
   - Check the ROS_DISCOVERY_SERVER settings

3. **Motor Control Issues**
   - Verify the motor connections
   - Check encoder feedback
   - Run calibration scripts in the calibration directories

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgements

- ROS2 Community
- FastDDS Team
- Contributors to the project

## Contact

For questions or support, please open an issue on the GitHub repository.