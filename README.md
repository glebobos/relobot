# ReloBot - ROS2 Robotics Platform

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![AGENTS.md](https://img.shields.io/badge/AGENTS.md-Configured-success)](AGENTS.md)


## Overview

ReloBot is a comprehensive robotics platform built on ROS2 (Robot Operating System 2) that integrates multiple hardware components including differential drive motors, cutting blades (knives), IMU sensors, ToF cameras for navigation and object detection, and joystick control for manual operation. The system is designed with a microcontroller architecture using Raspberry Pi Pico boards for hardware control, with a central ROS2 system coordinating the operation.

## System Architecture

The ReloBot platform consists of the following major components:

1. **Differential Drive System** - Controls robot movement with two independently driven wheels
2. **Cutting Mechanism** - Motor-controlled blades/knives with RPM control and PID stabilization
3. **Sensor Suite**:
   - IMU (Inertial Measurement Unit) - ICM20948 for orientation data
   - ToF (Time of Flight) Camera - For distance measurement and object detection
4. **Control Interfaces**:
   - Web-based Control Interface - For remote operation and monitoring
   - Joystick Controller - For direct manual control with haptic feedback
5. **SLAM** (Simultaneous Localization and Mapping) - For environment mapping and navigation

## Hardware Components

- Raspberry Pi 5 (main controller)
- Raspberry Pi Pico microcontrollers (for wheels and knife control)
- ICM20948 IMU sensor
- ToF Camera (B0410)
- DC Motors with encoders
- Bluetooth/USB Joystick controller
- Various serial interfaces and USB connections

## Software Stack

- **ROS2 Humble** - Core robotics framework
- **FastDDS** - For distributed communications
- **Python** - For high-level control and microcontroller programming
- **Docker** - For containerization of ROS2 nodes
- **Pygame** - For joystick control interface and haptic feedback
- **Systemd** - For autostart services on Raspberry Pi
- **Web Server** - For remote control interface

## Project Structure

```
├── finddevice.py              # Utility to find connected serial devices
├── start_app_helper.sh        # Startup script for the application (autostart)
├── B0410_Tof_Firmware/        # ToF camera firmware
├── helpers/                   # Utility scripts and tools
│   ├── joystic_test/          # Joystick testing utilities
│   │   └── joystic.py         # Joystick controller interface with haptic feedback
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

- Raspberry Pi 5 with Raspberry Pi OS
- Docker and Docker Compose
- ROS2 Humble (if running outside of Docker)
- Python 3.x
- USB access to hardware components
- Bluetooth or USB joystick controller

### Hardware Setup

1. Connect the Raspberry Pi Pico boards running the wheel and knife control firmware
2. Connect the IMU sensor to a USB port
3. Connect the ToF camera to a USB port
4. Connect the joystick controller via Bluetooth or USB

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
7. **Joystick Controller** (`ros2_joystick`) - Manages joystick input and haptic feedback

The container configurations are defined in the respective Dockerfiles and orchestrated through `docker-compose.yml`.

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
  -e XDG_RUNTIME_DIR -e ROS_DISCOVERY_SERVER=192.168.18.120:11811 \
  -e ROS_SUPER_CLIENT=True -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e PULSE_SERVER -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  --network host osrf/ros:jazzy-desktop rviz2
```

### Creating Convenient Aliases

To make running RViz2 and other ReloBot commands easier, you can create convenient aliases:

#### RViz2 Alias

1. Make the RViz2 script executable (if not already done):
   ```bash
   chmod +x /home/admin/Documents/relobot/start_rviz2.sh
   ```

2. Add the alias to your shell configuration:
   ```bash
   echo 'alias rviz="/home/admin/Documents/relobot/start_rviz2.sh"' >> ~/.bashrc
   ```

3. Reload your shell configuration:
   ```bash
   source ~/.bashrc
   ```

4. Now you can start RViz2 from any terminal with:
   ```bash
   rviz
   ```

#### Additional Useful Aliases

You can also create aliases for other common tasks:

```bash
# Add more aliases to ~/.bashrc
echo 'alias relobot-start="cd /home/admin/Documents/relobot/ros2_ws && docker compose up"' >> ~/.bashrc
echo 'alias relobot-stop="cd /home/admin/Documents/relobot/ros2_ws && docker compose down"' >> ~/.bashrc
echo 'alias relobot-logs="cd /home/admin/Documents/relobot/ros2_ws && docker compose logs -f"' >> ~/.bashrc
echo 'alias relobot-find="python3 /home/admin/Documents/relobot/finddevice.py"' >> ~/.bashrc

# Reload shell configuration
source ~/.bashrc
```

After setting up these aliases, you can use:
- `rviz` - Start RViz2 visualization
- `relobot-start` - Start all ReloBot services
- `relobot-stop` - Stop all ReloBot services  
- `relobot-logs` - View live logs from all services
- `relobot-find` - Find and identify connected devices

## Robot Control

### Basic Movement Commands

Control the robot via ROS2 topics:

```bash
# Move forward (0.2 m/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" --once

# Turn left (0.5 rad/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}" --once

# Turn right (-0.5 rad/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: -0.5}}" --once

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

### Joystick Controller

The ReloBot supports control via a joystick/gamepad controller, providing intuitive manual control with haptic feedback.

#### Setting Up the Controller

1. Connect your controller via USB or pair it via Bluetooth:
   ```bash
   # For Bluetooth controllers
   bluetoothctl
   # Then in the bluetoothctl prompt:
   scan on
   # Wait for your controller to appear, note its MAC address
   pair XX:XX:XX:XX:XX:XX
   connect XX:XX:XX:XX:XX:XX
   # Exit with:
   exit
   ```

2. Test the controller:
   ```bash
   python3 helpers/joystic_test/joystic.py
   ```

#### Controller Features

- Analog stick control for precise movement
- Haptic feedback for obstacles and system events
- Button mapping for special functions:
  - Start/Stop cutting blades
  - Reset navigation
  - Emergency stop

#### Controller Configuration

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

4. **Joystick Controller Issues**
   - Check controller battery/connection status
   - Run `python3 helpers/joystic_test/joystic.py` to test controller functionality
   - For Bluetooth controllers, reinitiate pairing if connection is lost

5. **Autostart Problems**
   - Check service status with `sudo systemctl status relobot.service`
   - Verify logs with `sudo journalctl -u relobot.service`
   - Ensure network connection is available before Docker containers start

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgements

- ROS2 Community
- FastDDS Team
- Contributors to the project

## Contact

For questions or support, please open an issue on the GitHub repository.

## Raspberry Pi 5 Autostart Configuration

To configure ReloBot to automatically start when the Raspberry Pi 5 boots up:

1. Create a systemd service file:
   ```bash
   sudo nano /etc/systemd/system/relobot.service
   ```

2. Add the following content:
   ```
   [Unit]
   Description=ReloBot ROS2 Service
   After=network.target docker.service
   Requires=docker.service

   [Service]
   Type=simple
   User=admin
   WorkingDirectory=/home/admin/Documents/relobot
   ExecStart=/home/admin/Documents/relobot/start_app_helper.sh
   Restart=on-failure
   RestartSec=10

   [Install]
   WantedBy=multi-user.target
   ```

3. Enable and start the service:
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable relobot.service
   sudo systemctl start relobot.service
   ```

4. Check service status:
   ```bash
   sudo systemctl status relobot.service
   ```

### Autostart Troubleshooting

If the autostart service fails:

1. Check the systemd logs:
   ```bash
   sudo journalctl -u relobot.service
   ```

2. Verify that the start_app_helper.sh script has execute permissions:
   ```bash
   chmod +x /home/admin/Documents/relobot/start_app_helper.sh
   ```

3. Make sure Docker service is running:
   ```bash
   sudo systemctl status docker
   ```