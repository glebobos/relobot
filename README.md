# ReloBot - ROS2 Robotics Platform

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![AGENTS.md](https://img.shields.io/badge/AGENTS.md-Configured-success)](AGENTS.md)


## Overview

ReloBot is a robotics platform built on ROS2 Humble that integrates differential-drive wheels, cutting blades (knives), IMU, power monitoring, and a camera for navigation. All microcontrollers run C micro-ROS firmware over USB CDC, communicating directly with the ROS2 graph. High-level navigation uses Nav2 with a LiDAR-based EKF stack.

## System Architecture

| Component | Hardware | Firmware | ROS2 interface |
| :--- | :--- | :--- | :--- |
| **Wheels** | XIAO RP2040 | `pico_ware_wheels_microros` | `/robot_joint_commands` + `/robot_joint_states` (JointState) |
| **Knives** | XIAO RP2040 | `pico_ware_knives_microros` | `knives/set_rpm` + `knives/current_rpm` (Float32) |
| **IMU** | RPi Pico RP2040 | `pico_ware_imu_microros` | `/imu/data` (Imu) |
| **Power** | RPi Pico 2 RP2350 | `pico_ware_ina226_microros` | `/battery_state` |
| **LiDAR** | USB | `ldlidar` | `/scan` |
| **Camera** | USB | `camera_ros` | image/depth topics |

The differential drive uses `topic_based_ros2_control/TopicBasedSystem` as the hardware plugin — no custom C++ serial driver. The controller manager talks to the wheels Pico purely through ROS2 topics.

## Hardware

- **Raspberry Pi 5** — main compute, runs all Docker containers
- **XIAO RP2040** ×2 — wheels and knives, USB CDC micro-ROS
- **RPi Pico RP2040** — IMU (ICM20948), USB CDC micro-ROS
- **RPi Pico 2 RP2350** — INA226 power monitor, USB CDC micro-ROS
- DC motors with single-channel 60 CPR encoders
- LiDAR (USB)
- USB camera

## Software Stack

- **ROS2 Humble** — core framework (all nodes run in Docker)
- **micro-ROS** — C firmware on all Pico boards (USB CDC transport)
- **ros2_control** + `topic_based_ros2_control` — hardware abstraction for wheels
- **diff_drive_controller** — velocity → wheel command conversion
- **robot_localization** (EKF) — fuses wheel odometry + IMU
- **Nav2** — autonomous navigation
- **Docker / Docker Compose** — container orchestration

## Project Structure

```
├── start_robot.sh              # Start/stop the full stack
├── FLASHING.md                 # How to flash all Pico boards
├── helpers/
│   ├── joystic_test/           # Joystick testing utilities
│   └── rviz2/                  # RViz2 Docker helper
├── pico_ware_wheels_microros/  # RP2040 micro-ROS firmware — wheels
├── pico_ware_knives_microros/  # RP2040 micro-ROS firmware — knife spindle
├── pico_ware_imu_microros/     # RP2040 micro-ROS firmware — IMU
├── pico_ware_ina226_microros/  # RP2350 micro-ROS firmware — power monitor
├── pico_ware_wheels/           # Legacy CircuitPython wheel code (reference only)
└── ros2_ws/                    # ROS2 workspace
    ├── docker-compose.yml      # All services
    ├── *.dockerfile            # Per-service Dockerfiles
    └── src/
        ├── diff_drive_hardware/    # URDF, controllers config, launch (no C++ plugin)
        ├── camera_ros/             # Camera driver
        ├── ldlidar/                # LiDAR driver
        ├── explore_lite/           # Autonomous exploration
        ├── nav2/                   # Nav2 configuration
        ├── opennav_coverage/       # Coverage path planning
        └── web_server/             # Web-based control UI
```

## Getting Started

### Prerequisites

- Raspberry Pi 5 with Raspberry Pi OS
- Docker and Docker Compose installed
- All Pico boards flashed with their respective micro-ROS firmware (see [FLASHING.md](FLASHING.md))
- USB devices plugged in (see device mapping in [FLASHING.md](FLASHING.md))

### Start the stack

```bash
./start_robot.sh up
```

Development mode (rebuilds ROS2 packages on startup):
```bash
./start_robot.sh up --dev
```

Stop everything:
```bash
./start_robot.sh down
```

Start a single service:
```bash
./start_robot.sh up ros2_diff_robot
```

### Docker Services

| Service | Description |
| :--- | :--- |
| `ros2_frontend` | nginx — serves the web UI |
| `ros2_lidar` | LiDAR driver (`ldlidar`) |
| `ros2_diff_robot` | ros2_control + diff_drive_controller + EKF + micro-ROS agent (wheels, knives, IMU, INA226) |
| `ros2_camera_rp` | Camera driver + AprilTag detection |
| `ros2_nav2` | Nav2 navigation stack |

### Running RViz2

```bash
./rviz2.sh
```

Or directly:
```bash
docker run -it --rm -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg \
  -e DISPLAY -e WAYLAND_DISPLAY -e XDG_RUNTIME_DIR -e PULSE_SERVER \
  --network host osrf/ros:jazzy-desktop rviz2
```

## Coverage Planning Workflow

1. Start the stack with `./start_robot.sh up`.
2. Open the web UI.
3. Click the map to place polygon vertices for the coverage area.
4. Right-click to remove the last point.
5. Click **Preview Coverage** to call the coverage planner.
6. Inspect the generated path in the web UI and RViz.
7. Click **Execute Coverage** to send the path to Nav2 `follow_path`.
8. Click **Stop Coverage** to cancel at any time.
9. Click `Clear Polygon` to remove the active area and preview.

### Headless test command

To verify the backend without using the browser, run:

```bash
docker exec ros2_ws-ros2_nav2-1 bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 run nav2 coverage_preview_test'
```

This publishes a simple rectangular test polygon and requests a coverage preview through the new coverage manager.

### RViz verification topics

For coverage inspection in RViz, add displays for these topics:

- `/coverage/polygon_active`
- `/coverage/preview_path`
- `/coverage_server/coverage_plan`
- `/coverage_server/field_boundary`
- `/coverage_server/planning_field`
- `/coverage_server/swaths`

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