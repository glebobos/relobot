# ReloBot - ROS2 Robotics Platform

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![AGENTS.md](https://img.shields.io/badge/AGENTS.md-Configured-success)](AGENTS.md)


## Overview

ReloBot is an autonomous robotics platform built on **ROS 2 Lyrical** with **Zenoh Shared Memory (`rmw_zenoh_cpp`)**. It integrates differential-drive wheels, cutting blades (knives), IMU, power monitoring, and a camera for navigation. All four microcontrollers run **native Pico-ROS (Zenoh)** communicating over USB serial via a central `ros2_zenoh_router`. High-level navigation uses Nav2 with a LiDAR-based EKF stack.

Architecture decisions, performance benchmarks, and hardware validation results are documented in [wiki/zenoh_picoros_architecture_and_decisions.md](wiki/zenoh_picoros_architecture_and_decisions.md).

## System Architecture

| Component | Hardware | Firmware | ROS 2 Interface |
| :--- | :--- | :--- | :--- |
| **Wheels** | XIAO RP2040 | `pico_ware_wheels_picoros` | `/robot_joint_commands` + `/robot_joint_states` (JointState) |
| **Knives** | XIAO RP2040 | `pico_ware_knives_picoros` | `knives/set_rpm` + `knives/current_rpm` (Float32) |
| **IMU** | RPi Pico RP2040 | `pico_ware_imu_picoros` | `/imu/data` (Imu) |
| **Power** | XIAO RP2350 | `pico_ware_ina226_picoros` | `/battery_state` |
| **LiDAR** | USB | `ldlidar` | `/scan` |
| **Camera** | USB | `camera_ros` | image/depth topics |

The differential drive uses `topic_based_ros2_control/TopicBasedSystem` as the hardware plugin — no custom C++ serial driver. The controller manager talks to the wheels Pico purely through ROS 2 topics over Zenoh.

## Hardware

- **Raspberry Pi 5** — main compute, runs all Docker containers
- **XIAO RP2040** ×2 — wheels and knives, native Pico-ROS USB serial
- **RPi Pico RP2040** — IMU (ICM20948), native Pico-ROS USB serial
- **XIAO RP2350** — INA226 power monitor, native Pico-ROS USB serial
- DC motors with single-channel 60 CPR encoders
- LiDAR (USB)
- USB camera

## Software Stack

- **ROS 2 Lyrical** — core framework (all nodes run in Docker with `rmw_zenoh_cpp`)
- **Zenoh Shared Memory** — zero-copy host transport (`transport/shared_memory/enabled=true`)
- **Native Pico-ROS** — FreeRTOS + Zenoh-Pico firmware on all Pico boards (USB CDC transport)
- **ros2_control** + `topic_based_ros2_control` — hardware abstraction for wheels
- **diff_drive_controller** — velocity → wheel command conversion
- **robot_localization** (EKF) — fuses wheel odometry + IMU
- **Nav2** — autonomous navigation
- **Docker / Docker Compose** — container orchestration

## Project Structure

```
├── start_robot.sh              # Start/stop the full stack
├── start_sim.sh                # Start Gazebo simulation launcher
├── run_firmware.sh             # Unified firmware build, scan, flash, & monitor CLI
├── start_wheel_calibration.sh  # Automated wheel feedforward calibration launcher
├── start_camera_calibration.sh # Camera calibration launcher
├── rviz2.sh                    # RViz2 visualization launcher
├── FLASHING.md                 # How to flash all Pico boards
├── helpers/
│   ├── wheel_calibration/      # Wheel velocity calibration & diagnostic scripts
│   ├── camera_calibration/     # Camera calibration tool
│   ├── cpu_profiler/           # CPU and Memory profiler for Docker & ROS 2
│   └── rviz2/                  # RViz2 Docker configuration
├── pico_firmware/              # Shared foundation (Pico SDK 2.2.0, FreeRTOS, Zenoh-Pico, Docker builder)
├── pico_ware_wheels_picoros/   # RP2040 Pico-ROS firmware — wheels
├── pico_ware_knives_picoros/   # RP2040 Pico-ROS firmware — knife spindle
├── pico_ware_imu_picoros/      # RP2040 Pico-ROS firmware — IMU
├── pico_ware_ina226_picoros/   # RP2350 Pico-ROS firmware — power monitor
├── wiki/                       # Architecture decision records & benchmarks
└── ros2_ws/                    # ROS 2 workspace
    ├── docker-compose.yml      # All services
    ├── ZenohRouter.dockerfile  # Zenoh router container
    ├── DiffDriveHardware.dockerfile # ros2_control diffbot hardware container
    ├── zenoh_devices.json      # Mapping of USB by-id serial endpoints
    └── src/
        ├── diff_drive_hardware/    # URDF, controllers config, launch
        ├── camera_ros/             # Camera driver
        ├── ldlidar/                # LiDAR driver
        ├── explore_lite/           # Autonomous exploration
        ├── nav2/                   # Nav2 configuration
        ├── opennav_coverage/       # Coverage path planning
        └── web_server/             # Web-based control UI
```

## Getting Started

### Prerequisites

- Raspberry Pi 5 with Raspberry Pi OS (64-bit)
- Docker and Docker Compose installed
- All Pico boards flashed with their respective Pico-ROS firmware (see [FLASHING.md](FLASHING.md))
- USB devices configured in `ros2_ws/zenoh_devices.json` (see device mapping in [FLASHING.md](FLASHING.md))

### Start the stack

```bash
# Physical Hardware Mode (auto-rebuilds packages with --dev):
./start_robot.sh up [--dev]

# Gazebo Simulation Mode:
./start_sim.sh up [--world garden|obstacles|empty] [--gui|--headless] [--dev]

# Stop everything:
./start_robot.sh down
```

### Docker Services

| Service | Description |
| :--- | :--- |
| `ros2_zenoh_router` | rmw_zenohd router connecting host nodes and microcontroller serial devices |
| `ros2_frontend` | nginx — serves the web UI |
| `ros2_lidar` | LiDAR driver (`ldlidar`) |
| `ros2_diff_robot` | ros2_control + diff_drive_controller + EKF |
| `ros2_camera_rp` | Camera driver + AprilTag detection |
| `ros2_nav2` | Nav2 navigation stack |

### Running RViz2

```bash
# Physical robot view:
./rviz2.sh

# Gazebo simulation view:
./rviz2.sh --sim
```

## Coverage Planning Workflow

1. Start the stack with `./start_robot.sh up`.
2. Open the web UI at `http://raspberrypi.local/` (or `http://localhost/`).
3. Click the map to place polygon vertices for the coverage area.
4. Right-click to remove the last point.
5. Click **Preview Coverage** to call the coverage planner.
6. Inspect the generated path in the web UI and RViz.
7. Click **Execute Coverage** to send the path to Nav2 `follow_path`.
8. Click **Stop Coverage** to cancel at any time.

## Monitoring

```bash
# Joint states from wheels Pico
docker compose -f ros2_ws/docker-compose.yml exec ros2_diff_robot bash -c "source /opt/ros/lyrical/setup.bash && ros2 topic echo /robot_joint_states"

# Odometry
docker compose -f ros2_ws/docker-compose.yml exec ros2_diff_robot bash -c "source /opt/ros/lyrical/setup.bash && ros2 topic echo /odom"

# LiDAR
docker compose -f ros2_ws/docker-compose.yml exec ros2_lidar bash -c "source /opt/ros/lyrical/setup.bash && ros2 topic echo /scan"
```

## Troubleshooting

- **Microcontroller not connecting** — check `python3 ros2_ws/zenoh_devices.py validate` and `ls -l /dev/serial/by-id/`
- **Motors not moving** — confirm `ros2_zenoh_router` is running and `ros2 topic echo /robot_joint_states` receives messages
- **Robot turns instead of going straight** — recalibrate feedforward with `./start_wheel_calibration.sh` and update constants in `pico_ware_wheels_picoros/src/main.c`
- **Pico needs reflashing** — see [FLASHING.md](FLASHING.md)

## License

Apache License 2.0 — see [LICENSE](LICENSE).
