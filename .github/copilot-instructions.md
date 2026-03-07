# ReloBot AI Agent Instructions

## Project Overview
ReloBot is a ROS 2 Humble robotics platform running on Raspberry Pi 5, integrating differential drive, mower blades, IMU, ToF camera, and Lidar. It uses a distributed architecture with Raspberry Pi Picos for low-level hardware control.

## Architecture & Patterns
- **Containerization**: All ROS 2 nodes run in Docker containers defined in `ros2_ws/docker-compose.yml`.
  - `network_mode: host` is used for all containers.
- **Hardware Abstraction**:
  - `pico_ware_*`: Firmware for RP2040 microcontrollers (Wheels, Knives, IMU).
  - `ros2_ws/src/diff_drive_hardware`: ROS 2 hardware interface for the base.
  - `ros2_ws/src/mower_knife_controller`: Control node for the cutting mechanism.
- **Workspace**: `ros2_ws` is the main ROS 2 workspace.

## Critical Workflows

### Lifecycle Management
- **Start Robot**: Always use `./start_robot.sh up` instead of raw `docker compose`.
- **Stop Robot**: `./start_robot.sh down`.
- **Single Service**: `./start_robot.sh up [service_name]` (e.g., `./start_robot.sh up ros2_lidar`).


## Code Conventions
- **Python**: Used for high-level nodes and microcontroller firmware.
- **C++**: Used for performance-critical hardware interfaces (e.g., `diff_drive_hardware`).
- **Dockerfiles**: Located in `ros2_ws/` root (e.g., `DiffDriveHardware.dockerfile`).
- **Launch Files**: Python-based launch files are preferred over XML.

## Key Files
- `ros2_ws/docker-compose.yml`: Service orchestration and config.
- `start_robot.sh`: Main entry point script.
- `ros2_ws/src/`: Source code for ROS 2 packages.
- `pico_ware_*/code.py`: Main logic for microcontroller firmware.

## Common Pitfalls
- **Serial Ports**: Device paths (e.g., `/dev/ttyACM0`) are hardcoded in `docker-compose.yml`. Ensure devices are plugged in correctly or update the compose file/udev rules.
