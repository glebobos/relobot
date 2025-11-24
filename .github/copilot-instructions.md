# ReloBot AI Agent Instructions

## Project Overview
ReloBot is a ROS 2 Humble robotics platform running on Raspberry Pi 5, integrating differential drive, mower blades, IMU, ToF camera, and Lidar. It uses a distributed architecture with Raspberry Pi Picos for low-level hardware control.

## Architecture & Patterns
- **Containerization**: All ROS 2 nodes run in Docker containers defined in `ros2_ws/docker-compose.yml`.
- **Communication**: Uses FastDDS with a centralized Discovery Server (`ds` service).
  - `ROS_DISCOVERY_SERVER=${HOST_IP}:11811` is required for all nodes.
  - `ROS_SUPER_CLIENT=True` is set for all nodes.
  - `network_mode: host` is used for all containers.
- **Hardware Abstraction**:
  - `pico_ware_*`: Firmware for RP2040 microcontrollers (Wheels, Knives, IMU).
  - `ros2_ws/src/diff_drive_hardware`: ROS 2 hardware interface for the base.
  - `ros2_ws/src/mower_knife_controller`: Control node for the cutting mechanism.
- **Workspace**: `ros2_ws` is the main ROS 2 workspace.

## Critical Workflows

### Lifecycle Management
- **Start Robot**: Always use `./start_robot.sh up` instead of raw `docker compose`.
  - This script handles `HOST_IP` export and internet connectivity checks.
- **Stop Robot**: `./start_robot.sh down`.
- **Single Service**: `./start_robot.sh up [service_name]` (e.g., `./start_robot.sh up ros2_lidar`).

### Development & Building
- **Build System**: `colcon` is used for building ROS 2 packages.
  - Inside a container: `colcon build --symlink-install`.
  - To rebuild a specific package: `colcon build --packages-select [package_name]`.
- **Device Detection**: Run `python3 finddevice.py` to identify and map serial devices before starting services if hardware configuration changes.

### Visualization
- **RViz2**: Use `./start_rviz2.sh` (or `rviz` alias if configured) to launch visualization with correct DDS settings.
  - Do not run `rviz2` directly without the environment variables for the discovery server.

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
- **Network**: `HOST_IP` must be set correctly for FastDDS to work. The startup script handles this.
- **Serial Ports**: Device paths (e.g., `/dev/ttyACM0`) are hardcoded in `docker-compose.yml`. Ensure devices are plugged in correctly or update the compose file/udev rules.
- **DDS**: If nodes can't see each other, check if they are pointing to the correct `ROS_DISCOVERY_SERVER`.
