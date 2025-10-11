# ROS 2 Workspace Overview

This document provides an overview of the ROS 2 workspace, its packages, and their relationships.

## System Architecture

The ROS 2 workspace is designed to control a robot with a differential drive, an IMU, a mower knife, and a web interface. The packages are organized as follows:

- **`diff_drive_hardware`**: Provides the hardware interface for the differential drive, communicating with the motor controller via a serial connection.
- **`icm_20948`**: A driver for the ICM-20948 IMU, which provides sensor data over a serial connection.
- **`mower_knife_controller`**: Controls the mower knife motor through a serial interface.
- **`slam_tool`**: A wrapper for the `slam_toolbox` package, configured for this specific robot.
- **`web_server`**: A web server that provides a user interface for controlling the robot.
- **`serial`**: This directory is currently empty, but it is likely intended to hold a shared serial communication library.

## Package Details

### `diff_drive_hardware`

- **Description**: This package provides a hardware interface for a differential drive robot. It communicates with a motor controller via a serial connection to control the robot's movement.
- **Dependencies**:
  - `hardware_interface`
  - `pluginlib`
  - `rclcpp`
  - `rclpy`
  - `ros2_control`
  - `ros2_controllers`
  - `controller_manager`
  - `realtime_tools`
  - `serial_driver`
  - `asio_cmake_module`
  - `io_context`
  - `rclcpp_lifecycle`
  - `launch`
  - `launch_ros`
  - `xacro`
  - `libserial-dev`

### `icm_20948`

- **Description**: This package is a driver for the ICM-20948 IMU. It reads sensor data from the IMU via a serial connection and publishes it as a ROS topic.
- **Dependencies**:
  - `rclcpp`
  - `sensor_msgs`
  - `serial`
  - `imu_tools`
  - `rqt`

### `mower_knife_controller`

- **Description**: This package controls the mower knife motor. It communicates with the motor controller via a serial interface.
- **Dependencies**:
  - `rclpy`
  - `std_msgs`
  - `pyserial`

### `slam_tool`

- **Description**: This package is a wrapper for the `slam_toolbox` package. It provides a launch file and configuration for running SLAM on this specific robot.
- **Dependencies**:
  - `rclpy`
  - `slam_toolbox`
  - `launch`
  - `launch_ros`

### `web_server`

- **Description**: This package provides a web server that allows users to control the robot through a web browser. It communicates with the other ROS nodes to send commands and receive status information.
- **Dependencies**:
  - `rclpy`
  - `geometry_msgs`

### `serial`

- **Description**: This directory is currently empty. It is likely intended to hold a shared serial communication library that can be used by the other packages.

## Dockerization

The project is containerized using Docker and orchestrated with `docker-compose`. Each ROS 2 package, along with its dependencies, is encapsulated in its own Docker container. This approach provides a consistent and reproducible environment for development and deployment.

### `docker-compose.yml`

The `docker-compose.yml` file defines the services that make up the ROS 2 system. Each service corresponds to a container and is configured with its own build context, Dockerfile, volumes, and environment variables. The services are all connected to the host network, which allows them to communicate with each other as if they were running on the same machine.

A central "ds" (discovery service) container running `fastdds discovery` is used to facilitate communication between the ROS 2 nodes. All other containers are configured to use this discovery service.

### Dockerfiles

Each service has a corresponding Dockerfile that defines how the container image is built. The Dockerfiles typically perform the following steps:

1.  **Base Image**: Start from a `ros:humble` base image.
2.  **Dependencies**: Install any additional system dependencies using `apt-get` and Python dependencies using `pip`.
3.  **Workspace**: Set up the ROS 2 workspace.
4.  **Build and Run**: Create a startup script that builds the package using `colcon` and then runs the ROS 2 node.

This modular approach allows for individual packages to be built and run independently, while still being able to communicate with each other as part of the larger ROS 2 system.

## Building and Running the Code

To build and run the code, use `docker-compose up`. This will build all the container images and start all the services.

## Agent Instructions

- When working with this codebase, be aware of the dependencies between the packages.
- If you are modifying a package, be sure to rebuild it using `colcon build`.
- If you are adding a new package, be sure to add it to the `src` directory and update the dependencies of any packages that use it.
- When working with the hardware interfaces, make sure that the correct serial ports are specified in the launch files.
- The `serial` directory is empty. If you need to add a shared serial library, this is the place to do it.
- Before submitting any changes, make sure to run all relevant tests and ensure that the code builds and runs correctly.