# Robot Web Control Server

A web interface for controlling a robot using ROS2, providing movement control and knife mechanism operation through a browser-based interface.

## Overview

The Robot Web Control Server provides a web-based interface for controlling a robot using ROS2. It allows operators to control the robot's movement and knife mechanism through a browser, with support for both gamepad controllers and on-screen controls.

## Features

- Web-based control interface accessible from any device with a browser
- Gamepad controller support with configurable mappings
- Real-time control of robot movement (linear and angular velocity)
- Knife mechanism control with adjustable RPM
- PID control toggle functionality
- Emergency stop capability
- Configurable controller settings
- REST API for programmatic control

## Requirements

- Python 3.8 or higher
- ROS2 (Humble or newer)
- Flask
- Modern web browser with JavaScript enabled
- Gamepad controller (optional)

### ROS2 Dependencies

- `geometry_msgs`
- `std_msgs`
- `std_srvs`

## Installation

1. Create a new directory for your project:

```bash
mkdir -p ~/robot_web_control/templates
cd ~/robot_web_control
```

2. Create the main script file:

```bash
# Copy the provided Python code into this file
nano robot_web_server.py
```

3. Make the script executable:

```bash
chmod +x robot_web_server.py
```

4. Install Python dependencies:

```bash
pip install flask rclpy
```

5. Create a basic HTML template file in the templates directory (you'll need to create your own HTML interface).

## Configuration

### Controller Configuration

The controller configuration is stored in `controller_config.json` and will be automatically created with default values on first run. You can modify it either directly or through the web interface.

### Configuration Parameters

| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `turn_axis` | int | Joystick axis for turning left/right | 0 |
| `drive_axis` | int | Joystick axis for forward/backward movement | 3 |
| `knife_button` | int | Button index for knife control | 7 |
| `pid_toggle_button` | int | Button index for toggling PID control | 9 |
| `scale_linear` | float | Maximum linear speed scaling factor | 1.0 |
| `scale_angular` | float | Maximum angular speed scaling factor | 2.0 |
| `invert_turn` | bool | Invert turn direction | true |
| `invert_drive` | bool | Invert drive direction | true |
| `invert_knife` | bool | Invert knife control | false |

### Updating Configuration

You can update the configuration in three ways:

1. **Web Interface**: Navigate to the settings page in the web interface to modify controller settings.

2. **API**: Send a POST request to the `/config` endpoint with the updated configuration values:

```bash
curl -X POST http://robot-ip/config \
  -H "Content-Type: application/json" \
  -d '{"scale_linear": 0.8, "scale_angular": 1.5}'
```

3. **Direct File Edit**: Modify the `controller_config.json` file directly and restart the server.

## Usage

### Starting the Server

First, ensure ROS2 is properly sourced:

```bash
source /opt/ros/humble/setup.bash  # Replace 'humble' with your ROS2 distribution
```

Run the server using:

```bash
cd ~/robot_web_control
sudo python3 robot_web_server.py  # sudo is needed to bind to port 80
```

To run on a different port (e.g., 8080) without sudo:

```bash
cd ~/robot_web_control
python3 robot_web_server.py
```

Then modify the Flask app.run line in the code to use port 8080 instead of 80.

### Accessing the Web Interface

Open a web browser and navigate to:

```
http://<robot-ip>/
```

Replace `<robot-ip>` with your robot's IP address or hostname.

### Controlling the Robot

- **Gamepad Controller**: Connect a gamepad to your device and use the configured axes and buttons to control the robot.
- **On-screen Controls**: Use the virtual joystick on the web interface to control movement.
- **Emergency Stop**: Press the emergency stop button on the web interface to immediately stop all robot movement.

## API Reference

### Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Serves the main web interface |
| `/gamepad` | POST | Processes gamepad controller input |
| `/set_motors` | POST | Sets motor speeds directly |
| `/config` | GET | Retrieves current configuration |
| `/config` | POST | Updates configuration |
| `/emergency_stop` | POST | Triggers emergency stop |

### Example API Usage

#### Set Motor Speed

```bash
curl -X POST http://robot-ip/set_motors \
  -H "Content-Type: application/json" \
  -d '{"x": 0.0, "y": 0.5}'
```

#### Get Configuration

```bash
curl -X GET http://robot-ip/config
```

#### Update Configuration

```bash
curl -X POST http://robot-ip/config \
  -H "Content-Type: application/json" \
  -d '{"scale_linear": 0.8, "invert_turn": false}'
```

#### Emergency Stop

```bash
curl -X POST http://robot-ip/emergency_stop
```

## Troubleshooting

### Common Issues

#### Server Won't Start

- Ensure ROS2 is properly installed and sourced
- Check that required ROS2 topics and services exist
- Verify port 80 is available (may require running as root)

#### Controller Not Responding

- Check controller connection to your device
- Verify controller mappings in configuration
- Inspect browser console for JavaScript errors

#### Robot Not Moving

- Ensure ROS2 topics are correctly configured
- Check that the robot's motor controllers are powered and operational
- Verify that the emergency stop is not activated

### Logs

The server logs to standard output and can be redirected to a file:

```bash
python3 robot_web_server.py > robot_server.log 2>&1
```

Set the `ENABLE_FLASK_LOGS` environment variable to enable detailed Flask logs:

```bash
ENABLE_FLASK_LOGS=true python3 robot_web_server.py
```

## Customizing the Server

### Changing ROS2 Topics

If your robot uses different ROS2 topics, modify the following lines in the `RobotROSNode` class:

```python
self.publisher = self.create_publisher(
    Twist, 
    '/diff_drive_controller/cmd_vel_unstamped',  # Change this to your topic
    10
)
self.knife_publisher = self.create_publisher(
    Float32, 
    'knives/set_rpm',  # Change this to your topic
    10
)
```

### Adjusting Knife RPM Range

To change the minimum and maximum knife RPM, modify these values in the `KnifeController` class:

```python
self.min_rpm = 500  # Change to your minimum RPM
self.max_rpm = 3000  # Change to your maximum RPM
```

### Running on System Startup

To run the server automatically on system startup, create a systemd service:

```bash
sudo nano /etc/systemd/system/robot-web-control.service
```

Add the following content:

```
[Unit]
Description=Robot Web Control Server
After=network.target

[Service]
User=root
WorkingDirectory=/home/your_username/robot_web_control
ExecStart=/usr/bin/python3 /home/your_username/robot_web_control/robot_web_server.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable and start the service:

```bash
sudo systemctl enable robot-web-control.service
sudo systemctl start robot-web-control.service
```

---

For questions or support, please contact the system administrator.