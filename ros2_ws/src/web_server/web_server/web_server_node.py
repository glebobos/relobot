#!/usr/bin/env python3
import threading
import gc
import time
import json
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
from flask import Flask, render_template, request, jsonify

# Flask
app = Flask(__name__, template_folder="./templates")
node = None

# Global values for motor control
_linear = 0.0
_angular = 0.0

# Knife motor control variables
_knife_speed_rpm = 0.0
_knife_min_rpm = 500
_knife_max_rpm = 3000
_knife_last_send_time = 0.0
_knife_send_interval = 0.2  # Send knife commands every 0.2 seconds

# PID control state
_pid_enabled = True
_last_pid_toggle = 0.0
_pid_toggle_cooldown = 0.5  # Cooldown period to avoid multiple toggles

# Controller configuration with defaults
_config = {
    "turn_axis": 0,              # Axis for turning left/right
    "drive_axis": 3,             # Axis for forward/backward
    "knife_button": 7,           # Button for knife control (trigger)
    "pid_toggle_button": 9,      # Button for PID toggle
    "scale_linear": 1.0,         # Maximum linear speed
    "scale_angular": 2.0,        # Maximum angular speed
    "invert_turn": True,         # Invert turn direction
    "invert_drive": True,        # Invert drive direction
    "invert_knife": False        # Invert knife control
}

def load_config():
    """Load controller configuration from file or use defaults."""
    global _config
    
    config_path = os.path.join(os.path.dirname(__file__), 'controller_config.json')
    
    try:
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                loaded_config = json.load(f)
                # Merge loaded config with defaults
                _config.update(loaded_config)
                print(f"Loaded controller config: {_config}")
        else:
            # Save default config for user to edit
            with open(config_path, 'w') as f:
                json.dump(_config, indent=2, fp=f)
                print(f"Created default controller config at {config_path}")
    except Exception as e:
        print(f"Error loading config: {e}")

def process_knife_button(value: float):
    """Process knife button value (0-1) and store RPM."""
    global _knife_speed_rpm, _config
    
    # Apply inversion if configured
    if _config.get("invert_knife", False):
        value = 1.0 - value
    
    # Calculate RPM based on button value (0-1 range)
    if value <= 0.05:  # Small threshold to detect completely off
        rpm = 0
    else:
        # Map button value from 0-1 to min_rpm-max_rpm
        rpm = _knife_min_rpm + value * (_knife_max_rpm - _knife_min_rpm)
    
    # Update the global variable
    _knife_speed_rpm = rpm
    # print(f"Knife button value: {value}, RPM: {rpm:.0f}")

def toggle_pid_control():
    """Toggle PID control."""
    global _pid_enabled, _last_pid_toggle, node
    
    current_time = time.time()
    if current_time - _last_pid_toggle < _pid_toggle_cooldown:
        return  # Avoid multiple toggles due to button bounce
    
    _last_pid_toggle = current_time
    _pid_enabled = not _pid_enabled
    
    if node and rclpy.ok():
        node.toggle_pid_control(_pid_enabled)
        print(f"PID control {'enabled' if _pid_enabled else 'disabled'}")

def send_knife_commands():
    """Periodically send knife RPM commands."""
    global _knife_speed_rpm, _knife_last_send_time
    
    # Flag to track if we've already shown the "set to 0" message
    zero_message_shown = False
    
    while rclpy.ok():
        current_time = time.time()
        
        # Send command if enough time has passed
        if current_time - _knife_last_send_time >= _knife_send_interval:
            if node and rclpy.ok():
                # Always send the most current value
                node.publish_knife_rpm(_knife_speed_rpm)
                
                # Print logic: always show non-zero RPM, but show zero only once
                if _knife_speed_rpm == 0:
                    if not zero_message_shown:
                        print("Setting knife RPM to 0")
                        zero_message_shown = True
                else:
                    print(f"Setting knife RPM to {_knife_speed_rpm:.0f}")
                    zero_message_shown = False  # Reset flag when RPM is non-zero
                    
            _knife_last_send_time = current_time
        
        time.sleep(0.1)  # Check frequently but don't overwhelm the CPU

class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server_node')
        self.publisher = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.knife_publisher = self.create_publisher(Float32, 'knives/set_rpm', 10)
        
        # Create client for PID control service
        self.pid_client = self.create_client(SetBool, 'knives/enable_pid')
        
        # Wait at most 5 seconds for service
        if not self.pid_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning('PID control service not available after waiting!')
        else:
            self.get_logger().info('PID control service is available')
            
        # Run garbage collector every 10 seconds
        self.create_timer(10.0, self.gc_callback)

    def gc_callback(self):
        gc.collect()
        self.get_logger().debug('Memory cleanup performed')

    def publish_motor_commands(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.publisher.publish(msg)
        
    def publish_knife_rpm(self, rpm):
        msg = Float32()
        msg.data = float(rpm)
        self.knife_publisher.publish(msg)
    
    def toggle_pid_control(self, enable):
        """Toggle PID control via service call."""
        if not self.pid_client.service_is_ready():
            self.get_logger().warning('PID service is not ready')
            return
            
        request = SetBool.Request()
        request.data = enable
        
        # Create the future
        future = self.pid_client.call_async(request)
        
        # Add a callback that will be executed when the future completes
        future.add_done_callback(lambda f: self._handle_pid_toggle_callback(f, enable))
    
    def _handle_pid_toggle_callback(self, future, enable_requested):
        """Callback for when the PID toggle future completes."""
        try:
            response = future.result()
            if response is None:
                self.get_logger().error('PID toggle service returned None')
                return
                
            self.get_logger().info(f'PID toggle response: success={response.success}, ' +
                                  f'message="{response.message}"')
        except Exception as e:
            self.get_logger().error(f'Failed to call PID toggle service: {e}')
            self.get_logger().info(f'Note: We requested PID to be {"enabled" if enable_requested else "disabled"}')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/gamepad', methods=['POST'])
def gamepad():
    global _linear, _angular, _config, node
    
    try:
        data = request.get_json(force=True, silent=True)
        # print(f"[GAMEPAD] Incoming data: {data}")
        
        if not data or not isinstance(data, dict):
            return jsonify(success=False, message="Invalid data format")
        
        # Get axes and buttons from the incoming data
        axes = data.get('axes', [])
        buttons = data.get('buttons', [])
        
        # Make sure we have enough data
        if len(axes) <= max(_config["turn_axis"], _config["drive_axis"]):
            return jsonify(success=False, message="Not enough axis data provided")
            
        if len(buttons) <= max(_config["knife_button"], _config["pid_toggle_button"]):
            return jsonify(success=False, message="Not enough button data provided")
        
        # Process movement controls
        turn_value = axes[_config["turn_axis"]]
        drive_value = axes[_config["drive_axis"]]
        
        # Apply inversions if configured
        if _config.get("invert_turn", False):
            turn_value = -turn_value
        if _config.get("invert_drive", False):
            drive_value = -drive_value
        
        # Calculate linear and angular velocities
        _angular = turn_value * _config["scale_angular"]
        _linear = drive_value * _config["scale_linear"]
        
        # Process knife control from button
        knife_value = buttons[_config["knife_button"]]
        process_knife_button(knife_value)
        
        # Check for PID toggle button press (1 = pressed, 0 = not pressed)
        pid_button = _config["pid_toggle_button"]
        if pid_button < len(buttons) and buttons[pid_button] == 1:
            toggle_pid_control()
            
        # Publish motor commands if node is initialized
        if node and rclpy.ok():
            node.publish_motor_commands(_linear, _angular)
            
        return jsonify(success=True)
    except Exception as e:
        print(f"Error processing gamepad data: {e}")
        return jsonify(success=False, error=str(e))

@app.route('/set_motors', methods=['POST'])
def set_motors():
    try:
        data = request.json
        x = float(data['x'])
        y = float(data['y'])
        # Apply scaling
        linear_velocity = y * _config["scale_linear"]
        angular_velocity = -x * _config["scale_angular"]
        if node and rclpy.ok():
            node.publish_motor_commands(linear_velocity, angular_velocity)
        return jsonify(success=True)
    except Exception as e:
        return jsonify(success=False, error=str(e))

@app.route('/config', methods=['GET'])
def get_config():
    return jsonify(success=True, config=_config)

@app.route('/config', methods=['POST'])
def update_config():
    global _config
    
    try:
        new_config = request.get_json(force=True, silent=True)
        if not new_config:
            return jsonify(success=False, message="Invalid data format")
            
        # Update the config
        _config.update(new_config)
        
        # Save to file
        config_path = os.path.join(os.path.dirname(__file__), 'controller_config.json')
        with open(config_path, 'w') as f:
            json.dump(_config, indent=2, fp=f)
        
        return jsonify(success=True, message="Configuration updated", config=_config)
    except Exception as e:
        return jsonify(success=False, error=str(e))

def main(args=None):
    global node
    
    rclpy.init(args=args)
    
    # Load controller configuration
    load_config()
    
    node = WebServerNode()

    # Run Flask in a separate thread
    flask_thread = threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=80, threaded=False, debug=False),
        daemon=True
    )
    flask_thread.start()
    
    # Start thread for sending knife commands
    knife_command_thread = threading.Thread(target=send_knife_commands, daemon=True)
    knife_command_thread.start()

    # Start rclpy.spin to process ROS callbacks
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()