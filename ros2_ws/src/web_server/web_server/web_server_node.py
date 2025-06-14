#!/usr/bin/env python3
"""
Robot Web Control Server with Video Streaming

Added video streaming capability for ToF camera images.
"""

import threading
import gc
import time
import json
import os
import logging
from dataclasses import dataclass, asdict
from typing import Dict, Any, Callable, Tuple
import io

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image  # Add this import
from flask import Flask, render_template, request, jsonify, Response
from flask.logging import default_handler
import cv2
from cv_bridge import CvBridge  # Add this import

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('robot_webserver')


@dataclass
class ControllerConfig:
    """Configuration for controller mappings and settings."""
    turn_axis: int = 0
    drive_axis: int = 3
    knife_button: int = 7
    pid_toggle_button: int = 9
    scale_linear: float = 1.0
    scale_angular: float = 2.0
    invert_turn: bool = True
    invert_drive: bool = True
    invert_knife: bool = False
    
    def validate(self) -> Tuple[bool, str]:
        """Validate configuration values."""
        if not all(isinstance(val, int) for val in [
            self.turn_axis, self.drive_axis, 
            self.knife_button, self.pid_toggle_button
        ]):
            return False, "Axes and button indices must be integers"
            
        if not all(isinstance(val, (int, float)) for val in [
            self.scale_linear, self.scale_angular
        ]):
            return False, "Scale values must be numeric"
            
        if self.scale_linear <= 0 or self.scale_angular <= 0:
            return False, "Scale values must be positive"
            
        return True, ""
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return asdict(self)
    
    @classmethod
    def from_dict(cls, config_dict: Dict[str, Any]) -> 'ControllerConfig':
        """Create instance from dictionary."""
        valid_fields = cls.__annotations__.keys()
        filtered_dict = {k: v for k, v in config_dict.items() if k in valid_fields}
        return cls(**filtered_dict)
    
    def save_to_file(self, filepath: str) -> None:
        """Save configuration to a JSON file."""
        try:
            with open(filepath, 'w') as f:
                json.dump(self.to_dict(), f, indent=2)
            logger.info(f"Configuration saved to {filepath}")
        except Exception as e:
            logger.error(f"Error saving configuration: {e}")
            
    @classmethod
    def load_from_file(cls, filepath: str) -> 'ControllerConfig':
        """Load configuration from JSON file or create default."""
        try:
            if os.path.exists(filepath):
                with open(filepath, 'r') as f:
                    config_dict = json.load(f)
                logger.info(f"Loaded configuration from {filepath}")
                return cls.from_dict(config_dict)
            else:
                config = cls()
                config.save_to_file(filepath)
                logger.info(f"Created default configuration at {filepath}")
                return config
        except Exception as e:
            logger.error(f"Error loading configuration: {e}")
            return cls()


class KnifeController:
    """Controls the knife mechanism."""
    
    def __init__(self, config: ControllerConfig, publish_callback: Callable[[float], None]):
        self.config = config
        self.publish_callback = publish_callback
        self.min_rpm = 500
        self.max_rpm = 3000
        self.rpm = 0.0
        self.last_send_time = 0.0
        self.send_interval = 0.2
        self.lock = threading.Lock()
        self.zero_message_shown = False
        self._running = True
        
    def process_button_value(self, value: float) -> None:
        """Process knife button value (0-1) and store RPM."""
        with self.lock:
            if self.config.invert_knife:
                value = 1.0 - value
            
            if value <= 0.05:
                rpm = 0
            else:
                rpm = self.min_rpm + value * (self.max_rpm - self.min_rpm)
            
            self.rpm = rpm
    
    def send_commands_loop(self) -> None:
        """Periodically send knife RPM commands."""
        logger.info("Starting knife command loop")
        try:
            while self._running and rclpy.ok():
                current_time = time.time()
                
                with self.lock:
                    rpm = self.rpm
                    
                    if current_time - self.last_send_time >= self.send_interval:
                        try:
                            self.publish_callback(rpm)
                            
                            if rpm == 0:
                                if not self.zero_message_shown:
                                    self.zero_message_shown = True
                            else:
                                self.zero_message_shown = False
                        except Exception as e:
                            logger.error(f"Error publishing knife RPM: {e}")
                            
                        self.last_send_time = current_time
                
                time.sleep(0.1)
        except Exception as e:
            logger.error(f"Knife command loop crashed: {e}", exc_info=True)
        finally:
            logger.info("Knife command loop stopped")
    
    def stop(self) -> None:
        """Stop the command loop."""
        self._running = False


class PIDController:
    """Manages PID control state."""
    
    def __init__(self, toggle_callback: Callable[[bool], None]):
        self.enabled = True
        self.toggle_callback = toggle_callback
        self.last_toggle_time = 0.0
        self.toggle_cooldown = 0.5
        self.lock = threading.Lock()
    
    def toggle(self) -> None:
        """Toggle PID control with cooldown."""
        with self.lock:
            current_time = time.time()
            if current_time - self.last_toggle_time < self.toggle_cooldown:
                return
            
            self.enabled = not self.enabled
            self.last_toggle_time = current_time
            
            try:
                self.toggle_callback(self.enabled)
                logger.info(f"PID control {'enabled' if self.enabled else 'disabled'}")
            except Exception as e:
                logger.error(f"Error toggling PID control: {e}")


class MotorController:
    """Controls the robot's motor commands."""
    
    def __init__(self, config: ControllerConfig, publish_callback: Callable[[float, float], None]):
        self.config = config
        self.publish_callback = publish_callback
        self.linear = 0.0
        self.angular = 0.0
        self.lock = threading.Lock()
    
    def process_joystick(self, turn_value: float, drive_value: float) -> None:
        """Process joystick values and update motor commands."""
        with self.lock:
            if self.config.invert_turn:
                turn_value = -turn_value
            if self.config.invert_drive:
                drive_value = -drive_value
            
            self.angular = turn_value * self.config.scale_angular
            self.linear = drive_value * self.config.scale_linear
            
            try:
                self.publish_callback(self.linear, self.angular)
            except Exception as e:
                logger.error(f"Error publishing motor commands: {e}")
    
    def process_xy_input(self, x: float, y: float) -> None:
        """Process x,y input values from non-joystick source."""
        with self.lock:
            linear = y * self.config.scale_linear
            angular = -x * self.config.scale_angular
            
            try:
                self.publish_callback(linear, angular)
            except Exception as e:
                logger.error(f"Error publishing motor commands: {e}")
    
    def stop(self) -> None:
        """Stop the robot by sending zero commands."""
        try:
            self.publish_callback(0.0, 0.0)
            logger.info("Emergency stop: Motors stopped")
        except Exception as e:
            logger.error(f"Failed to stop motors: {e}")


class RobotROSNode(Node):
    """ROS2 node for robot control with video streaming."""
    
    def __init__(self):
        super().__init__('robot_web_server_node')
        
        # Motor and knife publishers
        self.publisher = self.create_publisher(
            Twist, 
            '/diff_drive_controller/cmd_vel_unstamped', 
            10
        )
        self.knife_publisher = self.create_publisher(
            Float32, 
            'knives/set_rpm', 
            10
        )
        
        # Create client for PID control service
        self.pid_client = self.create_client(SetBool, 'knives/enable_pid')
        
        # Initialize CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Image storage with thread safety
        self.latest_depth_image = None
        self.latest_confidence_image = None
        self.image_lock = threading.Lock()
        
        # Subscribe to camera image topics
        self.depth_image_sub = self.create_subscription(
            Image,
            'depth_image',
            self.depth_image_callback,
            10
        )
        
        self.confidence_image_sub = self.create_subscription(
            Image,
            'confidence_image',
            self.confidence_image_callback,
            10
        )
        
        if not self.pid_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning('PID control service not available after waiting!')
        else:
            self.get_logger().info('PID control service is available')
            
        self.create_timer(10.0, self._gc_callback)
        
        self.get_logger().info('Robot ROS node with video streaming initialized')
    
    def depth_image_callback(self, msg: Image) -> None:
        """Callback for depth image topic."""
        try:
            with self.image_lock:
                self.latest_depth_image = msg
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")
    
    def confidence_image_callback(self, msg: Image) -> None:
        """Callback for confidence image topic."""
        try:
            with self.image_lock:
                self.latest_confidence_image = msg
        except Exception as e:
            self.get_logger().error(f"Error processing confidence image: {e}")
    
    def get_latest_depth_image(self) -> bytes:
        """Get latest depth image as JPEG bytes."""
        with self.image_lock:
            if self.latest_depth_image is None:
                return self._create_no_signal_image()
            
            try:
                cv_image = self.bridge.imgmsg_to_cv2(self.latest_depth_image, "bgr8")
                _, jpeg_data = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 85])
                return jpeg_data.tobytes()
            except Exception as e:
                self.get_logger().error(f"Error converting depth image: {e}")
                return self._create_no_signal_image()
    
    def get_latest_confidence_image(self) -> bytes:
        """Get latest confidence image as JPEG bytes."""
        with self.image_lock:
            if self.latest_confidence_image is None:
                return self._create_no_signal_image()
            
            try:
                cv_image = self.bridge.imgmsg_to_cv2(self.latest_confidence_image, "mono8")
                _, jpeg_data = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 85])
                return jpeg_data.tobytes()
            except Exception as e:
                self.get_logger().error(f"Error converting confidence image: {e}")
                return self._create_no_signal_image()
    
    def _create_no_signal_image(self) -> bytes:
        """Create a 'No Signal' placeholder image."""
        img = cv2.imread('no_signal.jpg') if os.path.exists('no_signal.jpg') else None
        if img is None:
            # Create a simple black image with text
            img = cv2.zeros((240, 320, 3), dtype=cv2.uint8)
            cv2.putText(img, 'No Signal', (80, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        _, jpeg_data = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return jpeg_data.tobytes()
    
    def _gc_callback(self) -> None:
        """Perform garbage collection."""
        gc.collect()
        self.get_logger().debug('Memory cleanup performed')
    
    def publish_motor_commands(self, linear_x: float, angular_z: float) -> None:
        """Publish motor commands to ROS topic."""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.publisher.publish(msg)
    
    def publish_knife_rpm(self, rpm: float) -> None:
        """Publish knife RPM to ROS topic."""
        msg = Float32()
        msg.data = float(rpm)
        self.knife_publisher.publish(msg)
    
    def toggle_pid_control(self, enable: bool) -> None:
        """Toggle PID control via service call."""
        if not self.pid_client.service_is_ready():
            self.get_logger().warning('PID service is not ready')
            return
            
        request = SetBool.Request()
        request.data = enable
        
        future = self.pid_client.call_async(request)
        future.add_done_callback(
            lambda f: self._handle_pid_toggle_callback(f, enable)
        )
    
    def _handle_pid_toggle_callback(self, future, enable_requested: bool) -> None:
        """Callback for when the PID toggle future completes."""
        try:
            response = future.result()
            if response is None:
                self.get_logger().error('PID toggle service returned None')
                return
                
            self.get_logger().info(
                f'PID toggle response: success={response.success}, ' +
                f'message="{response.message}"'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to call PID toggle service: {e}')


class RobotWebServer:
    """Main class that integrates ROS2, Flask, and robot control with video streaming."""
    
    def __init__(self):
        self.config_path = os.path.join(os.path.dirname(__file__), 'controller_config.json')
        self.config = ControllerConfig.load_from_file(self.config_path)
        
        # Initialize ROS node
        self.node = self._create_ros_node()
        
        # Initialize controllers
        self.knife_controller = KnifeController(
            self.config, 
            self.node.publish_knife_rpm
        )
        
        self.pid_controller = PIDController(
            self.node.toggle_pid_control
        )
        
        self.motor_controller = MotorController(
            self.config,
            self.node.publish_motor_commands
        )
        
        # Initialize Flask app
        self.app = Flask(__name__, template_folder="./templates")
        
        # Disable Flask logging if environment variable is set
        if os.environ.get('ENABLE_FLASK_LOGS', '').lower() not in ('true', '1', 'yes'):
            self.app.logger.removeHandler(default_handler)
            import logging
            log = logging.getLogger('werkzeug')
            log.setLevel(logging.ERROR)
        
        self._setup_routes()
        
        # Thread handles
        self.knife_thread = None
        self.flask_thread = None
    
    def _create_ros_node(self) -> RobotROSNode:
        """Create and initialize the ROS node."""
        return RobotROSNode()
    
    def _setup_routes(self) -> None:
        """Set up Flask routes."""
        
        @self.app.route('/')
        def index():
            return render_template('index.html')
            
        @self.app.route('/config')
        def config_page():
            return render_template('config.html')
        
        # === VIDEO STREAMING ROUTES ===
        @self.app.route('/video_feed/depth')
        def video_feed_depth():
            """Video streaming route for depth camera."""
            def generate():
                while True:
                    try:
                        frame = self.node.get_latest_depth_image()
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                        time.sleep(1/30)  # ~30 FPS
                    except Exception as e:
                        logger.error(f"Error in depth video feed: {e}")
                        time.sleep(0.1)
            
            return Response(generate(),
                          mimetype='multipart/x-mixed-replace; boundary=frame')
        
        @self.app.route('/video_feed/confidence')
        def video_feed_confidence():
            """Video streaming route for confidence camera."""
            def generate():
                while True:
                    try:
                        frame = self.node.get_latest_confidence_image()
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                        time.sleep(1/30)  # ~30 FPS
                    except Exception as e:
                        logger.error(f"Error in confidence video feed: {e}")
                        time.sleep(0.1)
            
            return Response(generate(),
                          mimetype='multipart/x-mixed-replace; boundary=frame')
        
        # === EXISTING ROUTES ===
        @self.app.route('/gamepad', methods=['POST'])
        def gamepad():
            try:
                data = request.get_json(force=True, silent=True)
                if not data or not isinstance(data, dict):
                    return jsonify(success=False, message="Invalid data format")
                
                axes = data.get('axes', [])
                buttons = data.get('buttons', [])
                
                if len(axes) <= max(self.config.turn_axis, self.config.drive_axis):
                    return jsonify(success=False, message="Not enough axis data provided")
                    
                if len(buttons) <= max(self.config.knife_button, self.config.pid_toggle_button):
                    return jsonify(success=False, message="Not enough button data provided")
                
                turn_value = axes[self.config.turn_axis]
                drive_value = axes[self.config.drive_axis]
                self.motor_controller.process_joystick(turn_value, drive_value)
                
                knife_value = buttons[self.config.knife_button]
                self.knife_controller.process_button_value(knife_value)
                
                pid_button = self.config.pid_toggle_button
                if buttons[pid_button] == 1:
                    self.pid_controller.toggle()
                    
                return jsonify(success=True)
            except Exception as e:
                logger.error(f"Error processing gamepad data: {e}", exc_info=True)
                return jsonify(success=False, error=str(e))
        
        @self.app.route('/set_motors', methods=['POST'])
        def set_motors():
            try:
                data = request.json
                x = float(data['x'])
                y = float(data['y'])
                
                self.motor_controller.process_xy_input(x, y)
                return jsonify(success=True)
            except Exception as e:
                logger.error(f"Error setting motors: {e}", exc_info=True)
                return jsonify(success=False, error=str(e))
        
        @self.app.route('/api/config', methods=['GET'])
        def get_config():
            return jsonify(success=True, config=self.config.to_dict())
        
        @self.app.route('/api/config', methods=['POST'])
        def update_config():
            try:
                new_config_dict = request.get_json(force=True, silent=True)
                if not new_config_dict:
                    return jsonify(success=False, message="Invalid data format")
                    
                merged_dict = {**self.config.to_dict(), **new_config_dict}
                updated_config = ControllerConfig.from_dict(merged_dict)
                
                valid, error_msg = updated_config.validate()
                if not valid:
                    return jsonify(success=False, message=f"Invalid configuration: {error_msg}")
                
                self.config = updated_config
                self.motor_controller.config = updated_config
                self.knife_controller.config = updated_config
                
                self.config.save_to_file(self.config_path)
                
                return jsonify(
                    success=True, 
                    message="Configuration updated", 
                    config=self.config.to_dict()
                )
            except Exception as e:
                logger.error(f"Error updating config: {e}", exc_info=True)
                return jsonify(success=False, error=str(e))
        
        @self.app.route('/emergency_stop', methods=['POST'])
        def emergency_stop():
            """Emergency stop endpoint to immediately stop the robot."""
            try:
                self.motor_controller.stop()
                return jsonify(success=True, message="Emergency stop activated")
            except Exception as e:
                logger.error(f"Error during emergency stop: {e}")
                return jsonify(success=False, error=str(e))
    
    def start(self) -> None:
        """Start the web server and ROS node."""
        self.knife_thread = threading.Thread(
            target=self.knife_controller.send_commands_loop,
            daemon=True
        )
        self.knife_thread.start()
        
        self.flask_thread = threading.Thread(
            target=lambda: self.app.run(host='0.0.0.0', port=80, threaded=True, debug=False),
            daemon=True
        )
        self.flask_thread.start()
        
        logger.info("Robot web server with video streaming started")
        
        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            logger.info("Keyboard interrupt detected, shutting down")
        finally:
            self.shutdown()
    
    def shutdown(self) -> None:
        """Clean up resources and shutdown."""
        logger.info("Shutting down robot web server...")
        
        self.motor_controller.stop()
        
        if self.knife_controller:
            self.knife_controller.stop()
        
        if self.node:
            self.node.destroy_node()
            
        logger.info("Robot web server shutdown complete")


def main():
    """Main entry point for the robot web server."""
    try:
        rclpy.init()
        server = RobotWebServer()
        server.start()
        return 0
    except Exception as e:
        logger.critical(f"Fatal error: {e}", exc_info=True)
        if rclpy.ok():
            rclpy.shutdown()
        return 1


if __name__ == '__main__':
    exit(main())