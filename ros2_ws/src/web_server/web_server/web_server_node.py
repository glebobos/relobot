#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory
from flask import Flask, render_template, request, jsonify
import threading
import os
import logging

# Configure Flask to minimize logging output
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

# Get the directory of the current script
current_dir = os.path.dirname(os.path.abspath(__file__))
template_dir = os.path.join(current_dir, "templates")

app = Flask(__name__, template_folder=template_dir)
node = None
lock = threading.Lock()  # For thread safety

class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server_node')
        self.publisher = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.get_logger().info('Web server node initialized')

    def publish_motor_commands(self, linear_x, angular_z):
        try:
            msg = Twist()
            msg.linear.x = float(linear_x)
            msg.angular.z = float(angular_z)
            self.publisher.publish(msg)
            self.get_logger().debug(f'Published: linear={linear_x}, angular={angular_z}')
            return True
        except Exception as e:
            self.get_logger().error(f'Error publishing motor commands: {e}')
            return False

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/set_motors', methods=['POST'])
def set_motors():
    global node
    
    try:
        data = request.json
        x = float(data.get('x', 0.0))
        y = float(data.get('y', 0.0))
        
        # Limit values to a reasonable range if needed
        x = max(min(x, 1.0), -1.0)
        y = max(min(y, 1.0), -1.0)
        
        # Calculate linear and angular velocities based on joystick position
        linear_velocity = y
        angular_velocity = x
        
        # Use lock for thread safety when accessing the node
        with lock:
            if node:
                success = node.publish_motor_commands(linear_velocity, angular_velocity)
                if success:
                    return jsonify(success=True)
            
        return jsonify(success=False, error="Failed to publish motor commands")
    except Exception as e:
        return jsonify(success=False, error=str(e))

def run_flask():
    app.run(host='0.0.0.0', port=5000, debug=False)

def main(args=None):
    global node
    
    rclpy.init(args=args)
    node = WebServerNode()
    
    # Run Flask in a separate thread
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.daemon = True
    flask_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    except Exception as e:
        node.get_logger().error(f'Error in ROS2 node: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()