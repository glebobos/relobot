#!/usr/bin/env python3

# Rest of your code...
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory
from flask import Flask, render_template, request, jsonify
import threading
import os
package_share_directory = get_package_share_directory('web_server')

# Construct the path to the templates directory
template_path = os.path.join(package_share_directory, 'templates')

# Initialize the Flask app with the correct template folder
app = Flask(__name__, template_folder=template_path) # Update template folder path
node = None

class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server_node')
        self.publisher = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)

    def publish_motor_commands(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.publisher.publish(msg)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/set_motors', methods=['POST'])
def set_motors():
    data = request.json
    x = float(data['x'])
    y = float(data['y'])
    
    # Calculate linear and angular velocities based on joystick position
    linear_velocity = y
    angular_velocity = -x
    
    # Publish to ROS2 topic
    node.publish_motor_commands(linear_velocity, angular_velocity)
    
    return jsonify(success=True)

def main(args=None):
    global node
    
    rclpy.init(args=args)
    node = WebServerNode()
    
    # Run Flask in a separate thread
    flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000))
    flask_thread.daemon = True
    flask_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
