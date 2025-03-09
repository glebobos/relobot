#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from flask import Flask, render_template, request, jsonify
import threading
import os
import gc  # Add garbage collection

# Initialize the Flask app with the correct template folder
app = Flask(__name__, template_folder="./templates")
node = None

class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server_node')
        self.publisher = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        # Add a timer to periodically force garbage collection
        self.create_timer(10.0, self.gc_callback)
        
    def gc_callback(self):
        # Force garbage collection periodically
        gc.collect()
        self.get_logger().debug('Memory cleanup performed')

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
    try:
        data = request.json
        x = float(data['x'])
        y = float(data['y'])
        
        # Calculate linear and angular velocities based on joystick position
        linear_velocity = y
        angular_velocity = -x
        
        # Publish to ROS2 topic
        if node and rclpy.ok():
            node.publish_motor_commands(linear_velocity, angular_velocity)
        
        return jsonify(success=True)
    except Exception as e:
        return jsonify(success=False, error=str(e))

def main(args=None):
    global node
    
    rclpy.init(args=args)
    node = WebServerNode()
    
    # Run Flask with minimal resources
    flask_thread = threading.Thread(
        target=lambda: app.run(
            host='0.0.0.0', 
            port=5000,
            threaded=False,  # Disable Flask threading
            debug=False      # Disable debug mode
        )
    )
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