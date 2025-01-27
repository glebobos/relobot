import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from flask import Flask, render_template, request, jsonify
import threading
import os

app = Flask(__name__, 
           template_folder='/ros2_ws/templates')  # Update template folder path
node = None
publisher = None

class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server_node')
        self.publisher = self.create_publisher(Float32MultiArray, 'motor_commands', 10)

    def publish_motor_commands(self, left, right):
        msg = Float32MultiArray()
        msg.data = [float(left), float(right)]
        self.publisher.publish(msg)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/set_motors', methods=['POST'])
def set_motors():
    data = request.json
    x = float(data['x'])
    y = float(data['y'])
    
    # Calculate motor speeds based on joystick position
    left_motor = y + x
    right_motor = y - x
    
    # Clamp values to [-1, 1] range
    left_motor = max(-1, min(1, left_motor))
    right_motor = max(-1, min(1, right_motor))
    
    # Publish to ROS2 topic
    node.publish_motor_commands(left_motor, right_motor)
    
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