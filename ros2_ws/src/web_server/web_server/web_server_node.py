#!/usr/bin/env python3
import threading
import gc

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from flask import Flask, render_template, request, jsonify

# Ваши модули из примера выше
from web_server.controller import Controller

# Flask
app = Flask(__name__, template_folder="./templates")
node = None

# текущие значения осей
_linear = 0.0
_angular = 0.0

def on_axis(axis_id: int, value: float):
    """Колбэк для осей 0 и 3: обновляем _linear/_angular и публикуем Twist."""
    global _linear, _angular, node
    if axis_id == 3:
        _linear = -value
    elif axis_id == 0:
        _angular = -value
    if node and rclpy.ok():
        node.publish_motor_commands(_linear, _angular)

class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server_node')
        self.publisher = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.create_timer(10.0, self.gc_callback)

    def gc_callback(self):
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
        linear_velocity  = y
        angular_velocity = -x
        if node and rclpy.ok():
            node.publish_motor_commands(linear_velocity, angular_velocity)
        return jsonify(success=True)
    except Exception as e:
        return jsonify(success=False, error=str(e))

def main(args=None):
    global node
    rclpy.init(args=args)
    node = WebServerNode()

    flask_thread = threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=80, threaded=False, debug=False),
        daemon=True
    )
    flask_thread.start()

    ctrl = Controller()
    ctrl.register_axis(0, on_axis)
    ctrl.register_axis(3, on_axis)
    ctrl_thread = threading.Thread(target=ctrl.run, daemon=True)
    ctrl_thread.start()

    # Старт rclpy.spin
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
