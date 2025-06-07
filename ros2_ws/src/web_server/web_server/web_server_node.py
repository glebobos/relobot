#!/usr/bin/env python3
import threading
import gc
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from flask import Flask, render_template, request, jsonify

# Ваши модули из примера выше
from web_server.controller import Controller

# Flask
app = Flask(__name__, template_folder="./templates")
node = None

# текущие значения осей (от  -1.0 до 1.0)
_linear = 0.0
_angular = 0.0

# Масштабные коэффициенты для джойстика
# Вы можете подстроить максимальные скорости робота здесь
SCALE_LINEAR = 1.0    # максимальная линейная скорость
SCALE_ANGULAR = 2.0   # максимальная угловая скорость

# Knife motor control variables
_knife_speed_rpm = 0.0
_knife_min_rpm = 500
_knife_max_rpm = 3000

# Store the controller instance for polling
controller_instance = None

def on_axis(axis_id: int, value: float):
    """Колбэк для осей 0 и 3: обновляем _linear/_angular и публикуем Twist."""
    global _linear, _angular, node
    if axis_id == 3:
        _linear = -value * SCALE_LINEAR
    elif axis_id == 0:
        _angular = -value * SCALE_ANGULAR
    if node and rclpy.ok():
        node.publish_motor_commands(_linear, _angular)

def process_knife_axis(value: float):
    """Process knife axis value and publish RPM."""
    global _knife_speed_rpm, node
    
    # Calculate RPM based on axis value
    if value <= -0.98:
        rpm = 0  # Set to 0 RPM when under -0.98
    else:
        # Map axis value from [-0.98,1] to [min_rpm, max_rpm]
        normalized_value = (value + 0.98) / 1.98
        rpm = _knife_min_rpm + normalized_value * (_knife_max_rpm - _knife_min_rpm)
    
    # Update the global variable
    _knife_speed_rpm = rpm
        
    # Send to knife motor controller
    if node and rclpy.ok():
        node.publish_knife_rpm(rpm)
        print(f"Setting knife RPM to {rpm:.0f}")

def on_axis_knife(axis_id: int, value: float):
    """Handle axis 4 for knife motor speed control."""
    if axis_id == 4:
        process_knife_axis(value)

def poll_knife_axis():
    """Continuously poll the knife axis position and update RPM."""
    global controller_instance
    
    while rclpy.ok():
        if controller_instance:
            # Assuming the Controller class has a method to get axis value
            # If it doesn't, you'll need to modify the Controller class to expose this
            knife_axis_value = controller_instance.get_axis_value(4)
            process_knife_axis(knife_axis_value)
        time.sleep(0.05)  # Poll at 20Hz

class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server_node')
        self.publisher = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.knife_publisher = self.create_publisher(Float32, 'knives/set_rpm', 10)
        # Запускаем сборщик мусора раз в 10 секунд
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

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/set_motors', methods=['POST'])
def set_motors():
    try:
        data = request.json
        x = float(data['x'])
        y = float(data['y'])
        # Применяем масштабирование
        linear_velocity  = y * SCALE_LINEAR
        angular_velocity = -x * SCALE_ANGULAR
        if node and rclpy.ok():
            node.publish_motor_commands(linear_velocity, angular_velocity)
        return jsonify(success=True)
    except Exception as e:
        return jsonify(success=False, error=str(e))

def main(args=None):
    global node, controller_instance
    rclpy.init(args=args)
    node = WebServerNode()

    # Запускаем Flask в отдельном потоке
    flask_thread = threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=80, threaded=False, debug=False),
        daemon=True
    )
    flask_thread.start()

    # Инициализируем контроллер джойстика
    ctrl = Controller()
    controller_instance = ctrl  # Save for polling
    ctrl.register_axis(0, on_axis)
    ctrl.register_axis(3, on_axis)
    # Register knife control callback for axis
    ctrl.register_axis(4, on_axis_knife)
    
    # Start controller thread
    ctrl_thread = threading.Thread(target=ctrl.run, daemon=True)
    ctrl_thread.start()

    # Start polling thread for continuous knife axis updates
    polling_thread = threading.Thread(target=poll_knife_axis, daemon=True)
    polling_thread.start()

    # Старт rclpy.spin для обработки колбэков ROS
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()