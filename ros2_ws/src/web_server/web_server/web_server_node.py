#!/usr/bin/env python3
import threading
import gc

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
_knife_speed_locked = False
_knife_speed_rpm = 0.0
_knife_min_rpm = 500
_knife_max_rpm = 3000

def on_axis(axis_id: int, value: float):
    """Колбэк для осей 0 и 3: обновляем _linear/_angular и публикуем Twist."""
    global _linear, _angular, node
    if axis_id == 3:
        _linear = -value * SCALE_LINEAR
    elif axis_id == 0:
        _angular = -value * SCALE_ANGULAR
    if node and rclpy.ok():
        node.publish_motor_commands(_linear, _angular)

def on_axis_knife(axis_id: int, value: float):
    """Handle axis 4 for knife motor speed control."""
    global _knife_speed_locked, _knife_speed_rpm, node
    
    # Only process if this is axis 4 and not locked
    if axis_id != 4 or _knife_speed_locked:
        return

    # Map axis value from [-1,1] to [min_rpm, max_rpm]
    # Normalize to 0-1 range first
    normalized_value = (value + 1) / 2
    rpm = _knife_min_rpm + normalized_value * (_knife_max_rpm - _knife_min_rpm)
    
    # Update the global variable
    _knife_speed_rpm = rpm

    # Send to knife motor controller if node is available
    if node and rclpy.ok():
        node.publish_knife_rpm(rpm)
        print(f"Setting knife RPM to {rpm:.0f}")

def on_button_knife(button: int):
    """Handle button 7 for knife motor speed locking."""
    global _knife_speed_locked, _knife_speed_rpm
    
    # Toggle lock state when button 7 is pressed
    if button == 7:
        _knife_speed_locked = not _knife_speed_locked
        print(f"Knife speed {'locked' if _knife_speed_locked else 'unlocked'} at {_knife_speed_rpm:.0f} RPM")

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
    global node
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
    ctrl.register_axis(0, on_axis)
    ctrl.register_axis(3, on_axis)
    # Register knife control callbacks
    ctrl.register_axis(4, on_axis_knife)
    ctrl.register_button_down(7, on_button_knife)
    ctrl_thread = threading.Thread(target=ctrl.run, daemon=True)
    ctrl_thread.start()

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