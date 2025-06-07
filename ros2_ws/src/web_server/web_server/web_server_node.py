#!/usr/bin/env python3
import threading
import gc
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
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
_knife_last_send_time = 0.0
_knife_send_interval = 0.2  # Send knife commands every 0.2 seconds

# PID control state
_pid_enabled = True
_last_pid_toggle = 0.0
_pid_toggle_cooldown = 0.5  # Cooldown period to avoid multiple toggles

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
    """Process knife axis value and store RPM."""
    global _knife_speed_rpm
    
    # Calculate RPM based on axis value
    if value <= -0.98:
        rpm = 0  # Set to 0 RPM when under -0.98
    else:
        # Map axis value from [-0.98,1] to [min_rpm, max_rpm]
        normalized_value = (value + 0.98) / 1.98
        rpm = _knife_min_rpm + normalized_value * (_knife_max_rpm - _knife_min_rpm)
    
    # Update the global variable
    _knife_speed_rpm = rpm

def on_axis_knife(axis_id: int, value: float):
    """Handle axis 4 for knife motor speed control."""
    if axis_id == 4:
        process_knife_axis(value)

def on_button_7_pressed(button_id: int):
    """Toggle PID control when button 7 is pressed."""
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
    
    while rclpy.ok():
        current_time = time.time()
        
        # Send command if enough time has passed
        if current_time - _knife_last_send_time >= _knife_send_interval:
            if node and rclpy.ok():
                # Always send the most current value
                node.publish_knife_rpm(_knife_speed_rpm)
                print(f"Setting knife RPM to {_knife_speed_rpm:.0f}")
            _knife_last_send_time = current_time
        
        time.sleep(0.01)  # Check frequently but don't overwhelm the CPU

def poll_knife_axis():
    """Continuously poll the knife axis position and update RPM."""
    global controller_instance
    
    while rclpy.ok():
        if controller_instance:
            # Poll the knife axis value and update the global RPM value
            knife_axis_value = controller_instance.get_axis_value(4)
            process_knife_axis(knife_axis_value)
        time.sleep(0.05)  # Poll at 20Hz

class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server_node')
        self.publisher = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.knife_publisher = self.create_publisher(Float32, 'knives/set_rpm', 10)
        
        # Create client for PID control service
        self.pid_client = self.create_client(SetBool, 'knives/enable_pid')
        while not self.pid_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('PID control service not available, waiting...')
            
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
    
    def toggle_pid_control(self, enable):
        """Toggle PID control via service call."""
        request = SetBool.Request()
        request.data = enable
        
        future = self.pid_client.call_async(request)
        # Use a separate thread to handle the response
        threading.Thread(target=self._handle_pid_toggle_response, args=(future,), daemon=True).start()
    
    def _handle_pid_toggle_response(self, future):
        """Handle the response from the PID toggle service call."""
        try:
            response = future.result()
            self.get_logger().info(f'PID toggle response: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Failed to call PID toggle service: {e}')

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
    # Register button 7 for PID control toggle
    ctrl.register_button_down(7, on_button_7_pressed)
    
    # Start controller thread
    ctrl_thread = threading.Thread(target=ctrl.run, daemon=True)
    ctrl_thread.start()

    # Start polling thread for continuous knife axis updates
    polling_thread = threading.Thread(target=poll_knife_axis, daemon=True)
    polling_thread.start()
    
    # Start thread for sending knife commands
    knife_command_thread = threading.Thread(target=send_knife_commands, daemon=True)
    knife_command_thread.start()

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