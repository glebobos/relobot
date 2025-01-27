import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        
        # Motor control parameters
        self.ACCELERATION = 0.1
        self.UPDATE_RATE = 0.02
        self.current_left = 0.0
        self.current_right = 0.0
        self.target_left = 0.0
        self.target_right = 0.0
        
        # Serial connection
        self.SERIAL_PORT = '/dev/ttyACM1'
        self.BAUD_RATE = 115200
        self.ser = serial.Serial(self.SERIAL_PORT, self.BAUD_RATE, timeout=1)
        
        # Create subscription
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'motor_commands',
            self.motor_callback,
            10)
        
        # Create timer for smooth control
        self.timer = self.create_timer(self.UPDATE_RATE, self.smooth_control_callback)

    def motor_callback(self, msg):
        self.target_left = msg.data[0]
        self.target_right = msg.data[1]

    def smooth_control_callback(self):
        # Calculate difference between current and target speeds
        diff_left = self.target_left - self.current_left
        diff_right = self.target_right - self.current_right
        
        # Apply acceleration/deceleration
        if abs(diff_left) > self.ACCELERATION:
            self.current_left += self.ACCELERATION if diff_left > 0 else -self.ACCELERATION
        else:
            self.current_left = self.target_left
            
        if abs(diff_right) > self.ACCELERATION:
            self.current_right += self.ACCELERATION if diff_right > 0 else -self.ACCELERATION
        else:
            self.current_right = self.target_right
        
        # Send command to motors
        command = f"{self.current_left},{self.current_right}"
        self.ser.write((command + '\n').encode())

    def cleanup(self):
        self.ser.write(b"0,0\n")
        self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()