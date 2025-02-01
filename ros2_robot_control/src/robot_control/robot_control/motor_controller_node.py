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
        self.connect_serial()
        
        # Create subscription
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'motor_commands',
            self.motor_callback,
            10)
        
        # Create timer for smooth control
        self.timer = self.create_timer(self.UPDATE_RATE, self.smooth_control_callback)
        
        # Add a flag to track if we're currently processing a command
        self.is_processing = False

    def connect_serial(self):
        try:
            self.ser = serial.Serial(
                port=self.SERIAL_PORT,
                baudrate=self.BAUD_RATE,
                timeout=1,
                write_timeout=1
            )
            time.sleep(2)  # Wait for serial connection to stabilize
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except serial.SerialException as e:
            self.get_logger().error(f'Serial connection failed: {str(e)}')
            raise

    def motor_callback(self, msg):
        try:
            self.target_left = float(msg.data[0])
            self.target_right = float(msg.data[1])
        except (IndexError, ValueError) as e:
            self.get_logger().error(f'Invalid motor command received: {str(e)}')

    def smooth_control_callback(self):
        if self.is_processing:
            return
        
        try:
            self.is_processing = True
            
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
            
            # Format command with fixed precision to reduce string length
            command = f"{self.current_left:.2f},{self.current_right:.2f}\n"
            
            # Check if serial is still connected
            if not self.ser.is_open:
                self.connect_serial()
            
            # Clear buffers before sending new command
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            # Send command to motors
            self.ser.write(command.encode())
            self.ser.flush()  # Wait until all data is written
            
        except serial.SerialException as e:
            self.get_logger().error(f'Serial communication error: {str(e)}')
            try:
                self.ser.close()
                self.connect_serial()
            except serial.SerialException:
                self.get_logger().error('Failed to reconnect to serial port')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {str(e)}')
        finally:
            self.is_processing = False

    def cleanup(self):
        try:
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.write(b"0.00,0.00\n")
                self.ser.flush()
                time.sleep(0.1)  # Give some time for the command to be sent
                self.ser.close()
        except Exception as e:
            self.get_logger().error(f'Error during cleanup: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = MotorControllerNode()
        rclpy.spin(node)
    except Exception as e:
        if node:
            node.get_logger().error(f'Node error: {str(e)}')
    finally:
        if node:
            node.cleanup()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()