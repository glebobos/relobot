import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time

from mower_knife_controller.knife_motor_controller import SerialKnifeMotorController


class KnifeControllerNode(Node):
    """ROS 2 node for controlling a mower knife motor via serial interface."""
    
    def __init__(self):
        super().__init__('knife_controller')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM4')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('current_rpm_topic', 'mower/current_rpm')
        self.declare_parameter('set_rpm_topic', 'mower/set_rpm')
        self.declare_parameter('update_rate', 0.1)  # seconds
        
        # Get parameters
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        current_rpm_topic = self.get_parameter('current_rpm_topic').value
        set_rpm_topic = self.get_parameter('set_rpm_topic').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Initialize motor controller
        self.get_logger().info(f"Connecting to knife motor controller on {serial_port}")
        self.motor_controller = SerialKnifeMotorController(port_name=serial_port, baud_rate=baud_rate)
        
        # Verify correct device
        if not self.motor_controller.identify_device():
            self.get_logger().error("Could not identify the knife motor controller. Is it connected?")
        else:
            self.get_logger().info("Knife motor controller identified successfully")
        
        # Create publisher for current RPM
        self.rpm_publisher = self.create_publisher(Float32, current_rpm_topic, 10)
        
        # Create subscription for RPM commands
        self.rpm_subscription = self.create_subscription(
            Float32,
            set_rpm_topic,
            self.set_rpm_callback,
            10)
            
        # Create timer for regular updates
        self.timer = self.create_timer(self.update_rate, self.timer_callback)
        
        self.get_logger().info('Mower knife controller node initialized')
    
    def set_rpm_callback(self, msg):
        """Handle RPM command messages."""
        target_rpm = msg.data
        self.get_logger().info(f"Setting knife motor RPM to {target_rpm}")
        self.motor_controller.set_rpm(target_rpm)
    
    def timer_callback(self):
        """Regularly publish current RPM."""
        rpm = self.motor_controller.read_rpm()
        if rpm is not None:
            msg = Float32()
            msg.data = rpm
            self.rpm_publisher.publish(msg)
            self.get_logger().debug(f"Published current RPM: {rpm}")
    
    def on_shutdown(self):
        """Stop the motor and close the connection when shutting down."""
        self.get_logger().info("Shutting down, stopping knife motor...")
        try:
            self.motor_controller.set_rpm(0)
            time.sleep(0.5)  # Give time for the command to be processed
            self.motor_controller.close()
        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = KnifeControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()