#!/usr/bin/env python3
import time
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data

class WheelCalibrationNode(Node):
    def __init__(self):
        super().__init__('wheel_calibration_node')
        self.pub = self.create_publisher(
            JointState, 
            '/robot_joint_commands', 
            10
        )
        self.sub = self.create_subscription(
            JointState, 
            '/robot_joint_states', 
            self.states_callback, 
            qos_profile_sensor_data
        )
        
        self.latest_left_vel = 0.0
        self.latest_right_vel = 0.0
        self.samples = []
        self.collecting = False
        self.get_logger().info("Wheel Calibration Node initialized.")

    def states_callback(self, msg):
        left_vel = None
        right_vel = None
        for i, name in enumerate(msg.name):
            if name == 'left_wheel_joint':
                left_vel = msg.velocity[i]
            elif name == 'right_wheel_joint':
                right_vel = msg.velocity[i]
        
        # Position-based fallback if names are not present
        if left_vel is None or right_vel is None:
            if len(msg.velocity) >= 2:
                left_vel = msg.velocity[0]
                right_vel = msg.velocity[1]
        
        if left_vel is not None and right_vel is not None:
            self.latest_left_vel = left_vel
            self.latest_right_vel = right_vel
            if self.collecting:
                self.samples.append((left_vel, right_vel))

    def publish_pwm(self, motor, pwm):
        msg = JointState()
        msg.name = ['left_wheel_joint', 'right_wheel_joint']
        if motor == 'left':
            msg.velocity = [float(pwm), 0.0]
        else:
            msg.velocity = [0.0, float(pwm)]
        self.pub.publish(msg)

    def run_calibration(self):
        pwm_sweep = [0.08, 0.12, 0.16, 0.20, 0.24, 0.28, 0.32]
        
        results = {
            'left': {'fwd': [], 'rev': []},
            'right': {'fwd': [], 'rev': []}
        }
        
        sweeps = [
            ('left', 'fwd', pwm_sweep),
            ('left', 'rev', [-x for x in pwm_sweep]),
            ('right', 'fwd', pwm_sweep),
            ('right', 'rev', [-x for x in pwm_sweep])
        ]
        
        self.get_logger().info("Starting Wheel Calibration Sweeps...")
        
        for motor, direction, pwms in sweeps:
            self.get_logger().info(f"\n=== Sweeping {motor.upper()} motor {direction.upper()} ===")
            for pwm in pwms:
                self.get_logger().info(f"Commanding PWM: {pwm:.2f}")
                
                # Phase 1: Stabilize (1.5s)
                self.samples = []
                self.collecting = False
                start_time = time.time()
                while time.time() - start_time < 1.5:
                    self.publish_pwm(motor, pwm)
                    rclpy.spin_once(self, timeout_sec=0.05)
                    time.sleep(0.05)
                
                # Phase 2: Collect samples (1.0s)
                self.samples = []
                self.collecting = True
                start_time = time.time()
                while time.time() - start_time < 1.0:
                    self.publish_pwm(motor, pwm)
                    rclpy.spin_once(self, timeout_sec=0.05)
                    time.sleep(0.05)
                
                self.collecting = False
                
                if not self.samples:
                    self.get_logger().warn("  Warning: No samples received during this step!")
                    continue
                
                # Retrieve the commanded motor's measured velocity
                idx = 0 if motor == 'left' else 1
                vels = [s[idx] for s in self.samples]
                avg_vel = sum(vels) / len(vels)
                
                self.get_logger().info(f"  Avg velocity: {avg_vel:.4f} rad/s (samples count: {len(vels)})")
                
                # We store absolute velocity and absolute commanded PWM for feedforward fitting
                results[motor][direction].append((abs(avg_vel), abs(pwm)))
                
                # Phase 3: Coast to stop (0.5s)
                start_time = time.time()
                while time.time() - start_time < 0.5:
                    self.publish_pwm(motor, 0.0)
                    rclpy.spin_once(self, timeout_sec=0.05)
                    time.sleep(0.05)
        
        # Shutdown motors at the end
        for _ in range(10):
            self.publish_pwm('left', 0.0)
            self.publish_pwm('right', 0.0)
            time.sleep(0.05)
            
        self.analyze_results(results)

    def analyze_results(self, results):
        self.get_logger().info("\n==========================================")
        self.get_logger().info("CALIBRATION ANALYSIS COMPLETE")
        self.get_logger().info("==========================================\n")
        
        fits = {}
        for motor in ['left', 'right']:
            fits[motor] = {}
            for direction in ['fwd', 'rev']:
                data = results[motor][direction]
                
                # Exclude any points where motor didn't spin (threshold: 0.05 rad/s)
                filtered_data = [pt for pt in data if pt[0] > 0.05]
                
                k, c = self.linear_regression(filtered_data)
                fits[motor][direction] = (k, c)
                
                self.get_logger().info(f"{motor.upper()} {direction.upper()} (fitted on {len(filtered_data)}/{len(data)} points):")
                for omega, pwm in filtered_data:
                    predicted_pwm = k * omega + c
                    err = pwm - predicted_pwm
                    self.get_logger().info(f"  omega={omega:6.3f} rad/s, pwm={pwm:5.3f}, fit={predicted_pwm:5.3f}, error={err:6.3f}")
                self.get_logger().info(f"  Result: K={k:.6f}, C={c:.6f}\n")
        
        print("\n// Copy-paste the following definitions into your main.cpp:")
        print(f"#define FF_K_LEFT_FWD               {fits['left']['fwd'][0]:.6f}f")
        print(f"#define FF_C_LEFT_FWD               {fits['left']['fwd'][1]:.6f}f")
        print(f"#define FF_K_LEFT_REV               {fits['left']['rev'][0]:.6f}f")
        print(f"#define FF_C_LEFT_REV               {fits['left']['rev'][1]:.6f}f")
        print(f"#define FF_K_RIGHT_FWD              {fits['right']['fwd'][0]:.6f}f")
        print(f"#define FF_C_RIGHT_FWD              {fits['right']['fwd'][1]:.6f}f")
        print(f"#define FF_K_RIGHT_REV              {fits['right']['rev'][0]:.6f}f")
        print(f"#define FF_C_RIGHT_REV              {fits['right']['rev'][1]:.6f}f\n")

    def linear_regression(self, data):
        # Fits: y = K * x + C
        # data: list of (x, y) where x = abs(omega), y = abs(pwm)
        n = len(data)
        if n < 2:
            return 0.0, 0.0
            
        sum_x = sum(d[0] for d in data)
        sum_y = sum(d[1] for d in data)
        sum_xx = sum(d[0]**2 for d in data)
        sum_xy = sum(d[0] * d[1] for d in data)
        
        denom = n * sum_xx - sum_x**2
        if abs(denom) < 1e-9:
            return 0.0, sum_y / n
            
        k = (n * sum_xy - sum_x * sum_y) / denom
        c = (sum_y - k * sum_x) / n
        return k, c

def main(args=None):
    rclpy.init(args=args)
    node = WheelCalibrationNode()
    try:
        # Spin once to ensure subscriptions are initialized and connections established
        rclpy.spin_once(node, timeout_sec=1.0)
        node.run_calibration()
    except KeyboardInterrupt:
        node.get_logger().info("Calibration interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
