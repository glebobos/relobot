import time
import math
import board
import pwmio
import countio
import digitalio
import usb_cdc

# Constants for motor control
FREQUENCY = 10000  # PWM frequency in Hz
COUNTS_PER_REVOLUTION = 1
RPM_UPDATE_INTERVAL = 0.6  # seconds
INACTIVITY_TIMEOUT = 30.0  # Stop motor after 3 seconds of inactivity
SPEED_SMOOTHING_FACTOR = 0.2  # Lower value = smoother transitions

# Motor calibration coefficients
K = 0.001381
C = 0.048623

# Initialize PWM outputs for forward and reverse control
FWD_PWM = pwmio.PWMOut(board.D0, frequency=FREQUENCY)
REV_PWM = pwmio.PWMOut(board.D3, frequency=FREQUENCY)

# Initialize enable pins
FWD_EN = digitalio.DigitalInOut(board.D1)
REV_EN = digitalio.DigitalInOut(board.D2)
for pin in (FWD_EN, REV_EN):
    pin.direction = digitalio.Direction.OUTPUT

# Initialize encoder
encoder = countio.Counter(board.D5, pull=digitalio.Pull.UP)


class MotorController:
    """Handles motor control with smooth speed transitions and PID for fine control."""
    
    def __init__(self):
        self.prev_count = 0
        self.prev_time = time.monotonic()
        self.current_rpm = 0.0
        self.target_rpm = 0.0
        self.measured_rpm = 0.0
        self.last_command_time = time.monotonic()
        
        # PID controller parameters
        self.kp = 0.00005  # Proportional gain
        self.ki = 0.00001  # Integral gain
        self.kd = 0.00001  # Derivative gain
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_pid_time = time.monotonic()
        self.pid_active = False
        self.pwm_adjustment = 0.0
    
    def set_pwm(self, norm_pwm, forward=True):
        """Set normalized PWM [0,1] for specified direction."""
        try:
            duty = int(65535 * max(0.0, min(1.0, norm_pwm)))
            
            if duty == 0:
                FWD_EN.value = False
                REV_EN.value = False
                FWD_PWM.duty_cycle = 0
                REV_PWM.duty_cycle = 0
                return
            
            FWD_EN.value = True
            REV_EN.value = True
            
            if forward:
                FWD_PWM.duty_cycle = duty
                REV_PWM.duty_cycle = 0
            else:
                FWD_PWM.duty_cycle = 0
                REV_PWM.duty_cycle = duty
        except Exception as e:
            print(f"Error setting PWM: {e}")
            # Safe shutdown on error
            try:
                FWD_EN.value = False
                REV_EN.value = False
                FWD_PWM.duty_cycle = 0
                REV_PWM.duty_cycle = 0
            except:
                pass
    
    def rpm_to_pwm(self, rpm):
        """Convert RPM to PWM value using linear model."""
        forward = rpm >= 0
        angular_velocity = abs(rpm) * 2 * math.pi / 60.0
        pwm_value = K * angular_velocity + C
        pwm_value = max(0.0, min(1.0, pwm_value))
        return pwm_value, forward
    
    def set_rpm(self, target_rpm):
        """Set target RPM and update command timestamp."""
        if self.target_rpm != target_rpm:
            # Reset PID controller when target changes
            self.integral = 0.0
            self.previous_error = 0.0
            self.pid_active = False
            self.pwm_adjustment = 0.0
        self.target_rpm = target_rpm
        self.last_command_time = time.monotonic()
    
    def update(self):
        """Update motor control with smooth transitions and PID near target."""
        now = time.monotonic()
        
        # Check for inactivity timeout
        if now - self.last_command_time > INACTIVITY_TIMEOUT and self.target_rpm != 0:
            print("Inactivity timeout. Stopping motor.")
            self.target_rpm = 0
            self.pid_active = False
        
        # Get measured RPM from encoder
        self.measured_rpm = self.calculate_rpm()
        
        # Handle the case when target is 0
        if self.target_rpm == 0:
            self.current_rpm = 0
            self.pid_active = False
            self.pwm_adjustment = 0.0
            self.set_pwm(0, True)
            return self.measured_rpm
        
        # Disable PID for reverse direction
        if self.target_rpm < 0:
            self.pid_active = False
        # Check if we should activate PID (when speed reaches 90% of target and not in reverse)
        elif not self.pid_active and self.target_rpm > 0 and abs(self.measured_rpm) >= 0.9 * abs(self.target_rpm):
            self.pid_active = True
            print("PID control activated")
            self.integral = 0.0
            self.previous_error = 0.0
            self.last_pid_time = now
        
        # Use either PID control or standard ramping
        if self.pid_active:
            # Calculate base PWM from feed-forward model
            base_pwm, forward = self.rpm_to_pwm(self.target_rpm)
            
            # Calculate PID adjustment
            self.update_pid()
            
            # Apply PID adjustment to base PWM
            adjusted_pwm = base_pwm + self.pwm_adjustment
            adjusted_pwm = max(0.0, min(1.0, adjusted_pwm))
            
            # Set motor PWM
            self.set_pwm(adjusted_pwm, self.target_rpm >= 0)
            print(f"PID control: RPM={self.measured_rpm:.1f} PWM={adjusted_pwm:.4f} Adj={self.pwm_adjustment:.4f}")
            
            # Update current_rpm tracking
            self.current_rpm = self.target_rpm
        else:
            # Gradually transition to target speed
            if abs(self.current_rpm - self.target_rpm) > 0.1:
                new_rpm = self.current_rpm + SPEED_SMOOTHING_FACTOR * (self.target_rpm - self.current_rpm)
                self.current_rpm = new_rpm
                
                # Apply PWM signal to motor
                pwm_value, forward = self.rpm_to_pwm(new_rpm)
                self.set_pwm(pwm_value, forward)
                print(f"Standard control: RPM={new_rpm:.1f} PWM={pwm_value:.4f}")
        
        return self.measured_rpm
    
    def update_pid(self):
        """Calculate PID adjustment to PWM."""
        now = time.monotonic()
        dt = now - self.last_pid_time
        
        # Skip if dt is too small
        if dt < 0.001:
            return
        
        # Calculate error (target - measured)
        error = self.target_rpm - self.measured_rpm
        
        # Proportional term
        p_out = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = max(-500, min(500, self.integral))  # Limit integral windup
        i_out = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        d_out = self.kd * derivative
        
        # Calculate new adjustment with smoothing
        pid_output = p_out + i_out + d_out
        smooth_factor = 0.2  # Lower = smoother but slower response
        self.pwm_adjustment = self.pwm_adjustment * (1 - smooth_factor) + pid_output * smooth_factor
        
        # Limit adjustment range
        self.pwm_adjustment = max(-0.2, min(0.2, self.pwm_adjustment))
        
        # Update state for next iteration
        self.previous_error = error
        self.last_pid_time = now
    
    def calculate_rpm(self):
        """Calculate RPM from encoder counts."""
        now = time.monotonic()
        count = encoder.count
        dt = now - self.prev_time
        
        if dt >= RPM_UPDATE_INTERVAL:
            count_delta = count - self.prev_count
            measured_rpm = (count_delta / COUNTS_PER_REVOLUTION) * (60.0 / dt)
            
            self.prev_count = count
            self.prev_time = now
            
            return measured_rpm
            
        return self.measured_rpm  # Return the last measured RPM


def process_command(cmd, motor_controller):
    """Process commands received over USB."""
    cmd = cmd.strip().lower()
    
    if cmd == 'whoyouare':
        usb_cdc.data.write(b'knives\n')
    elif cmd == 'test':
        motor_controller.set_rpm(700)
    else:
        try:
            target = float(cmd)
            motor_controller.set_rpm(target)
        except ValueError:
            # Invalid command, stop the motor
            motor_controller.set_rpm(0)


def main():
    """Main program loop."""
    try:
        motor = MotorController()
        motor.set_rpm(0)  # Start with motor stopped
        
        while True:
            try:
                # Process incoming commands
                if usb_cdc.data.in_waiting > 0:
                    line = usb_cdc.data.readline().decode()
                    process_command(line, motor)
                
                # Update motor control with smooth transitions
                rpm = motor.update()
                
                # Report current RPM
                status_msg = f"RPM: {rpm:.1f}\n"
                usb_cdc.data.write(status_msg.encode())
                if rpm != 0:
                    print(status_msg, end="")
                
                time.sleep(0.1)
            except Exception as e:
                print(f"Error in main loop: {e}")
                try:
                    motor.set_rpm(0)
                    motor.update()
                except:
                    pass
                time.sleep(0.5)
    except Exception as e:
        print(f"Critical error: {e}")
        # Emergency shutdown
        try:
            FWD_EN.value = False
            REV_EN.value = False
            FWD_PWM.duty_cycle = 0
            REV_PWM.duty_cycle = 0
        except:
            pass


if __name__ == '__main__':
    main()

