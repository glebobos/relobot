import time
import board
import pwmio
import usb_cdc
import countio
import digitalio

# Constants
MAX_SPEED = 1.0
MIN_SPEED = -1.0
FREQUENCY = 10000
COUNTS_PER_REVOLUTION = 1
RPM_UPDATE_INTERVAL = 0.1  # Update RPM every 100ms

# Pin configurations
try:
    FWD_PWM = pwmio.PWMOut(board.D0, frequency=FREQUENCY)  # Forward PWM
    FWD_EN = digitalio.DigitalInOut(board.D1)      # Forward Enable
    FWD_EN.direction = digitalio.Direction.OUTPUT

    REV_EN = digitalio.DigitalInOut(board.D2)   # Reverse Enable
    REV_EN.direction = digitalio.Direction.OUTPUT

    REV_PWM = pwmio.PWMOut(board.D3, frequency=FREQUENCY)  # Reverse PWM
    encoder = countio.Counter(board.D10)                    # Encoder input
except Exception as e:
    print(f"Error initializing hardware: {e}")
    raise

class MotorState:
    def __init__(self):
        self.prev_count = 0
        self.prev_time = time.monotonic()
        self.current_rpm = 0

motor_state = MotorState()

def calculate_rpm():
    """Calculate current RPM"""
    current_time = time.monotonic()
    current_count = encoder.count
    
    # Calculate time difference
    time_diff = current_time - motor_state.prev_time
    
    if time_diff >= RPM_UPDATE_INTERVAL:
        # Calculate count difference
        count_diff = current_count - motor_state.prev_count
        
        # Calculate RPM
        rpm = (count_diff / COUNTS_PER_REVOLUTION) * (60 / time_diff)
        
        # Update previous values
        motor_state.prev_count = current_count
        motor_state.prev_time = current_time
        motor_state.current_rpm = rpm
        
        return rpm
    
    return motor_state.current_rpm

def set_motor_speed(speed):
    """Set motor speed with direction control"""
    speed = max(MIN_SPEED, min(MAX_SPEED, speed))
    duty_cycle = int(65535 * abs(speed))    
    try:
        if speed > 0:
            FWD_EN.value = True  # Full on
            REV_EN.value = True      # Off
            FWD_PWM.duty_cycle = duty_cycle
            REV_PWM.duty_cycle = 0
        elif speed < 0:
            FWD_EN.value = True      # Off
            REV_EN.value = True  # Full on
            FWD_PWM.duty_cycle = 0
            REV_PWM.duty_cycle = duty_cycle
        else:
            FWD_EN.value = False      # Off
            REV_EN.value = False # Full on
            FWD_PWM.duty_cycle = 0
            REV_PWM.duty_cycle = 0
    except Exception as e:
        print(f"Error setting motor speed: {e}")

def test_motor():
    """Test function to gradually increase and decrease speed"""
    print("Starting motor test...")
    
    # Accelerate forward
    for speed in range(0, 101, 2):
        set_motor_speed(speed / 100.0)
        rpm = calculate_rpm()
        print(f"Speed: {speed}%, RPM: {rpm:.1f}")
        time.sleep(0.1)
    
    # Decelerate to zero
    for speed in range(100, -1, -2):
        set_motor_speed(speed / 100.0)
        rpm = calculate_rpm()
        print(f"Speed: {speed}%, RPM: {rpm:.1f}")
        time.sleep(0.1)
    
    # Accelerate reverse
    for speed in range(0, -101, -2):
        set_motor_speed(speed / 100.0)
        rpm = calculate_rpm()
        print(f"Speed: {speed}%, RPM: {rpm:.1f}")
        time.sleep(0.1)
    
    # Decelerate to zero
    for speed in range(-100, 1, 2):
        set_motor_speed(speed / 100.0)
        rpm = calculate_rpm()
        print(f"Speed: {speed}%, RPM: {rpm:.1f}")
        time.sleep(0.1)
    
    print("Motor test completed")

def process_command(command):
    """Process incoming command with error handling for a single motor"""
    try:
        command = command.strip()

        # Check for identification command
        if command.lower() == "whoyouare":
            usb_cdc.data.write(b"knives\n")
            return True

        # Check for test command
        if command.lower() == "test":
            test_motor()
            return True

        # Process speed command
        speed = float(command)
        set_motor_speed(speed)
        return True
    except Exception as e:
        print(f"Error processing command: {e}")
        set_motor_speed(0)
        return False

def main():
    while True:
        try:
            # Process incoming commands
            if usb_cdc.data.in_waiting > 0:
                command = usb_cdc.data.readline().decode().strip()
                process_command(command)
            
            # Calculate and send RPM data
            rpm = calculate_rpm()
            usb_cdc.data.write(f"RPM: {rpm:.1f}\n".encode())
            time.sleep(0.1)

        except Exception as e:
            print(f"Main loop error: {e}")
            set_motor_speed(0)
            time.sleep(0.1)

if __name__ == "__main__":
    main()