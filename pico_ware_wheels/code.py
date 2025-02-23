import time
import board
import pwmio
import usb_cdc
import countio
import math

# Constants
MAX_SPEED = 1.0
MIN_SPEED = -1.0
FREQUENCY = 1000
COUNTS_PER_REVOLUTION = 60  # Adjust based on your encoder specifications
GEAR_RATIO = 1  # Adjust based on your motor's gear ratio
DEBUG_FACTOR = 1.045 #Adjust based on left and right speeds

try:
    PWM1 = pwmio.PWMOut(board.D2, frequency=FREQUENCY)
    PWM2 = pwmio.PWMOut(board.D4, frequency=FREQUENCY)
    PWM3 = pwmio.PWMOut(board.D1, frequency=FREQUENCY)
    PWM4 = pwmio.PWMOut(board.D0, frequency=FREQUENCY)
    encoder_left = countio.Counter(board.D10)
    encoder_right = countio.Counter(board.D7)
except Exception as e:
    print(f"Error initializing hardware: {e}")
    raise

class EncoderState:
    def __init__(self):
        self.left_count = 0
        self.right_count = 0
        self.left_direction = 1
        self.right_direction = 1
        self.prev_left_speed = 0
        self.prev_right_speed = 0
        self.accumulated_left_count = 0
        self.accumulated_right_count = 0
        self.prev_raw_left_count = 0
        self.prev_raw_right_count = 0

encoder_state = EncoderState()

def counts_to_radians(counts):
    """Convert encoder counts to radians"""
    return (counts * 2 * math.pi) / (COUNTS_PER_REVOLUTION)

def clamp_speed(speed):
    """Ensure speed is within valid range"""
    return max(MIN_SPEED, min(MAX_SPEED, speed))

def get_encoder_counts():
    """Get current encoder counts with error handling"""
    try:
        return encoder_left.count, encoder_right.count
    except Exception as e:
        print(f"Error reading encoders: {e}")
        return 0, 0

def update_encoder_direction(motor, speed):
    """Update encoder direction based on motor speed"""
    if motor == 1:
        if speed != encoder_state.prev_left_speed:
            encoder_state.left_direction = 1 if speed >= 0 else -1
            encoder_state.prev_left_speed = speed
    else:  # motor == 2
        if speed != encoder_state.prev_right_speed:
            encoder_state.right_direction = 1 if speed >= 0 else -1
            encoder_state.prev_right_speed = speed

def set_motor_speed(motor, speed):
    """Set motor speed with input validation"""
    if motor not in [1, 2]:
        print(f"Invalid motor number: {motor}")
        return
    
    if motor == 1 and DEBUG_FACTOR is not None:
       speed *= DEBUG_FACTOR
    speed = clamp_speed(speed)
    update_encoder_direction(motor, speed)
    duty_cycle = int(65535 * abs(speed))
    
    try:
        if motor == 1:
            PWM1.duty_cycle = duty_cycle if speed > 0 else 0
            PWM2.duty_cycle = duty_cycle if speed < 0 else 0
        else:  # motor == 2
            PWM3.duty_cycle = duty_cycle if speed > 0 else 0
            PWM4.duty_cycle = duty_cycle if speed < 0 else 0
    except Exception as e:
        print(f"Error setting motor speed: {e}")

def process_command(command):
    """Process incoming command with error handling"""
    try:
        command = command.strip()

        # Check for identification command
        if command.lower() == "whoyouare":
            usb_cdc.data.write(b"wheels\n")
            return True

        # Process speed commands
        parts = command.split(',')
        if len(parts) != 2:
            raise ValueError("Expected 2 values")
        
        speed1 = float(parts[0])
        speed2 = float(parts[1])
        
        set_motor_speed(1, speed1)
        set_motor_speed(2, speed2)
        return True
    except Exception as e:
        print(f"Error processing command: {e}")
        set_motor_speed(1, 0)
        set_motor_speed(2, 0)
        return False

def update_accumulated_counts(raw_left, raw_right):
    """Update accumulated counts based on direction"""
    # Left encoder
    delta_left = raw_left - encoder_state.prev_raw_left_count
    encoder_state.accumulated_left_count += delta_left * encoder_state.left_direction
    encoder_state.prev_raw_left_count = raw_left

    # Right encoder
    delta_right = raw_right - encoder_state.prev_raw_right_count
    encoder_state.accumulated_right_count += delta_right * encoder_state.right_direction
    encoder_state.prev_raw_right_count = raw_right

import time

def debug_correction_run_ticks(duration=10.0, test_speed=0.8):
    """
    Debug mode to determine the correction factor based on raw encoder ticks.
    This function sets both motors to the same speed (test_speed),
    counts the ticks for the specified duration, and then computes their ratio.

    Args:
        duration (float): The duration of the test in seconds (default=40.0).
        test_speed (float): Motor speed from -1.0 to 1.0 (default=0.8).

    Returns:
        float or None: The correction factor (left/right) if available, otherwise None.
    """
    print("Starting encoder tick debug: both motors rotating...")

    # Start both motors at the given test speed
    set_motor_speed(1, test_speed)
    set_motor_speed(2, test_speed)

    start_time = time.monotonic()
    last_print_time = start_time

    PRINT_INTERVAL = 0.01  # Interval (in seconds) for print updates
    SLEEP_INTERVAL = 0.001 # Sleep time (in seconds) to avoid busy-wait

    while True:
        current_time = time.monotonic()
        elapsed_time = current_time - start_time

        # Exit the loop once the desired duration has passed
        if elapsed_time >= duration:
            break

        left_count, right_count = get_encoder_counts()

        # Print periodic status updates
        if current_time - last_print_time >= PRINT_INTERVAL:
            print(f"Time: {elapsed_time:.1f}s - ")
            print(f"Left: {left_count} ticks, ")
            print(f"Right: {right_count} ticks")
            last_print_time = current_time

        time.sleep(SLEEP_INTERVAL)

    # Stop the motors after the test completes
    set_motor_speed(1, 0)
    set_motor_speed(2, 0)

    # Final encoder readings
    left_count, right_count = get_encoder_counts()

    print("\nDebug results:")
    print(f"Left wheel: {left_count} ticks")
    print(f"Right wheel: {right_count} ticks")

    # Compute the correction factor
    if right_count != 0:
        correction_factor = left_count / right_count
        print(f"Correction factor (left/right): {correction_factor:.6f}, ")
        print(f"Test speed: {test_speed:.6f}")
    else:
        correction_factor = None
        print("Error: Right wheel produced 0 ticks. Unable to compute correction factor.")

    return correction_factor

def main():
    # debug_factor = debug_correction_run_ticks() ## Uncoment to find factor

    while True:
        try:
            # Process incoming commands
            if usb_cdc.data.in_waiting > 0:
                command = usb_cdc.data.readline().decode()
                process_command(command)

            # Update encoder counts
            left_count, right_count = get_encoder_counts()
            
            if left_count != encoder_state.prev_raw_left_count or right_count != encoder_state.prev_raw_right_count:
                update_accumulated_counts(left_count, right_count)
                
                # Convert accumulated counts to radians
                left_radians = counts_to_radians(encoder_state.accumulated_left_count)
                right_radians = counts_to_radians(encoder_state.accumulated_right_count)
                
                encoder_data = f"{left_radians:.6f},{right_radians:.6f}\n"
                usb_cdc.data.write(encoder_data.encode())

        except Exception as e:
            print(f"Main loop error: {e}")
            time.sleep(0.1)

if __name__ == "__main__":
    main()