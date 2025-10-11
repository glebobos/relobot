import time
import board
import pwmio
import usb_cdc
import countio
import math 

# Constants
MAX_SPEED_RAD_S = 7.5  # Specify maximum speed in rad/sec
MIN_SPEED_RAD_S = -7.5
FREQUENCY = 7000
COUNTS_PER_REVOLUTION = 60  # Specify encoder parameters

# Functions for converting speed to PWM
def speed_to_pwm_left(speed):
    abs_speed = abs(speed)
    if speed >= 0:  # Forward
        pwm_value = 0.12281 * abs_speed + 0.029637
    else:  # Backward
        pwm_value = 0.12281 * abs_speed + 0.029637  # Same as forward for now
    return max(0, min(1, pwm_value))

def speed_to_pwm_right(speed):
    abs_speed = abs(speed)
    if speed >= 0:  # Forward
        pwm_value = 0.13593 * abs_speed + 0.021466
    else:  # Backward
        pwm_value = 0.11793 * abs_speed + 0.021466  # Same as forward for now
    return max(0, min(1, pwm_value))

# Initialization of PWM and encoders
try:
    PWM1 = pwmio.PWMOut(board.D1, frequency=FREQUENCY)
    PWM2 = pwmio.PWMOut(board.D0, frequency=FREQUENCY)
    PWM3 = pwmio.PWMOut(board.D2, frequency=FREQUENCY)
    PWM4 = pwmio.PWMOut(board.D4, frequency=FREQUENCY)
    encoder_left = countio.Counter(board.D7)
    encoder_right = countio.Counter(board.D10)
except Exception as e:
    print(f"Error initializing hardware: {e}")
    raise

class EncoderState:
    def __init__(self):
        self.left_count = 0
        self.right_count = 0
        self.left_direction = 1
        self.right_direction = 1
        self.prev_left_speed = None
        self.prev_right_speed = None
        self.accumulated_left_count = 0
        self.accumulated_right_count = 0
        self.prev_raw_left_count = 0
        self.prev_raw_right_count = 0

encoder_state = EncoderState()

def counts_to_radians(counts):
    return (counts * 2 * math.pi) / COUNTS_PER_REVOLUTION

def get_encoder_counts():
    try:
        return encoder_left.count, encoder_right.count
    except Exception as e:
        print(f"Error reading encoders: {e}")
        return 0, 0

def update_accumulated_counts(raw_left, raw_right):
    delta_left = raw_left - encoder_state.prev_raw_left_count
    encoder_state.accumulated_left_count += delta_left * encoder_state.left_direction
    encoder_state.prev_raw_left_count = raw_left
    
    delta_right = raw_right - encoder_state.prev_raw_right_count
    encoder_state.accumulated_right_count += delta_right * encoder_state.right_direction
    encoder_state.prev_raw_right_count = raw_right

def set_motor_speed(motor, speed):
    if motor not in [1, 2]:
        print(f"Invalid motor number: {motor}")
        return
    
    speed = max(MIN_SPEED_RAD_S, min(MAX_SPEED_RAD_S, speed))
    duty_cycle = int(65535 * (speed_to_pwm_left(speed) if motor == 1 else speed_to_pwm_right(speed)))
    
    try:
        if motor == 1:
            PWM1.duty_cycle = duty_cycle if speed > 0 else 0
            PWM2.duty_cycle = duty_cycle if speed < 0 else 0
            encoder_state.left_direction = 1 if speed >= 0 else -1
        else:
            PWM3.duty_cycle = duty_cycle if speed > 0 else 0
            PWM4.duty_cycle = duty_cycle if speed < 0 else 0
            encoder_state.right_direction = 1 if speed >= 0 else -1
    except Exception as e:
        print(f"Error setting motor speed: {e}")


def process_command(command):
    try:
        command = command.strip()
        if command.lower() == "whoyouare":
            usb_cdc.data.write(b"wheels\n")
            return True
        
        if command.lower() == "reset_counts":
            # Reset accumulated encoder counts and stop motors
            encoder_state.accumulated_left_count = 0
            encoder_state.accumulated_right_count = 0
            encoder_state.prev_raw_left_count = encoder_left.count
            encoder_state.prev_raw_right_count = encoder_right.count
            set_motor_speed(1, 0)
            set_motor_speed(2, 0)
            encoder_state.prev_left_speed = 0
            encoder_state.prev_right_speed = 0
            print("Encoder counts reset and motors stopped")
            return True
        
        parts = command.split(',')
        if len(parts) != 2:
            raise ValueError("Expected 2 values")
        
        speed1 = float(parts[0])
        speed2 = float(parts[1])
        
        if speed1 != encoder_state.prev_left_speed:
            set_motor_speed(1, speed1)
            encoder_state.prev_left_speed = speed1
        if speed2 != encoder_state.prev_right_speed:
            set_motor_speed(2, speed2)
            encoder_state.prev_right_speed = speed2
        
        return True
    except Exception as e:
        print(f"Error processing command: {e}")
        set_motor_speed(1, 0)
        set_motor_speed(2, 0)
        return False
def main():
    while True:
        try:
            if usb_cdc.data.in_waiting > 0:
                command = usb_cdc.data.readline().decode()
                process_command(command)
            
            left_count, right_count = get_encoder_counts()
            if left_count != encoder_state.prev_raw_left_count or right_count != encoder_state.prev_raw_right_count:
                update_accumulated_counts(left_count, right_count)
                left_radians = counts_to_radians(encoder_state.accumulated_left_count)
                right_radians = counts_to_radians(encoder_state.accumulated_right_count)
                encoder_data = f"{left_radians:.6f},{right_radians:.6f}\n"
                # print(encoder_data)
                usb_cdc.data.write(encoder_data.encode())
        except Exception as e:
            print(f"Main loop error: {e}")
            time.sleep(0.1)

if __name__ == "__main__":
    main()
