import time
import board
import pwmio
import usb_cdc
import countio


PWM1 = pwmio.PWMOut(board.D1, frequency=10000)
PWM2 = pwmio.PWMOut(board.D0, frequency=10000)
PWM3 = pwmio.PWMOut(board.D2, frequency=10000)  # Changed from GP29 to GP26 for testing
PWM4 = pwmio.PWMOut(board.D4, frequency=10000)
encoder_left = countio.Counter(board.D7)
encoder_right = countio.Counter(board.D10)

def get_encoder_counts():
    return encoder_left.count, encoder_right.count
def set_motor_speed(motor, speed):
    if motor == 1:
        if speed > 0:
            PWM1.duty_cycle = int(65535 * speed)
            PWM2.duty_cycle = 0
        elif speed < 0:
            PWM1.duty_cycle = 0
            PWM2.duty_cycle = int(65535 * -speed)
        else:
            PWM1.duty_cycle = 0
            PWM2.duty_cycle = 0
    elif motor == 2:
        if speed > 0:
            PWM3.duty_cycle = int(65535 * speed)
            PWM4.duty_cycle = 0
        elif speed < 0:
            PWM3.duty_cycle = 0
            PWM4.duty_cycle = int(65535 * -speed)
        else:
            PWM3.duty_cycle = 0
            PWM4.duty_cycle = 0

def test_motors():
    print("Testing motors...")
    test_speeds = [0.5, -0.5]  # Test with 50% speed in both directions
    for motor in [1, 2]:
        for speed in test_speeds:
            set_motor_speed(motor, speed)
            print(f"Motor {motor} set to speed {speed}")
            time.sleep(1)
        set_motor_speed(motor, 0)
    print("Motor testing complete.")

# Главный цикл
print("Motor control ready. Send commands in format: M1,M2")

# Run motor test after start
# test_motors()
last_encoder_update = time.monotonic()
# set_motor_speed(1, 0.2)
# set_motor_speed(2, 0.2)
while True:
    if usb_cdc.data.in_waiting > 0:
        command = usb_cdc.data.readline().decode().strip()  # Read input from USB serial
        parts = command.split(',')
        if len(parts) == 2:
            try:
                speed1 = float(parts[0])
                speed2 = float(parts[1])
                set_motor_speed(1, speed1)
                set_motor_speed(2, speed2)
                print(f"Motors set to: M1={speed1}, M2={speed2}")
            except ValueError:
                print("Invalid command format")
                set_motor_speed(1, 0)
                set_motor_speed(2, 0)
        else:
            print("Invalid command format")
    current_time = time.monotonic()
    if current_time - last_encoder_update >= 1:
        left_count, right_count = get_encoder_counts()
        print(f"Encoder counts: Left={left_count}, Right={right_count}")
        last_encoder_update = current_time