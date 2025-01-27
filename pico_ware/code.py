import time
import board
import pwmio
import usb_cdc

# Настройка GPIO для моторов
PWM1 = pwmio.PWMOut(board.GP27, frequency=10000)
PWM2 = pwmio.PWMOut(board.GP28, frequency=10000)
PWM3 = pwmio.PWMOut(board.GP26, frequency=10000)  # Changed from GP29 to GP26 for testing
PWM4 = pwmio.PWMOut(board.GP6, frequency=10000)

# Функция для управления мотором
def set_motor_speed(motor, speed):
    if motor == 1:  # Мотор 1
        if speed > 0:
            PWM1.duty_cycle = int(65535 * speed)
            PWM2.duty_cycle = 0
        elif speed < 0:
            PWM1.duty_cycle = 0
            PWM2.duty_cycle = int(65535 * -speed)
        else:
            PWM1.duty_cycle = 0
            PWM2.duty_cycle = 0
    elif motor == 2:  # Мотор 2
        if speed > 0:
            PWM3.duty_cycle = int(65535 * speed)
            PWM4.duty_cycle = 0
        elif speed < 0:
            PWM3.duty_cycle = 0
            PWM4.duty_cycle = int(65535 * -speed)
        else:
            PWM3.duty_cycle = 0
            PWM4.duty_cycle = 0

# Функция для тестирования моторов
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
test_motors()

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
        else:
            print("Invalid command format")
