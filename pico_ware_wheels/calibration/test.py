import time
import board
import pwmio
import countio
import math
import ulab.numpy as np  # Lightweight library for numerical calculations

# Constants
FREQUENCY = 1000  # PWM frequency
COUNTS_PER_REVOLUTION = 60  # Number of ticks per revolution
TEST_DURATIONS = 2.0  # Test time for each PWM level (sec)

# PWM levels for testing
PWM_LEVELS = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]

# PWM initialization
PWM1 = pwmio.PWMOut(board.D2, frequency=FREQUENCY)
PWM2 = pwmio.PWMOut(board.D4, frequency=FREQUENCY)
PWM3 = pwmio.PWMOut(board.D1, frequency=FREQUENCY)
PWM4 = pwmio.PWMOut(board.D0, frequency=FREQUENCY)

# Encoders initialization
encoder_left = countio.Counter(board.D7)
encoder_right = countio.Counter(board.D10)

def counts_to_radians(counts):
    """Converts encoder tick count to radians."""
    return (counts * 2 * math.pi) / COUNTS_PER_REVOLUTION

def set_motor_speed(pwm_value):
    """Sets PWM for both motors."""
    duty_cycle = int(65535 * pwm_value)
    PWM1.duty_cycle = duty_cycle
    PWM2.duty_cycle = 0  # Left motor
    PWM3.duty_cycle = duty_cycle
    PWM4.duty_cycle = 0  # Right motor

def get_encoder_counts():
    """Returns current encoder tick count."""
    return encoder_left.count, encoder_right.count

def compute_linear_fit(x_values, y_values):
    """Performs manual linear approximation: calculates K and C for PWM = K * ω + C"""
    x = np.array(y_values)  # Speed (rad/s)
    y = np.array(x_values)  # PWM

    n = len(x)
    sum_x = np.sum(x)
    sum_y = np.sum(y)
    sum_xy = np.sum(x * y)
    sum_x2 = np.sum(x * x)

    # Least squares method formula
    K = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x**2)
    C = (sum_y - K * sum_x) / n

    return K, C

def run_speed_test():
    """Runs the test and calculates linear approximation equation."""
    results = []

    print("\nStarting speed test...\n")

    for pwm_value in PWM_LEVELS:
        print(f"Testing PWM = {pwm_value:.2f}...")

        # Reset encoders
        encoder_left.count = 0
        encoder_right.count = 0

        set_motor_speed(pwm_value)
        start_time = time.monotonic()
        prev_left_count, prev_right_count = get_encoder_counts()
        
        time.sleep(TEST_DURATIONS / 2)  # Let the motor stabilize

        while (time.monotonic() - start_time) < TEST_DURATIONS:
            time.sleep(0.1)  # Poll every 100 ms

        # Get final encoder values
        left_count, right_count = get_encoder_counts()
        elapsed_time = time.monotonic() - start_time

        # Calculate rotation speed (rad/s)
        left_speed = counts_to_radians(left_count - prev_left_count) / elapsed_time
        right_speed = counts_to_radians(right_count - prev_right_count) / elapsed_time

        print(f"PWM {pwm_value:.2f} → Left motor: {left_speed:.3f} rad/s, Right motor: {right_speed:.3f} rad/s")

        results.append((pwm_value, left_speed, right_speed))

    # Stop motors
    set_motor_speed(0)

    print("\nTest completed!")

    # Separate data for approximation
    pwm_values = [res[0] for res in results]
    left_speeds = [res[1] for res in results]
    right_speeds = [res[2] for res in results]

    # Calculate coefficients for both motors
    K_left, C_left = compute_linear_fit(pwm_values, left_speeds)
    K_right, C_right = compute_linear_fit(pwm_values, right_speeds)

    # Output equations
    print(f"\nEquation for LEFT motor: PWM = {K_left:.6f} * Speed + {C_left:.6f}")
    print(f"Equation for RIGHT motor: PWM = {K_right:.6f} * Speed + {C_right:.6f}")

    return K_left, C_left, K_right, C_right

if __name__ == "__main__":
    run_speed_test()

