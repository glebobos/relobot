import time
import board
import pwmio
import countio
import digitalio
import math
import ulab.numpy as np  # Lightweight library for numerical calculations

# PWM and encoder configuration (as in working code)
FREQUENCY = 10000
COUNTS_PER_REVOLUTION = 1    # Encoder ticks per revolution (configure for your encoder)
TEST_DURATION = 2.0           # Test duration at each PWM level (seconds)
PWM_LEVELS = [i / 10.0 for i in range(1, 11)]  # 0.1 … 1.0

# Hardware initialization
FWD_PWM = pwmio.PWMOut(board.D0, frequency=FREQUENCY)
REV_PWM = pwmio.PWMOut(board.D3, frequency=FREQUENCY)
FWD_EN = digitalio.DigitalInOut(board.D1)
REV_EN = digitalio.DigitalInOut(board.D2)
for pin in (FWD_EN, REV_EN):
    pin.direction = digitalio.Direction.OUTPUT
encoder = countio.Counter(board.D5, pull=digitalio.Pull.UP)


def counts_to_radians(counts):
    """Converts encoder ticks to radians."""
    return (counts * 2 * math.pi) / COUNTS_PER_REVOLUTION


def set_pwm(pwm_value):
    """Sets only PWM for forward movement (dependency check)."""
    duty = int(65535 * pwm_value)
    FWD_EN.value = True
    REV_EN.value = True
    FWD_PWM.duty_cycle = duty
    REV_PWM.duty_cycle = 0


def compute_linear_fit(x_vals, y_vals):
    """Linear approximation: pwm = K * ω + C."""
    x = np.array(y_vals)  # ω (rad/s)
    y = np.array(x_vals)  # pwm
    n = len(x)
    sum_x = np.sum(x)
    sum_y = np.sum(y)
    sum_xy = np.sum(x * y)
    sum_x2 = np.sum(x * x)

    K = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x**2)
    C = (sum_y - K * sum_x) / n
    return K, C


def run_calibration():
    """Launch PWM → ω testing cycle and get coefficients."""
    data = []
    print("\nStarting PWM → ω calibration...\n")

    for pwm in PWM_LEVELS:
        encoder.count = 0
        set_pwm(pwm)
        start = time.monotonic()
        time.sleep(TEST_DURATION)
        ticks = encoder.count
        ω = counts_to_radians(ticks) / TEST_DURATION
        print(f"PWM={pwm:.2f} → ω={ω:.3f} rad/s")
        data.append((pwm, ω))

    # Stop motor
    FWD_EN.value = False
    REV_EN.value = False
    FWD_PWM.duty_cycle = 0
    REV_PWM.duty_cycle = 0

    pwm_vals = [d[0] for d in data]
    omega_vals = [d[1] for d in data]
    K, C = compute_linear_fit(pwm_vals, omega_vals)
    print(f"\nCalibration result: PWM = {K:.6f} * ω + {C:.6f}")
    return K, C


if __name__ == '__main__':
    run_calibration()
