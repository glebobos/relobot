import time
import board
import pwmio
import countio
import digitalio
import math
import ulab.numpy as np  # Лёгкая библиотека для численных расчётов

# Конфигурация ШИМ и энкодера (как в рабочем коде)
FREQUENCY = 10000
COUNTS_PER_REVOLUTION = 1    # Тиков энкодера на оборот (настройте под ваш энкодер)
TEST_DURATION = 2.0           # Время теста на каждом уровне PWM (секунды)
PWM_LEVELS = [i / 10.0 for i in range(1, 11)]  # 0.1 … 1.0

# Инициализация аппаратуры
FWD_PWM = pwmio.PWMOut(board.D0, frequency=FREQUENCY)
REV_PWM = pwmio.PWMOut(board.D3, frequency=FREQUENCY)
FWD_EN = digitalio.DigitalInOut(board.D1)
REV_EN = digitalio.DigitalInOut(board.D2)
for pin in (FWD_EN, REV_EN):
    pin.direction = digitalio.Direction.OUTPUT
encoder = countio.Counter(board.D5, pull=digitalio.Pull.UP)


def counts_to_radians(counts):
    """Преобразует тики энкодера в радианы."""
    return (counts * 2 * math.pi) / COUNTS_PER_REVOLUTION


def set_pwm(pwm_value):
    """Устанавливает только ШИМ для прямого движения (проверка зависимости)."""
    duty = int(65535 * pwm_value)
    FWD_EN.value = True
    REV_EN.value = True
    FWD_PWM.duty_cycle = duty
    REV_PWM.duty_cycle = 0


def compute_linear_fit(x_vals, y_vals):
    """Линейная аппроксимация: pwm = K * ω + C."""
    x = np.array(y_vals)  # ω (рад/с)
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
    """Запуск цикла тестирования PWM → ω и получение коэффициентов."""
    data = []
    print("\nЗапуск калибровки PWM → ω...\n")

    for pwm in PWM_LEVELS:
        encoder.count = 0
        set_pwm(pwm)
        start = time.monotonic()
        time.sleep(TEST_DURATION)
        ticks = encoder.count
        ω = counts_to_radians(ticks) / TEST_DURATION
        print(f"PWM={pwm:.2f} → ω={ω:.3f} рад/с")
        data.append((pwm, ω))

    # Остановить мотор
    FWD_EN.value = False
    REV_EN.value = False
    FWD_PWM.duty_cycle = 0
    REV_PWM.duty_cycle = 0

    pwm_vals = [d[0] for d in data]
    omega_vals = [d[1] for d in data]
    K, C = compute_linear_fit(pwm_vals, omega_vals)
    print(f"\nРезультат калибровки: PWM = {K:.6f} * ω + {C:.6f}")
    return K, C


if __name__ == '__main__':
    run_calibration()
