import time
import board
import pwmio
import countio
import math
import ulab.numpy as np  # Лёгкая библиотека для численных расчётов

# Константы
FREQUENCY = 1000  # Частота ШИМ
COUNTS_PER_REVOLUTION = 60  # Количество тиков на один оборот
TEST_DURATIONS = 2.0  # Время теста на одном уровне PWM (сек)

# Уровни ШИМ для теста
PWM_LEVELS = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]

# Инициализация ШИМ
PWM1 = pwmio.PWMOut(board.D2, frequency=FREQUENCY)
PWM2 = pwmio.PWMOut(board.D4, frequency=FREQUENCY)
PWM3 = pwmio.PWMOut(board.D1, frequency=FREQUENCY)
PWM4 = pwmio.PWMOut(board.D0, frequency=FREQUENCY)

# Инициализация энкодеров
encoder_left = countio.Counter(board.D7)
encoder_right = countio.Counter(board.D10)

def counts_to_radians(counts):
    """Преобразует количество тиков энкодера в радианы."""
    return (counts * 2 * math.pi) / COUNTS_PER_REVOLUTION

def set_motor_speed(pwm_value):
    """Устанавливает ШИМ на оба двигателя."""
    duty_cycle = int(65535 * pwm_value)
    PWM1.duty_cycle = duty_cycle
    PWM2.duty_cycle = 0  # Левый мотор
    PWM3.duty_cycle = duty_cycle
    PWM4.duty_cycle = 0  # Правый мотор

def get_encoder_counts():
    """Возвращает текущее количество тиков энкодера."""
    return encoder_left.count, encoder_right.count

def compute_linear_fit(x_values, y_values):
    """Выполняет линейную аппроксимацию вручную: вычисляет K и C для PWM = K * ω + C"""
    x = np.array(y_values)  # Скорость (рад/с)
    y = np.array(x_values)  # PWM

    n = len(x)
    sum_x = np.sum(x)
    sum_y = np.sum(y)
    sum_xy = np.sum(x * y)
    sum_x2 = np.sum(x * x)

    # Формула метода наименьших квадратов
    K = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x**2)
    C = (sum_y - K * sum_x) / n

    return K, C

def run_speed_test():
    """Проводит тест и считает уравнение линейной аппроксимации."""
    results = []

    print("\nЗапуск теста скорости...\n")

    for pwm_value in PWM_LEVELS:
        print(f"Тестируем PWM = {pwm_value:.2f}...")

        # Сброс энкодеров
        encoder_left.count = 0
        encoder_right.count = 0

        set_motor_speed(pwm_value)
        start_time = time.monotonic()
        prev_left_count, prev_right_count = get_encoder_counts()
        
        time.sleep(TEST_DURATIONS / 2)  # Даем мотору стабилизироваться

        while (time.monotonic() - start_time) < TEST_DURATIONS:
            time.sleep(0.1)  # Опрос каждые 100 мс

        # Получаем конечные значения энкодера
        left_count, right_count = get_encoder_counts()
        elapsed_time = time.monotonic() - start_time

        # Вычисляем скорость вращения (рад/с)
        left_speed = counts_to_radians(left_count - prev_left_count) / elapsed_time
        right_speed = counts_to_radians(right_count - prev_right_count) / elapsed_time

        print(f"PWM {pwm_value:.2f} → Левый мотор: {left_speed:.3f} рад/с, Правый мотор: {right_speed:.3f} рад/с")

        results.append((pwm_value, left_speed, right_speed))

    # Останавливаем моторы
    set_motor_speed(0)

    print("\nТест завершён!")

    # Разделяем данные для аппроксимации
    pwm_values = [res[0] for res in results]
    left_speeds = [res[1] for res in results]
    right_speeds = [res[2] for res in results]

    # Вычисляем коэффициенты для обоих моторов
    K_left, C_left = compute_linear_fit(pwm_values, left_speeds)
    K_right, C_right = compute_linear_fit(pwm_values, right_speeds)

    # Вывод уравнений
    print(f"\nУравнение для ЛЕВОГО мотора: PWM = {K_left:.6f} * Скорость + {C_left:.6f}")
    print(f"Уравнение для ПРАВОГО мотора: PWM = {K_right:.6f} * Скорость + {C_right:.6f}")

    return K_left, C_left, K_right, C_right

if __name__ == "__main__":
    run_speed_test()

