import sys
import time
import pygame
from typing import Callable, Dict, Set

IDLE_ZONE_CONFIG = {
    0: 0.07,
    3: 0.07,
}

DISABLED_AXES = {1, 2}

AxisCallback   = Callable[[int, float], None]
ButtonCallback = Callable[[int], None]

class Controller:
    """Generic wrapper over pygame.Joystick для регистрации колбэков
       и непрерывной отправки, пока ось/кнопка удерживается."""
    def __init__(self, joystick_idx: int = 0):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            print("No joystick connected.")
            sys.exit(1)

        self.joystick = pygame.joystick.Joystick(joystick_idx)
        self.joystick.init()

        self._axis_callbacks: Dict[int, AxisCallback]     = {}
        self._button_down_callbacks: Dict[int, ButtonCallback] = {}
        self._button_up_callbacks:   Dict[int, ButtonCallback] = {}

        self._last_axis_values: Dict[int, float] = {}
        # множества удерживаемых осей и кнопок
        self._held_axes:    Set[int]   = set()
        self._held_buttons: Set[int]   = set()

    def register_axis(self, axis: int, callback: AxisCallback):
        """Будем вызывать callback(axis, processed_value)."""
        self._axis_callbacks[axis] = callback

    def register_button_down(self, button: int, callback: ButtonCallback):
        """Будем вызывать callback(button) при нажатии и удерживании."""
        self._button_down_callbacks[button] = callback

    def register_button_up(self, button: int, callback: ButtonCallback):
        """Будем вызывать callback(button) при отпускании."""
        self._button_up_callbacks[button] = callback

    def _process_axis(self, axis: int, raw_value: float) -> float:
        """Применяем dead-zone и рескейлинг."""
        if axis in DISABLED_AXES:
            return 0.0

        if axis in IDLE_ZONE_CONFIG:
            thr = IDLE_ZONE_CONFIG[axis]
            if abs(raw_value) < thr:
                return 0.0
            sign = 1 if raw_value > 0 else -1
            return (abs(raw_value) - thr) / (1 - thr) * sign

        return raw_value

    def _dispatch_axis(self, axis: int, value: float):
        """Вызываем колбэк, если изменилось достаточно (для одноразовой отправки)."""
        proc = self._process_axis(axis, value)
        last = self._last_axis_values.get(axis)
        if last is None or abs(proc - last) > 1e-3:
            self._last_axis_values[axis] = proc
            cb = self._axis_callbacks.get(axis)
            if cb:
                cb(axis, proc)

    def run(self, poll_delay: float = 0.01):
        """
        Главный цикл:
         - обрабатывает pygame-события,
         - отправляет одноразово при изменении,
         - и непрерывно, пока удерживается (с частотой 1/poll_delay).
        """
        try:
            while True:
                # 1) Обработка событий
                for ev in pygame.event.get():
                    if ev.type == pygame.JOYAXISMOTION:
                        # одноразово при изменении
                        self._dispatch_axis(ev.axis, ev.value)
                        # отслеживаем, удерживается ли ось вне dead-zone
                        proc = self._process_axis(ev.axis, ev.value)
                        if abs(proc) > 0:
                            self._held_axes.add(ev.axis)
                        else:
                            self._held_axes.discard(ev.axis)

                    elif ev.type == pygame.JOYBUTTONDOWN:
                        # колбэк при нажатии
                        self._held_buttons.add(ev.button)
                        cb = self._button_down_callbacks.get(ev.button)
                        if cb:
                            cb(ev.button)

                    elif ev.type == pygame.JOYBUTTONUP:
                        # колбэк при отпускании
                        self._held_buttons.discard(ev.button)
                        cb = self._button_up_callbacks.get(ev.button)
                        if cb:
                            cb(ev.button)

                # 2) Непрерывная отправка для удерживаемых осей
                for axis in list(self._held_axes):
                    raw = self.joystick.get_axis(axis)
                    proc = self._process_axis(axis, raw)
                    cb = self._axis_callbacks.get(axis)
                    if cb:
                        cb(axis, proc)

                # 3) Непрерывная отправка для удерживаемых кнопок
                for btn in list(self._held_buttons):
                    cb = self._button_down_callbacks.get(btn)
                    if cb:
                        cb(btn)

                time.sleep(poll_delay)

        except KeyboardInterrupt:
            print("Exiting...")
        finally:
            pygame.quit()
