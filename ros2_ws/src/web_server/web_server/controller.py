import sys
import time
import pygame
from typing import Callable, Dict, Set

# Конфиги dead-zone и отключённых осей
IDLE_ZONE_CONFIG = {
    0: 0.07,
    3: 0.07,
}
DISABLED_AXES = {1, 2}

AxisCallback   = Callable[[int, float], None]
ButtonCallback = Callable[[int], None]

class Controller:
    """Generic wrapper over pygame.Joystick:
       - регистрация колбэков
       - автоматическая повторная отправка при удержании
       - обработка подключения/отключения джойстика
       - периодический опрос на случай отсутствия udev-ивентов (например в Docker)
    """
    def __init__(self, joystick_idx: int = 0, retry_interval: float = 3.0):
        pygame.init()
        pygame.joystick.init()
        self.joystick_idx = joystick_idx
        self.retry_interval = retry_interval
        self.connected = False
        self.joystick = None
        # initial wait or immediate connect
        self._wait_for_joystick()

        self._axis_callbacks: Dict[int, AxisCallback]       = {}
        self._button_down_callbacks: Dict[int, ButtonCallback] = {}
        self._button_up_callbacks:   Dict[int, ButtonCallback] = {}

        self._last_axis_values: Dict[int, float] = {}
        self._held_axes:    Set[int]   = set()
        self._held_buttons: Set[int]   = set()

    def _wait_for_joystick(self):
        """Ожидаем подключения джойстика, проверяя с задержкой."""
        while True:
            pygame.joystick.init()
            try:
                count = pygame.joystick.get_count()
            except pygame.error:
                count = 0
            if count > 0:
                self._connect(self.joystick_idx)
                break
            print(f"No joystick connected. Retrying in {self.retry_interval} seconds...")
            time.sleep(self.retry_interval)

    def _connect(self, joystick_idx: int):
        """Инициализирует джойстик по индексу и ставит connected=True."""
        try:
            pygame.joystick.quit()
            pygame.joystick.init()
            self.joystick = pygame.joystick.Joystick(joystick_idx)
            self.joystick.init()
            self.connected = True
            print(f"Joystick {joystick_idx} initialized: {self.joystick.get_name()}")
        except pygame.error as e:
            print(f"Failed to initialize joystick {joystick_idx}: {e}")
            time.sleep(self.retry_interval)
            self._wait_for_joystick()

    def _process_axis(self, axis: int, raw_value: float) -> float:
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
        proc = self._process_axis(axis, value)
        last = self._last_axis_values.get(axis)
        if last is None or abs(proc - last) > 1e-3:
            self._last_axis_values[axis] = proc
            cb = self._axis_callbacks.get(axis)
            if cb:
                cb(axis, proc)

    def register_axis(self, axis: int, callback: AxisCallback):
        self._axis_callbacks[axis] = callback

    def register_button_down(self, button: int, callback: ButtonCallback):
        self._button_down_callbacks[button] = callback

    def register_button_up(self, button: int, callback: ButtonCallback):
        self._button_up_callbacks[button] = callback

    def run(self, poll_delay: float = 0.01):
        """Главный цикл: события + непрерывная отправка + периодический опрос подключения"""
        last_check = time.time()
        try:
            while True:
                # если джойстик внезапно пропал без события
                if not self.connected and time.time() - last_check >= self.retry_interval:
                    self._wait_for_joystick()
                    last_check = time.time()
                # обрабатываем события
                ev = pygame.event.wait()
                if ev.type == pygame.JOYAXISMOTION:
                    self._dispatch_axis(ev.axis, ev.value)
                    proc = self._process_axis(ev.axis, ev.value)
                    if abs(proc) > 0:
                        self._held_axes.add(ev.axis)
                    else:
                        self._held_axes.discard(ev.axis)
                elif ev.type == pygame.JOYBUTTONDOWN:
                    self._held_buttons.add(ev.button)
                    cb = self._button_down_callbacks.get(ev.button)
                    if cb:
                        cb(ev.button)
                elif ev.type == pygame.JOYBUTTONUP:
                    self._held_buttons.discard(ev.button)
                    cb = self._button_up_callbacks.get(ev.button)
                    if cb:
                        cb(ev.button)
                elif ev.type == pygame.JOYDEVICEREMOVED:
                    print("Joystick removed. Waiting for reconnection...")
                    self.connected = False
                elif ev.type == pygame.JOYDEVICEADDED:
                    print("Joystick added event received.")
                    self._connect(ev.device_index)
                    pygame.event.clear()

                # непрерывная отправка для удерживаемых
                for axis in list(self._held_axes):
                    raw = self.joystick.get_axis(axis)
                    proc = self._process_axis(axis, raw)
                    cb = self._axis_callbacks.get(axis)
                    if cb:
                        cb(axis, proc)
                for btn in list(self._held_buttons):
                    cb = self._button_down_callbacks.get(btn)
                    if cb:
                        cb(btn)

                time.sleep(poll_delay)
        except KeyboardInterrupt:
            print("Exiting...")
        finally:
            pygame.quit()
