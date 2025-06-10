import time
import pygame
from typing import Callable, Dict, Set

# Configs for dead-zone and disabled axes
IDLE_ZONE_CONFIG = {
    0: 0.07,
    3: 0.07,
}
DISABLED_AXES = {1, 2}

AxisCallback   = Callable[[int, float], None]
ButtonCallback = Callable[[int], None]

class Controller:
    """Generic wrapper over pygame.Joystick:
       - callback registration
       - automatic repeat sending when held
       - handling joystick connection/disconnection
       - periodic polling in case of missing udev events (for example in Docker)
    """
    def __init__(self, joystick_idx: int = 0, retry_interval: float = 3.0):
        pygame.init()
        pygame.joystick.init()
        self.joystick_idx = joystick_idx
        self.retry_interval = retry_interval
        self.connected = False
        self.joystick = None
        
        # Initialize attributes before calling _try_connect()
        self._axis_callbacks: Dict[int, AxisCallback] = {}
        self._button_down_callbacks: Dict[int, ButtonCallback] = {}
        self._button_up_callbacks: Dict[int, ButtonCallback] = {}
        self._last_axis_values: Dict[int, float] = {}
        self._held_axes: Set[int] = set()
        self._held_buttons: Set[int] = set()
        
        # Now trying to connect
        self._try_connect()

    def _try_connect(self):
        """Trying to connect to the joystick without blocking."""
        pygame.joystick.quit()
        pygame.joystick.init()
        try:
            count = pygame.joystick.get_count()
            if count > 0:
                self._connect(self.joystick_idx)
                return True
            return False
        except pygame.error:
            return False

    def _connect(self, joystick_idx: int):
        """Initializes the joystick by index and sets connected=True."""
        try:
            self.joystick = pygame.joystick.Joystick(joystick_idx)
            self.joystick.init()
            self.connected = True
            # Clear state on new connection
            self._held_axes.clear()
            self._held_buttons.clear()
            self._last_axis_values.clear()
            print(f"Joystick {joystick_idx} initialized: {self.joystick.get_name()}")
        except pygame.error as e:
            print(f"Failed to initialize joystick {joystick_idx}: {e}")
            self.connected = False

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
        """Main loop: events + continuous sending + periodic connection polling"""
        last_check = time.time()
        try:
            while True:
                # Periodically check the connection
                if time.time() - last_check >= self.retry_interval:
                    if not self.connected:
                        self._try_connect()
                    last_check = time.time()
                
                # Process events with timeout
                for event in pygame.event.get():
                    if event.type == pygame.JOYAXISMOTION and self.connected:
                        self._dispatch_axis(event.axis, event.value)
                        proc = self._process_axis(event.axis, event.value)
                        if abs(proc) > 0:
                            self._held_axes.add(event.axis)
                        else:
                            self._held_axes.discard(event.axis)
                    elif event.type == pygame.JOYBUTTONDOWN and self.connected:
                        self._held_buttons.add(event.button)
                        cb = self._button_down_callbacks.get(event.button)
                        if cb:
                            cb(event.button)
                    elif event.type == pygame.JOYBUTTONUP and self.connected:
                        self._held_buttons.discard(event.button)
                        cb = self._button_up_callbacks.get(event.button)
                        if cb:
                            cb(event.button)
                    elif event.type == pygame.JOYDEVICEREMOVED:
                        print("Joystick removed. Waiting for reconnection...")
                        self.connected = False
                        self._held_axes.clear()
                        self._held_buttons.clear()
                    elif event.type == pygame.JOYDEVICEADDED:
                        print("Joystick added event received.")
                        self._connect(event.device_index)

                # Continuous sending for held buttons and axes
                if self.connected:
                    try:
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
                    except pygame.error:
                        # Joystick was disconnected, but no event was received
                        print("Error accessing joystick. Marking as disconnected.")
                        self.connected = False
                        self._held_axes.clear()
                        self._held_buttons.clear()

                time.sleep(poll_delay)
        except KeyboardInterrupt:
            print("Exiting...")
        finally:
            pygame.quit()