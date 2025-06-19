import logging
import threading
import time
from typing import Callable

logger = logging.getLogger(__name__)


class PIDController:
    """Edge-triggered toggle with debounce for PID enable flag."""

    def __init__(self, toggle_cb: Callable[[bool], None]) -> None:
        self.enabled = True
        self._cb = toggle_cb
        self._lock = threading.Lock()
        self._last_toggle = 0.0
        self._cooldown = 0.5

    def toggle(self) -> None:
        with self._lock:
            now = time.time()
            if now - self._last_toggle < self._cooldown:
                return
            self.enabled = not self.enabled
            self._last_toggle = now
        try:
            self._cb(self.enabled)
            logger.info("PID %s", "enabled" if self.enabled else "disabled")
        except Exception:
            logger.exception("PID toggle callback failed")
