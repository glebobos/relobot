import logging
import threading
import time
from typing import Callable

from .config import ControllerConfig

logger = logging.getLogger(__name__)


class KnifeController:
    """
    Publishes the desired RPM for the knife mechanism at a fixed interval.
    """

    def __init__(self,
                 cfg: ControllerConfig,
                 publish_callback: Callable[[float], None]) -> None:
        self.cfg = cfg
        self._publish = publish_callback
        self.min_rpm = 500
        self.max_rpm = 3000
        self._rpm = 0.0
        self._last_send = 0.0
        self._interval = 0.2
        self._lock = threading.Lock()
        self._running = False

    # --------------------------------------------------------------- #
    #                         public API                              #
    # --------------------------------------------------------------- #
    def process_button_value(self, val: float) -> None:
        """Map button pressure [0-1] → RPM."""
        with self._lock:
            if self.cfg.invert_knife:
                val = 1.0 - val
            self._rpm = 0.0 if val <= 0.05 else \
                self.min_rpm + val * (self.max_rpm - self.min_rpm)

    def start(self) -> None:
        if not self._running:
            self._running = True
            threading.Thread(target=self._loop,
                             name="KnifeRPMLoop",
                             daemon=True).start()

    def stop(self) -> None:
        self._running = False

    # --------------------------------------------------------------- #
    #                        internal loop                            #
    # --------------------------------------------------------------- #
    def _loop(self) -> None:
        logger.info("Knife RPM loop started")
        try:
            while self._running:
                now = time.time()
                if now - self._last_send >= self._interval:
                    with self._lock:
                        rpm = self._rpm
                    try:
                        self._publish(float(rpm))
                    except Exception:                     # pragma: no cover
                        logger.exception("Knife publish failed")
                    self._last_send = now
                time.sleep(0.05)
        finally:
            logger.info("Knife RPM loop stopped")
