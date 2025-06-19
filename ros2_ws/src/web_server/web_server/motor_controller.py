import logging
import threading
from typing import Callable

from .config import ControllerConfig

logger = logging.getLogger(__name__)


class MotorController:
    """Scale + publish joystick values as Twist message components."""

    def __init__(self,
                 cfg: ControllerConfig,
                 publish_cb: Callable[[float, float], None]) -> None:
        self.cfg = cfg
        self._publish = publish_cb
        self._lock = threading.Lock()

    # ---------------------------------------------------------------- #
    #                          public API                              #
    # ---------------------------------------------------------------- #
    def process_joystick(self, turn: float, drive: float) -> None:
        with self._lock:
            linear = (-drive if self.cfg.invert_drive else drive) * self.cfg.scale_linear
            angular = (-turn if self.cfg.invert_turn else turn) * self.cfg.scale_angular
        self._safe_publish(linear, angular)

    def process_xy_input(self, x: float, y: float) -> None:
        self._safe_publish(y * self.cfg.scale_linear,
                           -x * self.cfg.scale_angular)

    def stop(self) -> None:
        self._safe_publish(0.0, 0.0)

    # ---------------------------------------------------------------- #
    #                         internal util                            #
    # ---------------------------------------------------------------- #
    def _safe_publish(self, lin: float, ang: float) -> None:
        try:
            self._publish(float(lin), float(ang))
        except Exception:                              # pragma: no cover
            logger.exception("Motor publish failed")
