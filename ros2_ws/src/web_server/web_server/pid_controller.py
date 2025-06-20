# Copyright 2025 ReloBot Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
