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
        # Cruise control variables
        self._cruise_rpm = 0.0
        self._cruise_active = False
        # Define cruise control button as button 4
        self.cruise_button = 4
        # Track button state for toggle action
        self._cruise_button_pressed = False
        # Cruise control variables
        self._cruise_rpm = 0.0
        self._cruise_active = False
        # Define cruise control button as button 4
        self.cruise_button = 4
        # Track button state for toggle action
        self._cruise_button_pressed = False

    # --------------------------------------------------------------- #
    #                         public API                              #
    # --------------------------------------------------------------- #
    def process_button_value(self, val: float) -> None:
        """Map button pressure [0-1] → RPM."""
        with self._lock:
            rpm_value = 0.0 if val <= 0.05 else \
                self.min_rpm + val * (self.max_rpm - self.min_rpm)
            
            # Apply inversion to the direction of rotation by negating the RPM value
            self._rpm = -rpm_value if self.cfg.invert_knife and rpm_value != 0 else rpm_value
            
    def process_cruise_control(self, knife_val: float, cruise_btn_val: float) -> None:
        """
        Handle toggle-based cruise control functionality for blade RPM.
        
        Args:
            knife_val: Value of knife button (0-1)
            cruise_btn_val: Value of cruise control button (0-1)
        """
        with self._lock:
            # Calculate the raw RPM value based on the knife button value
            raw_rpm = 0.0 if knife_val <= 0.05 else \
                self.min_rpm + knife_val * (self.max_rpm - self.min_rpm)
                
            # Apply inversion to the direction of rotation by negating the RPM value
            current_rpm = -raw_rpm if self.cfg.invert_knife and raw_rpm != 0 else raw_rpm
            
            # Keep track of knife button being actively pressed (>0.05)
            knife_is_active = knife_val > 0.05
            
            # Handle toggle behavior for cruise control button
            # Detect the rising edge of button press (wasn't pressed before, is pressed now)
            cruise_btn_is_pressed = cruise_btn_val > 0.5
            
            # Track when cruise button is first pressed or released
            cruise_button_just_pressed = cruise_btn_is_pressed and not self._cruise_button_pressed
            
            # Update the button state for next time
            self._cruise_button_pressed = cruise_btn_is_pressed
            
            if cruise_button_just_pressed:
                # Button 4 was just pressed - toggle cruise control state
                if not self._cruise_active:
                    # Activate cruise control and store current RPM
                    self._cruise_rpm = current_rpm
                    self._cruise_active = True
                    logger.info(f"Cruise control toggled ON at {self._cruise_rpm} RPM")
                else:
                    # Deactivate cruise control
                    self._cruise_active = False
                    logger.info("Cruise control toggled OFF")
            
            # Only check for knife button reset when cruise is active AND
            # the cruise button is NOT being pressed (to prevent immediate reset during activation)
            if self._cruise_active and knife_is_active and not cruise_btn_is_pressed:
                self._cruise_active = False
                logger.info("Cruise control reset - knife button pressed")
            
            # Apply the appropriate RPM value
            if self._cruise_active:
                self._rpm = self._cruise_rpm
            else:
                self._rpm = current_rpm

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
