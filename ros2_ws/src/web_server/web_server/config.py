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

import json
import logging
import os
from dataclasses import dataclass, asdict
from typing import Any, Dict, Tuple

logger = logging.getLogger(__name__)


@dataclass
class ControllerConfig:
    """Configuration for controller mappings and settings."""
    turn_axis: int = 0
    drive_axis: int = 3
    knife_button: int = 7
    pid_toggle_button: int = 9
    scale_linear: float = 0.3
    scale_angular: float = 1.0
    invert_turn: bool = True
    invert_drive: bool = True
    invert_knife: bool = False  # When true, inverts the knife rotation direction

    # ------------------------------------------------------------------ #
    #                            validation                              #
    # ------------------------------------------------------------------ #
    def validate(self) -> Tuple[bool, str]:
        if not all(isinstance(val, int) for val in
                   (self.turn_axis, self.drive_axis,
                    self.knife_button, self.pid_toggle_button)):
            return False, "Axes and button indices must be integers"

        if not all(isinstance(val, (int, float))
                   for val in (self.scale_linear, self.scale_angular)):
            return False, "Scale values must be numeric"

        if self.scale_linear <= 0 or self.scale_angular <= 0:
            return False, "Scale values must be positive"
        return True, ""

    # ------------------------------------------------------------------ #
    #                     (de)serialisation helpers                      #
    # ------------------------------------------------------------------ #
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ControllerConfig":
        valid_fields = cls.__annotations__.keys()
        return cls(**{k: v for k, v in data.items() if k in valid_fields})

    # ------------------------------------------------------------------ #
    #                           file helpers                             #
    # ------------------------------------------------------------------ #
    def save_to_file(self, path: str) -> None:
        try:
            with open(path, "w") as fp:
                json.dump(self.to_dict(), fp, indent=2)
            logger.info("Saved controller config → %s", path)
        except OSError as exc:
            logger.error("Could not save config: %s", exc, exc_info=True)

    @classmethod
    def load_from_file(cls, path: str) -> "ControllerConfig":
        try:
            if os.path.exists(path):
                with open(path) as fp:
                    return cls.from_dict(json.load(fp))
            # file absent – create default & persist
            cfg = cls()
            cfg.save_to_file(path)
            return cfg
        except Exception as exc:                      # pragma: no cover
            logger.error("Failed to load config: %s", exc, exc_info=True)
            return cls()
