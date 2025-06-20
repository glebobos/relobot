#!/usr/bin/env python3
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

"""
Executable entry point (`python -m web_server`) that wires everything together.
"""

import logging
import sys

import rclpy
from .server import RobotWebServer

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(name)s %(levelname)s: %(message)s",
)

def main() -> int:
    try:
        rclpy.init()
        RobotWebServer().start()
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as exc:                                  # pragma: no cover
        logging.critical("Fatal error: %s", exc, exc_info=True)
        return 1
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
