#!/usr/bin/env python3
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
