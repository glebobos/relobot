"""
Robot Web Control - package initialiser.

Nothing heavy here; we just expose the public façade so that
`from web_server import RobotWebServer` works straight away.
"""

from .server import RobotWebServer          # noqa: F401
from .config import ControllerConfig        # noqa: F401
