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
Robot Web Control - package initialiser.

Nothing heavy here; we just expose the public façade so that
`from web_server import RobotWebServer` works straight away.
"""

from .server import RobotWebServer          # noqa: F401
from .config import ControllerConfig        # noqa: F401
