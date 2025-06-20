#!/usr/bin/env bash
#
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
cd /home/admin/Documents/relobot/ros2_ws
until ping -c 1 8.8.8.8 &> /dev/null
do
    echo "No internet connection. Retrying in 5 seconds..."
    sleep 5
done
export HOST_IP=$(hostname -I | awk '{print $1}')

docker compose up $1
