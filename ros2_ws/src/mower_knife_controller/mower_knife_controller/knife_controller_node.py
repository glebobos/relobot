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
ROS 2 node that publishes BOTH knife-motor RPM and Vin (battery/input voltage).

Parameters (all have sensible defaults):
  serial_port        : serial device of controller      (/dev/ttyACM4)
  baud_rate          : serial baud                      (115200)
  current_rpm_topic  : topic to publish RPM             (knives/current_rpm)
  voltage_topic      : topic to publish Vin             (knives/vin)
  set_rpm_topic      : topic to subscribe RPM commands  (knives/set_rpm)
  update_rate        : seconds between polls            (0.1)

Services (same as before):
  knives/enable_pid        (std_srvs/SetBool)
  knives/get_pid_status    (std_srvs/Trigger)
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from std_srvs.srv import SetBool, Trigger

from mower_knife_controller.knife_motor_controller import SerialKnifeMotorController


class KnifeControllerWithVoltage(Node):
    """ROS 2 node handling knife-motor RPM control + Vin monitoring."""

    def __init__(self):
        super().__init__('knife_controller_with_voltage')

        # ─── parameters ──────────────────────────────────────────────────────
        self.declare_parameter('serial_port',        '/dev/ttyACM4')
        self.declare_parameter('baud_rate',          115200)
        self.declare_parameter('current_rpm_topic',  'knives/current_rpm')
        self.declare_parameter('voltage_topic',      'knives/vin')
        self.declare_parameter('set_rpm_topic',      'knives/set_rpm')
        self.declare_parameter('update_rate',        0.1)

        port_name    = self.get_parameter('serial_port').value
        baud_rate    = self.get_parameter('baud_rate').value
        rpm_topic    = self.get_parameter('current_rpm_topic').value
        vin_topic    = self.get_parameter('voltage_topic').value
        cmd_topic    = self.get_parameter('set_rpm_topic').value
        self.period  = float(self.get_parameter('update_rate').value)

        # ─── serial interface ───────────────────────────────────────────────
        self.get_logger().info(f"Connecting knife controller on {port_name}")
        self.ctrl = SerialKnifeMotorController(port_name=port_name,
                                               baud_rate=baud_rate)

        # ─── pubs / subs ─────────────────────────────────────────────────────
        self.rpm_pub  = self.create_publisher(Float32, rpm_topic, 10)
        self.vin_pub  = self.create_publisher(Float32, vin_topic, 10)

        self.rpm_sub  = self.create_subscription(
            Float32, cmd_topic, self.set_rpm_cb, 10)

        self.pid_pub  = self.create_publisher(String, 'knives/pid_status', 10)

        # periodic poll
        self.timer = self.create_timer(self.period, self.poll_cb)

        # services – identical to old node
        self.create_service(SetBool, 'knives/enable_pid',   self.enable_pid_cb)
        self.create_service(Trigger, 'knives/get_pid_status', self.pid_status_cb)

        self.last_cmd = None
        self.get_logger().info('Knife controller w/ voltage node ready')

    # ───────────────────────────── callbacks ────────────────────────────────
    def set_rpm_cb(self, msg):
        rpm = msg.data
        if rpm != 0 or self.last_cmd in (None, 0):
            self.get_logger().debug(f"Command: set RPM → {rpm}")
        self.last_cmd = rpm
        self.ctrl.set_rpm(rpm)

    def poll_cb(self):
        rpm, vin = self.ctrl.read_telemetry()

        if rpm is not None:
            self.rpm_pub.publish(Float32(data=rpm))
            self.get_logger().debug(f"RPM={rpm:.1f}")

        if vin is not None:
            self.vin_pub.publish(Float32(data=vin))
            self.get_logger().debug(f"Vin={vin:.1f}")

    # ───────────────────────────── services ────────────────────────────────
    def enable_pid_cb(self, req, res):
        try:
            cmd = "pid_enable" if req.data else "pid_disable"
            ok  = self.ctrl.send_command(cmd)
            self.publish_pid_status()
            res.success = bool(ok)
            res.message = "PID enabled" if req.data else "PID disabled"
        except Exception as e:
            res.success, res.message = False, str(e)
        return res

    def pid_status_cb(self, _, res):
        try:
            status = self.ctrl.send_command("pid_status")
            res.success, res.message = True, status or "No response"
        except Exception as e:
            res.success, res.message = False, str(e)
        return res

    def publish_pid_status(self):
        status = self.ctrl.send_command("pid_status")
        if status:
            self.pid_pub.publish(String(data=status))

    # ───────────────────────────── shutdown ────────────────────────────────
    def on_shutdown(self):
        self.get_logger().info("Node shutting down; stopping motor")
        try:
            self.ctrl.set_rpm(0)
            time.sleep(0.5)
            self.ctrl.close()
        except Exception as e:
            self.get_logger().error(f"Shutdown error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = KnifeControllerWithVoltage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
