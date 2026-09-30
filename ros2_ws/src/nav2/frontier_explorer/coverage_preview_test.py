from __future__ import annotations

import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CoveragePreviewTest(Node):
    def __init__(self) -> None:
        super().__init__('coverage_preview_test')

        self.declare_parameter('command_topic', '/coverage/command')
        self.declare_parameter('status_topic', '/coverage/status')
        self.declare_parameter('polygon_points', [0.0, 0.0, 8.0, 0.0, 8.0, 8.0, 0.0, 8.0])

        self._sent = False
        self._done = False
        self._success = False
        self._started = time.monotonic()
        self._preview_sent = False

        self._command_pub = self.create_publisher(
            String, self.get_parameter('command_topic').value, 10
        )
        self.create_subscription(
            String,
            self.get_parameter('status_topic').value,
            self._on_status,
            10,
        )
        self.create_timer(1.0, self._send_preview_once)

    def _send_preview_once(self) -> None:
        if self._sent:
            return

        polygon_points = self.get_parameter('polygon_points').value
        if len(polygon_points) < 6 or len(polygon_points) % 2 != 0:
            self.get_logger().error('polygon_points must contain x/y pairs for at least 3 vertices.')
            self._done = True
            return

        if self._command_pub.get_subscription_count() == 0:
            return
        polygon = [{'x': float(polygon_points[index]), 'y': float(polygon_points[index + 1])}
                   for index in range(0, len(polygon_points), 2)]
        self._sent = True
        self._command_pub.publish(String(data='set_zone:' + json.dumps(polygon)))
        self.get_logger().info('Requested test zone; waiting for acknowledgement.')

    def _on_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        state = payload.get('state')
        if not self._sent:
            return
        if state == 'polygon_ready' and not self._preview_sent:
            self._preview_sent = True
            self._command_pub.publish(String(data='preview'))
            return
        if state == 'preview_ready':
            self._success = True
            self._done = True
            self.get_logger().info(payload.get('message', 'Preview ready.'))
            return

        if state in {'failed', 'rejected', 'server_unavailable', 'polygon_invalid', 'busy', 'error'}:
            self._done = True
            self.get_logger().error(payload.get('message', f'Coverage preview ended in state {state}.'))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CoveragePreviewTest()
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.5)
            if time.monotonic() - node._started > 30.0:
                node.get_logger().error('Preview test timed out.')
                break
    finally:
        success = node._success
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if success else 1)