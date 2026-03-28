from __future__ import annotations

import json
import sys

from geometry_msgs.msg import Point32, PolygonStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CoveragePreviewTest(Node):
    def __init__(self) -> None:
        super().__init__('coverage_preview_test')

        self.declare_parameter('polygon_topic', '/coverage/polygon')
        self.declare_parameter('command_topic', '/coverage/command')
        self.declare_parameter('status_topic', '/coverage/status')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('polygon_points', [0.0, 0.0, 2.0, 0.0, 2.0, 2.0, 0.0, 2.0])

        self._sent = False
        self._done = False
        self._success = False

        self._polygon_pub = self.create_publisher(
            PolygonStamped, self.get_parameter('polygon_topic').value, 10
        )
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

        polygon = PolygonStamped()
        polygon.header.frame_id = self.get_parameter('frame_id').value
        for index in range(0, len(polygon_points), 2):
            polygon.polygon.points.append(
                Point32(
                    x=float(polygon_points[index]),
                    y=float(polygon_points[index + 1]),
                    z=0.0,
                )
            )
        self._polygon_pub.publish(polygon)
        self._command_pub.publish(String(data='preview'))
        self._sent = True
        self.get_logger().info('Published test polygon and preview command.')

    def _on_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        state = payload.get('state')
        if state == 'preview_ready':
            self._success = True
            self._done = True
            self.get_logger().info(payload.get('message', 'Preview ready.'))
            return

        if state in {'failed', 'rejected', 'server_unavailable', 'polygon_invalid'}:
            self._done = True
            self.get_logger().error(payload.get('message', f'Coverage preview ended in state {state}.'))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CoveragePreviewTest()
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        success = node._success
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if success else 1)