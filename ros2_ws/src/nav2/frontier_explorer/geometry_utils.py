from __future__ import annotations
import math
from typing import Any
from geometry_msgs.msg import Point32, PoseStamped, PolygonStamped, Quaternion

def points_equal(first: tuple[float, float], second: tuple[float, float]) -> bool:
    """Return True if two 2D points are within a threshold distance."""
    return math.isclose(first[0], second[0], abs_tol=1e-4) and math.isclose(
        first[1], second[1], abs_tol=1e-4
    )

def normalize_polygon(msg: PolygonStamped, default_frame_id: str = 'map') -> PolygonStamped | None:
    """Remove duplicate consecutive points and ensure the polygon is closed (first == last)."""
    points = []
    for point in msg.polygon.points:
        candidate = (float(point.x), float(point.y))
        if not points or not points_equal(points[-1], candidate):
            points.append(candidate)

    if len(points) >= 2 and points_equal(points[0], points[-1]):
        points.pop()

    if len(points) < 3:
        return None

    normalized = PolygonStamped()
    normalized.header = msg.header
    normalized.header.frame_id = msg.header.frame_id or default_frame_id

    for x_coord, y_coord in points:
        normalized.polygon.points.append(Point32(x=float(x_coord), y=float(y_coord), z=0.0))

    first = normalized.polygon.points[0]
    normalized.polygon.points.append(Point32(x=float(first.x), y=float(first.y), z=0.0))
    return normalized

def flip_quaternion(q: Quaternion) -> Quaternion:
    """Rotate a quaternion by 180 degrees (pi radians) around the Z axis."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    new_yaw = yaw + math.pi
    
    q_new = Quaternion()
    q_new.x = 0.0
    q_new.y = 0.0
    q_new.z = math.sin(new_yaw / 2.0)
    q_new.w = math.cos(new_yaw / 2.0)
    return q_new

def swath_to_waypoints(swaths: Any, frame_id: str, stamp: Any, endpoint_margin: float) -> list[PoseStamped]:
    """Convert swaths from coverage path planner to nav2 poses with margin offset."""
    waypoints = []
    for swath in swaths:
        sx, sy = float(swath.start.x), float(swath.start.y)
        ex, ey = float(swath.end.x), float(swath.end.y)
        dx = ex - sx
        dy = ey - sy
        length = math.hypot(dx, dy)
        if length > (2.0 * endpoint_margin) and endpoint_margin > 0.0:
            offset_x = (dx / length) * endpoint_margin
            offset_y = (dy / length) * endpoint_margin
            sx += offset_x
            sy += offset_y
            ex -= offset_x
            ey -= offset_y
        angle = math.atan2(ey - sy, ex - sx)
        qz = math.sin(angle / 2.0)
        qw = math.cos(angle / 2.0)
        for px, py in [(sx, sy), (ex, ey)]:
            ps = PoseStamped()
            ps.header.frame_id = frame_id
            ps.header.stamp = stamp
            ps.pose.position.x = px
            ps.pose.position.y = py
            ps.pose.position.z = 0.0
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            waypoints.append(ps)
    return waypoints
