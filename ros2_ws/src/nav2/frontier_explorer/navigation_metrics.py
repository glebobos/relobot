"""Pure geometry used by the passive navigation recorder."""

from __future__ import annotations

import math


def motion_kind(linear: float, angular: float) -> str:
    if not all(math.isfinite(value) for value in (linear, angular)):
        return 'invalid'
    if abs(linear) < 0.01:
        return 'stationary_turn' if abs(angular) >= 0.05 else 'stopped'
    return 'reverse' if linear < 0.0 else 'forward'


def wheel_speeds(linear: float, angular: float, separation: float, radius: float) -> dict:
    if separation <= 0.0 or radius <= 0.0:
        raise ValueError('Wheel separation and radius must be positive')
    return {
        'left_rad_s': (linear - angular * separation / 2.0) / radius,
        'right_rad_s': (linear + angular * separation / 2.0) / radius,
    }


def point_footprint_distance(point: tuple, footprint: list[tuple]) -> float:
    """Distance to the filled footprint; zero for points inside or on its edge."""
    if len(footprint) < 3:
        raise ValueError('Footprint needs at least three vertices')
    point_x, point_y = point
    inside = False
    distance = math.inf
    for start, end in zip(footprint, footprint[1:] + footprint[:1]):
        start_x, start_y = start
        end_x, end_y = end
        delta_x, delta_y = end_x - start_x, end_y - start_y
        length_squared = delta_x ** 2 + delta_y ** 2
        fraction = 0.0 if length_squared == 0.0 else max(0.0, min(
            1.0, ((point_x - start_x) * delta_x + (point_y - start_y) * delta_y)
            / length_squared,
        ))
        distance = min(distance, math.hypot(
            point_x - start_x - fraction * delta_x,
            point_y - start_y - fraction * delta_y,
        ))
        if (start_y > point_y) != (end_y > point_y):
            crossing_x = start_x + (point_y - start_y) * delta_x / delta_y
            if point_x < crossing_x:
                inside = not inside
    return 0.0 if inside else distance


def observed_scan_clearance(ranges, angle_min, angle_increment, range_min, range_max,
                            translation, rotation, footprint):
    quat_x, quat_y, quat_z, quat_w = rotation
    norm = math.sqrt(sum(value * value for value in rotation))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('invalid_scan_rotation')
    quat_x, quat_y, quat_z, quat_w = (value / norm for value in rotation)
    vertical_x = 2.0 * (quat_x * quat_z - quat_y * quat_w)
    vertical_y = 2.0 * (quat_y * quat_z + quat_x * quat_w)
    if math.hypot(vertical_x, vertical_y) > math.sin(0.1):
        raise ValueError('nonplanar_scan_transform')
    rotate_xx = 1.0 - 2.0 * (quat_y ** 2 + quat_z ** 2)
    rotate_xy = 2.0 * (quat_x * quat_y - quat_z * quat_w)
    rotate_yx = 2.0 * (quat_x * quat_y + quat_z * quat_w)
    rotate_yy = 1.0 - 2.0 * (quat_x ** 2 + quat_z ** 2)
    minimum = math.inf
    valid_returns = 0
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance) or not range_min <= distance <= range_max:
            continue
        angle = angle_min + index * angle_increment
        local_x, local_y = distance * math.cos(angle), distance * math.sin(angle)
        point = (translation[0] + rotate_xx * local_x + rotate_xy * local_y,
                 translation[1] + rotate_yx * local_x + rotate_yy * local_y)
        minimum = min(minimum, point_footprint_distance(point, footprint))
        valid_returns += 1
    return {'observed_clearance_m': minimum if valid_returns else None,
            'valid_returns': valid_returns}


class OrderedPathTracker:
    """Project onto a bounded forward arc-length window, not all neighboring rows."""

    def __init__(self, points: list[tuple], search_distance: float = 1.0):
        if search_distance <= 0.0 or not math.isfinite(search_distance):
            raise ValueError('Search distance must be finite and positive')
        self.segments = []
        self.progress = 0.0
        self.length = 0.0
        self.search_distance = search_distance
        if any(not all(math.isfinite(value) for value in point) for point in points):
            raise ValueError('Path coordinates must be finite')
        for index, (start, end) in enumerate(zip(points, points[1:])):
            delta_x, delta_y = end[0] - start[0], end[1] - start[1]
            length = math.hypot(delta_x, delta_y)
            if length <= 1e-9:
                continue
            self.segments.append((index, start, delta_x, delta_y, length, self.length))
            self.length += length

    def update(self, position_x: float, position_y: float, yaw: float) -> dict | None:
        if not all(math.isfinite(value) for value in (position_x, position_y, yaw)):
            return None
        best = None
        window_start = max(0.0, self.progress - 0.10)
        window_end = self.progress + self.search_distance
        for index, start, delta_x, delta_y, length, offset in self.segments:
            if offset + length < window_start:
                continue
            if offset > window_end:
                break
            fraction = (
                (position_x - start[0]) * delta_x + (position_y - start[1]) * delta_y
            ) / length ** 2
            fraction = max(max(0.0, (window_start - offset) / length), min(
                fraction, min(1.0, (window_end - offset) / length),
            ))
            error_x = position_x - start[0] - fraction * delta_x
            error_y = position_y - start[1] - fraction * delta_y
            distance = math.hypot(error_x, error_y)
            if best is None or distance < best['path_distance_m']:
                heading = yaw - math.atan2(delta_y, delta_x)
                best = {
                    'segment': index,
                    'progress_m': offset + fraction * length,
                    'path_distance_m': distance,
                    'cross_track_m': (delta_x * error_y - delta_y * error_x) / length,
                    'heading_error_rad': math.atan2(math.sin(heading), math.cos(heading)),
                }
        if best is not None:
            best['tracking_valid'] = best['path_distance_m'] <= 0.50
            if best['tracking_valid']:
                self.progress = max(self.progress, best['progress_m'])
        return best