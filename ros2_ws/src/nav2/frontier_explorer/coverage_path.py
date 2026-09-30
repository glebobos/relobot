"""Conservative coverage geometry checks against the original occupancy grid."""

from __future__ import annotations

import math
import time
from copy import deepcopy

import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def path_from_xyyaw(points, header):
    path = Path(header=deepcopy(header))
    for position_x, position_y, yaw in points:
        pose = PoseStamped(header=deepcopy(header))
        pose.pose.position.x, pose.pose.position.y = float(position_x), float(position_y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        path.poses.append(pose)
    return path


def join_forward_paths(ingress, route, min_radius, validate):
    from _coverage_geometry import dubins
    if not ingress.poses or not route.poses or ingress.header.frame_id != route.header.frame_id:
        raise ValueError('Ingress planner returned an empty path or the wrong frame')
    target = pose_xy_yaw(route.poses[0].pose)
    endpoint = pose_xy_yaw(ingress.poses[-1].pose)
    if math.hypot(endpoint[0] - target[0], endpoint[1] - target[1]) > 0.10:
        raise ValueError('Ingress endpoint is too far from the requested coverage start')
    removed = 0.0
    last_attempt = -1.0
    for index in range(len(ingress.poses) - 1, -1, -1):
        start = pose_xy_yaw(ingress.poses[index].pose)
        if index < len(ingress.poses) - 1:
            following = pose_xy_yaw(ingress.poses[index + 1].pose)
            removed += math.hypot(following[0] - start[0], following[1] - start[1])
        if removed > 1.5:
            break
        exact = math.hypot(start[0] - target[0], start[1] - target[1]) < 1e-6 and abs(
            angle_difference(start[2], target[2])) < 1e-6
        if not exact and (removed < 0.25 or removed - last_attempt < 0.15):
            continue
        last_attempt = removed
        candidate = deepcopy(ingress)
        candidate.poses = candidate.poses[:index + 1]
        try:
            if not exact:
                connector = path_from_xyyaw(dubins(start, target, min_radius * 1.25, 0.02), route.header)
                if validate_forward_path(connector, min_radius) > removed + 0.5:
                    continue
                candidate.poses.extend(connector.poses[1:])
            candidate.poses.extend(deepcopy(route.poses))
            validate_forward_path(candidate, min_radius)
            validate(candidate)
            return candidate
        except ValueError:
            continue
    raise ValueError('No collision-free forward connector reaches the exact coverage start')


def pose_xy_yaw(pose):
    position, rotation = pose.position, pose.orientation
    values = (position.x, position.y, rotation.x, rotation.y, rotation.z, rotation.w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Non-finite path pose')
    norm = sum(value * value for value in values[2:])
    if abs(norm - 1.0) > 0.01 or abs(rotation.x) > 0.01 or abs(rotation.y) > 0.01:
        raise ValueError('Path needs normalized planar orientations')
    return position.x, position.y, math.atan2(
        2.0 * rotation.w * rotation.z, 1.0 - 2.0 * rotation.z ** 2)


def angle_difference(first, second):
    return math.atan2(math.sin(first - second), math.cos(first - second))


def directional_coverage(points, header, validator, min_radius, spacing=0.24, holes=(), canceled=lambda: False):
    from _coverage_geometry import dubins
    from shapely.affinity import rotate
    from shapely.geometry import LineString, Polygon
    from shapely.geometry.polygon import orient

    field = Polygon(points, holes)
    if not field.is_valid or field.is_empty or not math.isfinite(spacing) or spacing <= 0.0:
        raise ValueError('Coverage needs a valid polygon and positive swath spacing')
    if field.area > 400.0 or max(field.bounds[2] - field.bounds[0], field.bounds[3] - field.bounds[1]) > 50.0:
        raise ValueError('Split large fields into zones below 400 square metres and 50 metres across')
    deadline = time.monotonic() + 20.0

    def check_budget():
        if canceled() or time.monotonic() > deadline:
            raise ValueError('Coverage layout canceled or exceeded its planning time budget')
    rectangle = list(field.minimum_rotated_rectangle.exterior.coords)
    longest = max(zip(rectangle, rectangle[1:]), key=lambda edge: math.dist(*edge))
    angle = math.atan2(longest[1][1] - longest[0][1], longest[1][0] - longest[0][0]) % math.pi
    local = rotate(field, -angle, origin=(0, 0), use_radians=True)
    lateral = float(np.abs(validator.footprint[:, 1]).max())
    longitudinal = float(np.abs(validator.footprint[:, 0]).max())
    padding = validator.clearance + 3.0 * validator.resolution
    radius = min_radius * 1.25
    side_margin = math.hypot(radius + lateral, longitudinal) - radius + padding
    inner = local.buffer(-side_margin, join_style=2)
    if inner.is_empty or inner.geom_type != 'Polygon':
        raise ValueError('Zone has no connected interior after body clearance')
    minimum_x, minimum_y, maximum_x, maximum_y = inner.bounds
    count = math.ceil((maximum_y - minimum_y) / spacing) + 1
    if count < 4 or count > 120:
        raise ValueError('Zone needs 4 to 120 feasible rows; resize or split the selected zone')
    offsets = np.linspace(minimum_y + 1e-4, maximum_y - 1e-4, count)
    actual_spacing = float(offsets[1] - offsets[0])
    stride = math.ceil(2.0 * radius / actual_spacing)
    end_margin = math.hypot(longitudinal, stride * actual_spacing / 2.0 + lateral) + padding
    rows = []
    for offset in offsets:
        section = inner.intersection(LineString([(minimum_x - 1, offset), (maximum_x + 1, offset)]))
        if section.geom_type != 'LineString' or section.is_empty:
            raise ValueError('Disconnected rows need separate coverage zones')
        begin, _, end, _ = section.bounds
        begin += end_margin - side_margin
        end -= end_margin - side_margin
        if end - begin < 0.20:
            raise ValueError('A row has insufficient length after reserving forward-turn space')
        rows.append((begin, end, float(offset)))

    cosine, sine = math.cos(angle), math.sin(angle)

    def world_pose(position_x, position_y, yaw):
        return (position_x * cosine - position_y * sine,
                position_x * sine + position_y * cosine, yaw + angle)

    half = len(rows) // 2
    if len(rows) % 2:
        ordered = [(index * half) % len(rows) for index in range(len(rows))]
    else:
        ordered = [index for lower in range(half - 1, -1, -1) for index in (lower, lower + half)]
    if any(abs(first - second) < stride for first, second in zip(ordered, ordered[1:])):
        raise ValueError('Zone is too narrow to separate all forward row turns by two turning radii')
    poses = []
    for position, index in enumerate(ordered):
        check_budget()
        begin, end, offset = rows[index]
        if position % 2:
            begin, end = end, begin
        yaw = 0.0 if end > begin else math.pi
        row = [world_pose(float(distance), offset, yaw)
               for distance in np.linspace(begin, end, math.ceil(abs(end - begin) / 0.03) + 1)]
        if poses:
            poses.extend(dubins(poses[-1], row[0], radius, 0.02)[1:])
        poses.extend(row)
        if len(poses) > 15000:
            raise ValueError('Coverage route exceeds the preview point budget; split the zone')
    path = path_from_xyyaw(poses, header)
    validate_forward_path(path, min_radius)
    validator.check_path(path)

    border_radius = max(0.4, radius)
    border_margin = math.hypot(border_radius + lateral, longitudinal) - border_radius + padding
    core = local.buffer(-(border_margin + border_radius), join_style=2)
    if core.is_empty or core.geom_type != 'Polygon' or core.interiors:
        raise ValueError('Zone cannot fit a connected rounded perimeter pass')
    boundary = orient(core.buffer(border_radius, resolution=24), sign=1.0)
    corners = list(boundary.exterior.coords)[:-1]
    ring = []
    for index, current in enumerate(corners):
        previous, following = corners[index - 1], corners[(index + 1) % len(corners)]
        incoming = math.atan2(current[1] - previous[1], current[0] - previous[0])
        outgoing = math.atan2(following[1] - current[1], following[0] - current[0])
        ring.append(world_pose(*current, incoming + angle_difference(outgoing, incoming) / 2.0))
    candidates = []
    for index in range(0, len(ring), max(1, len(ring) // 24)):
        connector = path_from_xyyaw(dubins(poses[-1], ring[index], radius, 0.02), header)
        try:
            length = validate_forward_path(connector, min_radius)
            candidates.append((length, index, connector))
        except ValueError:
            continue
    for _, index, connector in sorted(candidates, key=lambda candidate: candidate[0]):
        check_budget()
        candidate = deepcopy(path)
        candidate.poses.extend(connector.poses[1:])
        lap = ring[index:] + ring[:index] + [ring[index]]
        dense = []
        for start, end in zip(lap, lap[1:]):
            for fraction in np.linspace(0.0, 1.0, max(2, math.ceil(math.dist(start[:2], end[:2]) / 0.03) + 1))[:-1]:
                dense.append((start[0] + fraction * (end[0] - start[0]),
                              start[1] + fraction * (end[1] - start[1]),
                              start[2] + fraction * angle_difference(end[2], start[2])))
        dense.append(lap[-1])
        candidate.poses.extend(path_from_xyyaw(dense, header).poses)
        try:
            length = validate_forward_path(candidate, min_radius)
            validator.check_path(candidate)
            return candidate, dict(swath_count=len(rows), perimeter_passes=1,
                                   side_margin_m=side_margin, row_end_margin_m=end_margin,
                                   perimeter_margin_m=border_margin, path_length_m=length)
        except ValueError:
            continue
    raise ValueError('No validated forward connection reaches the rounded perimeter pass')


def validate_headland_bounds(points, headland_width):
    polygon = np.asarray(points, dtype=float)
    if (polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3
            or not np.isfinite(polygon).all()):
        raise ValueError('Coverage polygon must contain at least three finite points')
    if not math.isfinite(headland_width) or headland_width < 0.0:
        raise ValueError('Headland width must be finite and nonnegative')
    spans = sorted(cv2.minAreaRect(polygon.astype(np.float32))[1])
    if spans[0] <= 2.0 * headland_width + 1e-6:
        raise ValueError(
            f'Zone is too narrow for a {headland_width:.2f} m headland on each side: '
            f'enclosing rectangle {spans[0]:.2f} x {spans[1]:.2f} m. '
            f'Both dimensions must exceed {2.0 * headland_width:.2f} m just to leave an interior; '
            'turns need additional space. Draw a larger mapped zone.')


def validate_forward_path(path, min_radius):
    if not path.header.frame_id or len(path.poses) < 2:
        raise ValueError('Coverage path is empty or has no frame')
    if not math.isfinite(min_radius) or min_radius <= 0.0:
        raise ValueError('Minimum radius must be positive')
    poses = []
    for stamped in path.poses:
        if stamped.header.frame_id and stamped.header.frame_id != path.header.frame_id:
            raise ValueError('Path mixes coordinate frames')
        poses.append(pose_xy_yaw(stamped.pose))
    total = 0.0
    for start, end in zip(poses, poses[1:]):
        delta_x, delta_y = end[0] - start[0], end[1] - start[1]
        distance = math.hypot(delta_x, delta_y)
        turn = angle_difference(end[2], start[2])
        if distance < 1e-6:
            if abs(turn) > 0.01:
                raise ValueError('Path contains a stationary turn')
            continue
        if abs(turn) / distance > 1.0 / min_radius * 1.02:
            raise ValueError('Path curvature exceeds the configured turning limit')
        tangent = math.atan2(delta_y, delta_x)
        midpoint_heading = start[2] + turn / 2.0
        if abs(angle_difference(tangent, midpoint_heading)) > 0.15:
            raise ValueError(f'Path contains reverse motion or a discontinuous connector: {start} -> {end}')
        total += distance
    if total < 0.05:
        raise ValueError('Coverage path is too short')
    return total


class FreeSpaceValidator:
    def __init__(self, grid, footprint, clearance, zone=None, occupied_threshold=1):
        self.frame = grid.header.frame_id
        self.resolution = grid.info.resolution
        self.width, self.height = grid.info.width, grid.info.height
        if (not self.frame or not math.isfinite(self.resolution) or self.resolution <= 0.0
                or self.width < 2 or self.height < 2
                or len(grid.data) != self.width * self.height):
            raise ValueError('Invalid occupancy grid')
        if not math.isfinite(clearance) or clearance < 0.0:
            raise ValueError('Invalid clearance')
        self.clearance = clearance
        self.origin = pose_xy_yaw(grid.info.origin)
        self.footprint = np.asarray(footprint, dtype=float)
        if (self.footprint.ndim != 2 or self.footprint.shape[1] != 2
                or len(self.footprint) < 3 or not np.isfinite(self.footprint).all()):
            raise ValueError('Invalid robot footprint')
        self.radius = float(np.linalg.norm(self.footprint, axis=1).max())
        occupancy = np.asarray(grid.data, dtype=np.int16).reshape(self.height, self.width)
        free = np.uint8((occupancy >= 0) & (occupancy < occupied_threshold))
        if zone:
            mask = np.zeros_like(free)
            polygon = self.world_to_cells(np.asarray(zone, dtype=float))
            cv2.fillPoly(mask, [polygon], 1)
            free &= mask
        padded = np.pad(free, 1, constant_values=0)
        distances = cv2.distanceTransform(padded, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]
        self.safe = distances * self.resolution >= clearance + math.sqrt(2.0) * self.resolution

    def world_to_cells(self, points):
        delta = points - np.asarray(self.origin[:2])
        cosine, sine = math.cos(self.origin[2]), math.sin(self.origin[2])
        local = delta @ np.asarray([[cosine, -sine], [sine, cosine]])
        return np.floor(local / self.resolution).astype(np.int32)

    def check_pose(self, position_x, position_y, yaw):
        cosine, sine = math.cos(yaw), math.sin(yaw)
        points = self.footprint @ np.asarray([[cosine, sine], [-sine, cosine]])
        cells = self.world_to_cells(points + np.asarray([position_x, position_y]))
        minimum, maximum = cells.min(axis=0), cells.max(axis=0)
        if (minimum < 0).any() or maximum[0] >= self.width or maximum[1] >= self.height:
            raise ValueError('Swept footprint leaves the known map')
        mask = np.zeros((maximum[1] - minimum[1] + 1, maximum[0] - minimum[0] + 1), np.uint8)
        cv2.fillPoly(mask, [cells - minimum], 1)
        available = self.safe[minimum[1]:maximum[1] + 1, minimum[0]:maximum[0] + 1]
        if not available[mask.astype(bool)].all():
            raise ValueError('Swept footprint violates obstacle, unknown-space or zone clearance '
                             f'at ({position_x:.3f}, {position_y:.3f}, {yaw:.3f})')

    def check_path(self, path, transform=(0.0, 0.0, 0.0)):
        poses = [pose_xy_yaw(stamped.pose) for stamped in path.poses]
        if not poses:
            raise ValueError('Empty path')
        cosine, sine = math.cos(transform[2]), math.sin(transform[2])
        previous = poses[0]
        for current in poses:
            distance = math.hypot(current[0] - previous[0], current[1] - previous[1])
            turn = angle_difference(current[2], previous[2])
            count = max(1, math.ceil((distance + self.radius * abs(turn)) / (self.resolution / 2.0)))
            for step in range(count + 1):
                fraction = step / count
                position_x = previous[0] + fraction * (current[0] - previous[0])
                position_y = previous[1] + fraction * (current[1] - previous[1])
                self.check_pose(transform[0] + cosine * position_x - sine * position_y,
                                transform[1] + sine * position_x + cosine * position_y,
                                previous[2] + fraction * turn + transform[2])
            previous = current