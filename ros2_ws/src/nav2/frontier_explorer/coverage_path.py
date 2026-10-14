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


def work_paths(path, sections=None):
    if sections is None:
        return [path]
    result = []
    next_index = 0
    for section in sections:
        if (not isinstance(section, (list, tuple)) or len(section) != 2
            or any(type(index) is not int for index in section)
                or section[0] != next_index or not section[0] < section[1] < len(path.poses)):
            raise ValueError('Invalid coverage work-section boundaries')
        start, end = section
        result.append(Path(header=deepcopy(path.header), poses=path.poses[start:end + 1]))
        next_index = end + 1
    if not result or next_index != len(path.poses):
        raise ValueError('Coverage work sections must include every pose')
    return result


def path_from_swaths(coverage, max_path_points=250000):
    if not coverage.swaths_ordered or not coverage.swaths:
        raise ValueError('Coverage server must return ordered work swaths')
    path = Path(header=deepcopy(coverage.header))
    sections = []
    for swath in coverage.swaths:
        delta_x, delta_y = swath.end.x - swath.start.x, swath.end.y - swath.start.y
        distance = math.hypot(delta_x, delta_y)
        if not math.isfinite(distance) or distance < 0.05:
            raise ValueError('Coverage server returned an invalid work swath')
        count = math.ceil(distance / 0.03) + 1
        if len(path.poses) + count > max_path_points:
            raise ValueError('Coverage exceeds its path point budget')
        yaw = math.atan2(delta_y, delta_x)
        work = path_from_xyyaw([(swath.start.x + fraction * delta_x,
                                swath.start.y + fraction * delta_y, yaw)
                               for fraction in np.linspace(0.0, 1.0, count)], path.header)
        start = len(path.poses)
        path.poses.extend(work.poses)
        sections.append([start, len(path.poses) - 1])
    return path, sections


class CoveragePlanningLimit(ValueError):
    pass


def split_forward_segments(points, header, validator, min_radius, canceled=lambda: False,
                           split_on_turn=False, max_length=math.inf):
    if (len(points) > 3 and math.dist(points[0][:2], points[-1][:2]) < 1e-6
            and abs(angle_difference(points[0][2], points[-1][2])) < 1e-6):
        middle = len(points) // 2
        first, omitted_first = split_forward_segments(
            points[:middle + 1], header, validator, min_radius, canceled, split_on_turn, max_length)
        second, omitted_second = split_forward_segments(
            points[middle:], header, validator, min_radius, canceled, split_on_turn, max_length)
        return first + second, omitted_first + omitted_second
    segments = []
    current = []
    current_length = 0.0
    current_turn = None
    omitted_length = 0.0

    def finish():
        nonlocal current, current_length, omitted_length
        if current_length >= 0.20:
            segments.append(current)
        else:
            omitted_length += current_length
        current, current_length = [], 0.0

    for start, end in zip(points, points[1:]):
        if canceled():
            raise CoveragePlanningLimit('Coverage segmentation canceled')
        distance = math.dist(start[:2], end[:2])
        turning = abs(angle_difference(end[2], start[2])) > 1e-4
        pair = path_from_xyyaw([start, end], header)
        try:
            validate_forward_path(pair, min_radius, minimum_length=0.0)
            validator.check_path(pair)
        except ValueError:
            finish()
            omitted_length += distance
            current_turn = None
            continue
        if (current and current_length >= 0.20
                and (current_length + distance > max_length
                     or split_on_turn and turning != current_turn)):
            finish()
        if not current:
            current = [start]
            current_turn = turning
        current.append(end)
        current_length += distance
    finish()
    return segments, omitted_length


def directional_coverage(points, header, validator, min_radius, spacing=0.24, holes=(),
                         canceled=lambda: False, planning_timeout=120.0,
                         max_path_points=250000, open_segments=False, work_segment_length=6.0):
    from shapely.affinity import rotate
    from shapely.geometry import LineString, MultiLineString, Polygon
    from shapely.geometry.polygon import orient

    field = Polygon(points, holes)
    if (not field.is_valid or field.is_empty or not math.isfinite(spacing) or spacing <= 0.0
            or not math.isfinite(min_radius) or min_radius <= 0.0
            or not math.isfinite(planning_timeout) or planning_timeout <= 0.0
            or not math.isfinite(work_segment_length) or work_segment_length < 0.20
            or max_path_points < 2):
        raise ValueError('Coverage needs a valid polygon and positive swath spacing')
    deadline = time.monotonic() + planning_timeout
    started = time.monotonic()

    def check_budget():
        if canceled() or time.monotonic() > deadline:
            raise CoveragePlanningLimit('Coverage layout canceled or exceeded its planning time budget')

    def checked(poses):
        check_budget()
        candidate = path_from_xyyaw(poses, header)
        length = validate_forward_path(candidate, min_radius)
        validator.check_path(candidate, canceled=check_budget)
        return length

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
    if inner.is_empty:
        raise ValueError('Zone has no interior after body clearance')
    minimum_x, minimum_y, maximum_x, maximum_y = inner.bounds
    count = max(2, math.ceil((maximum_y - minimum_y) / spacing) + 1)
    if count > max_path_points:
        raise CoveragePlanningLimit('Coverage exceeds its path point budget')
    offsets = np.linspace(minimum_y + 1e-4, maximum_y - 1e-4, count)
    actual_spacing = float(offsets[1] - offsets[0])
    stride = math.ceil(2.0 * radius / actual_spacing)
    end_margin = math.hypot(longitudinal, stride * actual_spacing / 2.0 + lateral) + padding
    rows = []
    short_sections = 0
    for offset in offsets:
        check_budget()
        section = inner.intersection(LineString([(minimum_x - 1, offset), (maximum_x + 1, offset)]))
        sections = [section] if section.geom_type == 'LineString' else list(getattr(section, 'geoms', ()))
        row = []
        for part in sections:
            if part.is_empty or part.geom_type != 'LineString':
                continue
            begin, _, end, _ = part.bounds
            begin += max(0.0, end_margin - side_margin)
            end -= max(0.0, end_margin - side_margin)
            if end - begin < 0.20:
                short_sections += 1
                continue
            row.append((begin, end, float(offset)))
        rows.append(sorted(row))

    cosine, sine = math.cos(angle), math.sin(angle)

    def world_pose(position_x, position_y, yaw):
        return (position_x * cosine - position_y * sine,
                position_x * sine + position_y * cosine, yaw + angle)

    border_radius = max(0.4, radius)
    border_margin = math.hypot(border_radius + lateral, longitudinal) - border_radius + padding
    core = local.buffer(-(border_margin + border_radius), join_style=2)
    boundaries = []
    components = [core] if core.geom_type == 'Polygon' else list(getattr(core, 'geoms', ()))
    for component in components:
        if not component.is_empty and component.geom_type == 'Polygon':
            boundaries.append(orient(Polygon(component.exterior).buffer(border_radius, resolution=24), sign=1.0))
    for interior in local.interiors:
        expanded = Polygon(interior).buffer(max(0.0, border_margin - border_radius), join_style=2)
        boundaries.append(orient(expanded.buffer(border_radius, resolution=24), sign=-1.0))
    rings = []
    edge_segments = []
    omitted_boundaries = set()
    omitted_work_length = 0.0
    for boundary_index, boundary in enumerate(boundaries):
        corners = list(boundary.exterior.coords)[:-1]
        ring = []
        for index, current in enumerate(corners):
            previous, following = corners[index - 1], corners[(index + 1) % len(corners)]
            incoming = math.atan2(current[1] - previous[1], current[0] - previous[0])
            outgoing = math.atan2(following[1] - current[1], following[0] - current[0])
            ring.append(world_pose(*current, incoming + angle_difference(outgoing, incoming) / 2.0))
        dense = []
        for start, end in zip(ring, ring[1:] + ring[:1]):
            for fraction in np.linspace(0.0, 1.0, max(2, math.ceil(math.dist(start[:2], end[:2]) / 0.03) + 1))[:-1]:
                dense.append((start[0] + fraction * (end[0] - start[0]),
                              start[1] + fraction * (end[1] - start[1]),
                              start[2] + fraction * angle_difference(end[2], start[2])))
        if dense:
            try:
                checked(dense + dense[:1])
                rings.append(dense)
            except ValueError:
                check_budget()
            if open_segments:
                segments, omitted = split_forward_segments(
                    dense + dense[:1], header, validator, min_radius,
                    lambda: check_budget() or False, split_on_turn=True, max_length=work_segment_length)
                edge_segments.extend((segment, boundary_index) for segment in segments)
                omitted_work_length += omitted
                if omitted > 1e-6:
                    omitted_boundaries.add(boundary_index)

    poses = []
    work_sections = []
    swath_count = 0
    edge_segment_count = 0
    unconnected_segment_count = 0
    visited_boundaries = set()

    def append(segment):
        if len(poses) + len(segment) > max_path_points:
            raise CoveragePlanningLimit('Coverage exceeds its path point budget; no partial route was accepted')
        poses.extend(segment)

    def append_work(segment):
        if (math.dist(segment[0][:2], segment[-1][:2]) < 1e-6
                and abs(angle_difference(segment[0][2], segment[-1][2])) < 1e-6):
            parts, omitted = split_forward_segments(
                segment, header, validator, min_radius, lambda: check_budget() or False,
                max_length=work_segment_length)
            if omitted > 1e-6 or not parts:
                raise ValueError('Closed work section cannot be split without losing work')
            return all(append_work(part) for part in parts)
        for candidate in (segment, [(position_x, position_y, yaw + math.pi)
                                    for position_x, position_y, yaw in reversed(segment)]):
            try:
                checked(candidate)
            except CoveragePlanningLimit:
                raise
            except ValueError:
                check_budget()
                continue
            start = len(poses)
            append(candidate)
            work_sections.append([start, len(poses) - 1])
            return True
        return False

    for position, row_sections in enumerate(rows):
        check_budget()
        sections = row_sections if position % 2 == 0 else list(reversed(row_sections))
        for begin, end, offset in sections:
            if position % 2:
                begin, end = end, begin
            yaw = 0.0 if end > begin else math.pi
            row = [world_pose(float(distance), offset, yaw)
                   for distance in np.linspace(begin, end, math.ceil(abs(end - begin) / 0.03) + 1)]
            if open_segments:
                segments, omitted = split_forward_segments(
                    row, header, validator, min_radius, lambda: check_budget() or False,
                    max_length=work_segment_length)
                omitted_work_length += omitted
                for segment in segments:
                    if append_work(segment):
                        swath_count += 1
                    else:
                        omitted_work_length += sum(math.dist(start[:2], end[:2])
                                                   for start, end in zip(segment, segment[1:]))
                continue
            checked(row)
            append_work(row)
            swath_count += 1
    if not poses and not open_segments:
        raise ValueError('No swaths fit the footprint and forward-turn constraints')
    if open_segments:
        pending = list(edge_segments)
        while pending:
            check_budget()
            if poses:
                pending.sort(key=lambda item: min(math.dist(poses[-1][:2], item[0][0][:2]),
                                                  math.dist(poses[-1][:2], item[0][-1][:2])))
            segment, boundary_index = pending.pop(0)
            if append_work(segment):
                edge_segment_count += 1
                visited_boundaries.add(boundary_index)
            else:
                omitted_boundaries.add(boundary_index)
                omitted_work_length += sum(math.dist(start[:2], end[:2])
                                           for start, end in zip(segment, segment[1:]))
        if not poses:
            raise ValueError('No safe work segment satisfies the footprint and forward-motion constraints')
    for ring in (() if open_segments else rings):
        append_work(ring + ring[:1])
    length = sum(checked(poses[start:end + 1]) for start, end in work_sections)
    geometry = MultiLineString([[pose[:2] for pose in poses[start:end + 1]]
                                for start, end in work_sections])
    strip = geometry.simplify(validator.resolution / 4.0).buffer(
        spacing / 2.0, resolution=4)
    estimated_covered_area = strip.intersection(field).area
    check_budget()
    return path_from_xyyaw(poses, header), dict(
        swath_count=swath_count, perimeter_passes=len(visited_boundaries) if open_segments else len(rings),
        edge_segment_count=edge_segment_count, work_segment_count=len(work_sections),
        closed_perimeter_count=0 if open_segments else len(rings),
        unconnected_segment_count=unconnected_segment_count, omitted_work_length_m=omitted_work_length,
        work_sections=work_sections,
        open_segments=open_segments, connector_count=0,
        obstacle_detours=0, side_margin_m=side_margin, row_end_margin_m=end_margin,
        perimeter_margin_m=border_margin, path_length_m=length,
        short_section_count=short_sections,
        omitted_perimeter_count=len(omitted_boundaries) if open_segments else len(boundaries) - len(rings),
        estimated_covered_area_m2=estimated_covered_area,
        estimated_uncovered_area_m2=max(0.0, field.area - estimated_covered_area),
        planning_time_sec=time.monotonic() - started)


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


def validate_forward_path(path, min_radius, minimum_length=0.05):
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
            raise ValueError(f'Path curvature exceeds the configured turning limit: {start} -> {end}; '
                             f'distance={distance:.6f}, turn={turn:.6f}')
        tangent = math.atan2(delta_y, delta_x)
        midpoint_heading = start[2] + turn / 2.0
        alignment = abs(angle_difference(tangent, midpoint_heading))
        if alignment > 0.15:
            raise ValueError(f'Path contains reverse motion or a discontinuous connector: {start} -> {end}')
        total += distance
    if total < minimum_length:
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

    def check_path(self, path, transform=(0.0, 0.0, 0.0), canceled=None):
        poses = [pose_xy_yaw(stamped.pose) for stamped in path.poses]
        if not poses:
            raise ValueError('Empty path')
        cosine, sine = math.cos(transform[2]), math.sin(transform[2])
        previous = poses[0]
        for index, current in enumerate(poses):
            if canceled is not None and index % 64 == 0:
                canceled()
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