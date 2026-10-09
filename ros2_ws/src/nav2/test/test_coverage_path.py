import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import pytest

from frontier_explorer.coverage_path import FreeSpaceValidator, validate_forward_path, validate_headland_bounds, work_paths


def make_path(points):
    path = Path()
    path.header.frame_id = 'map'
    for position_x, position_y, yaw in points:
        pose = PoseStamped()
        pose.pose.position.x = float(position_x)
        pose.pose.position.y = float(position_y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        path.poses.append(pose)
    return path


def make_grid():
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 100
    grid.info.resolution = 0.05
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * 10000
    return grid


FOOTPRINT = [(-0.1, -0.245), (0.51, -0.245), (0.51, 0.175), (-0.1, 0.175)]


@pytest.mark.parametrize('sections', [[], [[0, 0], [1, 3]], [[0, 1], [3, 3]],
                                      [[0, 1], [1, 3]], [[0, 4]], [[0, 1]],
                                      [[False, 1], [2, 3]]])
def test_work_section_boundaries_reject_gaps_overlaps_and_single_poses(sections):
    from frontier_explorer.coverage_path import work_paths
    path = make_path([(1, 1, 0), (2, 1, 0), (3, 2, math.pi), (2, 2, math.pi)])
    with pytest.raises(ValueError, match='work.section'):
        work_paths(path, sections)


def test_legacy_swaths_keep_forward_work_and_exclude_connectors():
    from types import SimpleNamespace
    from geometry_msgs.msg import Point32
    from frontier_explorer.coverage_path import path_from_swaths
    coverage = SimpleNamespace(header=make_path([(1, 1, 0)]).header, swaths_ordered=True,
        swaths=[SimpleNamespace(start=Point32(x=1.0, y=1.0), end=Point32(x=2.0, y=1.0)),
                SimpleNamespace(start=Point32(x=2.0, y=2.0), end=Point32(x=1.0, y=2.0))])
    path, sections = path_from_swaths(coverage)
    assert len(sections) == 2
    assert sum(validate_forward_path(work, 0.20) for work in work_paths(path, sections)) == pytest.approx(2.0)
    assert validate_forward_path(path, 0.20) == pytest.approx(3.0)
    with pytest.raises(ValueError, match='point budget'):
        path_from_swaths(coverage, max_path_points=10)
    coverage.swaths_ordered = False
    with pytest.raises(ValueError, match='ordered'):
        path_from_swaths(coverage)


@pytest.mark.parametrize('angle', [0.0, math.pi / 4.0])
@pytest.mark.parametrize('width', [2.7, 3.0])
def test_headland_rejects_collapsed_interior_in_any_orientation(angle, width):
    polygon = [(0, 0), (width, 0), (width, 4.45), (0, 4.45), (0, 0)]
    rotated = [(position_x * math.cos(angle) - position_y * math.sin(angle),
                position_x * math.sin(angle) + position_y * math.cos(angle))
               for position_x, position_y in polygon]
    with pytest.raises(ValueError, match='Both dimensions must exceed 3.00 m'):
        validate_headland_bounds(rotated, 1.5)


def test_headland_bounds_allow_large_zone():
    validate_headland_bounds([(0, 0), (8, 0), (8, 8), (0, 8)], 1.5)


@pytest.mark.parametrize('width,height', [(2.7, 4.45), (3.65, 5.80), (8.0, 8.0)])
@pytest.mark.parametrize('angle', [0.0, 0.4])
def test_directional_rows_and_perimeter_use_zone_with_body_clearance(width, height, angle):
    from frontier_explorer.coverage_path import directional_coverage
    from shapely.affinity import rotate
    from shapely.geometry import Polygon
    grid = make_grid()
    grid.info.width = grid.info.height = 280
    grid.info.origin.position.x = grid.info.origin.position.y = -2.0
    grid.data = [0] * (280 * 280)
    zone = [(1, 1), (1 + width, 1), (1 + width, 1 + height), (1, 1 + height)]
    zone = list(rotate(Polygon(zone), angle, use_radians=True).exterior.coords)
    validator = FreeSpaceValidator(grid, FOOTPRINT, 0.25, zone)
    path, details = directional_coverage(zone, grid.header, validator, 0.20)
    assert details['swath_count'] >= 6
    assert details['perimeter_passes'] == 1
    assert details['side_margin_m'] < 0.90
    assert details['row_end_margin_m'] < 1.25
    assert all(math.dist((work.poses[0].pose.position.x, work.poses[0].pose.position.y),
                        (work.poses[-1].pose.position.x, work.poses[-1].pose.position.y)) > 0.10
               for work in work_paths(path, details['work_sections']))
    assert sum(validate_forward_path(work, 0.20) for work in work_paths(path, details['work_sections'])) > 10.0
    for work in work_paths(path, details['work_sections']):
        validator.check_path(work)


def test_directional_coverage_handles_a_central_obstacle():
    from frontier_explorer.coverage_path import directional_coverage
    grid = make_grid()
    grid.info.width = grid.info.height = 220
    grid.data = [0] * (220 * 220)
    for row in range(90, 110):
        for column in range(90, 110):
            grid.data[row * 220 + column] = 100
    zone = [(1, 1), (9, 1), (9, 9), (1, 9)]
    holes = [[(4.5, 4.5), (5.5, 4.5), (5.5, 5.5), (4.5, 5.5)]]
    validator = FreeSpaceValidator(grid, FOOTPRINT, 0.25, zone)
    path, details = directional_coverage(zone, grid.header, validator, 0.20, holes=holes)
    assert details['swath_count'] > 20
    assert 0.0 < details['estimated_covered_area_m2'] < 63.0
    assert details['estimated_uncovered_area_m2'] > 0.0
    assert sum(validate_forward_path(work, 0.20) for work in work_paths(path, details['work_sections'])) > 100.0
    for work in work_paths(path, details['work_sections']):
        validator.check_path(work)


def test_continuous_forward_arc():
    path = make_path([(2.0 + 0.3 * math.sin(angle), 2.0 + 0.3 * (1.0 - math.cos(angle)), angle)
                      for angle in [index * 0.05 for index in range(32)]])
    assert validate_forward_path(path, 0.2) > 0.4
    FreeSpaceValidator(make_grid(), FOOTPRINT, 0.2).check_path(path)


@pytest.mark.parametrize('occupancy', [100, -1])
def test_open_segments_preserve_safe_parts_on_both_sides_of_obstacle(occupancy):
    from frontier_explorer.coverage_path import split_forward_segments
    grid = make_grid()
    grid.data[40 * 100 + 50] = occupancy
    validator = FreeSpaceValidator(grid, FOOTPRINT, 0.25)
    points = [(0.5 + index * 0.025, 2.0, 0.0) for index in range(145)]
    segments, omitted_length = split_forward_segments(points, grid.header, validator, 0.20)
    assert len(segments) == 2
    assert omitted_length > 0.5
    assert segments[0][-1][0] < 2.5 < segments[1][0][0]
    for segment in segments:
        path = make_path(segment)
        assert validate_forward_path(path, 0.20) >= 0.20
        validator.check_path(path)


def test_open_coverage_keeps_rows_without_a_closed_perimeter():
    from frontier_explorer.coverage_path import directional_coverage
    grid = make_grid()
    grid.info.width = grid.info.height = 220
    grid.data = [0] * (220 * 220)
    zone = [(1, 1), (9, 1), (9, 9), (1, 9)]
    validator = FreeSpaceValidator(grid, FOOTPRINT, 0.25, zone)
    path, details = directional_coverage(zone, grid.header, validator, 0.20, open_segments=True)
    assert details['closed_perimeter_count'] == 0
    assert details['edge_segment_count'] >= 4
    assert details['work_segment_count'] >= details['swath_count'] + details['edge_segment_count']
    assert sum(validate_forward_path(work, 0.20) for work in work_paths(path, details['work_sections'])) > 100.0
    for work in work_paths(path, details['work_sections']):
        validator.check_path(work)


def test_open_segments_limit_length_without_stopping_or_losing_the_join():
    from frontier_explorer.coverage_path import split_forward_segments
    grid = make_grid()
    validator = FreeSpaceValidator(grid, FOOTPRINT, 0.25)
    points = [(0.5 + index * 0.025, 2.0, 0.0) for index in range(121)]
    segments, omitted = split_forward_segments(points, grid.header, validator, 0.20, max_length=1.25)
    assert len(segments) == 3
    assert omitted == pytest.approx(0.0)
    assert segments[0][0] == points[0]
    assert segments[-1][-1] == points[-1]
    for previous, following in zip(segments, segments[1:]):
        assert previous[-1] == following[0]
    combined = make_path([point for segment in segments for point in segment])
    assert validate_forward_path(combined, 0.20) == pytest.approx(3.0)
    validator.check_path(combined)


def test_open_segments_split_a_short_circle_without_omitting_a_lap():
    from frontier_explorer.coverage_path import split_forward_segments
    grid = make_grid()
    validator = FreeSpaceValidator(grid, FOOTPRINT, 0.25)
    points = [(2.5 + 0.4 * math.sin(index * math.pi / 60.0),
               2.5 + 0.4 * (1.0 - math.cos(index * math.pi / 60.0)), index * math.pi / 60.0)
              for index in range(121)]
    segments, omitted = split_forward_segments(points, grid.header, validator, 0.20,
                                              split_on_turn=True, max_length=6.0)
    assert len(segments) >= 2
    assert omitted == pytest.approx(0.0)
    assert all(math.dist(segment[0][:2], segment[-1][:2]) > 0.10 for segment in segments)
    assert segments[0][-1] == segments[1][0]
    path = make_path([point for segment in segments for point in segment])
    assert validate_forward_path(path, 0.20) == pytest.approx(validate_forward_path(make_path(points), 0.20))
    validator.check_path(path)


def test_open_coverage_preserves_work_without_forward_transits():
    from frontier_explorer.coverage_path import directional_coverage, pose_xy_yaw, angle_difference, work_paths

    class StraightOnlyValidator(FreeSpaceValidator):
        def check_path(self, path, **kwargs):
            heading = pose_xy_yaw(path.poses[0].pose)[2]
            if any(abs(angle_difference(pose_xy_yaw(pose.pose)[2], heading)) > 0.01
                   for pose in path.poses):
                raise ValueError('Forward turning lane unavailable')
            return super().check_path(path, **kwargs)

    grid = make_grid()
    grid.info.width = grid.info.height = 220
    grid.data = [0] * (220 * 220)
    zone = [(1, 1), (9, 1), (9, 9), (1, 9)]
    validator = StraightOnlyValidator(grid, FOOTPRINT, 0.25, zone)
    path, details = directional_coverage(zone, grid.header, validator, 0.20, open_segments=True)
    assert details['swath_count'] >= 1
    assert details['swath_count'] > 20
    assert details['unconnected_segment_count'] == 0
    assert details['connector_count'] == 0
    assert len(details['work_sections']) == details['work_segment_count']
    assert details['estimated_uncovered_area_m2'] > 0.0
    sections = work_paths(path, details['work_sections'])
    length = 0.0
    for section in sections:
        length += validate_forward_path(section, 0.20)
        validator.check_path(section)
    assert length == pytest.approx(details['path_length_m'])


def test_open_coverage_never_masks_a_planning_resource_limit():
    from frontier_explorer.coverage_path import CoveragePlanningLimit, directional_coverage
    grid = make_grid()
    grid.info.width = grid.info.height = 220
    grid.data = [0] * (220 * 220)
    zone = [(1, 1), (9, 1), (9, 9), (1, 9)]
    validator = FreeSpaceValidator(grid, FOOTPRINT, 0.25, zone)
    calls = 0

    def canceled():
        nonlocal calls
        calls += 1
        return calls > 20

    with pytest.raises(CoveragePlanningLimit, match='canceled'):
        directional_coverage(zone, grid.header, validator, 0.20,
                             canceled=canceled, open_segments=True)


def test_full_map_selects_robot_component_not_largest_contour():
    from frontier_explorer.map_processor import MapProcessor
    grid = make_grid()
    for row in range(100):
        grid.data[row * 100 + 30] = -1
    regions, details = MapProcessor(None).extract_coverage_regions(grid, robot_position=(0.5, 0.5))
    assert len(regions) == 1
    assert max(point.x for point in regions[0][0].polygon.points) < 1.5
    assert details['excluded_area_m2'] == pytest.approx(17.25)


@pytest.mark.parametrize('width,height', [(30.0, 30.0), (60.0, 10.0), (36.0, 36.0)])
def test_directional_full_map_exceeds_old_area_span_and_row_limits(width, height):
    from frontier_explorer.coverage_path import directional_coverage
    grid = make_grid()
    grid.info.resolution = 0.1
    grid.info.width = round((width + 4.0) / 0.1)
    grid.info.height = round((height + 4.0) / 0.1)
    grid.data = [0] * (grid.info.width * grid.info.height)
    zone = [(1, 1), (1 + width, 1), (1 + width, 1 + height), (1, 1 + height)]
    validator = FreeSpaceValidator(grid, FOOTPRINT, 0.25, zone)
    path, details = directional_coverage(zone, grid.header, validator, 0.20)
    assert len(path.poses) > 15000
    assert details['swath_count'] >= 20
    assert details['perimeter_passes'] == 1
    assert sum(validate_forward_path(work, 0.20) for work in work_paths(path, details['work_sections'])) > width * height / 0.30


def test_directional_point_budget_never_returns_a_truncated_route():
    from frontier_explorer.coverage_path import directional_coverage
    grid = make_grid()
    zone = [(0.5, 0.5), (4.5, 0.5), (4.5, 4.5), (0.5, 4.5)]
    validator = FreeSpaceValidator(grid, FOOTPRINT, 0.25, zone)
    with pytest.raises(ValueError, match='point budget'):
        directional_coverage(zone, grid.header, validator, 0.20, max_path_points=200)


@pytest.mark.parametrize('occupancy', [100, -1])
def test_full_map_preserves_cell_sized_holes_and_rotated_origin(occupancy):
    from frontier_explorer.map_processor import MapProcessor
    grid = make_grid()
    grid.info.origin.orientation.z = math.sin(math.pi / 4.0)
    grid.info.origin.orientation.w = math.cos(math.pi / 4.0)
    grid.data[50 * 100 + 50] = occupancy
    regions, details = MapProcessor(None).extract_coverage_regions(grid, robot_position=(-1.0, 1.0))
    assert len(regions) == 1
    polygon, holes = regions[0]
    assert len(holes) == 1
    assert max(point.x for point in polygon.polygon.points) < 0.0
    assert min(point.y for point in polygon.polygon.points) > 0.0
    assert holes[0][0][0] < 0.0
    assert details['selected_free_area_m2'] == pytest.approx(24.9975)


@pytest.mark.parametrize('points', [
    [(1, 1, 0), (1, 1, 1)], [(1, 1, 0), (0.5, 1, 0)],
    [(1, 1, 0), (1.02, 1.02, 1.0)], [(1, 1, 0), (1, 1.3, 0)],
])
def test_rejects_spin_reverse_sideways_and_excessive_curvature(points):
    with pytest.raises(ValueError):
        validate_forward_path(make_path(points), 0.2)


def test_forward_work_rejects_route_with_reverse_section():
    route = make_path([(2, 2, 0), (2.5, 2, 0), (2.5, 2, 0), (2.2, 2, 0), (3, 2, 0)])
    with pytest.raises(ValueError, match='reverse motion'):
        validate_forward_path(route, 0.2)


@pytest.mark.parametrize('occupancy', [100, -1])
def test_checks_between_sparse_points_and_inside_footprint(occupancy):
    grid = make_grid()
    grid.data[40 * 100 + 50] = occupancy
    with pytest.raises(ValueError, match='clearance'):
        FreeSpaceValidator(grid, FOOTPRINT, 0.2).check_path(make_path([(1, 2, 0), (3, 2, 0)]))


def test_outer_body_clearance_and_zone_containment():
    zone = [(1, 1), (4, 1), (4, 4), (1, 4)]
    validator = FreeSpaceValidator(make_grid(), FOOTPRINT, 0.2, zone)
    validator.check_path(make_path([(1.5, 2, 0), (3, 2, 0)]))
    with pytest.raises(ValueError):
        validator.check_path(make_path([(1.5, 2, 0), (3.4, 2, 0)]))


def test_rotated_map_origin():
    grid = make_grid()
    grid.info.origin.orientation.z = math.sin(math.pi / 4.0)
    grid.info.origin.orientation.w = math.cos(math.pi / 4.0)
    validator = FreeSpaceValidator(grid, FOOTPRINT, 0.2)
    validator.check_path(make_path([(-2, 1, math.pi / 2), (-2, 3, math.pi / 2)]))