import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import pytest

from frontier_explorer.coverage_path import FreeSpaceValidator, validate_forward_path, validate_headland_bounds


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


def test_recorded_overshooting_ingress_gets_an_exact_forward_connector():
    from copy import deepcopy
    from frontier_explorer.coverage_path import join_forward_paths, pose_xy_yaw
    endpoint = (-5.248040350154042, -3.1642318312078714, 1.4835299253463747)
    target = (-5.2433330128168905, -3.17458057403564, 1.5010369896849216)
    ingress = make_path([(endpoint[0] - distance * math.cos(endpoint[2]),
                          endpoint[1] - distance * math.sin(endpoint[2]), endpoint[2])
                         for distance in (0.6, 0.4, 0.2, 0.0)])
    route = make_path([target, (target[0] + math.cos(target[2]),
                                target[1] + math.sin(target[2]), target[2])])
    original = deepcopy(route)
    combined = deepcopy(ingress)
    combined.poses.extend(route.poses)
    with pytest.raises(ValueError, match='reverse motion'):
        validate_forward_path(combined, 0.20)
    joined = join_forward_paths(ingress, route, 0.20, lambda path: None)
    validate_forward_path(joined, 0.20)
    assert joined.poses[-2:] == route.poses
    assert pose_xy_yaw(joined.poses[-3].pose) == pytest.approx(target)
    assert route == original
    with pytest.raises(ValueError, match='No collision-free forward connector'):
        join_forward_paths(ingress, route, 0.20,
                           lambda path: (_ for _ in ()).throw(ValueError('blocked')))


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
    assert validate_forward_path(path, 0.20) > 10.0
    validator.check_path(path)


def test_continuous_forward_arc():
    path = make_path([(2.0 + 0.3 * math.sin(angle), 2.0 + 0.3 * (1.0 - math.cos(angle)), angle)
                      for angle in [index * 0.05 for index in range(32)]])
    assert validate_forward_path(path, 0.2) > 0.4
    FreeSpaceValidator(make_grid(), FOOTPRINT, 0.2).check_path(path)


@pytest.mark.parametrize('points', [
    [(1, 1, 0), (1, 1, 1)], [(1, 1, 0), (0.5, 1, 0)],
    [(1, 1, 0), (1.02, 1.02, 1.0)],
])
def test_rejects_spin_reverse_and_excessive_curvature(points):
    with pytest.raises(ValueError):
        validate_forward_path(make_path(points), 0.2)


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