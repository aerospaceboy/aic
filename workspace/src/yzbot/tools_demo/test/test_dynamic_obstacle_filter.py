"""Guard static-map matching and non-circular obstacle clearance."""

import math

from tools_demo.dynamic_obstacle_filter import (
    distance_to_box, static_return, transform_point,
)


def test_known_wall_is_filtered_but_free_space_moving_wall_is_not():
    grid = [0] * 100
    grid[5 * 10 + 5] = 100
    assert static_return(grid, 10, 10, .1, (0, 0, 0), .61, .55)
    assert not static_return(grid, 10, 10, .1, (0, 0, 0), .25, .25)
    grid[2 * 10 + 2] = -1
    assert not static_return(grid, 10, 10, .1, (0, 0, 0), .25, .25)


def test_matching_respects_rotated_map_origin():
    grid = [0] * 100
    grid[5 * 10 + 5] = 100
    assert static_return(grid, 10, 10, .1, (1, 2, math.pi / 2), .45, 2.55)


def test_wall_edge_clearance_is_not_small_cylinder_clearance():
    assert math.isclose(distance_to_box(7.2, -4.6, 7.2, -5.5, .12, 1.6), .1)
    assert distance_to_box(7.2, -5.5, 7.2, -5.5, .12, 1.6) == 0
    assert math.isclose(distance_to_box(8.6, -6, 7.9, -5.5, .12, 1.6), .64)


def test_laser_transform_preserves_sensor_height_and_heading():
    s = math.sqrt(.5)
    result = transform_point((1, 0, 0), (2, 3, .25), (0, 0, s, s))
    assert all(math.isclose(a, b, abs_tol=1e-8)
               for a, b in zip(result, (2, 4, .25)))


def test_complex_layout_keeps_original_spawn_staging():
    """Disabling old resource guides must retain the two live spawn gates."""
    from tools_demo.task_navigator import TaskNavigator
    from tools_demo.navigation_math import ZonePose

    node = TaskNavigator.__new__(TaskNavigator)
    node.use_legacy_layout_routes = False
    node.dock_pose = ZonePose("red4_entry", 9.1, -6.0, 0.0)
    stages = node._cube_approach_waypoints("RED_CUBE_4", 13.2, -.95)
    assert stages[-1].zone == "universal_origin_staging"
    assert (stages[-1].x, stages[-1].y) == (0.0, 0.0)
    assert len(stages) == 6
    assert node._cube_approach_waypoints("RED_CUBE_4", 0.0, 0.0) == []
