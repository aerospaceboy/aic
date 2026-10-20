"""The experimental route must be restricted to known start regions."""
from tools_demo.c_clearance_route import c_clearance_waypoints
from tools_demo.navigation_math import ZonePose


def test_known_south_starts_route_east_of_upper_wall_end():
    dock = ZonePose('C', -8.6, -7.0, 3.14)
    for x, y in [(9.26, -8.99), (4.72, -11.0), (-9.26, -3.01)]:
        points = c_clearance_waypoints(x, y, dock)
        assert points[-1].x == -2.1
        assert points[-1].y == 1.8
        assert all(p.zone.startswith('C_clearance_') for p in points)


def test_other_starts_retain_existing_routes():
    dock = ZonePose('C', -8.6, -7.0, 3.14)
    for x, y in [(5.1, 1.9), (-6.7, -7.0), (None, None), (0, 0)]:
        assert not c_clearance_waypoints(x, y, dock)
