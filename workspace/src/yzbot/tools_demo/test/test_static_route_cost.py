"""Wall detours must affect scheduling without weakening motion checks."""

import math
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
import yaml

from tools_demo.navigation_math import ZonePose
from tools_demo.static_route_cost import StaticRouteCost, load_static_route_cost
from tools_demo.transport_schedule import TransportTimeEstimator


def write_map(path, image):
    cv2.imwrite(str(path / 'map.pgm'), image)
    config = path / 'map.yaml'
    config.write_text(yaml.safe_dump({
        'image': 'map.pgm', 'resolution': .1, 'origin': [0., 0., 0.],
        'negate': 0, 'free_thresh': .25, 'occupied_thresh': .65}))
    return config


def test_wall_is_a_detour_and_cannot_be_crossed_by_a_straight_chord(tmp_path):
    image = np.full((60, 60), 255, np.uint8)
    image[10:, 29:31] = 0
    field = StaticRouteCost(write_map(tmp_path, image), clearance=.2)
    a, b = (2., 2.), (4., 2.)
    assert field.distance(a, b) > 6.
    assert field.distance(b, a) == field.distance(a, b)
    assert field.distance((1., 1.), (2., 1.)) == pytest.approx(1.)


def test_unknown_and_disconnected_cells_are_not_shortcuts(tmp_path):
    image = np.full((60, 60), 255, np.uint8)
    image[:, 29:31] = 128  # Between free and occupied: unknown is blocked.
    field = StaticRouteCost(write_map(tmp_path, image), clearance=0.)
    assert math.isinf(field.distance((2., 2.), (4., 2.)))
    assert math.isinf(field.distance((-3., 2.), (4., 2.)))
    assert math.isinf(field.distance((3., 2.), (4., 2.)))


def test_diagonal_cannot_cut_between_two_occupied_corner_cells(tmp_path):
    image = np.zeros((5, 5), np.uint8)
    image[3, 1] = image[2, 2] = 255
    field = StaticRouteCost(write_map(tmp_path, image), clearance=0., stride=1)
    assert math.isinf(field.distance((.15, .15), (.25, .25)))


def test_subsampling_never_erases_a_thin_wall(tmp_path):
    image = np.full((60, 60), 255, np.uint8)
    image[:, 29] = 0
    field = StaticRouteCost(write_map(tmp_path, image), clearance=0., stride=2)
    assert math.isinf(field.distance((2., 2.), (4., 2.)))


def test_coordinator_cache_reloads_changed_map_image(tmp_path):
    owner = SimpleNamespace()
    image = np.full((60, 60), 255, np.uint8)
    config = write_map(tmp_path, image)
    before = load_static_route_cost(owner, config)
    assert load_static_route_cost(owner, config) is before
    image[:, 29:31] = 0
    write_map(tmp_path, image)
    after = load_static_route_cost(owner, config)
    assert after is not before
    assert math.isinf(after.distance((2., 2.), (4., 2.)))


@pytest.mark.parametrize('a,b', [((math.nan, 1.), (1., 2.)),
                               ((1., 2.), (math.inf, 1.))])
def test_nonfinite_coordinates_cannot_enter_the_cost_graph(tmp_path, a, b):
    field = StaticRouteCost(write_map(tmp_path, np.full((20, 20), 255, np.uint8)))
    with pytest.raises(ValueError):
        field.distance(a, b)


def test_transport_cost_uses_wall_distance_for_empty_and_loaded_travel():
    pose = ZonePose('destination', 2., 0., 0.)
    plain = TransportTimeEstimator({}, {}, {}, {})
    mapped = TransportTimeEstimator({}, {}, {}, {}, route_distance=lambda a, b: 8.)
    for loaded, speed in ((False, plain.empty_speed), (True, plain.loaded_speed)):
        assert (mapped.transit_seconds((0., 0., 0.), [pose], loaded)
                - plain.transit_seconds((0., 0., 0.), [pose], loaded)
                == pytest.approx(6. / speed))
    mapped.route_distance = lambda a, b: math.inf
    assert math.isinf(mapped.transit_seconds((0., 0., 0.), [pose]))


def test_deployed_long_wall_no_longer_looks_like_ten_metre_free_space():
    path = Path(__file__).parents[2] / 'bot_navigation/maps/mapn3.yaml'
    field = StaticRouteCost(path)
    # Actual legal blue_1 entry to B: a long wall separates these anchors.
    distance = field.distance((6.5, 3.6), (11.3, -5.16))
    assert 23. < distance < 30.
