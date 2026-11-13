"""Keep transport optimization aligned with verified navigation evidence."""

import json
from types import SimpleNamespace

import pytest

from tools_demo.mission_coordinator import MissionCoordinator
from tools_demo.transport_schedule import current_task_response_proof


FIRST = 'competition_moving_obstacle_1'
SECOND = 'competition_moving_obstacle_2'
CART1 = 'crossing_cart_1'
CART2 = 'crossing_cart_2'


@pytest.mark.parametrize('effective,central,expected', [
    ([FIRST, SECOND], [], True),
    ([FIRST], [CART1], True),
    ([], [CART1, CART2], True),
    ([FIRST, FIRST], [], False),
    ([], [CART1, CART1], False),
    (['unverified_1', 'unverified_2'], [], False),
    ([FIRST, 'unverified_1'], [], False),
    ([FIRST], [FIRST], False),
    ([], [], False),
    (FIRST, [CART1, CART2], False),
    ([FIRST, None], [CART1, CART2], False),
])
def test_only_distinct_known_verified_responses_complete_the_contract(
        effective, central, expected):
    payload = {'encounter_task_id': 'round', 'encounter_epoch_sec': 11.,
               'verified_effective_responses': effective,
               'verified_central_responses': central}
    assert current_task_response_proof(payload, 'round', 10.) is expected


@pytest.mark.parametrize('task,epoch', [
    ('old-round', 11.), ('round', 9.), ('round', None),
    ('round', float('nan')), ('round', float('inf')),
])
def test_prior_or_invalid_epoch_does_not_release_encounter_constraint(task, epoch):
    payload = {'encounter_task_id': task, 'encounter_epoch_sec': epoch,
               'verified_effective_responses': [FIRST, SECOND]}
    assert not current_task_response_proof(payload, 'round', 10.)


def test_two_spawn_responses_reach_the_real_coordinator_boundary():
    node = SimpleNamespace(
        machine=SimpleNamespace(state='navigating_pickup'),
        navigation_location='RED_CUBE_1', _transport_automatic=True,
        active_task_id='round', _transport_epoch_sec=10.,
        _transport_responses_complete=False)
    payload = {'zone': 'RED_CUBE_1', 'state': 'active',
               'encounter_task_id': 'round', 'encounter_epoch_sec': 11.,
               'verified_effective_responses': [FIRST, SECOND],
               'verified_central_responses': []}
    MissionCoordinator.navigation_state_callback(
        node, SimpleNamespace(data=json.dumps(payload)))
    assert node._transport_responses_complete
    assert node.navigation_seen_active
