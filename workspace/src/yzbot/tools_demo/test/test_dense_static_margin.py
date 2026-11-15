import pytest
import numpy as np
from tools_demo.dense_static_margin import static_margin_cells


def test_static_clearance_never_erases_walls_or_turns_unknown_into_free_space():
    cells = np.zeros((25, 25), dtype=int)
    cells[12, :] = 100
    cells[:, 0] = -1
    padded = np.array(static_margin_cells(cells.ravel().tolist(), 25, 25, .05)).reshape(25, 25)
    assert np.all(padded[cells == 100] == 100)
    assert np.all(padded[cells == -1] == -1)
    assert padded[8, 12] == 100
    assert padded[7, 12] == 0


def test_broad_corridor_remains_passable_and_original_input_is_unchanged():
    cells = np.zeros((30, 30), dtype=int)
    cells[:, 3] = cells[:, 26] = 100
    original = cells.copy()
    padded = np.array(static_margin_cells(cells.ravel().tolist(), 30, 30, .05)).reshape(30, 30)
    assert padded[15, 15] == 0
    assert np.array_equal(cells, original)


def test_missing_or_invalid_static_map_is_not_republished_as_free_space():
    with pytest.raises(ValueError):
        static_margin_cells([0], 10, 10, .05)
    with pytest.raises(ValueError):
        static_margin_cells([0], 1, 1, float('nan'))
