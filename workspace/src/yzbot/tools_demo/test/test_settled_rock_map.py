"""Settled rotated furniture must be represented without raster holes."""
import importlib.util
import json
import math
from pathlib import Path

import pytest

REPO=Path(__file__).resolve().parents[2]
SPEC=importlib.util.spec_from_file_location('static_map_generator',REPO/'tools_demo/scripts/generate_static_map.py')
GEN=importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GEN)


@pytest.mark.parametrize('yaw',[0.,.2623,-.2618,.2548])
def test_rotated_rock_interior_has_no_free_holes(yaw):
    grid=bytearray([GEN.FREE])*(GEN.MAP_W*GEN.MAP_H)
    GEN.rasterize_oriented_rock_cells(grid,1.5685,-7.2636,1.,1.,yaw)
    for i in range(41):
        for j in range(41):
            x,y=-.48+i*.024,-.48+j*.024
            wx=1.5685+math.cos(yaw)*x-math.sin(yaw)*y
            wy=-7.2636+math.sin(yaw)*x+math.cos(yaw)*y
            px,py=GEN.world_to_grid_px(wx,wy)
            assert grid[py*GEN.MAP_W+px] == GEN.OCCUPIED


def test_oriented_fill_does_not_extend_beyond_its_grid_cell_overlap_band():
    grid=bytearray([GEN.FREE])*(GEN.MAP_W*GEN.MAP_H)
    GEN.rasterize_oriented_rock_cells(grid,1.5685,-7.2636,1.,1.,.2623)
    for ix in range(GEN.MAP_W):
        for iy in range(GEN.MAP_H):
            if grid[iy*GEN.MAP_W+ix] != GEN.OCCUPIED:continue
            wx=GEN.ORIGIN_X+(ix+.5)*GEN.RESOLUTION
            wy=GEN.ORIGIN_Y+(GEN.MAP_H-1-iy+.5)*GEN.RESOLUTION
            assert math.hypot(wx-1.5685,wy+7.2636) <= math.sqrt(.5)+math.sqrt(2)*GEN.RESOLUTION/2+1e-9


def test_all_settled_positions_have_a_measured_orientation():
    positions=json.loads((REPO/'config/rocks_settled.json').read_text())
    yaws=json.loads((REPO/'config/rocks_settled_yaw.json').read_text())
    assert set(positions) == set(yaws)
    assert all(math.isfinite(value) for value in yaws.values())
    assert yaws['link_7'] == .2623
    assert yaws['link_9'] == -.2618
    assert yaws['link_10'] == .2548
