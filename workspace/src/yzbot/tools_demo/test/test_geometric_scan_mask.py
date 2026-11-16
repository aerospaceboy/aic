import math
import pytest
from sensor_msgs.msg import LaserScan
from tools_demo.payload_scan_mask import geometric_masked_scan


def scan(ranges):
    s=LaserScan();s.ranges=ranges;s.angle_min=0.;s.angle_increment=0.;return s


def test_close_real_wall_survives_where_old_rectangular_mask_blinded_it():
    s=scan([.40,.48,.48,.60]);s.angle_increment=.5
    transforms={'link6':((-.40,0.,0.),(0.,0.,0.,1.))}
    out=geometric_masked_scan(s,transforms)
    assert math.isnan(out.ranges[0])  # Actual wrist cylinder.
    assert out.ranges[1:]==pytest.approx([.48,.48,.60])
    assert s.ranges==pytest.approx([.40,.48,.48,.60])


def test_missing_or_invalid_tf_never_creates_a_blind_zone():
    s=scan([.4,.45,.5])
    assert geometric_masked_scan(s,{}).ranges==s.ranges
    assert geometric_masked_scan(s,{'link6':((0.,0.,0.),(0.,0.,0.,0.))}).ranges==s.ranges


def test_only_actual_measured_cube_volume_is_masked_not_entire_carry_window():
    s=scan([.40,.46,.48,.50])
    transforms={'link6':((-.30,0.,.075),(0.,0.,0.,1.))}
    payload=((.10,0.,.075),(0.,0.,0.,1.))
    out=geometric_masked_scan(s,transforms,payload)
    assert math.isnan(out.ranges[0])
    assert out.ranges[1:]==pytest.approx([.46,.48,.50])


def test_geometry_filter_preserves_infinite_unknown_beams():
    s=scan([float('inf'),float('nan'),.7])
    out=geometric_masked_scan(s,{})
    assert math.isinf(out.ranges[0]) and math.isnan(out.ranges[1])
    assert out.ranges[2]==pytest.approx(.7)
