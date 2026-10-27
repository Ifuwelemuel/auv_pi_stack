import math

import pytest

from auv_control.geo import LocalFrame, wrap, yaw_from_quat
from auv_control.los import los_bearing


def test_wrap_short_way():
    assert wrap(math.radians(350)) == pytest.approx(math.radians(-10))
    assert wrap(-math.radians(190)) == pytest.approx(math.radians(170))


def test_yaw_from_quat_north():
    h = math.pi / 2       # facing north in ENU
    assert yaw_from_quat(0, 0, math.sin(h / 2), math.cos(h / 2)) == pytest.approx(h)


def test_local_frame_round_trip_and_scale():
    f = LocalFrame(53.3439, -1.4994)               # Sheffield test site
    e, n = f.to_local(53.3439 + 0.0001, -1.4994 + 0.0001)
    assert n == pytest.approx(11.12, abs=0.05)      # 1e-4 deg lat ~ 11.1 m
    assert e == pytest.approx(6.64, abs=0.05)       # shrunk by cos(lat)
    lat, lon = f.to_global(e, n)
    assert lat == pytest.approx(53.3440) and lon == pytest.approx(-1.4993)


def test_los_on_line_aims_along_line():
    b, xt = los_bearing((0, 0), (100, 0), (10, 0), 5.0)
    assert b == pytest.approx(0.0) and xt == pytest.approx(0.0)


def test_los_left_of_line_steers_back_right():
    b, xt = los_bearing((0, 0), (100, 0), (10, 5), 5.0)
    assert xt == pytest.approx(5.0)
    assert b == pytest.approx(-math.pi / 4)          # 45 deg back to the line


def test_los_never_aims_past_target():
    b, _ = los_bearing((0, 0), (10, 0), (9, 1), 5.0)
    assert b == pytest.approx(math.atan2(-1, 1))


def test_los_degenerate_segment():
    b, _ = los_bearing((5, 5), (5, 5), (0, 0), 5.0)
    assert b == pytest.approx(math.pi / 4)
