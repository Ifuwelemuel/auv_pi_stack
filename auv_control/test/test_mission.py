import pytest

from auv_control.mission import load_mission

GOOD = """
mission:
  frame: global
  waypoints: [[53.3439, -1.4994], [53.3438, -1.4992]]
  acceptance_radius_m: 3.0
  mission_timeout_s: 600
  geofence: {east_min: -60, east_max: 60, north_min: -60, north_max: 60}
"""


def write(tmp_path, text):
    f = tmp_path / 'm.yaml'
    f.write_text(text)
    return str(f)


def test_good_mission_with_defaults(tmp_path):
    m = load_mission(write(tmp_path, GOOD))
    assert m['frame'] == 'global' and len(m['waypoints']) == 2
    assert m['cruise'] == 0.3 and m['depth'] == 0.0 and m['lookahead'] == 5.0


def test_missing_frame_rejected(tmp_path):
    with pytest.raises(ValueError, match='frame'):
        load_mission(write(tmp_path, GOOD.replace('frame: global', 'frame: wgs84')))


def test_bad_waypoint_rejected(tmp_path):
    with pytest.raises(ValueError, match='waypoints'):
        load_mission(write(tmp_path, GOOD.replace('[53.3438, -1.4992]', '[1, 2, 3]')))


def test_inverted_fence_rejected(tmp_path):
    with pytest.raises(ValueError, match='geofence'):
        load_mission(write(tmp_path, GOOD.replace('east_max: 60', 'east_max: -70')))


def test_cruise_out_of_range_rejected(tmp_path):
    with pytest.raises(ValueError, match='cruise'):
        load_mission(write(tmp_path, GOOD + '  cruise_thrust: 1.5\n'))
