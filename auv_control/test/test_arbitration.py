from auv_control.arbitration import AUTONOMY, SAFE, TELEOP, select_source

T, S = 0.3, 0.5


def test_estop_wins():
    assert select_source(True, 0.0, 0.0, 0.0, T, S) == SAFE


def test_dead_supervisor_is_safe():
    assert select_source(False, None, 0.0, 0.0, T, S) == SAFE
    assert select_source(False, 0.9, 0.0, 0.0, T, S) == SAFE


def test_teleop_beats_autonomy():
    assert select_source(False, 0.0, 0.1, 0.1, T, S) == TELEOP


def test_stale_teleop_falls_to_autonomy():
    assert select_source(False, 0.0, 0.5, 0.1, T, S) == AUTONOMY


def test_nothing_fresh_is_safe():
    assert select_source(False, 0.0, None, 1.0, T, S) == SAFE
