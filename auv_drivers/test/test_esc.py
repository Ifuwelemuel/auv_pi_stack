import pytest

from auv_drivers.esc import thrust_to_us


@pytest.mark.parametrize('cmd, want', [(0.0, 1500), (1.0, 1900), (-1.0, 1100),
                                       (0.5, 1700), (3.0, 1900), (-3.0, 1100)])
def test_bidirectional(cmd, want):
    assert thrust_to_us(cmd, 1500, 400, 1, True) == want


def test_bidirectional_sign_flip():
    assert thrust_to_us(1.0, 1500, 400, -1, True) == 1100


@pytest.mark.parametrize('cmd, want', [(0.0, 1000), (0.5, 1500), (1.0, 2000),
                                       (-0.2, 1000), (-1.0, 1000)])
def test_forward_only_nose_down_mounting(cmd, want):
    assert thrust_to_us(cmd, 1000, 1000, 1, False) == want


@pytest.mark.parametrize('cmd, want', [(-1.0, 2000), (-0.5, 1500), (0.7, 1000)])
def test_forward_only_nose_up_mounting(cmd, want):
    assert thrust_to_us(cmd, 1000, 1000, -1, False) == want


def test_nan_is_stop():
    assert thrust_to_us(float('nan'), 1000, 1000, 1, False) == 1000
    assert thrust_to_us(float('nan'), 1500, 400, 1, True) == 1500


def test_never_leaves_calibrated_band():
    for i in range(-300, 301):
        x = i / 100.0
        assert 1000 <= thrust_to_us(x, 1000, 1000, 1, False) <= 2000
        assert 1100 <= thrust_to_us(x, 1500, 400, -1, True) <= 1900
