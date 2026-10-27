import pytest

from auv_control.pid import Pid


def test_plain_pid_inside_limits():
    pid = Pid(0.6, 0.05, 0.4, 0.25, 0.0, 1.0)
    out = pid.update(0.2, -0.1, 0.1)
    assert pid.integral == pytest.approx(0.02)
    assert out == pytest.approx(0.6 * 0.2 + 0.05 * 0.02 + 0.4 * -0.1)


def test_one_sided_output():
    pid = Pid(0.6, 0.05, 0.4, 0.25, 0.0, 1.0)
    assert pid.update(-1.0, 0.0, 0.1) == 0.0


def test_no_windup_while_too_deep():
    pid = Pid(0.6, 0.05, 0.4, 0.25, 0.0, 1.0)
    for _ in range(500):
        assert pid.update(-0.5, 0.0, 0.1) == 0.0
    assert pid.integral == 0.0


def test_trim_kept_while_pinned_at_zero():
    pid = Pid(0.6, 0.05, 0.4, 0.25, 0.0, 1.0)
    pid.integral = 0.2
    assert pid.update(-0.1, 0.0, 0.1) == 0.0
    assert pid.integral == 0.2


def test_unwinds_when_not_saturated():
    pid = Pid(0.6, 0.05, 0.4, 0.25, 0.0, 1.0)
    pid.integral = 0.2
    out = pid.update(-0.01, 0.0, 0.1)
    assert 0.0 < out < 1.0 and pid.integral < 0.2


def test_i_limit_bounds_i_term_output():
    pid = Pid(0.1, 0.05, 0.0, 0.25, 0.0, 1.0)
    for _ in range(5000):
        pid.update(0.1, 0.0, 0.1)
    assert pid.ki * pid.integral == pytest.approx(0.25)


def test_i_term_can_hold_buoyancy_trim():
    """At zero error the I-term alone must be able to supply trim thrust."""
    pid = Pid(0.6, 0.1, 0.4, 0.3, 0.0, 1.0)
    pid.integral = 3.0                      # 0.1 * 3.0 = 0.3 = i_limit
    assert pid.update(0.0, 0.0, 0.1) == pytest.approx(0.3)


def test_zero_ki_keeps_integral_at_zero():
    pid = Pid(1.0, 0.0, 0.0, 0.2)
    for _ in range(100):
        pid.update(0.5, 0.0, 0.1)
    assert pid.integral == 0.0


def test_bad_limits_rejected():
    with pytest.raises(ValueError):
        Pid(1, 0, 0, 0, 1.0, 0.0)
