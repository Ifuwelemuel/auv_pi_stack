import pytest

from auv_drivers.actuators import (failsafe, neutral, pulse, servo_to_us,
                                   validate)


def esc(**kw):
    c = dict(kind='esc', stage='pixhawk', input='surge', aux=3, channel=2, sign=1, stop_us=1500,
             span_us=400, bidirectional=True)
    c.update(kw)
    return c


def fin(**kw):
    c = dict(kind='servo', stage='pca9685', input='yaw', aux=0, channel=0, sign=1, center_us=1500,
             span_us=300, min_us=1200, max_us=1800, failsafe_us=1500)
    c.update(kw)
    return c


@pytest.mark.parametrize('cmd, want', [(0.0, 1500), (1.0, 1800), (-1.0, 1200),
                                       (0.5, 1650), (5.0, 1800), (float('nan'), 1500)])
def test_servo_basic(cmd, want):
    assert servo_to_us(cmd, 1500, 300, 1, 1200, 1800) == want


def test_servo_sign_and_trim():
    assert servo_to_us(1.0, 1520, 300, -1, 1100, 1900) == 1220
    assert servo_to_us(0.0, 1520, 300, -1, 1100, 1900) == 1520


def test_servo_never_past_mechanical_limits():
    # span wider than the limits allow: clamped, both directions
    assert servo_to_us(1.0, 1500, 500, 1, 1250, 1700) == 1700
    assert servo_to_us(-1.0, 1500, 500, 1, 1250, 1700) == 1250


def test_pulse_neutral_failsafe_per_kind():
    assert pulse(esc(), 0.5) == 1700
    assert pulse(fin(), 0.5) == 1650
    assert neutral(esc()) == 1500 and neutral(fin(center_us=1510)) == 1510
    assert failsafe(esc(stop_us=1490)) == 1490
    assert failsafe(fin(center_us=1510, failsafe_us=1500)) == 1500


# torpedo as wired: T200 on Pixhawk AUX3, fins on PCA9685 ch0-3
TORPEDO = {'surge': esc(), 'fin_top': fin(channel=0), 'fin_bottom': fin(channel=1),
           'fin_left': fin(channel=2, input='pitch'), 'fin_right': fin(channel=3, input='pitch')}


def test_torpedo_valid():
    validate(TORPEDO)


def test_same_number_on_different_stages_is_fine():
    # Pixhawk AUX3 and PCA channel 3 are different pins
    outs = {'surge': esc(aux=3), 'fin_right': fin(channel=3)}
    validate(outs)


@pytest.mark.parametrize('name, change, msg', [
    ('fin_top', dict(input='fin_top'), 'fin_top_input'),
    ('fin_top', dict(kind='motor'), 'fin_top_kind'),
    ('fin_top', dict(sign=0), 'fin_top_sign'),
    ('fin_top', dict(stage='arduino'), 'fin_top_stage'),
    ('fin_top', dict(channel=16), 'fin_top_channel'),
    ('fin_top', dict(channel=1), 'share a channel'),
    ('surge', dict(aux=4), 'surge_aux'),
    ('fin_top', dict(center_us=1900), 'center_us'),
    ('fin_top', dict(failsafe_us=1100), 'failsafe_us'),
    ('surge', dict(kind='servo', center_us=1500, min_us=1100, max_us=1900,
                   failsafe_us=1500), 'no ESC follows surge'),
])
def test_validate_rejects(name, change, msg):
    outs = {n: dict(c) for n, c in TORPEDO.items()}
    outs[name].update(change)
    with pytest.raises(ValueError, match=msg):
        validate(outs)


def test_each_output_checked_on_its_own_stage():
    # a Pixhawk output needs no PCA channel, a PCA output needs no AUX
    validate({'surge': esc(channel=-1), 'fin_top': fin(aux=0)})
