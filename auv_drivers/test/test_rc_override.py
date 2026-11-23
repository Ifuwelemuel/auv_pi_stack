import pytest

from auv_drivers.rc_override import (CHAN_IGNORE, CHAN_NOCHANGE, NO_CMD, NUM_RC, TOGGLE_HI,
                                     TOGGLE_LO, build_override, explain_failsafe,
                                     output_mismatches, rc_channel, servo_index,
                                     validate_map)

MAP = {'yaw': 1, 'pitch': 2, 'surge': 3}


def test_aux_numbering_matches_the_lua_script():
    # auv_watchdog.lua: AUXi = SERVO(8+i), commanded on RC(8+i), i = 1..3
    assert [rc_channel(a) for a in (1, 2, 3)] == [9, 10, 11]
    assert [servo_index(a) for a in (1, 2, 3)] == [8, 9, 10]


def test_torpedo_only_aux3_used():
    # torpedo: T200 on AUX3; AUX1/2 get an explicit "no command", not stale values
    ch = build_override({3: 1700}, False, 16)
    assert ch[8:11] == [NO_CMD, NO_CMD, 1700]
    assert ch[15] == TOGGLE_LO


def test_override_layout():
    ch = build_override({1: 1500, 2: 1000, 3: 1610.4}, True, 16)
    assert len(ch) == NUM_RC
    assert ch[:8] == [CHAN_NOCHANGE] * 8          # ArduSub pilot channels untouched
    assert ch[8:11] == [1500, 1000, 1610]
    assert ch[15] == TOGGLE_HI
    assert all(v == CHAN_IGNORE for i, v in enumerate(ch) if i >= 11 and i != 15)


def test_no_cmd_is_outside_the_scripts_valid_band():
    assert NO_CMD < 950          # auv_watchdog.lua PULSE_MIN


def test_toggle_alternates():
    assert build_override({1: 1500}, True, 16)[15] == TOGGLE_HI
    assert build_override({1: 1500}, False, 16)[15] == TOGGLE_LO


@pytest.mark.parametrize('aux, toggle', [({'yaw': 0, 'pitch': 2, 'surge': 3}, 16),
                                         ({'yaw': 4, 'pitch': 2, 'surge': 3}, 16),
                                         ({'yaw': 1, 'pitch': 1, 'surge': 3}, 16),
                                         (MAP, 11), (MAP, 17)])
def test_validate_rejects(aux, toggle):
    with pytest.raises(ValueError):
        validate_map(aux, toggle)


def test_validate_accepts():
    validate_map(MAP, 16)
    validate_map(MAP, 12)
    validate_map({'surge': 3}, 16)


def test_mismatches():
    out = [0] * 8 + [1500, 1000, 1500] + [0] * 5
    assert output_mismatches(out, {1: 1500, 2: 1000, 3: 1500}) == []
    assert output_mismatches(out, {1: 1501, 2: 1000, 3: 1500}) == []      # within 2 us
    assert output_mismatches(out, {2: 1500}) == [(2, 1000, 1500)]
    assert output_mismatches(out[:8], {1: 1500}) == [(1, None, 1500)]   # MAVLink1: 8 ch


def test_explanations_name_the_fix():
    text = '\n'.join(explain_failsafe([(1, 0, 1500), (2, 1500, 1000), (3, None, 1500)]))
    assert 'SERVO9_FUNCTION = 94' in text
    assert 'AUVW_STOP2 = 1000' in text
    assert 'MAVLink2' in text
