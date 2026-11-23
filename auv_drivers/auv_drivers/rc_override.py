"""Wire format between thruster_driver and pixhawk/auv_watchdog.lua (D18).

Pure functions, no ROS imports, so they unit-test anywhere.

Each RC_CHANNELS_OVERRIDE the driver sends through mavros carries:
    RC 9..11      pulse width for AUX1..3 (SERVO9..11), in microseconds;
                  NO_CMD for every AUX output this configuration does not use.
                  ArduSub keeps the last value of an RC override forever, so
                  an AUX left out would replay whatever an earlier session
                  sent; NO_CMD is outside the script's valid band (950-2050),
                  so the script holds that output at its failsafe.
    RC toggle     (default 16) TOGGLE_LO / TOGGLE_HI, flipped on every message;
                  the Pixhawk script follows the Pi only while it keeps flipping
    RC 1..8       CHAN_NOCHANGE: never touch ArduSub's own pilot channels
    the rest      0, which MAVLink defines as "ignore" for channels 9-18
The Pixhawk reports what it actually outputs in SERVO_OUTPUT_RAW
(/mavros/rc/out), index 0 = SERVO1, so AUXn is index 8 + n - 1.
"""

NUM_AUX = 3            # AUX outputs auv_watchdog.lua drives (its NUM_AUX)
TOGGLE_LO = 1100
TOGGLE_HI = 1900
CHAN_NOCHANGE = 65535
CHAN_IGNORE = 0
NO_CMD = 900           # "no command for this AUX": script holds its failsafe
NUM_RC = 18            # length of mavros_msgs/OverrideRCIn.channels


def rc_channel(aux):
    """RC channel number (1-based) that carries the command for AUX `aux`."""
    return 8 + aux


def servo_index(aux):
    """0-based index of AUX `aux` in /mavros/rc/out channels (SERVO9 = 8)."""
    return 8 + aux - 1


def validate_map(aux_by_name, toggle_rc):
    """Raise ValueError if the AUX assignment or toggle channel cannot work."""
    for name, aux in aux_by_name.items():
        if not 1 <= aux <= NUM_AUX:
            raise ValueError(f'{name}_aux = {aux}: must be 1..{NUM_AUX} '
                             f'(auv_watchdog.lua drives AUX1-AUX{NUM_AUX})')
    if len(set(aux_by_name.values())) != len(aux_by_name):
        raise ValueError(f'two outputs share an AUX output: {aux_by_name}')
    if not rc_channel(NUM_AUX) < toggle_rc <= 16:
        raise ValueError(f'toggle_rc = {toggle_rc}: must be '
                         f'{rc_channel(NUM_AUX) + 1}..16 and equal AUVW_TOG_RC')


def build_override(pulses_by_aux, toggle_high, toggle_rc):
    """The 18 OverrideRCIn channel values for one message."""
    ch = [CHAN_NOCHANGE] * 8 + [CHAN_IGNORE] * (NUM_RC - 8)
    for aux in range(1, NUM_AUX + 1):
        ch[rc_channel(aux) - 1] = NO_CMD
    for aux, us in pulses_by_aux.items():
        ch[rc_channel(aux) - 1] = int(round(us))
    ch[toggle_rc - 1] = TOGGLE_HI if toggle_high else TOGGLE_LO
    return ch


def output_mismatches(servo_out, want_by_aux, tol_us=2):
    """[(aux, reported_us or None, wanted_us)] for every AUX that is off."""
    bad = []
    for aux in sorted(want_by_aux):
        i = servo_index(aux)
        got = int(servo_out[i]) if i < len(servo_out) else None
        want = int(round(want_by_aux[aux]))
        if got is None or abs(got - want) > tol_us:
            bad.append((aux, got, want))
    return bad


def explain_failsafe(bad):
    """Operator-facing lines for a failed start-up check (Pi still silent)."""
    lines = []
    for aux, got, want in bad:
        if got is None:
            lines.append(f'AUX{aux}: not in SERVO_OUTPUT_RAW. Is the link MAVLink2 '
                         f'(SERIAL2_PROTOCOL = 2)?')
        elif got == 0:
            lines.append(f'AUX{aux}: no pulse. Is auv_watchdog.lua running '
                         f'(SCR_ENABLE = 1, file in APM/scripts on the SD card), '
                         f'SERVO{8 + aux}_FUNCTION = {93 + aux}, '
                         f'AUVW_STOP{aux} = {want}?')
        else:
            lines.append(f'AUX{aux}: the Pixhawk failsafe on this output is {got} us '
                         f'but the configuration says {want} us. Set AUVW_STOP{aux} = '
                         f'{want} in QGC (or correct *_stop_us / *_failsafe_us).')
    return lines


def explain_follow(bad):
    """One line: commanded vs reported, for a running vehicle."""
    return ', '.join(f'AUX{aux} sent {want} us, Pixhawk outputs '
                     f'{"?" if got is None else got} us' for aux, got, want in bad)
