"""ESC pulse maths. Pure Python, no ROS, unit-tested in test/test_esc.py."""


def thrust_to_us(cmd, stop_us, span_us, sign, bidirectional):
    """Normalised thrust [-1, 1] -> pulse width (microseconds) for one ESC.

    stop_us        pulse that means "motor stopped". Bidirectional ESC
                   (T200 Basic ESC): neutral, about 1500. Forward-only ESC:
                   minimum throttle, about 1000.
    span_us        pulse change at full command.
    sign           +1 or -1. Maps the body-frame convention onto this ESC's
                   wiring and propeller. For a forward-only ESC it also picks
                   which body direction the thruster can produce.
    bidirectional  False for a forward-only ESC: after applying the sign,
                   negative commands become "stopped" (never a pulse below
                   stop_us).

    The result is always inside the calibrated band, for any input
    including NaN (which maps to stop).
    """
    if cmd != cmd:            # NaN: a broken upstream must never reach an ESC
        return int(stop_us)
    x = max(-1.0, min(1.0, float(cmd))) * sign
    if not bidirectional:
        x = max(0.0, x)
    return int(round(stop_us + x * span_us))
