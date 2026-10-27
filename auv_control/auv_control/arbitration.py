"""Which command source drives the thrusters. Pure Python, unit-tested."""

SAFE, TELEOP, AUTONOMY = 'safe', 'teleop', 'autonomy'


def _fresh(age, timeout):
    return age is not None and age <= timeout


def select_source(estopped, safety_age, teleop_age, autonomy_age,
                  source_timeout, safety_timeout):
    """Priority: safe > teleop > autonomy. Ages are seconds since the last
    message on OUR clock, or None if never received.

    Safe is chosen if the e-stop is latched, OR the safety supervisor has
    gone quiet (a dead supervisor means a dead watchdog, so nothing else may
    drive), OR no command source is fresh (a crashed node's last command
    expires instead of persisting).
    """
    if estopped or not _fresh(safety_age, safety_timeout):
        return SAFE
    if _fresh(teleop_age, source_timeout):
        return TELEOP
    if _fresh(autonomy_age, source_timeout):
        return AUTONOMY
    return SAFE
