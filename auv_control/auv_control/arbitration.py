"""Which command source drives the thruster """

SAFE, TELEOP, AUTONOMY = 'safe', 'teleop','autonomy'

def _fresh(age, timeout):
    return age is not None and age <= timeout


def select_source(estopped, safety_age,teleop_age,autonomy_age,source_timeout,safety_timeout):
    """Priority: safe > teleop > autonomy. Ages are seconds since the last
    message on OUR clock, or None if never received.
    """
    if estopped or not _fresh(safety_age, safety_timeout):
        return SAFE
    if _fresh(teleop_age,source_timeout):
        return TELEOP
    if _fresh(autonomy_age,source_timeout):
        return AUTONOMY

    return SAFE

