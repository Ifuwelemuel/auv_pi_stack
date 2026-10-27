"""Mission file loading and validation. Pure Python, unit-tested."""
import yaml


def load_mission(path):
    """Parse and validate a mission file. Raises ValueError with a clear
    message rather than guessing at anything ambiguous."""
    with open(path) as f:
        m = yaml.safe_load(f)['mission']
    if m.get('frame') not in ('local', 'global'):
        raise ValueError(f"mission 'frame' must be 'local' or 'global', "
                         f"got {m.get('frame')!r}")
    wps = [tuple(float(v) for v in w) for w in m['waypoints']]
    if not wps or any(len(w) != 2 for w in wps):
        raise ValueError('waypoints must be a non-empty list of [a, b] pairs')
    g = m['geofence']
    mission = {
        'frame': m['frame'], 'waypoints': wps,
        'accept': float(m['acceptance_radius_m']),
        'timeout': float(m['mission_timeout_s']),
        'fence': (float(g['east_min']), float(g['east_max']),
                  float(g['north_min']), float(g['north_max'])),
        'cruise': float(m.get('cruise_thrust', 0.3)),
        'lookahead': float(m.get('lookahead_m', 5.0)),
        'depth': float(m.get('depth_m', 0.0)),
    }
    if not (mission['fence'][0] < mission['fence'][1] and
            mission['fence'][2] < mission['fence'][3]):
        raise ValueError('geofence min must be < max')
    if not 0.0 <= mission['cruise'] <= 1.0:
        raise ValueError('cruise_thrust must be in [0, 1]')
    return mission
