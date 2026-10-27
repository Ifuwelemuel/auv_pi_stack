"""Angles and a local east/north frame. Pure Python, unit-tested.

Heading convention everywhere in this stack: ENU yaw, radians, measured
counter-clockwise from EAST (REP-103). mavros publishes IMU orientation in
ENU, so yaw_from_quat() and the LOS bearing use the same convention and no
offsets are needed. (0 = east, +pi/2 = north.)
"""
import math

R_EARTH = 6_371_000.0   # mean Earth radius [m]


def wrap(angle):
    """Map any angle to (-pi, pi] — always turn the short way round."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quat(x, y, z, w):
    """ENU yaw from a quaternion (ZYX extraction, yaw only)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class LocalFrame:
    """Flat-earth east/north metres about a fixed origin (fine over a few km).

    x = east, y = north. The origin is fixed once (first good GPS fix) and
    never moves for the rest of the session.
    """

    def __init__(self, lat0_deg, lon0_deg):
        self.lat0_deg, self.lon0_deg = lat0_deg, lon0_deg
        self._lat0 = math.radians(lat0_deg)
        self._lon0 = math.radians(lon0_deg)
        self._coslat0 = math.cos(self._lat0)

    def to_local(self, lat_deg, lon_deg):
        east = R_EARTH * (math.radians(lon_deg) - self._lon0) * self._coslat0
        north = R_EARTH * (math.radians(lat_deg) - self._lat0)
        return east, north

    def to_global(self, east_m, north_m):
        lat = self._lat0 + north_m / R_EARTH
        lon = self._lon0 + east_m / (R_EARTH * self._coslat0)
        return math.degrees(lat), math.degrees(lon)
