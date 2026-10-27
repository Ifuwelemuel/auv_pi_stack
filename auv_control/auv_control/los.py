"""Line-of-sight guidance (Fossen). Pure Python, unit-tested.

Follow the LINE from the previous waypoint to the target, not just point at
the target: project the vehicle onto the line, aim at a point `lookahead`
metres further along it, and steer towards that point. Small lookahead =
aggressive return to the line, large = gentle.
"""
import math


def los_bearing(seg_start, target, pos, lookahead):
    """Return (bearing_rad_enu, cross_track_m).

    bearing: ENU, CCW from east — same convention as the IMU yaw.
    cross_track: signed distance from the line, + = left of the line.
    """
    px, py = seg_start
    tx, ty = target
    vx, vy = tx - px, ty - py
    seg_len = math.hypot(vx, vy)
    if seg_len < 1e-6:                     # degenerate segment: go to target
        return math.atan2(ty - pos[1], tx - pos[0]), 0.0
    ux, uy = vx / seg_len, vy / seg_len
    dx, dy = pos[0] - px, pos[1] - py
    along = dx * ux + dy * uy
    cross = ux * dy - uy * dx
    s = min(along + lookahead, seg_len)    # never aim past the target
    ax, ay = px + ux * s, py + uy * s
    return math.atan2(ay - pos[1], ax - pos[0]), cross
