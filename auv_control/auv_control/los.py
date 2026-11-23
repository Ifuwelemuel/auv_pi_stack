import math

def los_bearing(seg_start,target, pos, lookahead):
    """
    Return (bearing_rad_enu, cross_track_m)"""

    px, py = seg_start
    tx, ty = target

    vx, vy = tx - px, ty - py
    seg_len = math.hypot(vx, vy)

    if seg_len < 1e-6: #degenerate segment: go to start
        return math.atan2(ty - pos[1], tx - pos[0]), 0.0

    ux, uy = vx / seg_len, vy / seg_len
    dx, dy = pos[0] - px, pos[1] - py
    along = dx * ux + dy * uy
    cross = ux * dy - uy * dx
    s = min(along + lookahead, seg_len)    # never aim past the target
    ax, ay = px + ux * s, py + uy * s
    return math.atan2(ay - pos[1], ax - pos[0]), cross

