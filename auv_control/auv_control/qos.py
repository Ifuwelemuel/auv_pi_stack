"""QoS profiles shared by every node, so publisher and subscriber can't drift.

LATCHED: reliable + transient-local, depth 1. For STATE (/estop): a node
that starts late still receives the current value. Publisher and subscriber
must BOTH use it: a volatile subscriber connects but misses the value that
was published before it started (a late node would never learn it is
stopped), and a transient-local subscriber does not connect to a volatile
publisher at all (D8).

COMMAND: reliable + volatile, depth 1. For EVENTS and commands: never
replayed to late joiners, and never queued (a stale command is worse than a
dropped one).
"""
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST)
COMMAND = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.VOLATILE,
                     history=QoSHistoryPolicy.KEEP_LAST)
