#!/usr/bin/env python3
"""Move ONE Pixhawk AUX output by hand, through the watchdog path (D18, D19).

Use it to find a fin servo's centre and mechanical limits (or an ESC's stop).
Every other AUX output is held at its Pixhawk failsafe while this runs.

    # stack stopped (kill_stack.sh), mavros running:
    python3 scripts/aux_jog.py --aux 1                 # starts at 1500 us
    python3 scripts/aux_jog.py --aux 4 --min 1100 --max 1900

Keys:  u / d   +5 / -5 us        U / D   +25 / -25 us
       c       back to the start pulse
       q       quit (the Pixhawk returns the output to its failsafe in 0.3 s)

For a fin: step to where it is straight -> *_center_us. Step slowly each way
until the linkage is about to bind -> *_min_us / *_max_us (stay ~20 us
inside). Write them into config/torpedo.yaml.
PROPELLER OFF if the output is an ESC.
"""
import argparse
import select
import sys
import termios
import time
import tty

import rclpy
from mavros_msgs.msg import OverrideRCIn

from auv_drivers.rc_override import NUM_AUX, build_override

ap = argparse.ArgumentParser(description=__doc__,
                             formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('--aux', type=int, required=True, help=f'AUX output 1..{NUM_AUX}')
ap.add_argument('--start', type=int, default=1500)
ap.add_argument('--min', type=int, default=1100, dest='us_min')
ap.add_argument('--max', type=int, default=1900, dest='us_max')
ap.add_argument('--toggle-rc', type=int, default=16, help='= AUVW_TOG_RC')
args = ap.parse_args()
if not 1 <= args.aux <= NUM_AUX:
    sys.exit(f'--aux must be 1..{NUM_AUX}')

rclpy.init()
node = rclpy.create_node('aux_jog')
end = time.monotonic() + 1.5            # let discovery see other publishers
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
if node.count_publishers('/mavros/rc/override') > 0:
    sys.exit('Something else is publishing /mavros/rc/override (thruster_driver?). '
             'Run scripts/kill_stack.sh first.')
pub = node.create_publisher(OverrideRCIn, '/mavros/rc/override', 1)

us = max(args.us_min, min(args.us_max, args.start))
toggle = False
print(f'AUX{args.aux} at {us} us. u/d +-5, U/D +-25, c start, q quit.')
old = termios.tcgetattr(sys.stdin)
try:
    tty.setcbreak(sys.stdin.fileno())
    next_t = time.monotonic()
    while True:
        if select.select([sys.stdin], [], [], 0.0)[0]:
            k = sys.stdin.read(1)
            step = {'u': 5, 'd': -5, 'U': 25, 'D': -25}.get(k)
            if step:
                us = max(args.us_min, min(args.us_max, us + step))
            elif k == 'c':
                us = max(args.us_min, min(args.us_max, args.start))
            elif k == 'q':
                break
            print(f'\rAUX{args.aux} = {us} us   ', end='', flush=True)
        now = time.monotonic()
        if now >= next_t:                     # 20 Hz, toggle flips each message
            toggle = not toggle
            msg = OverrideRCIn()
            msg.channels = build_override({args.aux: us}, toggle, args.toggle_rc)
            pub.publish(msg)
            next_t += 0.05
        time.sleep(0.005)
finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
    print(f'\nStopped. AUX{args.aux} returns to its failsafe within 0.3 s.')
    node.destroy_node()
    rclpy.shutdown()
