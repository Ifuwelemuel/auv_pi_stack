#!/usr/bin/env python3
"""Find one fin's centre and mechanical limits on the PCA9685.

    ~/auv_ws/src/auv_pi_stack/scripts/kill_stack.sh      # driver must not run
    python3 ~/auv_ws/src/auv_pi_stack/scripts/fin_calibrate.py --fin top --channel 0

Keys (no Enter needed):
    d / u   -5 / +5 us        D / U   -25 / +25 us
    j / k   -1 / +1 us (the PCA9685 resolves ~5 us, so a single j/k may not move it)
    c       mark CENTRE here  (fin straight, in line with the hull)
    [       mark: binds at the LOW end here
    ]       mark: binds at the HIGH end here
    g       go to the centre mark
    s       sweep check: centre -> low limit -> high limit -> centre, slowly
    p       print the marks
    q       quit: the fin goes to its centre and the YAML lines are printed

Limits: step slowly towards an end until the linkage starts to strain or the
servo hums, mark it, then come back. The tool keeps --margin (20 us) inside
each mark for *_min_us / *_max_us.

Before running: servo BEC on; this channel is a servo, not an ESC.
"""
import argparse
import os
import select
import subprocess
import sys
import termios
import time
import tty

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'auv_drivers'))   # works unsourced
from auv_drivers.pca9685 import PCA9685                       # noqa: E402

ap = argparse.ArgumentParser(description=__doc__,
                             formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('--fin', required=True, choices=['top', 'bottom', 'left', 'right'])
ap.add_argument('--channel', type=int, required=True, help='PCA9685 channel (pca_servo_finder.py)')
ap.add_argument('--start', type=int, default=1500)
ap.add_argument('--hard-min', type=int, default=900, help='never go below, us')
ap.add_argument('--hard-max', type=int, default=2100, help='never go above, us')
ap.add_argument('--margin', type=int, default=20, help='kept inside each bind mark, us')
ap.add_argument('--bus', type=int, default=1)
ap.add_argument('--osc', type=int, default=28_730_000,
                help='MEASURED PCA9685 oscillator, Hz (D5; calibration.yaml)')
args = ap.parse_args()

if not 0 <= args.channel <= 15:
    sys.exit('--channel must be 0..15')
if subprocess.run(['pgrep', '-f', 'lib/auv_drivers/thruster_driver'],
                  capture_output=True).returncode == 0:
    sys.exit('thruster_driver is running and owns the PCA9685. '
             'Run scripts/kill_stack.sh first.')

pca = PCA9685(args.bus, 0x40, 50, args.osc)
ok, got, want, asleep = pca.verify()
if not ok:
    sys.exit(f'PCA9685 not configured (prescale {got}/{want}, asleep={asleep}).')

name = f'fin_{args.fin}'
us = max(args.hard_min, min(args.hard_max, args.start))
marks = {'center': None, 'low': None, 'high': None}


def put(v):
    global us
    us = max(args.hard_min, min(args.hard_max, int(v)))
    pca.set_us(args.channel, us)


def glide(target, seconds=1.0):
    start, steps = us, 40
    for i in range(1, steps + 1):
        put(start + (target - start) * i / steps)
        time.sleep(seconds / steps)


def show(msg=''):
    m = '  '.join(f'{k}={v if v is not None else "-"}' for k, v in marks.items())
    print(f'\r{name} ch{args.channel}: {us:4d} us   [{m}]  {msg}'.ljust(78), end='', flush=True)


put(us)
print(__doc__.split('Keys')[1].split('Limits:')[0].replace(' (no Enter needed):', 'Keys:'))
old = termios.tcgetattr(sys.stdin)
try:
    tty.setcbreak(sys.stdin.fileno())
    show()
    while True:
        if not select.select([sys.stdin], [], [], 0.1)[0]:
            pca.set_us(args.channel, us)            # keep refreshing
            continue
        k = sys.stdin.read(1)
        step = {'j': -1, 'k': 1, 'd': -5, 'u': 5, 'D': -25, 'U': 25}.get(k)
        if step:
            put(us + step); show()
        elif k == 'c':
            marks['center'] = us; show('centre marked')
        elif k == '[':
            marks['low'] = us; show('low bind marked')
        elif k == ']':
            marks['high'] = us; show('high bind marked')
        elif k == 'g' and marks['center'] is not None:
            glide(marks['center']); show('at centre')
        elif k == 's':
            if None in marks.values():
                show('mark centre, [ and ] first')
                continue
            c, lo, hi = marks['center'], marks['low'] + args.margin, marks['high'] - args.margin
            for target in (c, lo, hi, c):
                glide(target, 1.5); show('sweep'); time.sleep(0.4)
            show('sweep done')
        elif k == 'p':
            show()
        elif k == 'q':
            break
finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
    back = marks['center'] if marks['center'] is not None else args.start
    try:
        glide(back, 0.8)
    finally:
        pca.close()        # the PCA9685 keeps outputting this pulse

print('\n' + '=' * 60)
c, lo, hi = marks['center'], marks['low'], marks['high']
if c is None:
    print('No centre marked: nothing to write.')
    sys.exit(0)
print(f'Put these in auv_bringup/config/torpedo.yaml ({name}):')
print(f'    {name}_channel: {args.channel}')
print(f'    {name}_center_us: {c}')
if lo is not None and hi is not None:
    mn, mx = lo + args.margin, hi - args.margin
    if not mn < c < mx:
        print(f'  !! centre {c} is not between the limits {mn}..{mx}: re-mark')
    else:
        span = min(c - mn, mx - c)
        print(f'    {name}_min_us: {mn}')
        print(f'    {name}_max_us: {mx}')
        print(f'    {name}_span_us: {span}       # full stick reaches the nearer limit')
        print(f'  travel: {c - mn} us down, {mx - c} us up from centre')
else:
    print('  (limits not marked: keep the min_us / max_us placeholders for now)')
    