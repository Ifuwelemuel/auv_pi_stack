#!/usr/bin/env python3
"""Find which PCA9685 channel each fin servo is on.

Wiggles ONE channel at a time (centre -> +swing -> -swing -> centre), the
others held at centre, and asks which fin moved. At the end it prints the
lines for config/torpedo.yaml.

    ~/auv_ws/src/auv_pi_stack/scripts/kill_stack.sh      # driver must not run
    python3 ~/auv_ws/src/auv_pi_stack/scripts/pca_servo_finder.py
    python3 .../pca_servo_finder.py --channels 0-3 --swing 100

Before running: servo BEC on; NO ESC on the PCA9685 (or its battery off):
a wiggle on an ESC channel would spin that thruster.

Answers after each wiggle:
    t top   b bottom   l left   r right   (the fin that moved)
    n nothing moved    a again (wiggle the same channel once more)
    q quit
Ctrl+C at any time: every scanned channel goes back to centre.
"""
import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'auv_drivers'))   # works unsourced
from auv_drivers.pca9685 import PCA9685                       # noqa: E402

FINS = {'t': 'fin_top', 'b': 'fin_bottom', 'l': 'fin_left', 'r': 'fin_right'}


def channel_list(text):
    out = []
    for part in text.split(','):
        a, _, b = part.partition('-')
        out += list(range(int(a), int(b or a) + 1))
    if not out or min(out) < 0 or max(out) > 15:
        raise argparse.ArgumentTypeError('channels must be within 0-15')
    return out


ap = argparse.ArgumentParser(description=__doc__,
                             formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('--channels', type=channel_list, default=channel_list('0-15'),
                help='e.g. 0-3 or 0,1,4 (default 0-15)')
ap.add_argument('--center', type=int, default=1500, help='centre pulse, us')
ap.add_argument('--swing', type=int, default=150,
                help='wiggle amplitude, us (default 150: small, safe for linkages)')
ap.add_argument('--bus', type=int, default=1)
ap.add_argument('--osc', type=int, default=28_730_000,
                help='MEASURED PCA9685 oscillator, Hz (D5; calibration.yaml)')
args = ap.parse_args()

if subprocess.run(['pgrep', '-f', 'lib/auv_drivers/thruster_driver'],
                  capture_output=True).returncode == 0:
    sys.exit('thruster_driver is running and owns the PCA9685. '
             'Run scripts/kill_stack.sh first.')
if not 50 <= args.swing <= 400:
    sys.exit('--swing must be 50..400 us')

print(__doc__.split('Before running:')[1].split('Answers')[0].strip())
if input('\nNo ESC on the PCA (or its battery off), servo BEC on? [y/N] ').strip().lower() != 'y':
    sys.exit('Stopped.')

pca = PCA9685(args.bus, 0x40, 50, args.osc)
ok, got, want, asleep = pca.verify()
if not ok:
    sys.exit(f'PCA9685 not configured (prescale {got}/{want}, asleep={asleep}). '
             f'Check I2C: i2cdetect -y {args.bus} should show 40.')


def centre_all():
    for c in args.channels:
        pca.set_us(c, args.center)


def ramp(ch, start, end, seconds=0.4):
    steps = 20
    for i in range(1, steps + 1):
        pca.set_us(ch, start + (end - start) * i / steps)
        time.sleep(seconds / steps)


def wiggle(ch):
    c, s = args.center, args.swing
    ramp(ch, c, c + s)
    time.sleep(0.3)
    ramp(ch, c + s, c - s, 0.8)
    time.sleep(0.3)
    ramp(ch, c - s, c)


found = {}
try:
    centre_all()
    print(f'\nAll of channels {args.channels} centred at {args.center} us.')
    time.sleep(1.0)
    for ch in args.channels:
        while True:
            print(f'\nChannel {ch}: wiggling ...', flush=True)
            wiggle(ch)
            ans = input(f'Channel {ch}: which fin moved? '
                        '[t]op [b]ottom [l]eft [r]ight, [n]othing, [a]gain, [q]uit: ').strip().lower()
            if ans == 'a':
                continue
            if ans in FINS:
                fin = FINS[ans]
                if fin in found.values():
                    prev = [k for k, v in found.items() if v == fin][0]
                    print(f'  note: {fin} was already on channel {prev}; now channel {ch}')
                    del found[prev]
                found[ch] = fin
                print(f'  channel {ch} = {fin}')
            break
        if ans == 'q':
            break
finally:
    centre_all()
    pca.close()        # the PCA9685 keeps outputting the centre pulses

print('\n' + '=' * 60)
if not found:
    print('No fins identified. Check the servo BEC, the signal wires and that the')
    print('servos are on the PCA9685 (not the Pixhawk).')
else:
    print('Fins found:')
    for ch in sorted(found):
        print(f'  PCA ch{ch}: {found[ch]}')
    missing = [f for f in FINS.values() if f not in found.values()]
    if missing:
        print('Not found: ' + ', '.join(missing))
    print('\nPut these in auv_bringup/config/torpedo.yaml:')
    for ch in sorted(found):
        print(f'    {found[ch]}_channel: {ch}')
    print('\nNext: find each fin\'s centre and limits, e.g.')
    first = min(found)
    print(f'    python3 {os.path.join(HERE, "esc_calibrate.py")} --channel {first} '
          f'--stop 1500 --min 1100 --max 1900')
