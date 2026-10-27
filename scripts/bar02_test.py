#!/usr/bin/env python3
"""Bar02 smoke test, no ROS. Prove the sensor before anything else uses it.

    python3 scripts/bar02_test.py
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'auv_drivers'))
from auv_drivers import ms5837  # noqa: E402

s = ms5837.MS5837_02BA(bus=1)     # Pi I2C bus 1, via the PCA9685 pass-through
if not s.init():
    raise SystemExit('Bar02 init failed — run scripts/i2c_check.sh; expect 0x76.')
print('Bar02 up. Ctrl-C to stop.')
while True:
    if s.read():
        print(f'pressure {s.pressure():8.2f} mbar   temp {s.temperature():5.2f} C   '
              f'depth(fresh, vs 1013 mbar) {s.depth():+6.3f} m')
    else:
        print('read failed')
    time.sleep(0.2)
