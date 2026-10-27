#!/bin/bash
# Check the Pi's I2C bus sees exactly what it should (D3).
#   0x40  PCA9685        (0x70 = its all-call address, normal)
#   0x76  Bar02 (MS5837)
# Anything else, or a slow/garbled scan, suggests a second master (Pixhawk
# wired to this bus) or a wiring fault.
set -e
BUS=${1:-1}
i2cdetect -y "$BUS"
OUT=$(i2cdetect -y "$BUS")
for addr in 40 76; do
  if echo "$OUT" | grep -qE "(^| )$addr( |$)"; then echo "OK   0x$addr present"
  else echo "FAIL 0x$addr missing"; fi
done
