"""Minimal PCA9685 driver over smbus2, with read-back verification.

Two hard-won lessons live here (see docs/DECISIONS.md, D5 and D6):
  * The chip's internal oscillator is NOT 25 MHz on every board. This board
    measured 28.73 MHz, which scaled every pulse by ~15% and stopped the ESC
    arming. The prescale must be computed from the MEASURED oscillator.
  * A failed init is silent: the chip stays asleep, every write "succeeds",
    and nothing moves. verify() reads the configuration back.
"""
import time

from smbus2 import SMBus

MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
RESTART, SLEEP, AI, ALLCALL = 0x80, 0x10, 0x20, 0x01
OUTDRV = 0x04


class PCA9685:

    def __init__(self, bus_num, addr=0x40, freq_hz=50, osc_hz=25_000_000):
        self._bus = SMBus(bus_num)
        self._addr = addr
        self._freq = freq_hz
        self._prescale = int(round(osc_hz / (4096.0 * freq_hz)) - 1)
        self.init()

    def _w(self, reg, val):
        self._bus.write_byte_data(self._addr, reg, val)

    def _r(self, reg):
        return self._bus.read_byte_data(self._addr, reg)

    def init(self):
        """Totem-pole outputs, wake, set frequency. PRESCALE can only be
        written while asleep, hence sleep / write / restore."""
        self._w(MODE2, OUTDRV)
        self._w(MODE1, ALLCALL)
        time.sleep(0.005)
        self._w(MODE1, self._r(MODE1) & ~SLEEP)
        time.sleep(0.005)
        old = self._r(MODE1)
        self._w(MODE1, (old & 0x7F) | SLEEP)
        self._w(PRESCALE, self._prescale)
        self._w(MODE1, old)
        time.sleep(0.005)
        self._w(MODE1, old | RESTART | AI)

    def verify(self):
        """(ok, prescale_read, prescale_wanted, asleep)."""
        try:
            got = self._r(PRESCALE)
            asleep = bool(self._r(MODE1) & SLEEP)
        except OSError:
            return False, -1, self._prescale, True
        return (got == self._prescale and not asleep), got, self._prescale, asleep

    def set_us(self, channel, microseconds):
        ticks = int(4096 * microseconds / (1_000_000.0 / self._freq))
        ticks = max(0, min(4095, ticks))
        base = LED0_ON_L + 4 * channel
        self._w(base + 0, 0)
        self._w(base + 1, 0)
        self._w(base + 2, ticks & 0xFF)
        self._w(base + 3, (ticks >> 8) & 0x0F)

    def close(self):
        self._bus.close()
