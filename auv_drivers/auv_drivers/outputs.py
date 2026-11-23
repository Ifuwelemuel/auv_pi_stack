"""Output stages for thruster_driver: where the pulse widths go (D18, D19).

Each output names its stage (actuators.py): the Pixhawk AUX outputs with the
on-board watchdog, or the PCA9685. Both stages, and the Stages wrapper that
combines them, take pulses as {output name: us} and offer the same methods,
so the driver's safety logic does not care what is fitted where:
    needs_check        True if begin_check()/check() must pass before arming
    start_streaming()  from now on, send what write() was given
    write(pulses)      new pulse widths
    tick()             called at 20 Hz
    health(now)        list of (key, warning) for the 1 Hz health log
    label(name)        where that thruster is wired, for log lines
    close()
"""
import time

from rclpy.qos import (QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)

from auv_drivers.actuators import failsafe
from auv_drivers.rc_override import (build_override, explain_failsafe,
                                     explain_follow, output_mismatches,
                                     validate_map)


def make_output(node, outs, param, now):
    """outs: output name -> config (actuators.py), each with its 'stage';
    param(name) reads a node param; now() returns monotonic seconds.
    Returns one object that sends every output to its own stage."""
    groups = {}
    for n, c in outs.items():
        groups.setdefault(c['stage'], {})[n] = c
    stages = {}
    if 'pixhawk' in groups:
        stages['pixhawk'] = PixhawkOutput(node, groups['pixhawk'], param, now)
    if 'pca9685' in groups:
        stages['pca9685'] = Pca9685Output(node, groups['pca9685'], param)
    return Stages(stages, groups)


class Stages:
    """Pixhawk and/or PCA9685 behind the interface of a single stage."""

    def __init__(self, stages, groups):
        self._stages = stages
        self._of = {n: stages[st] for st, g in groups.items() for n in g}
        self._names = {st: set(g) for st, g in groups.items()}
        self.needs_check = any(s.needs_check for s in stages.values())

    def begin_check(self):
        for s in self._stages.values():
            if s.needs_check:
                s.begin_check()

    def check(self):
        for s in self._stages.values():
            if s.needs_check:
                return s.check()      # only the Pixhawk stage has a check
        return 'ok', 'Nothing to check.'

    def label(self, name):
        return self._of[name].label(name)

    def start_streaming(self):
        for s in self._stages.values():
            s.start_streaming()

    def write(self, pulses):
        for st, s in self._stages.items():
            s.write({n: us for n, us in pulses.items() if n in self._names[st]})

    def tick(self):
        for s in self._stages.values():
            s.tick()

    def health(self, now):
        return [w for s in self._stages.values() for w in s.health(now)]

    def close(self):
        for s in self._stages.values():
            s.close()


class Pca9685Output:
    """PCA9685 on I2C. Holds the last pulse by itself, so nothing to stream.
    Its RC oscillator drifts (D5); kept as a fallback."""

    needs_check = False

    def __init__(self, node, outs, param):
        from auv_drivers.pca9685 import PCA9685
        self._log = node.get_logger()
        self._chan = {n: c['channel'] for n, c in outs.items()}
        if len(set(self._chan.values())) != len(self._chan):
            raise RuntimeError(f'Two thrusters share a PCA9685 channel: {self._chan}')
        self._pca = PCA9685(param('i2c_bus'), param('i2c_addr'), param('pwm_freq_hz'),
                            param('oscillator_hz'))
        for attempt in range(1, 6):
            ok, got, want, asleep = self._pca.verify()
            if ok:
                self._log.info(f'PCA9685 verified (prescale={got}, awake).')
                return
            self._log.warn(f'PCA9685 not configured (prescale {got}/{want}, '
                           f'asleep={asleep}), re-init {attempt}/5')
            time.sleep(0.2)
            try:
                self._pca.init()
            except OSError as e:
                self._log.warn(f're-init raised: {e}')
        raise RuntimeError('PCA9685 failed to configure: check I2C wiring '
                           '(i2cdetect -y 1 should show 0x40).')

    def label(self, name):
        return f'ch{self._chan[name]}'

    def start_streaming(self):
        pass

    def write(self, pulses):
        try:
            for n, us in pulses.items():
                self._pca.set_us(self._chan[n], us)
        except OSError as e:
            self._log.error(f'I2C write failed: {e}')

    def tick(self):
        pass

    def health(self, now):
        return []

    def close(self):
        self._pca.close()


OVERRIDE_QOS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                          history=QoSHistoryPolicy.KEEP_LAST)


class PixhawkOutput:
    """Pixhawk AUX1-3 via RC overrides and pixhawk/auv_watchdog.lua.

    Sends nothing until start_streaming(): during the start-up check the Pi
    must be silent, so /mavros/rc/out shows the Pixhawk's own failsafe stops.
    """

    needs_check = True
    SETTLE_S = 0.5        # > AUVW_TMO_MS: a previous driver's stream has expired
    TOL_US = 2
    RC_OUT_STALE_S = 2.0
    STEADY_S = 0.6        # compare commanded vs reported only once settled

    def __init__(self, node, outs, param, now):
        from mavros_msgs.msg import OverrideRCIn, RCOut
        self._msg = OverrideRCIn
        self._now = now
        self._aux = {n: int(c['aux']) for n, c in outs.items()}
        self._toggle_rc = int(param('toggle_rc'))
        try:
            validate_map(self._aux, self._toggle_rc)
        except ValueError as e:
            raise RuntimeError(str(e)) from None
        # what the Pixhawk must hold on each AUX while the Pi is silent
        self._stops = {self._aux[n]: failsafe(c) for n, c in outs.items()}
        self._check_enabled = bool(param('watchdog_check'))
        self._wait_s = float(param('watchdog_wait_s'))
        self._pub = node.create_publisher(OverrideRCIn, '/mavros/rc/override',
                                          OVERRIDE_QOS)
        # mavros sensor-style topics: best effort receives from any publisher
        node.create_subscription(RCOut, '/mavros/rc/out', self._on_rc_out,
                                 qos_profile_sensor_data)
        self._streaming = False
        self._toggle = False
        self._by_aux = None
        self._changed_t = None
        self._rc_out = None
        self._rc_out_t = None
        self._check_t0 = None

    def label(self, name):
        return f'AUX{self._aux[name]}'

    # ----------------------------------------------------------- feedback
    def _on_rc_out(self, msg):
        self._rc_out = list(msg.channels)
        self._rc_out_t = self._now()

    def begin_check(self):
        self._check_t0 = self._now()

    def check(self):
        """('wait', None) | ('ok', text) | ('skipped', text) | ('fail', text)."""
        if not self._check_enabled:
            return 'skipped', ('watchdog_check is false: NOT verifying the Pixhawk '
                               'failsafe. Bench debugging only.')
        now = self._now()
        latest = None
        if self._rc_out_t is not None and self._rc_out_t - self._check_t0 >= self.SETTLE_S:
            latest = self._rc_out
            bad = output_mismatches(latest, self._stops, self.TOL_US)
            if not bad:
                stops = ', '.join(f'AUX{a}={us}' for a, us in sorted(self._stops.items()))
                return 'ok', f'Pixhawk watchdog is holding every output at its failsafe ({stops} us).'
        if now - self._check_t0 < self._wait_s:
            return 'wait', None
        if latest is None:
            return 'fail', (f'No /mavros/rc/out from the Pixhawk within {self._wait_s:.0f} s. '
                            'Is mavros running and connected (mavros:=true, fcu_url)? '
                            'Thrusters will not be driven.')
        return 'fail', ('The Pixhawk failsafe does not match calibration.yaml; '
                        'thrusters will not be driven:\n  '
                        + '\n  '.join(explain_failsafe(bad)))

    # ----------------------------------------------------------- commands
    def start_streaming(self):
        self._streaming = True
        self._send()

    def write(self, pulses):
        by_aux = {self._aux[n]: int(round(us)) for n, us in pulses.items()}
        if by_aux != self._by_aux:
            self._by_aux = by_aux
            self._changed_t = self._now()
            self._send()

    def tick(self):
        # The toggle flips HERE ONLY, at a steady 20 Hz. The script samples it
        # at 50 Hz; extra flips from write() would come in pairs a few ms
        # apart, cancel out between two samples, and look like a dead Pi.
        self._toggle = not self._toggle
        self._send()

    def _send(self):
        """Publish the current pulses. write() calls this for low latency,
        repeating the current toggle value."""
        if not self._streaming or self._by_aux is None:
            return
        msg = self._msg()
        msg.channels = build_override(self._by_aux, self._toggle, self._toggle_rc)
        self._pub.publish(msg)

    def health(self, now):
        if not self._streaming:
            return []
        if self._rc_out_t is None or now - self._rc_out_t > self.RC_OUT_STALE_S:
            return [('link', 'No /mavros/rc/out from the Pixhawk: mavros or the '
                             'TELEM2 link is down. The Pixhawk watchdog stops the '
                             'thrusters when our overrides stop arriving.')]
        if (self._by_aux is not None and now - self._changed_t >= self.STEADY_S
                and self._rc_out_t - self._changed_t >= self.STEADY_S - 0.2):
            bad = output_mismatches(self._rc_out, self._by_aux, self.TOL_US)
            if bad:
                return [('follow', 'Pixhawk AUX outputs do not follow the commands ('
                                   + explain_follow(bad) + '). Is auv_watchdog.lua '
                                   'running and AUVW_ENABLE = 1?')]
        return []

    def close(self):
        pass
