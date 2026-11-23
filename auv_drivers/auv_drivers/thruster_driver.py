"""Thruster driver: the ONLY node that commands the ESCs and fin servos.

Subscribes
    /cmd/actuators   auv_interfaces/ActuatorCommand  (from actuator_mixer only)
    /estop           std_msgs/Bool, latched           (from safety_supervisor)

Outputs (parameter `outputs`, D19, see actuators.py). Each is an ESC or a
servo that follows one of surge / pitch / yaw, with its own measured
calibration and its own stage (`*_stage`, default `output`):
    three_thruster (calibration.yaml)   surge, pitch, yaw ESCs on Pixhawk AUX
    torpedo (torpedo.yaml on top)       surge ESC on Pixhawk AUX3
                                        + fin_top, fin_bottom (yaw) and
                                        fin_left, fin_right (pitch) on PCA9685

Output stages (D18, see outputs.py):
    pixhawk  Pixhawk AUX1-3, crystal-timed PWM. Pulse widths go to the
             Pixhawk as RC overrides through mavros, with a toggle channel
             flipped on every message. pixhawk/auv_watchdog.lua passes them to
             the AUX pins only while the toggle keeps flipping, and stops every
             ESC within AUVW_TMO_MS if this node, mavros or the Pi dies.
    pca9685  the PCA9685 on I2C. Its oscillator drifts (D5), and it holds the
             last pulse forever if the Pi dies: fine for fin servos (a few
             us of drift is a fraction of a degree; a stuck fin with the
             Pixhawk-stopped thruster does nothing), not for ESCs.

Start-up, one state after another, at 20 Hz (never blocks the executor):
    check  pixhawk only. Send nothing; read /mavros/rc/out and confirm the
           Pixhawk is already holding each ESC at the stop pulse this file
           expects. If not, log why and exit: a failsafe "stop" that is not
           the ESC's real stop pulse would be a running thruster.
    arm    stream stop on all ESCs for esc_arm_hold_s
    ready  follow /cmd/actuators

Stops all thrusters and centres all fins (each at its OWN neutral) when
    * /estop is true — independently of the mixer (defence in depth)
    * no /cmd/actuators for cmd_timeout_s — the mixer publishes at 20 Hz,
      so silence means it crashed
    * shutting down
"""
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Bool

from auv_interfaces.msg import ActuatorCommand
from auv_drivers import actuators
from auv_drivers.outputs import make_output

# Nominal defaults only. Real values are MEASURED and live in calibration.yaml
# (and torpedo.yaml for the fins).
#   channel = PCA9685 channel 0..15 (stage pca9685)
#   aux     = Pixhawk AUX output 1..3 (stage pixhawk)
WIRING = {'surge': (2, 3), 'pitch': (1, 2), 'yaw': (0, 1)}   # name: (channel, aux)
KIND_DEFAULTS = {
    'esc': dict(stop_us=1500, span_us=400, bidirectional=True),
    'servo': dict(center_us=1500, span_us=300, min_us=1100, max_us=1900,
                  failsafe_us=1500),
}

TICK_S = 0.05          # 20 Hz: stream rate to the Pixhawk, state machine step
HEALTH_REPEAT_S = 5.0  # repeat a persisting health warning this often

LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST)
CMD_QOS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST)


class ThrusterDriver(Node):

    def __init__(self):
        super().__init__('thruster_driver')
        # Timeouts use the MONOTONIC clock: a wall-clock step (NTP sync on a Pi
        # with no RTC) must never make stale data look fresh (D16).
        self._mono = Clock(clock_type=ClockType.STEADY_TIME)
        self.declare_parameter('output', 'pixhawk')
        # output: pca9685
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_addr', 0x40)
        self.declare_parameter('oscillator_hz', 25_000_000)   # nominal; see D5
        self.declare_parameter('pwm_freq_hz', 50)
        # output: pixhawk
        self.declare_parameter('toggle_rc', 16)
        self.declare_parameter('watchdog_check', True)
        self.declare_parameter('watchdog_wait_s', 10.0)
        # both
        self.declare_parameter('esc_arm_hold_s', 2.0)
        self.declare_parameter('cmd_timeout_s', 0.5)
        self.declare_parameter('outputs', ['surge', 'pitch', 'yaw'])

        p = lambda k: self.get_parameter(k).value   # noqa: E731
        self._outs = {n: self._declare_output(n) for n in p('outputs')}
        try:
            actuators.validate(self._outs)
        except ValueError as e:
            raise RuntimeError(f'Bad output configuration: {e}') from None
        self._neutral = {n: actuators.neutral(c) for n, c in self._outs.items()}
        self._cmd_timeout = p('cmd_timeout_s')
        self._arm_hold = p('esc_arm_hold_s')

        self._out = make_output(self, self._outs, p, self._now)

        self._estopped = True
        self._last_cmd_time = None
        self._timed_out = True
        self._warned = {}
        self._state = None
        self._out.write(self._neutral)        # PCA: written now; Pixhawk: queued
        self._enter('check' if self._out.needs_check else 'arm')

        self.create_subscription(Bool, '/estop', self._on_estop, LATCHED)
        self.create_subscription(ActuatorCommand, '/cmd/actuators', self._on_cmd,
                                 CMD_QOS)
        self.create_timer(TICK_S, self._tick)
        self.create_timer(1.0, self._health)

    def _declare_output(self, n):
        """Declare and read the parameters of output n: {n}_kind, _stage,
        _input, _aux, _channel, _sign, then the kind's own (stop/centre,
        span...). {n}_stage defaults to the `output` parameter."""
        chan, aux = WIRING.get(n, (-1, 0))
        kind = self.declare_parameter(f'{n}_kind', 'esc').value
        cfg = {'kind': kind,
               'stage': self.declare_parameter(f'{n}_stage',
                                               self.get_parameter('output').value).value,
               'input': self.declare_parameter(f'{n}_input', n).value,
               'aux': self.declare_parameter(f'{n}_aux', aux).value,
               'channel': self.declare_parameter(f'{n}_channel', chan).value,
               'sign': self.declare_parameter(f'{n}_sign', 1).value}
        for key, val in KIND_DEFAULTS.get(kind, {}).items():
            cfg[key] = self.declare_parameter(f'{n}_{key}', val).value
        return cfg

    # ------------------------------------------------------------ states
    def _now(self):
        return self._mono.now().nanoseconds * 1e-9

    def _enter(self, state):
        self._state = state
        self._state_t = self._now()
        log = self.get_logger()
        if state == 'check':
            self._out.begin_check()
            log.info('Checking the Pixhawk watchdog before sending anything '
                     '(waiting for /mavros/rc/out)...')
        elif state == 'arm':
            self._stop_all()
            self._out.start_streaming()
            log.info(f'Streaming stop on all ESCs for {self._arm_hold} s. '
                     f'Switch the thruster battery on now if it is off.')
        elif state == 'ready':
            log.info('Thruster driver ready, booted STOPPED. ' + ', '.join(
                f'{n}: {self._out.label(n)} {actuators.describe(c)}'
                for n, c in self._outs.items()))

    def _tick(self):
        if self._state == 'check':
            verdict, text = self._out.check()
            if verdict == 'wait':
                return
            if verdict == 'fail':
                raise RuntimeError(text)
            (self.get_logger().warn if verdict == 'skipped' else self.get_logger().info)(text)
            self._enter('arm')
            return
        if self._state == 'arm' and self._now() - self._state_t >= self._arm_hold:
            self._enter('ready')
        self._check_timeout()
        self._out.tick()

    # ------------------------------------------------------------ inputs
    def _on_estop(self, msg):
        self._estopped = msg.data
        if self._estopped:
            self._stop_all()

    def _on_cmd(self, msg):
        if self._state != 'ready':
            return                      # arming: stop pulses only
        self._last_cmd_time = self._now()
        if self._timed_out:
            self.get_logger().info('Commands arriving.')
            self._timed_out = False
        if self._estopped:
            self._stop_all()
            return
        cmds = {'surge': msg.thrust_surge, 'pitch': msg.thrust_pitch,
                'yaw': msg.thrust_yaw}
        self._out.write({n: actuators.pulse(c, cmds[c['input']])
                         for n, c in self._outs.items()})

    def _check_timeout(self):
        if self._last_cmd_time is None or self._timed_out:
            return
        age = self._now() - self._last_cmd_time
        if age > self._cmd_timeout:
            self._timed_out = True
            self.get_logger().error(
                f'No /cmd/actuators for {age:.2f} s (mixer dead?) — STOPPING.')
            self._stop_all()

    def _health(self):
        now = self._now()
        active = set()
        for key, text in self._out.health(now):
            active.add(key)
            if now - self._warned.get(key, -1e9) >= HEALTH_REPEAT_S:
                self.get_logger().error(text)
                self._warned[key] = now
        for key in list(self._warned):
            if key not in active:
                del self._warned[key]

    def _stop_all(self):
        self._out.write(self._neutral)

    def destroy_node(self):
        try:
            self._stop_all()
            self._out.tick()
            self._out.close()
        except Exception:
            pass    # shutting down; the Pixhawk watchdog stops the ESCs anyway
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ThrusterDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        get_logger('thruster_driver').fatal(str(e))
        raise SystemExit(1)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
