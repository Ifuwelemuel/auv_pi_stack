"""Thruster driver: the ONLY node that writes to the PCA9685.

Subscribes
    /cmd/actuators   auv_interfaces/ActuatorCommand  (from actuator_mixer only)
    /estop           std_msgs/Bool, latched           (from safety_supervisor)

Three ESCs, each with its own measured calibration (config/calibration.yaml):
    surge  stern T200, bidirectional
    pitch  generic thruster, FORWARD-ONLY ESC
    yaw    lateral T200, bidirectional

Stops all thrusters (each at its OWN stop pulse) when:
    * booting — before any subscription exists
    * /estop is true — independently of the mixer (defence in depth)
    * no /cmd/actuators for cmd_timeout_s — the mixer publishes at 20 Hz,
      so silence means it crashed. Without this the PCA9685 would keep
      outputting the last pulse forever.
    * shutting down
"""
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Bool

from auv_interfaces.msg import ActuatorCommand
from auv_drivers.esc import thrust_to_us
from auv_drivers.pca9685 import PCA9685

THRUSTERS = ('surge', 'pitch', 'yaw')

# Nominal defaults only. Real values are MEASURED and live in calibration.yaml.
DEFAULTS = {
    'surge': dict(channel=2, stop_us=1500, span_us=400, sign=1, bidirectional=True),
    'pitch': dict(channel=1, stop_us=1000, span_us=1000, sign=1, bidirectional=False),
    'yaw': dict(channel=0, stop_us=1500, span_us=400, sign=1, bidirectional=True),
}

LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST)
CMD_QOS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST)


class ThrusterDriver(Node):

    def __init__(self):
        super().__init__('thruster_driver')
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_addr', 0x40)
        self.declare_parameter('oscillator_hz', 25_000_000)   # nominal; see D5
        self.declare_parameter('pwm_freq_hz', 50)
        self.declare_parameter('esc_arm_hold_s', 2.0)
        self.declare_parameter('cmd_timeout_s', 0.5)
        for name in THRUSTERS:
            for key, val in DEFAULTS[name].items():
                self.declare_parameter(f'{name}_{key}', val)

        p = lambda k: self.get_parameter(k).value   # noqa: E731
        self._thr = {n: {k: p(f'{n}_{k}') for k in DEFAULTS[n]} for n in THRUSTERS}
        chans = [t['channel'] for t in self._thr.values()]
        if len(set(chans)) != len(chans):
            raise RuntimeError(f'Two thrusters share a PCA9685 channel: {self._thr}')
        self._cmd_timeout = p('cmd_timeout_s')

        # --- hardware ---------------------------------------------------
        self._pca = PCA9685(p('i2c_bus'), p('i2c_addr'), p('pwm_freq_hz'),
                            p('oscillator_hz'))
        for attempt in range(1, 6):
            ok, got, want, asleep = self._pca.verify()
            if ok:
                self.get_logger().info(f'PCA9685 verified (prescale={got}, awake).')
                break
            self.get_logger().warn(
                f'PCA9685 not configured (prescale {got}/{want}, asleep={asleep}),'
                f' re-init {attempt}/5')
            time.sleep(0.2)
            try:
                self._pca.init()
            except OSError as e:
                self.get_logger().warn(f're-init raised: {e}')
        else:
            raise RuntimeError('PCA9685 failed to configure — check I2C wiring '
                               '(i2cdetect -y 1 should show 0x40).')

        # --- safe before anything else, then arm ------------------------
        self._estopped = True
        self._last_cmd_time = None
        self._timed_out = True
        self._stop_all()
        self.get_logger().info(
            f'Streaming stop on all ESCs for {p("esc_arm_hold_s")} s — '
            f'switch the thruster battery on now if it is off.')
        time.sleep(p('esc_arm_hold_s'))

        self.create_subscription(Bool, '/estop', self._on_estop, LATCHED)
        self.create_subscription(ActuatorCommand, '/cmd/actuators', self._on_cmd,
                                 CMD_QOS)
        self.create_timer(0.1, self._check_timeout)
        self.get_logger().info('Thruster driver ready, booted STOPPED. ' + ', '.join(
            f"{n}: ch{t['channel']} stop={t['stop_us']} span={t['span_us']} "
            f"sign={t['sign']:+d} {'bi' if t['bidirectional'] else 'FWD-ONLY'}"
            for n, t in self._thr.items()))

    # ------------------------------------------------------------------
    def _on_estop(self, msg):
        self._estopped = msg.data
        if self._estopped:
            self._stop_all()

    def _on_cmd(self, msg):
        self._last_cmd_time = self.get_clock().now()
        if self._timed_out:
            self.get_logger().info('Commands arriving.')
            self._timed_out = False
        if self._estopped:
            self._stop_all()
            return
        cmds = {'surge': msg.thrust_surge, 'pitch': msg.thrust_pitch,
                'yaw': msg.thrust_yaw}
        try:
            for n, t in self._thr.items():
                self._pca.set_us(t['channel'], thrust_to_us(
                    cmds[n], t['stop_us'], t['span_us'], t['sign'],
                    t['bidirectional']))
        except OSError as e:
            self.get_logger().error(f'I2C write failed: {e}')

    def _check_timeout(self):
        if self._last_cmd_time is None or self._timed_out:
            return
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if age > self._cmd_timeout:
            self._timed_out = True
            self.get_logger().error(
                f'No /cmd/actuators for {age:.2f} s (mixer dead?) — STOPPING.')
            self._stop_all()

    def _stop_all(self):
        try:
            for t in self._thr.values():
                self._pca.set_us(t['channel'], t['stop_us'])
        except OSError as e:
            self.get_logger().error(f'I2C write failed while stopping: {e}')

    def destroy_node(self):
        self._stop_all()
        try:
            self._pca.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
