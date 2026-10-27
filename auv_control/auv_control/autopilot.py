"""Autopilot: heading and depth hold -> /cmd/autonomy.

Subscribes
    <imu_topic>                 sensor_msgs/Imu    (mavros: yaw + yaw rate)
    /depth                      Float32  m, +down  (depth_sensor)
    /autopilot/heading_setpoint Float32  rad, ENU (CCW from east)
    /autopilot/depth_setpoint   Float32  m, +down
    /autopilot/surge_setpoint   Float32  normalised [-1, 1]
Publishes
    /cmd/autonomy               ActuatorCommand at rate_hz

Setpoints EXPIRE after setpoint_timeout_s: whoever is steering (navigator,
or you with `ros2 topic pub -r 5 ...`) must keep sending them. The
autopilot publishes while ANY setpoint is fresh; each axis without a fresh
setpoint, or whose sensor is stale, is commanded 0. If the heading loop is
not running, surge is also 0 — never drive forward unsteered.

Heading -> yaw thruster. Yaw authority does not depend on speed, so the
vehicle can turn on the spot. Optional pivot: above pivot_error_rad of
heading error, surge is held at 0 until the nose comes round (0 = off).
D-term uses the gyro yaw rate (no derivative kick on setpoint steps).

Depth -> pitch thruster, FORWARD-ONLY: output limited to
[depth_out_min, depth_out_max] (nose-down mounting: [0, 1]) with
anti-windup (pid.py). Positive buoyancy provides the way back up.
D-term uses a low-pass filtered depth rate.

Integrators reset only when a loop (re)starts or its setpoint jumps by more
than *_reset_threshold — not on every setpoint message, since the navigator
re-sends at 5 Hz and that would make the I-term useless.
"""
import math

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

from auv_interfaces.msg import ActuatorCommand
from auv_control.geo import wrap, yaw_from_quat
from auv_control.pid import Pid
from auv_control.qos import COMMAND


class Autopilot(Node):

    def __init__(self):
        super().__init__('autopilot')
        params = {
            'rate_hz': 10.0, 'imu_topic': '/mavros/imu/data',
            'imu_timeout_s': 0.5, 'depth_timeout_s': 0.7,
            'setpoint_timeout_s': 1.0,
            'heading_kp': 0.4, 'heading_ki': 0.0, 'heading_kd': 0.2,
            'heading_i_limit': 0.2, 'heading_reset_threshold_rad': 0.5,
            'pivot_error_rad': 0.0,
            'depth_kp': 0.6, 'depth_ki': 0.1, 'depth_kd': 0.4,
            'depth_i_limit': 0.3, 'depth_reset_threshold_m': 0.2,
            'depth_out_min': 0.0, 'depth_out_max': 1.0,
            'depth_rate_alpha': 0.3, 'max_depth_m': 2.0,
        }
        for k, v in params.items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value   # noqa: E731
        self._dt = 1.0 / p('rate_hz')
        self._imu_to, self._depth_to = p('imu_timeout_s'), p('depth_timeout_s')
        self._sp_to = p('setpoint_timeout_s')
        self._pivot = p('pivot_error_rad')
        self._max_depth = p('max_depth_m')
        self._alpha = p('depth_rate_alpha')
        self._h_reset = p('heading_reset_threshold_rad')
        self._d_reset = p('depth_reset_threshold_m')
        self._hpid = Pid(p('heading_kp'), p('heading_ki'), p('heading_kd'),
                         p('heading_i_limit'), -1.0, 1.0)
        self._dpid = Pid(p('depth_kp'), p('depth_ki'), p('depth_kd'),
                         p('depth_i_limit'), p('depth_out_min'), p('depth_out_max'))

        # latest measurements, each (value, time)
        self._yaw = self._yaw_rate = None
        self._imu_t = None
        self._depth = None
        self._depth_t = None
        self._depth_rate = 0.0
        # setpoints, each (value, time)
        self._sp = {'heading': None, 'depth': None, 'surge': None}
        self._loop_on = {'heading': False, 'depth': False}

        self.create_subscription(Imu, p('imu_topic'), self._on_imu,
                                 qos_profile_sensor_data)
        self.create_subscription(Float32, '/depth', self._on_depth, 10)
        for name in self._sp:
            self.create_subscription(
                Float32, f'/autopilot/{name}_setpoint',
                lambda m, n=name: self._on_setpoint(n, m.data), 10)
        self._pub = self.create_publisher(ActuatorCommand, '/cmd/autonomy', COMMAND)
        self.create_timer(self._dt, self._on_timer)
        self.get_logger().info(f'Autopilot up, IMU on {p("imu_topic")}. '
                               f'Idle until a setpoint arrives.')

    # ---------------------------------------------------------------- inputs
    def _now(self):
        return self.get_clock().now()

    def _age(self, t):
        return math.inf if t is None else (self._now() - t).nanoseconds * 1e-9

    def _on_imu(self, msg):
        q = msg.orientation
        self._yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self._yaw_rate = msg.angular_velocity.z        # FLU z up = ENU yaw rate
        self._imu_t = self._now()

    def _on_depth(self, msg):
        now = self._now()
        if self._depth is not None and self._depth_t is not None:
            dt = (now - self._depth_t).nanoseconds * 1e-9
            if 1e-3 < dt < 1.0:
                raw = (msg.data - self._depth) / dt
                self._depth_rate += self._alpha * (raw - self._depth_rate)
        self._depth, self._depth_t = msg.data, now

    def _on_setpoint(self, name, value):
        if name == 'heading':
            value = wrap(value)
        elif name == 'depth':
            if value > self._max_depth:
                self.get_logger().warn(f'Depth setpoint {value:.2f} m clamped to '
                                       f'ceiling {self._max_depth} m.',
                                       throttle_duration_sec=5.0)
            value = max(0.0, min(self._max_depth, value))
        elif name == 'surge':
            value = max(-1.0, min(1.0, value))
        old = self._sp[name]
        if old is not None:
            jump = abs(wrap(value - old[0])) if name == 'heading' else abs(value - old[0])
            if name == 'heading' and jump > self._h_reset:
                self._hpid.reset()
            if name == 'depth' and jump > self._d_reset:
                self._dpid.reset()
        self._sp[name] = (value, self._now())

    def _fresh_sp(self, name):
        sp = self._sp[name]
        return sp[0] if sp is not None and self._age(sp[1]) <= self._sp_to else None

    def _set_loop(self, name, on, pid):
        if on and not self._loop_on[name]:
            pid.reset()                                  # clean start
            self.get_logger().info(f'{name} loop ON')
        elif not on and self._loop_on[name]:
            self.get_logger().info(f'{name} loop OFF')
        self._loop_on[name] = on

    # ------------------------------------------------------------- control
    def _on_timer(self):
        h_sp, d_sp, s_sp = (self._fresh_sp(n) for n in ('heading', 'depth', 'surge'))
        if h_sp is None and d_sp is None and s_sp is None:
            self._set_loop('heading', False, self._hpid)
            self._set_loop('depth', False, self._dpid)
            return                                       # idle: publish nothing

        cmd = ActuatorCommand()
        cmd.header.stamp = self._now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.source = ActuatorCommand.SOURCE_AUTONOMY

        h_ok = h_sp is not None and self._age(self._imu_t) <= self._imu_to
        if h_sp is not None and not h_ok:
            self.get_logger().warn('IMU stale — heading loop and surge muted.',
                                   throttle_duration_sec=2.0)
        self._set_loop('heading', h_ok, self._hpid)
        if h_ok:
            err = wrap(h_sp - self._yaw)
            cmd.thrust_yaw = self._hpid.update(err, -self._yaw_rate, self._dt)
            pivoting = self._pivot > 0.0 and abs(err) > self._pivot
            cmd.thrust_surge = 0.0 if (pivoting or s_sp is None) else s_sp

        d_ok = d_sp is not None and self._age(self._depth_t) <= self._depth_to
        if d_sp is not None and not d_ok:
            self.get_logger().warn('/depth stale (tared?) — depth loop muted.',
                                   throttle_duration_sec=2.0)
        self._set_loop('depth', d_ok, self._dpid)
        if d_ok:
            err = d_sp - self._depth                     # + = too shallow = dive
            cmd.thrust_pitch = self._dpid.update(err, -self._depth_rate, self._dt)

        self._pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Autopilot()
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
