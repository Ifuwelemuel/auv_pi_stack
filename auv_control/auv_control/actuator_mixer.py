"""Actuator mixer: the single choke point for thruster commands.

Subscribes
    /estop          Bool, LATCHED
    /cmd/safety     ActuatorCommand  (safety_supervisor, 20 Hz)
    /cmd/teleop     ActuatorCommand  (teleop_node, only while deadman held)
    /cmd/autonomy   ActuatorCommand  (autopilot)
Publishes
    /cmd/actuators  ActuatorCommand, 20 Hz — the ONLY input of thruster_driver

Picks safe > teleop > autonomy (see arbitration.py). Ages are measured on
this node's clock, so a crashed source's last command expires after
source_timeout_s, and a silent safety supervisor forces safe.
Clamps each axis to its limit here — the last stage that still deals in
normalised intent.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from auv_interfaces.msg import ActuatorCommand
from auv_control.arbitration import AUTONOMY, SAFE, TELEOP, select_source
from auv_control.qos import COMMAND, LATCHED


def _clamp(v, lim):
    return max(-lim, min(lim, v))


class ActuatorMixer(Node):

    def __init__(self):
        super().__init__('actuator_mixer')
        self.declare_parameter('source_timeout_s', 0.3)
        self.declare_parameter('safety_timeout_s', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('surge_limit', 1.0)
        self.declare_parameter('pitch_limit', 1.0)
        self.declare_parameter('yaw_limit', 1.0)
        p = lambda k: self.get_parameter(k).value   # noqa: E731
        self._src_timeout = p('source_timeout_s')
        self._safety_timeout = p('safety_timeout_s')
        self._lim = (p('surge_limit'), p('pitch_limit'), p('yaw_limit'))

        self._estopped = True
        self._last = {SAFE: None, TELEOP: None, AUTONOMY: None}   # (msg, time)
        self._active = None

        self.create_subscription(Bool, '/estop', self._on_estop, LATCHED)
        for src, topic in ((SAFE, '/cmd/safety'), (TELEOP, '/cmd/teleop'),
                           (AUTONOMY, '/cmd/autonomy')):
            self.create_subscription(
                ActuatorCommand, topic,
                lambda m, s=src: self._on_cmd(s, m), COMMAND)
        self._pub = self.create_publisher(ActuatorCommand, '/cmd/actuators', COMMAND)
        self.create_timer(1.0 / p('publish_rate_hz'), self._on_timer)
        self.get_logger().info(f'Mixer up. limits surge/pitch/yaw={self._lim}')

    def _on_estop(self, msg):
        self._estopped = msg.data

    def _on_cmd(self, src, msg):
        self._last[src] = (msg, self.get_clock().now())

    def _age(self, src):
        entry = self._last[src]
        if entry is None:
            return None
        return (self.get_clock().now() - entry[1]).nanoseconds * 1e-9

    def _on_timer(self):
        src = select_source(self._estopped, self._age(SAFE), self._age(TELEOP),
                            self._age(AUTONOMY), self._src_timeout,
                            self._safety_timeout)
        if src != self._active:
            self.get_logger().info(f'Active source: {src}')
            self._active = src
        out = ActuatorCommand()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        if src == SAFE:
            out.source = ActuatorCommand.SOURCE_SAFETY      # all zeros
        else:
            cmd = self._last[src][0]
            out.thrust_surge = _clamp(cmd.thrust_surge, self._lim[0])
            out.thrust_pitch = _clamp(cmd.thrust_pitch, self._lim[1])
            out.thrust_yaw = _clamp(cmd.thrust_yaw, self._lim[2])
            out.source = cmd.source
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorMixer()
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
