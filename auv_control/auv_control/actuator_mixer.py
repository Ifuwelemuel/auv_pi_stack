import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Bool

from auv_interfaces.msg import ActuatorCommand
from auv_control.arbitration import AUTONOMY, SAFE, TELEOP, select_source
from auv_control.qos import COMMAND, LATCHED


def _clamp(v, lim):
    return max(-lim, min(lim,v))

class ActuatorMixer(Node):

    def __init__(self):
        super().__init__('actuator_mixer')
        self._mono = Clock(clock_type=ClockType.STEADY_TIME)
        self.declare_parameter('source_timeout_s', 0.3)
        self.declare_parameter('safety_timeout_s', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('surge_limit', 1.0)
        self.declare_parameter('pitch_limit', 1.0)
        self.declare_parameter('yaw_limit', 1.0)
        p = lambda k: self.get_parameter(k).value
        self._src_timeout = p('source_timeout_s')
        self._safety_timeout = p('safety_timeout_s')
        self._lim = (p('surge_limit'), p('pitch_limit'), p('yaw_limit'))

        self._estopped = True
        self._last = {SAFE: None, TELEOP:None, AUTONOMY:None}
        self._active =None

        self.create_subscription(Bool,'/estop', self._on_estop,LATCHED)
        self.create_subscription(ActuatorCommand, '/cmd/safety', self._on_safety, COMMAND)
        self.create_subscription(ActuatorCommand, '/cmd/teleop', self._on_teleop, COMMAND)
        self.create_subscription(ActuatorCommand, '/cmd/autonomy', self._on_autonomy, COMMAND)

        self._pub = self.create_publisher(ActuatorCommand, '/cmd/actuators', COMMAND)
        self.create_timer(1.0 / p('publish_rate_hz'), self._on_timer)
        self.get_logger().info(f'Mixer up. limits surge/pitch/yaw= {self._lim}')


    ## subs callbacl
    def _on_safety(self, msg):
        self._on_cmd(SAFE, msg)

    def _on_teleop(self, msg):
        self._on_cmd(TELEOP, msg)

    def _on_autonomy(self, msg):
        self._on_cmd(AUTONOMY, msg)

    ###
    def _on_estop(self,msg):
        self._estopped = msg.data

    def _on_cmd(self,src,msg):
        self._last[src] = (msg, self._mono.now())

    def _age(self,src):
        entry = self._last[src]
        if entry is None:
            return None
        return (self._mono.now() - entry[1]).nanoseconds * 1e-9

    def _on_timer(self):
        src = select_source(self._estopped,self._age(SAFE),self._age(TELEOP),self._age(AUTONOMY),
                            self._src_timeout,self._safety_timeout)

        if src != self._active:
            self.get_logger().info(f'Active source: {src}')
            self._active = src

        out = ActuatorCommand()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'

        if src == SAFE:
            out.source = ActuatorCommand.SOURCE_SAFETY
        else:
            cmd, stamp = self._last[src] ### <-
            out.thrust_surge = _clamp(cmd.thrust_surge,self._lim[0])
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





