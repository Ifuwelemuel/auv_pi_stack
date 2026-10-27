"""Safety supervisor: owns the e-stop latch and the watchdog.

Publishes
    /estop        std_msgs/Bool, LATCHED     current stop state
    /cmd/safety   ActuatorCommand, 20 Hz     the safe command (all stopped);
                                              also the mixer's proof we're alive
Subscribes
    /heartbeat      std_msgs/Header   from whoever is in command
                                      (teleop_node, navigator)
    /estop_request  std_msgs/Bool     ENGAGE ONLY — `false` is ignored
Service
    ~/clear_estop   std_srvs/Trigger  the ONLY way out of the stop; refused
                                      unless a heartbeat is live

Asymmetric authority (D7): anyone can stop the vehicle, any time, and a
replayed stop is harmless. Un-stopping needs a deliberate, acknowledged
service call while someone is actually in command.

Boots STOPPED. The watchdog latches the stop if heartbeats go quiet for
watchdog_timeout_s, measured on THIS node's clock (no trust in remote
stamps). Safe = all thrusters stopped; the vehicle is trimmed positively
buoyant, so stopped means it floats up (D1).
"""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Header
from std_srvs.srv import Trigger

from auv_interfaces.msg import ActuatorCommand
from auv_control.qos import COMMAND, LATCHED


class SafetySupervisor(Node):

    def __init__(self):
        super().__init__('safety_supervisor')
        self.declare_parameter('watchdog_timeout_s', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)
        self._timeout = self.get_parameter('watchdog_timeout_s').value
        rate = self.get_parameter('publish_rate_hz').value

        self._estopped = True
        # Epoch, not now(): nobody has been heard yet, so clear is refused.
        self._last_hb = Time(clock_type=self.get_clock().clock_type)

        self._estop_pub = self.create_publisher(Bool, '/estop', LATCHED)
        self._safe_pub = self.create_publisher(ActuatorCommand, '/cmd/safety', COMMAND)
        self.create_subscription(Header, '/heartbeat', self._on_heartbeat, COMMAND)
        self.create_subscription(Bool, '/estop_request', self._on_request, COMMAND)
        self.create_service(Trigger, '~/clear_estop', self._on_clear)
        self.create_timer(1.0 / rate, self._on_timer)

        self._publish_estop()
        self.get_logger().info(f'Safety supervisor up, booted E-STOPPED, '
                               f'watchdog {self._timeout} s.')

    def _hb_age(self):
        return (self.get_clock().now() - self._last_hb).nanoseconds * 1e-9

    def _on_heartbeat(self, msg):
        self._last_hb = self.get_clock().now()

    def _on_request(self, msg):
        if msg.data:
            if not self._estopped:
                self.get_logger().warn('E-STOP engaged by request.')
            self._estopped = True
            self._publish_estop()
        else:
            self.get_logger().warn('Ignored `false` on /estop_request — clear '
                                   'only via the clear_estop service.')

    def _on_clear(self, request, response):
        age = self._hb_age()
        if age > self._timeout:
            response.success = False
            response.message = (f'Refused: no live heartbeat ({age:.1f} s). Start '
                                f'teleop or a mission first.')
            self.get_logger().warn(response.message)
            return response
        self._estopped = False
        self._publish_estop()
        response.success, response.message = True, 'E-stop cleared.'
        self.get_logger().info('E-STOP CLEARED.')
        return response

    def _on_timer(self):
        age = self._hb_age()
        if age > self._timeout and not self._estopped:
            self.get_logger().error(f'WATCHDOG: no heartbeat for {age:.2f} s — '
                                    f'E-STOP engaged.')
            self._estopped = True
            self._publish_estop()
        cmd = ActuatorCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.source = ActuatorCommand.SOURCE_SAFETY      # all thrust fields 0.0
        self._safe_pub.publish(cmd)

    def _publish_estop(self):
        self._estop_pub.publish(Bool(data=self._estopped))


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._estopped = True
        try:
            node._publish_estop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
