import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Header
from std_srvs.srv import Trigger

from auv_interfaces.msg import ActuatorCommand
from auv_control.qos import COMMAND


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        for k, v in (('deadman_button', 4), ('estop_button', 1), ('clear_button', 3),
                        ('yaw_axis', 0), ('pitch_axis', 4), ('surge_axis', 5),
                        ('yaw_scale', 0.5), ('pitch_scale', 0.5), ('surge_scale', 1.0),
                        ('surge_is_trigger', True)):
                self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value

        self._b = {k: g(f'{k}_button') for k in ('deadman', 'estop', 'clear')}
        self._ax = {k: g(f'{k}_axis') for k in ('yaw', 'pitch', 'surge')}
        self._sc = {k: g(f'{k}_scale') for k in ('yaw', 'pitch', 'surge')}
        self._trigger = g('surge_is_trigger')
        self._prev = {'estop': 0, 'clear': 0}

        self._cmd_pub = self.create_publisher(ActuatorCommand, '/cmd/teleop', COMMAND)
        self._hb_pub = self.create_publisher(Header,'/heartbeat', COMMAND)
        self._estop_pub = self.create_publisher(Bool, '/estop_request', COMMAND)
        self._clear = self.create_client(Trigger,'/safety_supervisor/clear_estop')

        self.create_subscription(Joy, '/joy', self._on_joy, qos_profile_sensor_data)
        self.get_logger().info('Teleop up. Hold THE DEADMAN TO DRIIVE')



    @staticmethod
    def _axis(msg, i):
            return msg.axes[i] if 0 <= i < len(msg.axes) else 0.0

    @staticmethod
    def _button(msg, i):
            return msg.buttons[i] if 0 <= i < len(msg.buttons) else 0


    def _on_joy(self, msg):
        now = self.get_clock().now()
        self._hb_pub.publish(Header(stamp=now.to_msg(), frame_id='teleop'))

        for name in ('estop', 'clear'):                 # act on press, not hold
            pressed = self._button(msg, self._b[name])
            if pressed and not self._prev[name]:
                self._on_press(name)
            self._prev[name] = pressed

        if not self._button(msg, self._b['deadman']):
            return                                      # silence -> mixer drops us
        raw = self._axis(msg, self._ax['surge'])
        surge = (1.0 - raw) / 2.0 if self._trigger else raw   # trigger rests at +1
        cmd = ActuatorCommand()
        cmd.header.stamp = now.to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.source = ActuatorCommand.SOURCE_TELEOP
        cmd.thrust_surge = surge * self._sc['surge']
        cmd.thrust_yaw = self._axis(msg, self._ax['yaw']) * self._sc['yaw']
        cmd.thrust_pitch = self._axis(msg, self._ax['pitch']) * self._sc['pitch']
        self._cmd_pub.publish(cmd)

    def _on_press(self, name):
        if name == 'estop':
            self._estop_pub.publish(Bool(data=True))
            self.get_logger().warn('E-STOP requested from controller.')
            return
        if not self._clear.service_is_ready():
            self.get_logger().warn('Clear pressed but supervisor not reachable.')
            return
        # call_async only: a blocking call inside a callback deadlocks.
        self._clear.call_async(Trigger.Request()).add_done_callback(self._on_cleared)

    def _on_cleared(self, future):
        try:
            r = future.result()
        except Exception as e:
            self.get_logger().error(f'clear_estop failed: {e}')
            return
        (self.get_logger().info if r.success else self.get_logger().warn)(
            f'Supervisor: {r.message}')


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
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









