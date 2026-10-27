"""Bar02 depth sensor node.

The Bar02 sits on the PCA9685's I2C pass-through header, so it is on the
Pi's I2C bus 1 at 0x76 (the Pi is the only I2C master — D3).

Publishes
    /depth      std_msgs/Float32         metres, +down, ONLY after tare
    /pressure   sensor_msgs/FluidPressure Pa, always
Service
    ~/tare      std_srvs/Trigger   call with the vehicle AT THE SURFACE

No depth is published until tared. A depth on the wrong datum is worse than
none: the depth controller would happily dive to a wrong depth. Tare is
deliberately manual — an automatic tare after a node restart underwater
would read the current depth as zero.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

from auv_drivers import ms5837

MBAR_TO_PA = 100.0
G = 9.80665


class DepthSensor(Node):

    def __init__(self):
        super().__init__('depth_sensor')
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('fluid_density', 997.0)   # fresh 997, sea ~1025
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('max_failures', 5)
        p = lambda k: self.get_parameter(k).value   # noqa: E731
        self._density = p('fluid_density')
        self._max_fail = p('max_failures')

        self._sensor = ms5837.MS5837_02BA(bus=p('i2c_bus'))
        if not self._sensor.init():
            raise RuntimeError('Bar02 init failed — i2cdetect -y 1 should show 0x76.')

        self._surface_pa = None
        self._failures = 0
        self._depth_pub = self.create_publisher(Float32, '/depth', 10)
        self._press_pub = self.create_publisher(FluidPressure, '/pressure',
                                                qos_profile_sensor_data)
        self.create_service(Trigger, '~/tare', self._on_tare)
        self.create_timer(1.0 / p('rate_hz'), self._on_timer)
        self.get_logger().info(
            f'Bar02 up, density={self._density}. UNTARED — call '
            f'/depth_sensor/tare at the surface before diving.')

    def _read_pa(self):
        """Pressure in Pa, or None. Counts consecutive failures; after
        max_failures it tries one re-init per cycle."""
        try:
            if self._sensor.read():
                if self._failures:
                    self.get_logger().info(f'Bar02 recovered after {self._failures}.')
                self._failures = 0
                return self._sensor.pressure() * MBAR_TO_PA
            raise OSError('read() returned False')
        except OSError as e:
            self._failures += 1
            self.get_logger().warn(f'Bar02 read failed ({self._failures}): {e}',
                                   throttle_duration_sec=1.0)
            if self._failures >= self._max_fail:
                try:
                    self._sensor.init()
                except OSError:
                    pass
            return None

    def _on_tare(self, request, response):
        pa = self._read_pa()
        if pa is None:
            response.success, response.message = False, 'Tare failed: no reading.'
        else:
            self._surface_pa = pa
            response.success, response.message = True, f'Tared at {pa:.1f} Pa.'
            self.get_logger().info(response.message)
        return response

    def _on_timer(self):
        pa = self._read_pa()
        if pa is None:
            return                         # silence = stale = depth loop mutes
        msg = FluidPressure()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'bar02_link'
        msg.fluid_pressure = pa
        self._press_pub.publish(msg)
        if self._surface_pa is not None:
            depth = (pa - self._surface_pa) / (self._density * G)
            self._depth_pub.publish(Float32(data=float(depth)))


def main(args=None):
    rclpy.init(args=args)
    node = DepthSensor()
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
