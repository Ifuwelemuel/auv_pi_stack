"""GPS node. Receiver on the surface buoy, serial line down the tether to
the Pi in the vehicle (D2). Runs on the vehicle Pi.

Publishes
    /gps/fix      sensor_msgs/NavSatFix  every GGA sentence, fix or not
    /gps/status   std_msgs/String        every status_every fixes

A NavSatFix is published even without a fix (status NO_FIX, NaN position)
so the ABSENCE of a fix is recorded rather than appearing as a gap.
Consumers must check status. Fixes are stamped on arrival on the Pi clock —
the same clock as the IMU and depth, so the dataset has one timebase.

Note: the fix is the BUOY's position, not the vehicle's. The two differ by
the tether geometry; acceptance radii must allow for that.
"""
import pynmea2
import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String


class GpsNode(Node):

    def __init__(self):
        super().__init__('gps_node')
        self.declare_parameter('port', '/dev/serial0')   # Pi GPIO UART
        self.declare_parameter('baud', 9600)
        self.declare_parameter('frame_id', 'gps_buoy')
        # sigma at HDOP=1; covariance = (base * HDOP)^2. Refine from a static
        # scatter test on open water.
        self.declare_parameter('base_accuracy_m', 2.5)
        self.declare_parameter('status_every', 10)
        p = lambda k: self.get_parameter(k).value   # noqa: E731
        self._frame = p('frame_id')
        self._base_acc = p('base_accuracy_m')
        self._status_every = p('status_every')

        self._ser = serial.Serial(p('port'), p('baud'), timeout=0.05)
        self.get_logger().info(f'GPS serial open on {p("port")} @ {p("baud")}.')
        self._fix_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self._status_pub = self.create_publisher(String, '/gps/status', 10)
        self._sats_in_view = 0
        self._count = 0
        self._buf = b''
        self.create_timer(0.05, self._poll)

    def _poll(self):
        try:
            self._buf += self._ser.read(self._ser.in_waiting or 1)
        except serial.SerialException as e:
            self.get_logger().error(f'Serial read failed: {e}',
                                    throttle_duration_sec=2.0)
            return
        while b'\n' in self._buf:
            line, self._buf = self._buf.split(b'\n', 1)
            self._handle_line(line.decode('ascii', errors='ignore').strip())
        if len(self._buf) > 4096:          # garbage with no newlines
            self._buf = b''

    def _handle_line(self, raw):
        if not raw.startswith('$'):
            return
        try:
            msg = pynmea2.parse(raw)
        except pynmea2.ParseError:
            return
        kind = getattr(msg, 'sentence_type', None)
        if kind == 'GSV':
            try:
                self._sats_in_view = int(msg.num_sv_in_view)
            except (TypeError, ValueError):
                pass
        elif kind == 'GGA':
            self._publish_gga(msg)

    def _publish_gga(self, msg):
        try:
            quality = int(msg.gps_qual)
        except (TypeError, ValueError):
            quality = 0
        try:
            hdop = float(msg.horizontal_dil)
        except (TypeError, ValueError):
            hdop = 99.99

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self._frame
        fix.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS
        fix.status.status = (NavSatStatus.STATUS_NO_FIX if quality == 0 else
                             NavSatStatus.STATUS_SBAS_FIX if quality == 2 else
                             NavSatStatus.STATUS_FIX)
        has_pos = quality > 0 and msg.latitude and msg.longitude
        if has_pos:
            fix.latitude = float(msg.latitude)
            fix.longitude = float(msg.longitude)
            fix.altitude = float(msg.altitude) if msg.altitude else 0.0
        else:
            fix.status.status = NavSatStatus.STATUS_NO_FIX
            fix.latitude = fix.longitude = fix.altitude = float('nan')
        if not has_pos or hdop >= 99.0:
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        else:
            sh = self._base_acc * hdop
            sv = 2.0 * sh
            fix.position_covariance = [sh * sh, 0.0, 0.0, 0.0, sh * sh, 0.0,
                                       0.0, 0.0, sv * sv]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self._fix_pub.publish(fix)

        self._count += 1
        if self._count % self._status_every == 0:
            s = String(data=f'quality={quality} used={msg.num_sats} '
                            f'in_view={self._sats_in_view} hdop={hdop:.2f}')
            self._status_pub.publish(s)
            self.get_logger().info(s.data)

    def destroy_node(self):
        try:
            self._ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpsNode()
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
