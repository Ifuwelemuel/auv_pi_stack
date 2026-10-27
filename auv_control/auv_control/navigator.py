"""Navigator: runs a waypoint mission from GPS. (Replaces v1's los_guidance
+ waypoint_manager pair — one node, one state machine.)

Subscribes
    /gps/fix                    NavSatFix (gps_node; the BUOY's position)
Publishes
    /autopilot/heading_setpoint Float32 rad ENU   } at rate_hz while RUNNING
    /autopilot/surge_setpoint   Float32           }
    /autopilot/depth_setpoint   Float32 m         } only if mission depth_m > 0
    /heartbeat                  Header            } only while RUNNING and healthy
    /estop_request              Bool(true)        on abort or ~/stop
    /navigator/position         PointStamped      east/north m about origin
    /navigator/origin           NavSatFix, LATCHED  the local frame's origin
    /navigator/status           String
Services
    ~/start   std_srvs/Trigger   refused without a GPS fix
    ~/stop    std_srvs/Trigger   abort: idle + e-stop

States: IDLE -> RUNNING -> (COMPLETE | ABORTED) -> IDLE.
Aborts (and requests the e-stop) on: GPS fix stale, geofence violation,
mission timeout. On COMPLETE it simply stops sending setpoints and
heartbeats: the autopilot goes idle, the mixer falls to safe and the vehicle
floats; with no gamepad heartbeat, the watchdog also latches the e-stop.

Frame: x = east, y = north metres; origin = first good fix, fixed for the
session. Guidance is LOS (los.py) along the line between waypoints.

Mission YAML (config/missions/*.yaml):
    mission:
      frame: global            # 'global' [lat, lon] or 'local' [east, north]
      waypoints: [[53.3439, -1.4994], ...]
      acceptance_radius_m: 3.0  # > GPS error + buoy/tether offset
      mission_timeout_s: 600
      geofence: {east_min: -60, east_max: 60, north_min: -60, north_max: 60}
      cruise_thrust: 0.3        # optional, default 0.3
      lookahead_m: 5.0          # optional, default 5.0
      depth_m: 0.0              # optional, 0 = surface run (no depth loop)
"""
import math

import rclpy
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float32, Header, String
from std_srvs.srv import Trigger

from auv_control.geo import LocalFrame
from auv_control.los import los_bearing
from auv_control.mission import load_mission
from auv_control.qos import COMMAND, LATCHED

IDLE, RUNNING = 'IDLE', 'RUNNING'


class Navigator(Node):

    def __init__(self):
        super().__init__('navigator')
        self.declare_parameter('mission_file', '')
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('fix_timeout_s', 3.0)      # GPS ~1 Hz
        path = self.get_parameter('mission_file').value
        if not path:
            raise RuntimeError('mission_file parameter is required.')
        self._m = load_mission(path)
        self._fix_timeout = self.get_parameter('fix_timeout_s').value

        self._frame = None
        self._wps = None              # local (east, north) once the frame exists
        self._pos = None
        self._fix_t = None
        self._state = IDLE
        self._idx = 0
        self._seg_start = None
        self._t0 = None
        self._cross = 0.0

        self.create_subscription(NavSatFix, '/gps/fix', self._on_fix, 10)
        self._pub = {n: self.create_publisher(Float32, f'/autopilot/{n}_setpoint', 10)
                     for n in ('heading', 'surge', 'depth')}
        self._hb_pub = self.create_publisher(Header, '/heartbeat', COMMAND)
        self._estop_pub = self.create_publisher(Bool, '/estop_request', COMMAND)
        self._pos_pub = self.create_publisher(PointStamped, '/navigator/position', 10)
        self._origin_pub = self.create_publisher(NavSatFix, '/navigator/origin', LATCHED)
        self._status_pub = self.create_publisher(String, '/navigator/status', 10)
        self.create_service(Trigger, '~/start', self._on_start)
        self.create_service(Trigger, '~/stop', self._on_stop)
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self._on_timer)
        self.get_logger().info(
            f"Navigator up: {len(self._m['waypoints'])} {self._m['frame']} waypoints, "
            f"accept {self._m['accept']} m, timeout {self._m['timeout']} s, "
            f"depth {self._m['depth']} m. IDLE — call /navigator/start.")

    # ------------------------------------------------------------------
    def _now(self):
        return self.get_clock().now()

    def _on_fix(self, msg):
        if msg.status.status < NavSatStatus.STATUS_FIX or math.isnan(msg.latitude):
            return                                  # no fix = no data
        if self._frame is None:
            self._frame = LocalFrame(msg.latitude, msg.longitude)
            origin = NavSatFix()
            origin.header.stamp = self._now().to_msg()
            origin.header.frame_id = 'map'
            origin.latitude, origin.longitude = msg.latitude, msg.longitude
            self._origin_pub.publish(origin)
            self._wps = (list(self._m['waypoints']) if self._m['frame'] == 'local'
                         else [self._frame.to_local(*w) for w in self._m['waypoints']])
            self.get_logger().info(
                f'Origin fixed at {msg.latitude:.6f}, {msg.longitude:.6f}. '
                f'First waypoint at E {self._wps[0][0]:.1f} N {self._wps[0][1]:.1f}.')
        self._pos = self._frame.to_local(msg.latitude, msg.longitude)
        self._fix_t = self._now()
        out = PointStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'map'
        out.point.x, out.point.y = self._pos
        self._pos_pub.publish(out)

    def _fix_age(self):
        return math.inf if self._fix_t is None else \
            (self._now() - self._fix_t).nanoseconds * 1e-9

    def _on_start(self, request, response):
        if self._state == RUNNING:
            response.success, response.message = False, 'Already running.'
        elif self._pos is None or self._fix_age() > self._fix_timeout:
            response.success, response.message = False, 'Refused: no live GPS fix.'
        else:
            self._state, self._idx = RUNNING, 0
            self._seg_start, self._t0 = self._pos, self._now()
            response.success = True
            response.message = (f'Mission started ({len(self._wps)} waypoints). '
                                f'Now clear the e-stop.')
            self.get_logger().info(response.message)
        return response

    def _on_stop(self, request, response):
        self._abort('stopped by operator')
        response.success, response.message = True, 'Stopped; e-stop requested.'
        return response

    def _abort(self, reason):
        self.get_logger().error(f'MISSION ABORT: {reason} — requesting e-stop.')
        self._estop_pub.publish(Bool(data=True))
        self._state = IDLE
        self._status(f'ABORTED: {reason}')

    def _status(self, text):
        self._status_pub.publish(String(data=text))

    # ------------------------------------------------------------------
    def _on_timer(self):
        if self._state != RUNNING:
            return
        age = self._fix_age()
        if age > self._fix_timeout:
            return self._abort(f'GPS fix stale ({age:.1f} s)')
        e, n = self._pos
        emin, emax, nmin, nmax = self._m['fence']
        if not (emin <= e <= emax and nmin <= n <= nmax):
            return self._abort(f'geofence violation at E {e:.1f} N {n:.1f}')
        elapsed = (self._now() - self._t0).nanoseconds * 1e-9
        if elapsed > self._m['timeout']:
            return self._abort(f'mission timeout ({elapsed:.0f} s)')

        target = self._wps[self._idx]
        dist = math.hypot(target[0] - e, target[1] - n)
        if dist <= self._m['accept']:
            self.get_logger().info(f'Waypoint {self._idx + 1}/{len(self._wps)} reached.')
            self._seg_start = target
            self._idx += 1
            if self._idx >= len(self._wps):
                self._state = IDLE
                self.get_logger().info('MISSION COMPLETE — going quiet; the '
                                       'watchdog will stop the vehicle.')
                self._status('COMPLETE')
                return
            target = self._wps[self._idx]
            dist = math.hypot(target[0] - e, target[1] - n)

        bearing, self._cross = los_bearing(self._seg_start, target, (e, n),
                                           self._m['lookahead'])
        self._hb_pub.publish(Header(stamp=self._now().to_msg(), frame_id='navigator'))
        self._pub['heading'].publish(Float32(data=float(bearing)))
        self._pub['surge'].publish(Float32(data=self._m['cruise']))
        if self._m['depth'] > 0.0:
            self._pub['depth'].publish(Float32(data=self._m['depth']))
        self._status(f'RUNNING wp {self._idx + 1}/{len(self._wps)} dist {dist:.1f} m '
                     f'xtrack {self._cross:+.1f} m t {elapsed:.0f} s')


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
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
