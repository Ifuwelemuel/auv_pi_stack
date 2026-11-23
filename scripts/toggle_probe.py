import time, rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn

class Probe(Node):
    def __init__(self):
        super().__init__('toggle_probe')
        self.last_val = None; self.last_flip = time.monotonic(); self.n = 0; self.t0 = time.monotonic()
        self.create_subscription(OverrideRCIn, '/mavros/rc/override', self.cb, 10)
    def cb(self, m):
        now = time.monotonic(); self.n += 1
        v = m.channels[15]
        if v != self.last_val:
            gap = (now - self.last_flip) * 1000
            if gap > 70:
                print(f'slow flip: {gap:.0f} ms since last flip (AUX {list(m.channels[8:11])})')
            self.last_val, self.last_flip = v, now
        if now - self.t0 >= 2.0:
            print(f'{self.n / (now - self.t0):.0f} msgs/s, toggle now {v}, AUX {list(m.channels[8:11])}')
            self.n, self.t0 = 0, now

rclpy.init(); rclpy.spin(Probe())
