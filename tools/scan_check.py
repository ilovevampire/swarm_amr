#!/usr/bin/env python3
"""Quick lidar health check: python3 scan_check.py  (ROS sourced, sim running)."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanCheck(Node):
    def __init__(self):
        super().__init__('scan_check')
        self.count = 0
        self.create_subscription(LaserScan, '/scan', self.cb, qos_profile_sensor_data)
        self.timer = self.create_timer(5.0, self.no_data)

    def no_data(self):
        if self.count == 0:
            print('NO /scan messages in 5 s -> bridge or Gazebo lidar sensor is not publishing')
            raise SystemExit

    def cb(self, m):
        self.count += 1
        r = m.ranges
        finite = [(x, i) for i, x in enumerate(r) if math.isfinite(x)]
        pos_inf = sum(1 for x in r if x == math.inf)
        neg_inf = sum(1 for x in r if x == -math.inf)
        nans = sum(1 for x in r if math.isnan(x))
        print(f'--- msg {self.count}: frame_id={m.header.frame_id!r}  rays={len(r)}  '
              f'range limits={m.range_min:.2f}..{m.range_max:.2f} m')
        print(f'    hits={len(finite)}  +inf(no hit)={pos_inf}  -inf(too close)={neg_inf}  nan={nans}')
        if finite:
            lo = min(finite)
            hi = max(finite)
            deg = lambda i: math.degrees(m.angle_min + i * m.angle_increment)
            print(f'    closest hit {lo[0]:.2f} m at {deg(lo[1]):.0f} deg, '
                  f'farthest {hi[0]:.2f} m at {deg(hi[1]):.0f} deg')
        if self.count >= 3:
            raise SystemExit


def main():
    rclpy.init()
    node = ScanCheck()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
