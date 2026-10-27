#!/usr/bin/env python3
"""Live view of the 3 ultrasonic sensors: python3 ultra_check.py  (Ctrl+C to stop).
Shows the closest valid range per sensor; 'clear' = nothing within range, 'too close' = inside min range."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

NAMES = ('left', 'center', 'right')


class UltraCheck(Node):
    def __init__(self):
        super().__init__('ultra_check')
        self.state = {n: 'no data' for n in NAMES}
        for n in NAMES:
            self.create_subscription(
                LaserScan, f'/ultrasonic/{n}/scan',
                lambda m, n=n: self.cb(n, m), qos_profile_sensor_data)
        self.create_timer(0.5, self.show)

    def cb(self, name, m):
        finite = [x for x in m.ranges if math.isfinite(x)]
        if finite:
            self.state[name] = f'{min(finite):5.2f} m'
        elif any(x == -math.inf for x in m.ranges):
            self.state[name] = 'too close'
        else:
            self.state[name] = '  clear'

    def show(self):
        print('  |  '.join(f'{n}: {self.state[n]:>9}' for n in NAMES), flush=True)


def main():
    rclpy.init()
    node = UltraCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
