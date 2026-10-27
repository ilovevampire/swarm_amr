#!/usr/bin/env python3
"""Odometry / SLAM stability check.

Leave the robot STANDING STILL (no teleop keys) and run:  python3 stability_check.py
It listens for 12 s and tells you whether wheels, EKF or SLAM are moving when they should not.
"""
import math
import statistics as st
import subprocess
import time

import rclpy
import tf2_ros
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

WINDOW = 12.0


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def sd(v):
    return st.pstdev(v) if len(v) > 1 else float('nan')


def span(v):
    return (max(v) - min(v)) if v else float('nan')


class Track:
    def __init__(self):
        self.vx, self.wz, self.x, self.y, self.yaw = [], [], [], [], []

    def add(self, m):
        self.vx.append(m.twist.twist.linear.x)
        self.wz.append(m.twist.twist.angular.z)
        self.x.append(m.pose.pose.position.x)
        self.y.append(m.pose.pose.position.y)
        self.yaw.append(yaw_of(m.pose.pose.orientation))

    def moved(self):
        if len(self.x) < 2:
            return float('nan')
        return math.hypot(self.x[-1] - self.x[0], self.y[-1] - self.y[0])

    def turned(self):
        return wrap(self.yaw[-1] - self.yaw[0]) if len(self.yaw) > 1 else float('nan')


class Check(Node):
    def __init__(self):
        super().__init__('stability_check', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.raw, self.filt = Track(), Track()
        self.gyro = []
        self.mo = []                      # map -> odom samples (x, y, yaw)
        self.t0 = time.time()
        self.buf = tf2_ros.Buffer()
        self.lst = tf2_ros.TransformListener(self.buf, self)
        self.create_subscription(Odometry, '/diff_drive_controller/odom',
                                 self.raw.add, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry/filtered',
                                 self.filt.add, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data', lambda m: self.gyro.append(m.angular_velocity.z),
                                 qos_profile_sensor_data)
        self.create_timer(0.25, self.sample_tf)
        self.create_timer(1.0, self.tick)

    def sample_tf(self):
        try:
            t = self.buf.lookup_transform('map', 'odom', rclpy.time.Time()).transform
            self.mo.append((t.translation.x, t.translation.y, yaw_of(t.rotation)))
        except Exception:
            pass

    def tick(self):
        left = WINDOW - (time.time() - self.t0)
        if left > 0:
            print(f'  collecting... {left:4.0f} s', end='\r', flush=True)
        else:
            self.report()
            raise SystemExit

    def report(self):
        print('\n')
        try:
            p = subprocess.run(['ros2', 'param', 'get', '/diff_drive_controller', 'enable_odom_tf'],
                               capture_output=True, text=True, timeout=8).stdout.strip()
        except Exception as e:
            p = f'could not read ({e})'
        print(f'controller enable_odom_tf : {p}   (must be False)')
        print(f'messages received         : wheel odom {len(self.raw.x)}, EKF {len(self.filt.x)}, '
              f'IMU {len(self.gyro)}, map->odom samples {len(self.mo)}\n')

        print('                      speed std (vx / yaw rate)   moved      turned')
        for name, t in (('wheel odom (raw)', self.raw), ('EKF filtered', self.filt)):
            print(f'{name:18s}   {sd(t.vx):8.4f} / {sd(t.wz):8.4f}      '
                  f'{t.moved():6.3f} m   {math.degrees(t.turned()):6.2f} deg')
        print(f'IMU gyro z std            : {sd(self.gyro):.4f} rad/s')
        if self.mo:
            print(f'map->odom range           : x {span([a[0] for a in self.mo]):.3f} m, '
                  f'y {span([a[1] for a in self.mo]):.3f} m, '
                  f'yaw {math.degrees(span([a[2] for a in self.mo])):.2f} deg')
        print('\nVerdict (robot must have been standing still):')
        bad = False
        if sd(self.raw.vx) > 0.01 or sd(self.raw.wz) > 0.02:
            print(' - WHEELS JITTER while stopped -> physics instability (contacts / wheel torque).')
            bad = True
        if self.raw.moved() > 0.02:
            print(' - wheel odometry DRIFTS while stopped -> wheels are creeping or vibrating.')
            bad = True
        if self.filt.moved() > 0.02 or abs(self.filt.turned()) > 0.02:
            print(' - EKF output moves while stopped -> EKF is being fed moving data.')
            bad = True
        if self.mo and (span([a[0] for a in self.mo]) > 0.05 or span([a[1] for a in self.mo]) > 0.05
                        or span([a[2] for a in self.mo]) > 0.03):
            print(' - SLAM map->odom JUMPS while stopped -> scan matching is unstable.')
            bad = True
        if not self.mo:
            print(' - no map->odom transform: SLAM is not running or has not started mapping.')
            bad = True
        if not bad:
            print(' - all quiet: wheels, EKF and SLAM are stable at rest.')


def main():
    rclpy.init()
    node = Check()
    print('Robot must stand still, no teleop keys. Collecting for 12 s...')
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
