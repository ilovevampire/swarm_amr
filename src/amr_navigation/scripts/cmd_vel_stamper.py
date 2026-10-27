#!/usr/bin/env python3
"""Turns Nav2's plain Twist velocity commands into the TwistStamped that
diff_drive_controller (Jazzy) expects.   /cmd_vel  ->  /diff_drive_controller/cmd_vel"""
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class CmdVelStamper(Node):
    def __init__(self):
        super().__init__('cmd_vel_stamper')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/diff_drive_controller/cmd_vel')
        self.declare_parameter('frame_id', 'base_footprint')
        self.frame_id = self.get_parameter('frame_id').value
        self.pub = self.create_publisher(TwistStamped, self.get_parameter('output_topic').value, 10)
        self.create_subscription(Twist, self.get_parameter('input_topic').value, self.cb, 10)

    def cb(self, msg):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.twist = msg
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CmdVelStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()