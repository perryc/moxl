#!/usr/bin/env python3
"""Relay Gazebo IMU data to localization topics for simulation.

In simulation, heading_to_imu_node has no NMEA/FarmTRX input.
This relay bridges Gazebo's /imu/data_raw to the /gps/imu and
/gps/imu_gyro topics that the EKF and navsat_transform expect.

Gazebo's IMU sensor reports yaw relative to its initial heading (always
starts at 0), not absolute world-frame heading. The heading_offset
parameter rotates the orientation to match the robot's spawn heading.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def quat_multiply(q1, q2):
    """Multiply two quaternions (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


class SimImuRelay(Node):
    def __init__(self):
        super().__init__('sim_imu_relay')
        self.declare_parameter('heading_offset', 0.0)
        offset = self.get_parameter('heading_offset').value
        # Pre-compute yaw-only quaternion for the heading offset
        half = offset / 2.0
        self.q_offset = (0.0, 0.0, math.sin(half), math.cos(half))
        self.get_logger().info(
            f'Relaying /imu/data_raw -> /gps/imu + /gps/imu_gyro '
            f'(heading_offset={offset:.4f} rad)')

        self.pub_heading = self.create_publisher(Imu, '/gps/imu', 10)
        self.pub_gyro = self.create_publisher(Imu, '/gps/imu_gyro', 10)
        self.create_subscription(Imu, '/imu/data_raw', self.callback, 10)

    def callback(self, msg):
        # Apply heading offset to orientation
        q_in = (msg.orientation.x, msg.orientation.y,
                msg.orientation.z, msg.orientation.w)
        qx, qy, qz, qw = quat_multiply(self.q_offset, q_in)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        self.pub_heading.publish(msg)
        self.pub_gyro.publish(msg)


def main():
    rclpy.init()
    node = SimImuRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
