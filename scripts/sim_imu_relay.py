#!/usr/bin/env python3
"""Relay Gazebo IMU data to localization topics for simulation.

In simulation, heading_to_imu_node has no NMEA/FarmTRX input.
This relay bridges Gazebo's /imu/data_raw to the /gps/imu and
/gps/imu_gyro topics that the EKF and navsat_transform expect.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class SimImuRelay(Node):
    def __init__(self):
        super().__init__('sim_imu_relay')
        self.pub_heading = self.create_publisher(Imu, '/gps/imu', 10)
        self.pub_gyro = self.create_publisher(Imu, '/gps/imu_gyro', 10)
        self.create_subscription(Imu, '/imu/data_raw', self.callback, 10)
        self.get_logger().info('Relaying /imu/data_raw -> /gps/imu + /gps/imu_gyro')

    def callback(self, msg):
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
