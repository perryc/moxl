#!/usr/bin/env python3
"""Bridge node: converts GPS heading (QuaternionStamped) to IMU message.

The FarmTRX GPS magnetometer provides true heading via NMEA HDT sentences.
nmea_navsat_driver publishes this as geometry_msgs/QuaternionStamped on /heading.
robot_localization EKF expects sensor_msgs/Imu.

Speed-dependent blending:
    At low speed (<1 m/s), the magnetometer heading is trusted (low yaw covariance).
    As speed increases, covariance rises so the EKF favors GPS-derived heading
    from successive position fixes, which is more accurate at speed.

Subscriptions:
    /heading (geometry_msgs/QuaternionStamped): GPS magnetometer heading
    /diff_drive_base_controller/odom (nav_msgs/Odometry): for current velocity

Publications:
    /gps/imu (sensor_msgs/Imu): Heading as IMU orientation for robot_localization
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class HeadingToImuNode(Node):
    def __init__(self):
        super().__init__("heading_to_imu")

        # Parameters for speed-dependent covariance blending
        self.declare_parameter("blend_speed_threshold", 1.0)  # m/s
        self.declare_parameter("min_yaw_covariance", 0.01)    # ~5° — magnetometer confidence
        self.declare_parameter("max_yaw_covariance", 1.0e6)   # effectively ignore at high speed

        self.blend_speed = self.get_parameter("blend_speed_threshold").value
        self.min_cov = self.get_parameter("min_yaw_covariance").value
        self.max_cov = self.get_parameter("max_yaw_covariance").value

        self.current_speed = 0.0

        self.sub_heading = self.create_subscription(
            QuaternionStamped, "heading", self.heading_callback, 10
        )
        self.sub_odom = self.create_subscription(
            Odometry, "diff_drive_base_controller/odom", self.odom_callback, 10
        )
        self.pub = self.create_publisher(Imu, "gps/imu", 10)

        self.get_logger().info(
            f"Bridging GPS heading → IMU, speed blending threshold {self.blend_speed} m/s"
        )

    def odom_callback(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.sqrt(vx * vx + vy * vy)

    def heading_callback(self, msg: QuaternionStamped):
        imu = Imu()
        imu.header = msg.header
        imu.header.frame_id = "gps"

        # Orientation from GPS magnetometer
        imu.orientation = msg.quaternion

        # Speed-dependent yaw covariance:
        # At 0 m/s → min_cov (trust magnetometer fully)
        # At blend_speed m/s → max_cov (let GPS track heading dominate)
        # Linear interpolation between
        if self.blend_speed > 0:
            t = min(self.current_speed / self.blend_speed, 1.0)
        else:
            t = 0.0

        yaw_cov = self.min_cov + t * (self.max_cov - self.min_cov)

        # [row-major 3x3: roll, pitch, yaw]
        imu.orientation_covariance = [
            1.0e6, 0.0,   0.0,       # roll — unknown
            0.0,   1.0e6, 0.0,       # pitch — unknown
            0.0,   0.0,   yaw_cov,   # yaw — speed-dependent
        ]

        # No angular velocity or acceleration from GPS
        imu.angular_velocity_covariance[0] = -1.0  # flag: data unavailable
        imu.linear_acceleration_covariance[0] = -1.0

        self.pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = HeadingToImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
