#!/usr/bin/env python3
"""Bridge node: converts GPS heading (QuaternionStamped) to IMU message.

The FarmTRX GPS magnetometer provides true heading via NMEA HDT sentences.
nmea_navsat_driver publishes this as geometry_msgs/QuaternionStamped on /heading.
robot_localization EKF expects sensor_msgs/Imu.

Speed-dependent blending:
    At low speed (<1 m/s), the magnetometer heading is trusted (low yaw covariance).
    As speed increases, covariance rises so the EKF favors GPS-derived heading
    from successive position fixes, which is more accurate at speed.

NMEA sentence parsing (via /nmea_sentence topic):
    $HCHDG — Magnetic heading with deviation/variation. Used as fallback heading
             source when HDT is unavailable for longer than hdg_fallback_timeout.
    $TIROT — Rate of turn from gyroscope (deg/min). Published as angular velocity
             on /gps/imu_gyro for the EKF to fuse as vyaw.
    $GPXDR — Transducer measurement. MACHINE_WORK digital input reports blade
             engagement state (1 = working, 0 = idle). Published on /gps/work_state.

Subscriptions:
    /heading (geometry_msgs/QuaternionStamped): GPS magnetometer heading
    /diff_drive_base_controller/odom (nav_msgs/Odometry): for current velocity
    /nmea_sentence (nmea_msgs/Sentence): raw NMEA sentences for HCHDG + TIROT

Publications:
    /gps/imu (sensor_msgs/Imu): Heading as IMU orientation for robot_localization
    /gps/imu_gyro (sensor_msgs/Imu): Angular velocity z from TIROT for EKF
    /gps/work_state (std_msgs/Bool): Blade engagement from GPXDR MACHINE_WORK
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, QuaternionStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from nmea_msgs.msg import Sentence


def validate_nmea_checksum(sentence: str) -> bool:
    """Validate NMEA checksum: XOR all chars between '$' and '*'."""
    if '$' not in sentence or '*' not in sentence:
        return False
    try:
        start = sentence.index('$') + 1
        end = sentence.index('*')
        payload = sentence[start:end]
        expected_hex = sentence[end + 1:end + 3].strip()
        computed = 0
        for ch in payload:
            computed ^= ord(ch)
        return f"{computed:02X}" == expected_hex.upper()
    except (ValueError, IndexError):
        return False


class HeadingToImuNode(Node):
    def __init__(self):
        super().__init__("heading_to_imu")

        # Parameters for speed-dependent covariance blending
        self.declare_parameter("blend_speed_threshold", 1.0)  # m/s
        self.declare_parameter("min_yaw_covariance", 0.01)    # ~5° — magnetometer confidence
        self.declare_parameter("max_yaw_covariance", 1.0e6)   # effectively ignore at high speed

        # Parameters for NMEA sentence parsing
        self.declare_parameter("gyro_covariance", 0.01)          # rad²/s² for TIROT angular velocity
        self.declare_parameter("hdg_fallback_timeout", 2.0)      # seconds without HDT before HCHDG fallback
        self.declare_parameter("magnetic_declination_deg", 7.2)  # CDS2 Disley, SK

        self.blend_speed = self.get_parameter("blend_speed_threshold").value
        self.min_cov = self.get_parameter("min_yaw_covariance").value
        self.max_cov = self.get_parameter("max_yaw_covariance").value
        self.gyro_cov = self.get_parameter("gyro_covariance").value
        self.hdg_fallback_timeout = self.get_parameter("hdg_fallback_timeout").value
        self.magnetic_declination_deg = self.get_parameter("magnetic_declination_deg").value

        self.current_speed = 0.0
        self.last_hdt_time = self.get_clock().now()

        self.sub_heading = self.create_subscription(
            QuaternionStamped, "heading", self.heading_callback, 10
        )
        self.sub_odom = self.create_subscription(
            Odometry, "diff_drive_base_controller/odom", self.odom_callback, 10
        )
        self.sub_nmea = self.create_subscription(
            Sentence, "nmea_sentence", self.nmea_callback, 10
        )

        self.pub = self.create_publisher(Imu, "gps/imu", 10)
        self.pub_gyro = self.create_publisher(Imu, "gps/imu_gyro", 10)
        self.pub_work_state = self.create_publisher(Bool, "gps/work_state", 10)

        self.get_logger().info(
            f"Bridging GPS heading → IMU, speed blending threshold {self.blend_speed} m/s"
        )
        self.get_logger().info(
            f"NMEA parsing: TIROT gyro cov={self.gyro_cov}, "
            f"HCHDG fallback timeout={self.hdg_fallback_timeout}s, "
            f"declination={self.magnetic_declination_deg}°"
        )

    def odom_callback(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.sqrt(vx * vx + vy * vy)

    def heading_callback(self, msg: QuaternionStamped):
        self.last_hdt_time = self.get_clock().now()
        self._publish_heading_imu(msg.header.stamp, msg.quaternion)

    def _publish_heading_imu(self, stamp, quaternion):
        """Publish heading as IMU orientation on /gps/imu."""
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "gps"

        imu.orientation = quaternion

        # Speed-dependent yaw covariance:
        # At 0 m/s → min_cov (trust magnetometer fully)
        # At blend_speed m/s → max_cov (let GPS track heading dominate)
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

    def nmea_callback(self, msg: Sentence):
        """Parse raw NMEA sentences for HCHDG, TIROT, and GPXDR."""
        sentence = msg.sentence.strip()

        if not validate_nmea_checksum(sentence):
            return

        if sentence.startswith("$HCHDG"):
            self._parse_hchdg(sentence, msg.header)
        elif sentence.startswith("$TIROT"):
            self._parse_tirot(sentence, msg.header)
        elif sentence.startswith("$GPXDR"):
            self._parse_gpxdr(sentence)

    def _parse_hchdg(self, sentence: str, header):
        """Parse $HCHDG — magnetic heading with deviation/variation.

        Format: $HCHDG,heading,deviation,dev_dir,variation,var_dir*checksum
        """
        try:
            # Strip checksum portion
            data_part = sentence.split('*')[0]
            fields = data_part.split(',')

            if len(fields) < 6:
                self.get_logger().debug(f"HCHDG: not enough fields: {sentence}")
                return

            if not fields[1]:
                return

            mag_heading = float(fields[1])

            # Extract variation if present in the sentence
            variation = 0.0
            if fields[4] and fields[5]:
                variation = float(fields[4])
                if fields[5].upper() == 'W':
                    variation = -variation

            self.get_logger().debug(
                f"HCHDG: mag_heading={mag_heading:.1f}°, variation={variation:.1f}°"
            )

            # Use as fallback if no HDT for longer than timeout
            elapsed = (self.get_clock().now() - self.last_hdt_time).nanoseconds / 1e9
            if elapsed > self.hdg_fallback_timeout:
                # Compute true heading: mag heading + variation from sentence,
                # or use configured declination if variation not in sentence
                if fields[4] and fields[5]:
                    true_heading = mag_heading + variation
                else:
                    true_heading = mag_heading + self.magnetic_declination_deg

                # Normalize to [0, 360)
                true_heading = true_heading % 360.0

                self.get_logger().info(
                    f"HCHDG fallback: true_heading={true_heading:.1f}° "
                    f"(HDT absent for {elapsed:.1f}s)"
                )

                # Convert heading (degrees clockwise from north) to ENU yaw (radians CCW from east)
                yaw_rad = math.radians(90.0 - true_heading)

                # Yaw-only quaternion: [0, 0, sin(yaw/2), cos(yaw/2)]
                quat = Quaternion()
                quat.x = 0.0
                quat.y = 0.0
                quat.z = math.sin(yaw_rad / 2.0)
                quat.w = math.cos(yaw_rad / 2.0)

                self._publish_heading_imu(header.stamp, quat)

        except (ValueError, IndexError) as e:
            self.get_logger().warning(f"HCHDG parse error: {e} in '{sentence}'")

    def _parse_tirot(self, sentence: str, header):
        """Parse $TIROT — rate of turn from gyroscope.

        Format: $TIROT,rate,status*checksum
        rate: degrees/minute, negative = turning left
        status: A = valid, V = invalid
        """
        try:
            data_part = sentence.split('*')[0]
            fields = data_part.split(',')

            if len(fields) < 3:
                self.get_logger().debug(f"TIROT: not enough fields: {sentence}")
                return

            status = fields[2].strip().upper()
            if status != 'A':
                return

            if not fields[1]:
                return

            rate_deg_min = float(fields[1])

            # Convert deg/min → rad/s
            angular_velocity_z = rate_deg_min * (math.pi / 180.0) / 60.0

            self.get_logger().debug(
                f"TIROT: rate={rate_deg_min:.1f} deg/min → {angular_velocity_z:.5f} rad/s"
            )

            imu = Imu()
            imu.header.stamp = header.stamp
            imu.header.frame_id = "gps"

            imu.angular_velocity.z = angular_velocity_z

            # Gyro covariance — only z populated
            imu.angular_velocity_covariance = [
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, self.gyro_cov,
            ]

            # Orientation and linear acceleration not available
            imu.orientation_covariance[0] = -1.0
            imu.linear_acceleration_covariance[0] = -1.0

            self.pub_gyro.publish(imu)

        except (ValueError, IndexError) as e:
            self.get_logger().warning(f"TIROT parse error: {e} in '{sentence}'")

    def _parse_gpxdr(self, sentence: str):
        """Parse $GPXDR — transducer measurement for blade work state.

        Format: $GPXDR,G,x,,MACHINE_WORK*checksum
        G = generic transducer, x = 1 (working) or 0 (idle)
        """
        try:
            data_part = sentence.split('*')[0]
            fields = data_part.split(',')

            if len(fields) < 5:
                return

            # Only handle MACHINE_WORK transducer
            transducer_id = fields[4].strip()
            if transducer_id != "MACHINE_WORK":
                return

            if not fields[2]:
                return

            value = int(float(fields[2]))
            working = value == 1

            self.get_logger().debug(f"GPXDR MACHINE_WORK: {'active' if working else 'idle'}")

            msg = Bool()
            msg.data = working
            self.pub_work_state.publish(msg)

        except (ValueError, IndexError) as e:
            self.get_logger().warning(f"GPXDR parse error: {e} in '{sentence}'")


def main(args=None):
    rclpy.init(args=args)
    node = HeadingToImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
