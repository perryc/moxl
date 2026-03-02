#!/usr/bin/env python3
"""Blade controller node.

Publishes BladeStatus at 1 Hz and serves SetBlades service.
Drives GPIO 25 relay for blade engagement (TBD hardware).
Subscribes to /gps/work_state for hardware feedback: the blade actuator is
wired to the FarmTRX GPS digital input, which reports engagement via
$GPXDR MACHINE_WORK NMEA sentences.

Services:
    /moxl/blades/set (moxl/srv/SetBlades)

Subscriptions:
    /gps/work_state (std_msgs/Bool): Hardware feedback from GPS digital input

Publications:
    /moxl/blades/status (moxl/msg/BladeStatus)
"""

import rclpy
from rclpy.node import Node

from moxl.msg import BladeStatus
from moxl.srv import SetBlades
from std_msgs.msg import Bool


class BladeControllerNode(Node):
    def __init__(self):
        super().__init__("blade_controller")

        self.declare_parameter("engage_timeout", 5.0)  # seconds to wait for hardware confirmation
        self.engage_timeout = self.get_parameter("engage_timeout").value

        self.state = BladeStatus.DISENGAGED
        self.error_msg = ""
        self.commanded_engage = False
        self.command_time = None

        self.pub_status = self.create_publisher(
            BladeStatus, "moxl/blades/status", 10
        )

        self.srv_set = self.create_service(
            SetBlades, "moxl/blades/set", self.set_callback
        )

        self.sub_work_state = self.create_subscription(
            Bool, "gps/work_state", self.work_state_callback, 10
        )

        self.timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info("Blade controller ready (GPS work_state feedback)")

    def publish_status(self):
        # Check for engage/disengage timeout
        if self.command_time and self.state in (BladeStatus.ENGAGING, BladeStatus.DISENGAGING):
            elapsed = (self.get_clock().now() - self.command_time).nanoseconds / 1e9
            if elapsed > self.engage_timeout:
                action = "engage" if self.state == BladeStatus.ENGAGING else "disengage"
                self.get_logger().error(
                    f"Blade {action} timed out after {self.engage_timeout}s "
                    f"— no GPS work_state confirmation"
                )
                self.state = BladeStatus.ERROR
                self.error_msg = f"Blade {action} timeout — check actuator/GPS"
                self.command_time = None

        msg = BladeStatus()
        msg.state = self.state
        msg.error_message = self.error_msg
        self.pub_status.publish(msg)

    def work_state_callback(self, msg: Bool):
        """Handle hardware feedback from GPS digital input."""
        hw_engaged = msg.data

        if self.state == BladeStatus.ENGAGING and hw_engaged:
            self.get_logger().info("Blades ENGAGED (confirmed by GPS work_state)")
            self.state = BladeStatus.ENGAGED
            self.error_msg = ""

        elif self.state == BladeStatus.DISENGAGING and not hw_engaged:
            self.get_logger().info("Blades DISENGAGED (confirmed by GPS work_state)")
            self.state = BladeStatus.DISENGAGED
            self.error_msg = ""

        elif self.state == BladeStatus.ENGAGED and not hw_engaged:
            self.get_logger().warn("Blades disengaged unexpectedly!")
            self.state = BladeStatus.ERROR
            self.error_msg = "Unexpected disengage — check belt/actuator"

        elif self.state == BladeStatus.DISENGAGED and hw_engaged:
            self.get_logger().warn("Blades engaged unexpectedly!")
            self.state = BladeStatus.ERROR
            self.error_msg = "Unexpected engage — check actuator"

    def set_callback(self, request, response):
        if request.engage:
            if self.state == BladeStatus.ENGAGED:
                response.success = True
                response.message = "Blades already engaged"
                return response

            self.get_logger().info("Engaging blades — waiting for GPS work_state confirmation")
            # TODO: drive GPIO 25 relay here
            self.state = BladeStatus.ENGAGING
            self.commanded_engage = True
            self.command_time = self.get_clock().now()
            response.success = True
            response.message = "Blade engage commanded, awaiting hardware confirmation"
        else:
            if self.state == BladeStatus.DISENGAGED:
                response.success = True
                response.message = "Blades already disengaged"
                return response

            self.get_logger().info("Disengaging blades — waiting for GPS work_state confirmation")
            # TODO: release GPIO 25 relay here
            self.state = BladeStatus.DISENGAGING
            self.commanded_engage = False
            self.command_time = self.get_clock().now()
            response.success = True
            response.message = "Blade disengage commanded, awaiting hardware confirmation"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = BladeControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
