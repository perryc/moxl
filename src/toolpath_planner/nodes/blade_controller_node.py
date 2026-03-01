#!/usr/bin/env python3
"""Blade controller stub node.

Publishes BladeStatus at 1 Hz and serves SetBlades service.
Stub: toggles state immediately. Real implementation drives GPIO 25 relay
+ belt engagement actuator (TBD).

Services:
    /moxl/blades/set (moxl/srv/SetBlades)

Publications:
    /moxl/blades/status (moxl/msg/BladeStatus)
"""

import rclpy
from rclpy.node import Node

from moxl.msg import BladeStatus
from moxl.srv import SetBlades


class BladeControllerNode(Node):
    def __init__(self):
        super().__init__("blade_controller")

        self.state = BladeStatus.DISENGAGED
        self.error_msg = ""

        self.pub_status = self.create_publisher(
            BladeStatus, "moxl/blades/status", 10
        )

        self.srv_set = self.create_service(
            SetBlades, "moxl/blades/set", self.set_callback
        )

        self.timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info("Blade controller (stub) ready")

    def publish_status(self):
        msg = BladeStatus()
        msg.state = self.state
        msg.error_message = self.error_msg
        self.pub_status.publish(msg)

    def set_callback(self, request, response):
        if request.engage:
            if self.state == BladeStatus.ENGAGED:
                response.success = True
                response.message = "Blades already engaged"
                return response

            self.get_logger().info("Engaging blades (stub — immediate success)")
            self.state = BladeStatus.ENGAGED
            response.success = True
            response.message = "Blades engaged"
        else:
            if self.state == BladeStatus.DISENGAGED:
                response.success = True
                response.message = "Blades already disengaged"
                return response

            self.get_logger().info("Disengaging blades (stub — immediate success)")
            self.state = BladeStatus.DISENGAGED
            response.success = True
            response.message = "Blades disengaged"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = BladeControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
