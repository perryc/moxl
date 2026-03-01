#!/usr/bin/env python3
"""Engine controller stub node.

Publishes EngineStatus at 1 Hz and serves StartEngine/StopEngine services.
All calls succeed immediately — real GPIO implementation added when hardware is wired.

Future hardware:
    - Starter relay: GPIO 17
    - Choke servo PWM: GPIO 18
    - RPM tach input: GPIO 4 (edge counting)
    - CHT sensor: I2C ADC

Services:
    /moxl/engine/start (moxl/srv/StartEngine)
    /moxl/engine/stop  (moxl/srv/StopEngine)

Publications:
    /moxl/engine/status (moxl/msg/EngineStatus)
"""

import rclpy
from rclpy.node import Node

from moxl.msg import EngineStatus
from moxl.srv import StartEngine, StopEngine


class EngineControllerNode(Node):
    def __init__(self):
        super().__init__("engine_controller")

        self.state = EngineStatus.OFF
        self.rpm = 0.0
        self.cht = 0.0
        self.choke = False
        self.error_msg = ""

        self.pub_status = self.create_publisher(
            EngineStatus, "moxl/engine/status", 10
        )

        self.srv_start = self.create_service(
            StartEngine, "moxl/engine/start", self.start_callback
        )
        self.srv_stop = self.create_service(
            StopEngine, "moxl/engine/stop", self.stop_callback
        )

        self.timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info("Engine controller (stub) ready")

    def publish_status(self):
        msg = EngineStatus()
        msg.state = self.state
        msg.rpm = self.rpm
        msg.cht_celsius = self.cht
        msg.choke_engaged = self.choke
        msg.error_message = self.error_msg
        self.pub_status.publish(msg)

    def start_callback(self, request, response):
        if self.state == EngineStatus.RUNNING:
            response.success = True
            response.message = "Engine already running"
            return response

        self.get_logger().info("Starting engine (stub — immediate success)")
        self.state = EngineStatus.RUNNING
        self.rpm = 3600.0  # simulated idle RPM
        response.success = True
        response.message = "Engine started"
        return response

    def stop_callback(self, request, response):
        if self.state == EngineStatus.OFF:
            response.success = True
            response.message = "Engine already off"
            return response

        self.get_logger().info("Stopping engine (stub — immediate success)")
        self.state = EngineStatus.OFF
        self.rpm = 0.0
        response.success = True
        response.message = "Engine stopped"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = EngineControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
