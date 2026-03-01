#!/usr/bin/env python3
"""Safety monitor node.

Watches GPS fix quality and publishes e-stop when conditions are unsafe.
LiDAR obstacle detection is stubbed — no hardware selected yet.

Behavior:
    - GPS fix timeout (>2s no message): e-stop + disengage blades
    - GPS fix quality degradation (not RTK fix): e-stop + disengage blades
    - When safe conditions return: release e-stop

Subscriptions:
    /gps/fix (sensor_msgs/NavSatFix): GPS position + fix status

Publications:
    /e_stop (std_msgs/Bool): twist_mux lock (True = locked)

Service clients:
    /moxl/blades/set (moxl/srv/SetBlades): emergency blade disengage
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool

from moxl.srv import SetBlades


class SafetyMonitorNode(Node):
    # NavSatFix status constants
    FIX_RTK = 2       # STATUS_GBAS_FIX — RTK fix
    FIX_SBAS = 1      # STATUS_SBAS_FIX
    FIX_NONE = -1     # STATUS_NO_FIX

    def __init__(self):
        super().__init__("safety_monitor")

        self.declare_parameter("gps_timeout_sec", 2.0)
        self.declare_parameter("require_rtk", True)
        self.declare_parameter("watchdog_rate_hz", 5.0)

        self.gps_timeout = self.get_parameter("gps_timeout_sec").value
        self.require_rtk = self.get_parameter("require_rtk").value
        watchdog_hz = self.get_parameter("watchdog_rate_hz").value

        self.cb_group = ReentrantCallbackGroup()

        self.last_fix_time = None
        self.last_fix_status = self.FIX_NONE
        self.estopped = False

        # Subscriber
        self.create_subscription(
            NavSatFix, "/gps/fix", self._gps_cb, 10,
            callback_group=self.cb_group,
        )

        # Publisher
        self.estop_pub = self.create_publisher(Bool, "e_stop", 10)

        # Blade client for emergency disengage
        self.cli_blades = self.create_client(
            SetBlades, "moxl/blades/set", callback_group=self.cb_group
        )

        # Watchdog timer
        self.create_timer(
            1.0 / watchdog_hz, self._watchdog_cb,
            callback_group=self.cb_group,
        )

        self.get_logger().info(
            f"Safety monitor active: GPS timeout={self.gps_timeout}s, "
            f"require_rtk={self.require_rtk}"
        )

    def _gps_cb(self, msg: NavSatFix):
        self.last_fix_time = self.get_clock().now()
        self.last_fix_status = msg.status.status

    def _watchdog_cb(self):
        now = self.get_clock().now()
        should_estop = False
        reason = ""

        if self.last_fix_time is None:
            # No GPS fix received yet — don't e-stop until we've had one
            self.estop_pub.publish(Bool(data=False))
            return

        elapsed = (now - self.last_fix_time).nanoseconds / 1e9

        if elapsed > self.gps_timeout:
            should_estop = True
            reason = f"GPS timeout ({elapsed:.1f}s > {self.gps_timeout}s)"
        elif self.require_rtk and self.last_fix_status < self.FIX_RTK:
            should_estop = True
            reason = f"GPS fix quality degraded (status={self.last_fix_status})"

        if should_estop and not self.estopped:
            self.get_logger().warn(f"E-STOP: {reason}")
            self.estopped = True
            self.estop_pub.publish(Bool(data=True))
            self._emergency_blades_off()
        elif not should_estop and self.estopped:
            self.get_logger().info("GPS fix restored — releasing e-stop")
            self.estopped = False
            self.estop_pub.publish(Bool(data=False))
        else:
            # Maintain current state
            self.estop_pub.publish(Bool(data=self.estopped))

    def _emergency_blades_off(self):
        if not self.cli_blades.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Blade service unavailable for emergency stop")
            return
        req = SetBlades.Request()
        req.engage = False
        future = self.cli_blades.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info("Emergency blade disengage sent")
        )


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
