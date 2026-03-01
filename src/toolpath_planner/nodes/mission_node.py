#!/usr/bin/env python3
"""Mission orchestrator for autonomous mowing.

Implements a MowMission action server that sequences:
    PREFLIGHT → TRANSITING → MOWING (N strips) → RETURNING → PARKED

Uses Nav2 for path following, toolpath_node for strip generation,
and engine/blade controller stubs for subsystem coordination.

Action server:
    /moxl/mission (moxl/action/MowMission)

Subscriptions:
    /moxl/toolpath/current_strip (nav_msgs/Path): Current strip waypoints (lat/lon)
    /moxl/engine/status (moxl/msg/EngineStatus): Engine state
    /moxl/blades/status (moxl/msg/BladeStatus): Blade state

Publications:
    /moxl/mission/status (moxl/msg/MissionStatus): Current mission state
"""

import math
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from geographic_msgs.msg import GeoPoint
from std_srvs.srv import Trigger

from moxl.msg import EngineStatus, BladeStatus, MissionStatus
from moxl.srv import StartEngine, StopEngine, SetBlades
from moxl.action import MowMission

from nav2_msgs.action import NavigateThroughPoses
from robot_localization.srv import FromLL


class MissionNode(Node):
    def __init__(self):
        super().__init__("mission_node")

        self.declare_parameter("park_lat", 50.63860)
        self.declare_parameter("park_lon", -105.04030)

        self.cb_group = ReentrantCallbackGroup()

        # State
        self.state = MissionStatus.IDLE
        self.current_strip = 0
        self.total_strips = 0
        self.current_strip_path = None
        self.engine_status = None
        self.blade_status = None
        self._goal_lock = threading.Lock()
        self._active_nav_goal: ClientGoalHandle | None = None

        # Subscribers
        self.create_subscription(
            Path, "/moxl/toolpath/current_strip",
            self._strip_path_cb, 10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            EngineStatus, "moxl/engine/status",
            self._engine_status_cb, 10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            BladeStatus, "moxl/blades/status",
            self._blade_status_cb, 10,
            callback_group=self.cb_group,
        )

        # Publisher
        self.status_pub = self.create_publisher(
            MissionStatus, "moxl/mission/status", 10
        )
        self.create_timer(1.0, self._publish_status, callback_group=self.cb_group)

        # Service clients
        self.cli_start_engine = self.create_client(
            StartEngine, "moxl/engine/start", callback_group=self.cb_group
        )
        self.cli_stop_engine = self.create_client(
            StopEngine, "moxl/engine/stop", callback_group=self.cb_group
        )
        self.cli_set_blades = self.create_client(
            SetBlades, "moxl/blades/set", callback_group=self.cb_group
        )
        self.cli_generate = self.create_client(
            Trigger, "/moxl/toolpath/generate", callback_group=self.cb_group
        )
        self.cli_next_strip = self.create_client(
            Trigger, "/moxl/toolpath/next_strip", callback_group=self.cb_group
        )
        self.cli_from_ll = self.create_client(
            FromLL, "/fromLL", callback_group=self.cb_group
        )

        # Nav2 action client
        self._nav_client = rclpy.action.ActionClient(
            self, NavigateThroughPoses, "navigate_through_poses",
            callback_group=self.cb_group,
        )

        # Mission action server
        self._action_server = ActionServer(
            self,
            MowMission,
            "moxl/mission",
            execute_callback=self._execute_mission,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info("Mission orchestrator ready")

    # ── Subscription callbacks ───────────────────────────────────────

    def _strip_path_cb(self, msg: Path):
        self.current_strip_path = msg

    def _engine_status_cb(self, msg: EngineStatus):
        self.engine_status = msg

    def _blade_status_cb(self, msg: BladeStatus):
        self.blade_status = msg

    # ── Status publishing ────────────────────────────────────────────

    def _publish_status(self):
        msg = MissionStatus()
        msg.state = self.state
        msg.current_strip = self.current_strip
        msg.total_strips = self.total_strips
        msg.progress_pct = (
            (self.current_strip / self.total_strips * 100.0)
            if self.total_strips > 0 else 0.0
        )
        msg.status_text = self._state_name(self.state)
        self.status_pub.publish(msg)

    @staticmethod
    def _state_name(state: int) -> str:
        names = {
            MissionStatus.IDLE: "Idle",
            MissionStatus.PREFLIGHT: "Preflight",
            MissionStatus.TRANSITING: "Transiting to start",
            MissionStatus.MOWING: "Mowing",
            MissionStatus.RETURNING: "Returning to park",
            MissionStatus.PARKED: "Parked",
            MissionStatus.ERROR: "Error",
            MissionStatus.PAUSED: "Paused",
        }
        return names.get(state, "Unknown")

    # ── Action server callbacks ──────────────────────────────────────

    def _goal_callback(self, goal_request):
        if self.state not in (MissionStatus.IDLE, MissionStatus.PARKED):
            self.get_logger().warn("Rejecting mission — already active")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel requested")
        # Cancel active Nav2 goal if any
        with self._goal_lock:
            if self._active_nav_goal is not None:
                self._active_nav_goal.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def _execute_mission(self, goal_handle):
        """Main mission execution loop."""
        request = goal_handle.request
        feedback = MowMission.Feedback()
        result = MowMission.Result()

        self.get_logger().info(
            f"Starting mission: {request.airstrip_id} runway {request.runway_id}"
        )

        try:
            # ── PREFLIGHT ────────────────────────────────────────
            self.state = MissionStatus.PREFLIGHT
            self._send_feedback(goal_handle, feedback)

            # Start engine
            ok = await self._call_start_engine()
            if not ok:
                return self._abort(goal_handle, result, "Engine start failed")
            if goal_handle.is_cancel_requested:
                return self._cancel(goal_handle, result)

            # Engage blades
            ok = await self._call_set_blades(True)
            if not ok:
                return self._abort(goal_handle, result, "Blade engagement failed")
            if goal_handle.is_cancel_requested:
                return self._cancel(goal_handle, result)

            # Generate toolpath
            ok = await self._call_generate_toolpath()
            if not ok:
                return self._abort(goal_handle, result, "Toolpath generation failed")
            if goal_handle.is_cancel_requested:
                return self._cancel(goal_handle, result)

            # Wait for first strip path to arrive
            for _ in range(50):  # 5 second timeout
                if self.current_strip_path and len(self.current_strip_path.poses) > 0:
                    break
                await self._sleep(0.1)
            else:
                return self._abort(goal_handle, result, "No strip path received")

            # Count strips by checking how many next_strip calls succeed
            self.total_strips = await self._count_strips()
            if self.total_strips == 0:
                return self._abort(goal_handle, result, "No strips generated")

            # Regenerate to reset strip index
            await self._call_generate_toolpath()
            await self._sleep(1.0)  # Let first strip path arrive

            self.get_logger().info(f"Toolpath ready: {self.total_strips} strips")

            # ── MOWING ───────────────────────────────────────────
            self.state = MissionStatus.MOWING
            self.current_strip = 0

            for strip_idx in range(self.total_strips):
                if goal_handle.is_cancel_requested:
                    return self._cancel(goal_handle, result)

                self.current_strip = strip_idx + 1
                self._send_feedback(goal_handle, feedback)
                self.get_logger().info(
                    f"Mowing strip {self.current_strip}/{self.total_strips}"
                )

                # Wait for current strip path
                await self._sleep(0.5)
                if self.current_strip_path is None:
                    return self._abort(goal_handle, result, "Strip path missing")

                # Convert lat/lon waypoints to map frame
                map_poses = await self._convert_strip_to_map(
                    self.current_strip_path
                )
                if not map_poses:
                    return self._abort(
                        goal_handle, result,
                        f"Failed to convert strip {self.current_strip} to map frame"
                    )

                # Navigate the strip via Nav2
                nav_ok = await self._navigate_through(map_poses)
                if not nav_ok:
                    if goal_handle.is_cancel_requested:
                        return self._cancel(goal_handle, result)
                    return self._abort(
                        goal_handle, result,
                        f"Navigation failed on strip {self.current_strip}"
                    )

                # Advance to next strip
                if strip_idx < self.total_strips - 1:
                    await self._call_next_strip()

            # ── RETURNING ────────────────────────────────────────
            self.state = MissionStatus.RETURNING
            self._send_feedback(goal_handle, feedback)

            park_lat = self.get_parameter("park_lat").value
            park_lon = self.get_parameter("park_lon").value
            park_pose = await self._latlon_to_map(park_lat, park_lon)

            if park_pose:
                await self._navigate_through([park_pose])

            # ── PARKED ───────────────────────────────────────────
            self.state = MissionStatus.PARKED

            # Disengage blades
            await self._call_set_blades(False)
            # Stop engine
            await self._call_stop_engine()

            self._send_feedback(goal_handle, feedback)
            result.success = True
            result.strips_completed = self.total_strips
            result.message = f"Mission complete: {self.total_strips} strips mowed"

            self.get_logger().info(result.message)
            goal_handle.succeed()
            self.state = MissionStatus.IDLE
            return result

        except Exception as e:
            self.get_logger().error(f"Mission exception: {e}")
            return self._abort(goal_handle, result, str(e))

    # ── Helper methods ───────────────────────────────────────────────

    def _send_feedback(self, goal_handle, feedback):
        feedback.state = self.state
        feedback.current_strip = self.current_strip
        feedback.total_strips = self.total_strips
        feedback.progress_pct = (
            (self.current_strip / self.total_strips * 100.0)
            if self.total_strips > 0 else 0.0
        )
        feedback.status_text = self._state_name(self.state)
        goal_handle.publish_feedback(feedback)

    def _abort(self, goal_handle, result, message):
        self.get_logger().error(f"Mission aborted: {message}")
        self.state = MissionStatus.ERROR
        result.success = False
        result.strips_completed = max(0, self.current_strip - 1)
        result.message = message
        goal_handle.abort()
        return result

    def _cancel(self, goal_handle, result):
        self.get_logger().info("Mission cancelled")
        self.state = MissionStatus.PAUSED
        result.success = False
        result.strips_completed = max(0, self.current_strip - 1)
        result.message = "Mission cancelled by user"
        goal_handle.canceled()
        return result

    async def _sleep(self, seconds):
        """Non-blocking sleep."""
        rate = self.create_rate(int(1.0 / seconds) if seconds > 0 else 10)
        rate.sleep()

    # ── Service call helpers ─────────────────────────────────────────

    async def _call_start_engine(self):
        if not self.cli_start_engine.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Engine start service not available")
            return False
        resp = await self.cli_start_engine.call_async(StartEngine.Request())
        if not resp.success:
            self.get_logger().error(f"Engine start failed: {resp.message}")
        return resp.success

    async def _call_stop_engine(self):
        if not self.cli_stop_engine.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Engine stop service not available")
            return False
        resp = await self.cli_stop_engine.call_async(StopEngine.Request())
        return resp.success

    async def _call_set_blades(self, engage: bool):
        if not self.cli_set_blades.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Blade service not available")
            return False
        req = SetBlades.Request()
        req.engage = engage
        resp = await self.cli_set_blades.call_async(req)
        if not resp.success:
            self.get_logger().error(f"Blade control failed: {resp.message}")
        return resp.success

    async def _call_generate_toolpath(self):
        if not self.cli_generate.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Toolpath generate service not available")
            return False
        resp = await self.cli_generate.call_async(Trigger.Request())
        if not resp.success:
            self.get_logger().error(f"Toolpath generation failed: {resp.message}")
        return resp.success

    async def _call_next_strip(self):
        if not self.cli_next_strip.wait_for_service(timeout_sec=2.0):
            return False
        resp = await self.cli_next_strip.call_async(Trigger.Request())
        return resp.success

    async def _count_strips(self):
        """Count strips by repeatedly calling next_strip until it reports complete."""
        count = 1  # current strip is #1
        for _ in range(200):  # safety limit
            if not self.cli_next_strip.wait_for_service(timeout_sec=2.0):
                break
            resp = await self.cli_next_strip.call_async(Trigger.Request())
            if not resp.success or "complete" in resp.message.lower():
                break
            count += 1
        return count

    # ── Coordinate conversion ────────────────────────────────────────

    async def _latlon_to_map(self, lat: float, lon: float) -> PoseStamped | None:
        """Convert a lat/lon to map frame PoseStamped via fromLL service."""
        if not self.cli_from_ll.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("fromLL service not available")
            return None

        req = FromLL.Request()
        req.ll_point = GeoPoint(latitude=lat, longitude=lon, altitude=0.0)
        resp = await self.cli_from_ll.call_async(req)

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = resp.map_point
        pose.pose.orientation.w = 1.0
        return pose

    async def _convert_strip_to_map(self, strip_path: Path) -> list[PoseStamped]:
        """Convert a strip path from lat/lon to map frame poses."""
        map_poses = []
        for pose in strip_path.poses:
            # toolpath_node stores lat in x, lon in y
            lat = pose.pose.position.x
            lon = pose.pose.position.y

            map_pose = await self._latlon_to_map(lat, lon)
            if map_pose is None:
                return []

            # Compute heading from consecutive points
            map_poses.append(map_pose)

        # Set orientation based on direction of travel
        for i in range(len(map_poses) - 1):
            dx = map_poses[i + 1].pose.position.x - map_poses[i].pose.position.x
            dy = map_poses[i + 1].pose.position.y - map_poses[i].pose.position.y
            yaw = math.atan2(dy, dx)
            map_poses[i].pose.orientation.z = math.sin(yaw / 2.0)
            map_poses[i].pose.orientation.w = math.cos(yaw / 2.0)

        # Last pose gets same orientation as second-to-last
        if len(map_poses) >= 2:
            map_poses[-1].pose.orientation = map_poses[-2].pose.orientation

        return map_poses

    # ── Nav2 integration ─────────────────────────────────────────────

    async def _navigate_through(self, poses: list[PoseStamped]) -> bool:
        """Send poses to Nav2 NavigateThroughPoses and wait for completion."""
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 navigate_through_poses not available")
            return False

        goal = NavigateThroughPoses.Goal()
        goal.poses = poses

        send_goal_future = self._nav_client.send_goal_async(goal)
        goal_handle = await send_goal_future

        if not goal_handle.accepted:
            self.get_logger().error("Nav2 goal rejected")
            return False

        with self._goal_lock:
            self._active_nav_goal = goal_handle

        result_future = goal_handle.get_result_async()
        result = await result_future

        with self._goal_lock:
            self._active_nav_goal = None

        return result.status == 4  # STATUS_SUCCEEDED


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
