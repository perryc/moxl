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
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from moxl.msg import EngineStatus, BladeStatus, MissionStatus, RadioDetection
from moxl.srv import StartEngine, StopEngine, SetBlades
from moxl.action import MowMission

from nav2_msgs.action import NavigateThroughPoses, NavigateToPose, FollowPath
from robot_localization.srv import FromLL


class MissionNode(Node):
    def __init__(self):
        super().__init__("mission_node")

        self.declare_parameter("park_lat", 50.63787)
        self.declare_parameter("park_lon", -105.03854)
        self.declare_parameter("quiet_period_sec", 300.0)

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
        self._radio_active = False
        self._radio_detection: RadioDetection | None = None
        self._waypoint_resume_index = 0
        self._evacuated = False  # True when mower has reached clear zone
        self._clear_zones: list[dict] = []  # loaded from toolpath_node

        # Subscribers
        self.full_toolpath = None
        self.create_subscription(
            Path, "/moxl/toolpath/all",
            self._full_toolpath_cb, 10,
            callback_group=self.cb_group,
        )
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
        self.create_subscription(
            Bool, "/sdr/radio_active",
            self._radio_active_cb, 10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            RadioDetection, "/sdr/detection",
            self._radio_detection_cb, 10,
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

        # Service client for clear zone data from toolpath_node
        self.cli_clear_zones = self.create_client(
            Trigger, "/moxl/toolpath/clear_zones", callback_group=self.cb_group
        )

        # TF buffer for checking frame readiness
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Nav2 action clients
        self._nav_client = rclpy.action.ActionClient(
            self, NavigateThroughPoses, "navigate_through_poses",
            callback_group=self.cb_group,
        )
        self._follow_path_client = rclpy.action.ActionClient(
            self, FollowPath, "follow_path",
            callback_group=self.cb_group,
        )
        self._nav_to_pose_client = rclpy.action.ActionClient(
            self, NavigateToPose, "navigate_to_pose",
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

    def _full_toolpath_cb(self, msg: Path):
        self.full_toolpath = msg

    def _strip_path_cb(self, msg: Path):
        self.current_strip_path = msg

    def _engine_status_cb(self, msg: EngineStatus):
        self.engine_status = msg

    def _blade_status_cb(self, msg: BladeStatus):
        self.blade_status = msg

    def _radio_active_cb(self, msg: Bool):
        self._radio_active = msg.data

    def _radio_detection_cb(self, msg: RadioDetection):
        self._radio_detection = msg

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
        # State 8 = EVACUATING (radio traffic)
        if state == 8:
            return "Evacuating runway"
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

            # Wait for full toolpath to arrive
            for _ in range(50):  # 5 second timeout
                if self.full_toolpath and len(self.full_toolpath.poses) > 0:
                    break
                await self._sleep(0.1)
            else:
                return self._abort(goal_handle, result, "No toolpath received")

            # Count strips
            self.total_strips = await self._count_strips()
            if self.total_strips == 0:
                return self._abort(goal_handle, result, "No strips generated")

            self.get_logger().info(
                f"Toolpath ready: {self.total_strips} strips, "
                f"{len(self.full_toolpath.poses)} waypoints"
            )

            # ── LOAD CLEAR ZONES ──────────────────────────────────
            await self._load_clear_zones()

            # ── MOWING (per-strip) ────────────────────────────────
            # Send each strip as a separate FollowPath goal to prevent
            # the RPP controller from matching to adjacent spiral rings
            # (rings are only 1.47m apart but 1400m on the path).
            self.state = MissionStatus.MOWING
            self.current_strip = 1
            self._send_feedback(goal_handle, feedback)

            # Re-generate to reset strip iterator (count_strips consumed it)
            await self._call_generate_toolpath()
            await self._sleep(0.5)

            for strip_num in range(1, self.total_strips + 1):
                if goal_handle.is_cancel_requested:
                    return self._cancel(goal_handle, result)

                self.current_strip = strip_num
                self._send_feedback(goal_handle, feedback)

                # Wait for current strip path
                self.current_strip_path = None
                for _ in range(50):
                    if self.current_strip_path and len(self.current_strip_path.poses) > 0:
                        break
                    await self._sleep(0.1)

                if not self.current_strip_path or len(self.current_strip_path.poses) == 0:
                    self.get_logger().warn(f"Strip {strip_num}: no path, skipping")
                    await self._call_next_strip()
                    continue

                # Convert strip to map frame
                map_poses = await self._convert_strip_to_map(self.current_strip_path)
                if not map_poses:
                    self.get_logger().warn(f"Strip {strip_num}: conversion failed, skipping")
                    await self._call_next_strip()
                    continue

                # Prepend transit from robot to first waypoint
                next_wp = map_poses[1] if len(map_poses) > 1 else None
                transit = self._build_transit_path(map_poses[0], next_target=next_wp)
                if transit:
                    map_poses = transit + map_poses

                self.get_logger().info(
                    f"Strip {strip_num}/{self.total_strips}: "
                    f"{len(map_poses)} waypoints"
                )

                # Follow strip with radio evacuation support
                self._waypoint_resume_index = 0
                while self._waypoint_resume_index < len(map_poses):
                    if goal_handle.is_cancel_requested:
                        return self._cancel(goal_handle, result)

                    remaining = map_poses[self._waypoint_resume_index:]
                    nav_ok = await self._follow_path_with_radio_check(
                        remaining, goal_handle, feedback
                    )

                    if nav_ok:
                        break

                    if goal_handle.is_cancel_requested:
                        return self._cancel(goal_handle, result)

                    if not self._radio_active and not self._evacuated:
                        return self._abort(
                            goal_handle, result,
                            f"Navigation failed on strip {strip_num}"
                        )

                    self._evacuated = False

                # Advance to next strip
                await self._call_next_strip()

            self.current_strip = self.total_strips

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

    # ── Radio evacuation ────────────────────────────────────────────

    async def _load_clear_zones(self):
        """Load clear zone centroids from toolpath_node."""
        import json
        if not self.cli_clear_zones.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Clear zones service not available")
            return
        resp = await self.cli_clear_zones.call_async(Trigger.Request())
        if resp.success:
            self._clear_zones = json.loads(resp.message)
            self.get_logger().info(
                f"Loaded {len(self._clear_zones)} clear zones for evacuation"
            )
        else:
            self.get_logger().warn("No clear zones available for radio evacuation")

    async def _follow_path_with_radio_check(
        self, poses, goal_handle, feedback
    ) -> bool:
        """Follow path but cancel and evacuate if radio becomes active.

        Returns True if path completed, False if interrupted by radio or error.
        """
        if not self._follow_path_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 follow_path not available")
            return False

        # Wait for map→odom TF to be available before sending path
        try:
            self._tf_buffer.lookup_transform(
                "odom", "map", Time(seconds=0), timeout=Duration(seconds=10)
            )
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"TF not ready (map→odom): {e}")
            return False

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = Time(seconds=0).to_msg()  # latest TF available
        for p in poses:
            p.header.stamp = Time(seconds=0).to_msg()
            p.header.frame_id = "map"
            path_msg.poses.append(p)

        goal = FollowPath.Goal()
        goal.path = path_msg
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "general_goal_checker"

        send_goal_future = self._follow_path_client.send_goal_async(goal)
        goal_handle_nav = await send_goal_future

        if not goal_handle_nav.accepted:
            self.get_logger().error("FollowPath goal rejected")
            return False

        with self._goal_lock:
            self._active_nav_goal = goal_handle_nav

        result_future = goal_handle_nav.get_result_async()

        # Poll for completion or radio detection
        while not result_future.done():
            await self._sleep(0.2)

            if self._radio_active and self.state == MissionStatus.MOWING:
                self.get_logger().warn(
                    "Radio traffic detected — cancelling mowing, evacuating!"
                )
                # Cancel the active navigation
                with self._goal_lock:
                    if self._active_nav_goal is not None:
                        self._active_nav_goal.cancel_goal_async()
                        self._active_nav_goal = None

                # Estimate resume index: fraction of remaining path completed
                # (approximate — we'll resume from nearest waypoint)
                await self._evacuate_to_clear_zone()
                await self._wait_for_quiet(goal_handle, feedback)

                # Resume mowing from where we left off
                self.state = MissionStatus.MOWING
                self._send_feedback(goal_handle, feedback)
                return False  # Signal to retry from resume index

            if goal_handle.is_cancel_requested:
                with self._goal_lock:
                    if self._active_nav_goal is not None:
                        self._active_nav_goal.cancel_goal_async()
                        self._active_nav_goal = None
                return False

        with self._goal_lock:
            self._active_nav_goal = None

        result = result_future.result()
        self.get_logger().info(
            f"FollowPath finished with status={result.status} "
            f"(4=OK, 5=CANCELED, 6=ABORTED)"
        )
        if result.status == 4:  # STATUS_SUCCEEDED
            self._waypoint_resume_index = len(poses)  # all done
            return True
        return False

    async def _evacuate_to_clear_zone(self):
        """Navigate to the nearest clear zone centroid."""
        self.state = 8  # EVACUATING
        self._evacuated = True

        if not self._clear_zones:
            self.get_logger().warn(
                "No clear zones configured — stopping in place"
            )
            return

        # Get current GPS position from the toolpath poses
        # Use park position as rough fallback for nearest-zone calculation
        park_lat = self.get_parameter("park_lat").value
        park_lon = self.get_parameter("park_lon").value

        # Find nearest clear zone
        best = None
        best_dist = float("inf")
        for zone in self._clear_zones:
            clat, clon = zone["centroid"]
            dist = math.sqrt((park_lat - clat) ** 2 + (park_lon - clon) ** 2)
            if dist < best_dist:
                best_dist = dist
                best = zone

        if best is None:
            self.get_logger().error("Could not determine nearest clear zone")
            return

        clat, clon = best["centroid"]
        self.get_logger().info(
            f"Evacuating to clear zone '{best['name']}' "
            f"at ({clat:.6f}, {clon:.6f})"
        )

        # Navigate to clear zone centroid
        target = await self._latlon_to_map(clat, clon)
        if target:
            await self._navigate_to_pose(target)

        self.get_logger().info(f"Arrived at clear zone '{best['name']}'")

    async def _wait_for_quiet(self, goal_handle, feedback):
        """Wait until radio has been quiet for quiet_period_sec."""
        quiet_period = self.get_parameter("quiet_period_sec").value
        self.state = MissionStatus.PAUSED
        self._send_feedback(goal_handle, feedback)

        self.get_logger().info(
            f"Waiting for {quiet_period:.0f}s of radio silence before resuming"
        )

        while True:
            await self._sleep(1.0)

            if goal_handle.is_cancel_requested:
                return

            if self._radio_detection is not None:
                quiet = self._radio_detection.quiet_duration_sec
            elif not self._radio_active:
                # No detection messages yet but radio isn't active — count up
                quiet = quiet_period  # assume quiet
            else:
                quiet = 0.0

            if quiet >= quiet_period:
                self.get_logger().info(
                    f"Radio quiet for {quiet:.0f}s — resuming mission"
                )
                return

    async def _navigate_to_pose(self, pose: PoseStamped) -> bool:
        """Send a single pose to Nav2 NavigateToPose and wait."""
        if not self._nav_to_pose_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 navigate_to_pose not available")
            return False

        goal = NavigateToPose.Goal()
        goal.pose = pose

        send_goal_future = self._nav_to_pose_client.send_goal_async(goal)
        goal_handle = await send_goal_future

        if not goal_handle.accepted:
            self.get_logger().error("NavigateToPose goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        result = await result_future
        return result.status == 4  # STATUS_SUCCEEDED

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
        pose.header.stamp = Time(seconds=0).to_msg()  # latest TF available
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

    # ── Transit path ────────────────────────────────────────────────

    def _build_transit_path(
        self, target: PoseStamped, spacing: float = 1.5,
        next_target: PoseStamped | None = None,
    ) -> list[PoseStamped]:
        """Build a transit path from the robot to *target* with lead-in.

        If *next_target* is provided, the path includes a lead-in segment
        that aligns the robot's heading with the first toolpath leg
        (target → next_target) before arriving at *target*.  This prevents
        the cross-track overshoot that occurs when the transit heading
        differs significantly from the first mowing leg heading.

        Returns a list of PoseStamped in the map frame, spaced every *spacing*
        meters. Returns empty list if robot pose is unavailable or already
        within *spacing* of the target.
        """
        # Get current robot pose in map frame via TF
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                "map", "base_link", Time(seconds=0),
                timeout=Duration(seconds=5),
            )
        except Exception as e:
            self.get_logger().warn(f"Cannot get robot pose for transit: {e}")
            return []

        rx = tf_msg.transform.translation.x
        ry = tf_msg.transform.translation.y
        tx = target.pose.position.x
        ty = target.pose.position.y

        # Compute lead-in approach point aligned with first toolpath segment
        lead_in_dist = 20.0  # meters of straight approach before toolpath
        if next_target is not None:
            ntx = next_target.pose.position.x
            nty = next_target.pose.position.y
            leg_dx = ntx - tx
            leg_dy = nty - ty
            leg_len = math.sqrt(leg_dx * leg_dx + leg_dy * leg_dy)
            if leg_len > 0.01:
                # Approach point: back up along the first leg direction
                approach_x = tx - (leg_dx / leg_len) * lead_in_dist
                approach_y = ty - (leg_dy / leg_len) * lead_in_dist
            else:
                approach_x, approach_y = tx, ty
        else:
            approach_x, approach_y = tx, ty

        # Build two-segment path: robot → approach point → target
        segments = []
        if next_target is not None and (
            abs(approach_x - tx) > 0.1 or abs(approach_y - ty) > 0.1
        ):
            segments.append((rx, ry, approach_x, approach_y))
            segments.append((approach_x, approach_y, tx, ty))
            self.get_logger().info(
                f"Transit lead-in: approach at ({approach_x:.1f}, "
                f"{approach_y:.1f}) → target ({tx:.1f}, {ty:.1f})"
            )
        else:
            segments.append((rx, ry, tx, ty))

        poses: list[PoseStamped] = []
        for sx, sy, ex, ey in segments:
            dx, dy = ex - sx, ey - sy
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < spacing:
                continue
            n_pts = max(2, int(math.ceil(dist / spacing)) + 1)
            yaw = math.atan2(dy, dx)
            oz = math.sin(yaw / 2.0)
            ow = math.cos(yaw / 2.0)
            # Skip first point of second segment (same as last of first)
            start_i = 1 if poses else 0
            for i in range(start_i, n_pts):
                frac = i / (n_pts - 1)
                p = PoseStamped()
                p.header.frame_id = "map"
                p.header.stamp = Time(seconds=0).to_msg()
                p.pose.position.x = sx + frac * dx
                p.pose.position.y = sy + frac * dy
                p.pose.position.z = 0.0
                p.pose.orientation.z = oz
                p.pose.orientation.w = ow
                poses.append(p)

        if not poses:
            return []
        return poses

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

    async def _follow_path(self, poses: list[PoseStamped]) -> bool:
        """Send a pre-computed path directly to the controller via FollowPath.

        This bypasses the global planner entirely — ideal for open-field
        mowing where the toolpath IS the plan and there are no obstacles.
        """
        if not self._follow_path_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 follow_path not available")
            return False

        # Build a nav_msgs/Path from the poses
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = Time(seconds=0).to_msg()  # latest TF available
        path_msg.poses = poses

        goal = FollowPath.Goal()
        goal.path = path_msg
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "general_goal_checker"

        send_goal_future = self._follow_path_client.send_goal_async(goal)
        goal_handle = await send_goal_future

        if not goal_handle.accepted:
            self.get_logger().error("FollowPath goal rejected")
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
