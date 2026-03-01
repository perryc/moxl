#!/usr/bin/env python3
"""ROS2 node that generates and publishes airstrip mowing toolpaths.

Loads runway data from ppa_airports JSON, generates parallel mowing strips,
and publishes them as nav_msgs/Path messages for Nav2 to follow.

Published topics:
    /moxl/toolpath/all (nav_msgs/Path): Complete toolpath with all strips
    /moxl/toolpath/current_strip (nav_msgs/Path): Currently active strip
    /moxl/toolpath/markers (visualization_msgs/MarkerArray): RViz visualization

Services:
    /moxl/toolpath/generate (std_srvs/Trigger): Regenerate the toolpath
    /moxl/toolpath/next_strip (std_srvs/Trigger): Advance to the next strip

Parameters:
    airstrip_file (str): Path to the airport JSON file
    runway (str): Runway designation (e.g., "11/29")
    cutting_width (float): Mower cutting width in meters
    overlap (float): Overlap between strips in meters
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, ColorRGBA
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Time

from toolpath_planner.polygon_loader import load_airport, get_runway, get_corner_points
from toolpath_planner.strip_generator import generate_strips, compute_strip_stats


class ToolpathNode(Node):
    def __init__(self):
        super().__init__("toolpath_node")

        # Parameters
        self.declare_parameter("airstrip_file", "")
        self.declare_parameter("runway", "11/29")
        self.declare_parameter("cutting_width", 1.52)
        self.declare_parameter("overlap", 0.15)
        self.declare_parameter("heading_override_deg", -1.0)  # -1 = auto-detect

        # State
        self.strips: list[list[tuple[float, float]]] = []
        self.current_strip_index = 0

        # Publishers
        self.all_path_pub = self.create_publisher(Path, "/moxl/toolpath/all", 10)
        self.current_strip_pub = self.create_publisher(
            Path, "/moxl/toolpath/current_strip", 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "/moxl/toolpath/markers", 10
        )

        # Services
        self.create_service(
            Trigger, "/moxl/toolpath/generate", self.generate_callback
        )
        self.create_service(
            Trigger, "/moxl/toolpath/next_strip", self.next_strip_callback
        )

        # Generate on startup if airstrip_file is provided
        airstrip_file = self.get_parameter("airstrip_file").value
        if airstrip_file:
            self.generate_toolpath()
        else:
            self.get_logger().warn(
                "No airstrip_file parameter set. "
                "Call /moxl/toolpath/generate after setting parameters."
            )

        # Periodic republish (for late subscribers)
        self.create_timer(5.0, self.republish)

    def generate_toolpath(self):
        """Generate mowing strips from the configured airstrip data."""
        airstrip_file = self.get_parameter("airstrip_file").value
        runway_designation = self.get_parameter("runway").value
        cutting_width = self.get_parameter("cutting_width").value
        overlap = self.get_parameter("overlap").value
        heading_override = self.get_parameter("heading_override_deg").value

        if not airstrip_file:
            self.get_logger().error("airstrip_file parameter is empty")
            return False

        try:
            airport = load_airport(airstrip_file)
            runway = get_runway(airport, runway_designation)
            corners = get_corner_points(runway)
        except (FileNotFoundError, ValueError) as e:
            self.get_logger().error(f"Failed to load airstrip data: {e}")
            return False

        heading = heading_override if heading_override >= 0 else None

        self.strips = generate_strips(corners, cutting_width, overlap, heading)
        self.current_strip_index = 0

        stats = compute_strip_stats(corners, cutting_width, overlap)
        self.get_logger().info(
            f"Generated {len(self.strips)} strips for {airport['icao_code']} "
            f"runway {runway_designation}: "
            f"{stats['runway_length_m']:.0f}m x {stats['runway_width_m']:.0f}m, "
            f"heading {stats['heading_deg']:.1f}°, "
            f"total mowing distance {stats['total_mowing_distance_m']:.0f}m"
        )

        self.publish_all()
        return True

    def generate_callback(self, request, response):
        """Service callback to regenerate the toolpath."""
        success = self.generate_toolpath()
        response.success = success
        response.message = (
            f"Generated {len(self.strips)} strips"
            if success
            else "Failed to generate toolpath"
        )
        return response

    def next_strip_callback(self, request, response):
        """Service callback to advance to the next strip."""
        if not self.strips:
            response.success = False
            response.message = "No toolpath generated"
            return response

        self.current_strip_index += 1
        if self.current_strip_index >= len(self.strips):
            response.success = True
            response.message = "All strips complete"
            self.current_strip_index = len(self.strips)  # clamp
        else:
            response.success = True
            response.message = (
                f"Strip {self.current_strip_index + 1}/{len(self.strips)}"
            )
            self.publish_current_strip()

        return response

    def republish(self):
        """Periodic republish for late-joining subscribers."""
        if self.strips:
            self.publish_all()

    def publish_all(self):
        """Publish the complete toolpath and visualization markers."""
        self.publish_full_path()
        self.publish_current_strip()
        self.publish_markers()

    def publish_full_path(self):
        """Publish all strips as a single Path (for visualization)."""
        path = Path()
        path.header = Header(frame_id="map", stamp=self.get_clock().now().to_msg())

        for strip in self.strips:
            for lat, lon in strip:
                pose = PoseStamped()
                pose.header = path.header
                # Store lat/lon in x/y — the mission controller will convert
                # to the local map frame using navsat_transform
                pose.pose.position.x = lat
                pose.pose.position.y = lon
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)

        self.all_path_pub.publish(path)

    def publish_current_strip(self):
        """Publish the currently active strip as a Path."""
        if self.current_strip_index >= len(self.strips):
            return

        strip = self.strips[self.current_strip_index]
        path = Path()
        path.header = Header(frame_id="map", stamp=self.get_clock().now().to_msg())

        for lat, lon in strip:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = lat
            pose.pose.position.y = lon
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.current_strip_pub.publish(path)

    def publish_markers(self):
        """Publish visualization markers for RViz/Foxglove."""
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, strip in enumerate(self.strips):
            marker = Marker()
            marker.header = Header(frame_id="map", stamp=now)
            marker.ns = "mowing_strips"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.3  # line width in meters

            # Color: green for completed, yellow for current, white for pending
            if i < self.current_strip_index:
                marker.color = ColorRGBA(r=0.0, g=0.8, b=0.0, a=0.6)
            elif i == self.current_strip_index:
                marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
            else:
                marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.4)

            for lat, lon in strip:
                from geometry_msgs.msg import Point
                marker.points.append(Point(x=lat, y=lon, z=0.0))

            markers.markers.append(marker)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ToolpathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
