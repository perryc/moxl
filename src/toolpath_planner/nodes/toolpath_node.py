#!/usr/bin/env python3
"""ROS2 node that generates and publishes airstrip mowing toolpaths.

Loads runway data from ppa_airports JSON, generates a CCW inward-spiral
toolpath, and publishes it as nav_msgs/Path messages for Nav2 to follow.

Published topics:
    /moxl/toolpath/all (nav_msgs/Path): Complete toolpath with all rings
    /moxl/toolpath/current_strip (nav_msgs/Path): Currently active ring
    /moxl/toolpath/markers (visualization_msgs/MarkerArray): RViz visualization

Services:
    /moxl/toolpath/generate (std_srvs/Trigger): Regenerate the toolpath
    /moxl/toolpath/next_strip (std_srvs/Trigger): Advance to the next ring

Parameters:
    airstrip_file (str): Path to the airport JSON file
    runway (str): Runway designation (e.g., "11/29")
    cutting_width (float): Mower cutting width in meters
    overlap (float): Overlap between adjacent passes in meters
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

from toolpath_planner.polygon_loader import (
    load_airport, get_runway, get_corner_points, get_clear_zones,
)
from toolpath_planner.strip_generator import generate_strips, compute_strip_stats


class ToolpathNode(Node):
    def __init__(self):
        super().__init__("toolpath_node")

        # Parameters
        self.declare_parameter("airstrip_file", "")
        self.declare_parameter("runway", "11/29")
        self.declare_parameter("cutting_width", 1.52)
        self.declare_parameter("overlap", 0.15)
        self.declare_parameter("start_lat", float("nan"))
        self.declare_parameter("start_lon", float("nan"))

        # State
        self.strips: list[list[tuple[float, float]]] = []
        self.current_strip_index = 0
        self.clear_zones: list[dict] = []

        # Publishers
        self.all_path_pub = self.create_publisher(Path, "/moxl/toolpath/all", 10)
        self.current_strip_pub = self.create_publisher(
            Path, "/moxl/toolpath/current_strip", 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "/moxl/toolpath/markers", 10
        )
        self.clear_zone_marker_pub = self.create_publisher(
            MarkerArray, "/moxl/clear_zone/markers", 10
        )

        # Services
        self.create_service(
            Trigger, "/moxl/toolpath/generate", self.generate_callback
        )
        self.create_service(
            Trigger, "/moxl/toolpath/next_strip", self.next_strip_callback
        )
        self.create_service(
            Trigger, "/moxl/toolpath/clear_zones", self.clear_zones_callback
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

        import math
        start_lat = self.get_parameter("start_lat").value
        start_lon = self.get_parameter("start_lon").value
        start_latlon = None
        if not (math.isnan(start_lat) or math.isnan(start_lon)):
            start_latlon = (start_lat, start_lon)

        self.strips = generate_strips(
            corners, cutting_width, overlap, start_latlon=start_latlon,
        )
        self.current_strip_index = 0

        # Load clear zones for radio traffic evacuation
        self.clear_zones = get_clear_zones(runway)
        if self.clear_zones:
            self.get_logger().info(
                f"Loaded {len(self.clear_zones)} clear zones: "
                f"{[z['name'] for z in self.clear_zones]}"
            )
        else:
            self.get_logger().warn("No clear zones defined for this runway")

        stats = compute_strip_stats(corners, cutting_width, overlap)
        self.get_logger().info(
            f"Generated {stats['ring_count']} rings for {airport['icao_code']} "
            f"runway {runway_designation}: "
            f"area {stats['polygon_area_m2']:.0f} m², "
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

    def clear_zones_callback(self, request, response):
        """Service callback returning clear zone data as JSON."""
        import json
        if self.clear_zones:
            response.success = True
            response.message = json.dumps([
                {"name": z["name"], "centroid": list(z["centroid"])}
                for z in self.clear_zones
            ])
        else:
            response.success = False
            response.message = "No clear zones loaded"
        return response

    def nearest_clear_zone(self, lat: float, lon: float) -> dict | None:
        """Find the nearest clear zone centroid to a given lat/lon.

        Uses simple Euclidean distance on lat/lon (sufficient for ~1 km scale).

        Returns:
            Clear zone dict with 'name', 'vertices', 'centroid', or None.
        """
        if not self.clear_zones:
            return None

        best = None
        best_dist = float("inf")
        for zone in self.clear_zones:
            clat, clon = zone["centroid"]
            dist = math.sqrt((lat - clat) ** 2 + (lon - clon) ** 2)
            if dist < best_dist:
                best_dist = dist
                best = zone
        return best

    def publish_clear_zone_markers(self):
        """Publish clear zone polygons as visualization markers."""
        if not self.clear_zones:
            return

        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, zone in enumerate(self.clear_zones):
            # Polygon outline
            marker = Marker()
            marker.header = Header(frame_id="map", stamp=now)
            marker.ns = "clear_zones"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.5
            marker.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.8)

            from geometry_msgs.msg import Point
            for lat, lon in zone["vertices"]:
                marker.points.append(Point(x=lat, y=lon, z=0.0))
            # Close the polygon
            first = zone["vertices"][0]
            marker.points.append(Point(x=first[0], y=first[1], z=0.0))
            markers.markers.append(marker)

            # Label at centroid
            label = Marker()
            label.header = Header(frame_id="map", stamp=now)
            label.ns = "clear_zone_labels"
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.scale.z = 3.0
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            clat, clon = zone["centroid"]
            label.pose.position = Point(x=clat, y=clon, z=5.0)
            label.pose.orientation.w = 1.0
            label.text = zone["name"]
            markers.markers.append(label)

        self.clear_zone_marker_pub.publish(markers)

    def republish(self):
        """Periodic republish for late-joining subscribers."""
        if self.strips:
            self.publish_all()

    def publish_all(self):
        """Publish the complete toolpath and visualization markers."""
        self.publish_full_path()
        self.publish_current_strip()
        self.publish_markers()
        self.publish_clear_zone_markers()

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
