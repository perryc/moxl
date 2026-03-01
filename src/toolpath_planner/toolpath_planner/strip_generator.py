"""Generate parallel mowing strips within a runway polygon.

The algorithm:
1. Convert corner points from WGS84 to UTM (flat Cartesian plane)
2. Compute the runway heading from the polygon geometry
3. Generate parallel strip centerlines perpendicular to the cross-track direction
4. Clip each strip to the polygon boundary
5. Order strips in boustrophedon (alternating) pattern
"""

import math
import numpy as np
from shapely.geometry import Polygon, LineString, MultiLineString

from .coordinate_utils import latlon_to_utm, utm_to_latlon


def corners_to_utm(corners_latlon: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Convert a list of (lat, lon) corner points to UTM (easting, northing)."""
    return [latlon_to_utm(lat, lon) for lat, lon in corners_latlon]


def compute_heading_from_polygon(corners_utm: list[tuple[float, float]]) -> float:
    """Compute the runway heading from the polygon shape.

    Assumes the polygon is roughly rectangular with 4 corners.
    The heading is along the longer axis of the rectangle.

    Returns:
        Heading in radians, measured from UTM north (Y-axis), clockwise.
        This matches the aviation convention (0=North, 90=East).
    """
    pts = np.array(corners_utm)

    # Try both diagonals to find the two pairs of "long sides"
    # Corners are ordered: C1, C2, C3, C4
    # Possible long-side pairings: (C1-C4, C2-C3) or (C1-C2, C3-C4)
    mid_12 = (pts[0] + pts[1]) / 2
    mid_34 = (pts[2] + pts[3]) / 2
    mid_14 = (pts[0] + pts[3]) / 2
    mid_23 = (pts[1] + pts[2]) / 2

    axis_a = mid_34 - mid_12  # C1C2 midpoint to C3C4 midpoint
    axis_b = mid_23 - mid_14  # C1C4 midpoint to C2C3 midpoint

    # The longer axis is the runway direction
    if np.linalg.norm(axis_a) >= np.linalg.norm(axis_b):
        heading_vec = axis_a
    else:
        heading_vec = axis_b

    # Convert to heading angle (from north, clockwise)
    # In UTM: X=easting (east), Y=northing (north)
    # atan2(east, north) gives bearing from north
    heading_rad = math.atan2(heading_vec[0], heading_vec[1])

    # Normalize to [0, 2*pi)
    if heading_rad < 0:
        heading_rad += 2 * math.pi

    return heading_rad


def generate_strips(
    corners_latlon: list[tuple[float, float]],
    cutting_width: float,
    overlap: float = 0.0,
    heading_override_deg: float | None = None,
) -> list[list[tuple[float, float]]]:
    """Generate parallel mowing strips within a runway polygon.

    Args:
        corners_latlon: List of 4 (lat, lon) tuples defining the runway polygon.
        cutting_width: Mower cutting width in meters.
        overlap: Overlap between adjacent strips in meters.
        heading_override_deg: If set, use this heading (degrees true from north)
            instead of computing from the polygon. Useful if the polygon shape
            doesn't clearly indicate the mowing direction.

    Returns:
        List of strips, where each strip is a list of (lat, lon) waypoints.
        Strips alternate direction (boustrophedon pattern).
    """
    # Convert to UTM
    corners_utm = corners_to_utm(corners_latlon)
    polygon = Polygon(corners_utm)

    if not polygon.is_valid:
        # Try to fix by reordering to convex hull
        polygon = polygon.convex_hull

    # Compute heading
    if heading_override_deg is not None:
        heading_rad = math.radians(heading_override_deg)
    else:
        heading_rad = compute_heading_from_polygon(corners_utm)

    # The mowing direction is along the heading.
    # Strips are generated perpendicular (cross-track).
    # heading_vec: unit vector along the runway (direction of travel)
    heading_vec = np.array([math.sin(heading_rad), math.cos(heading_rad)])
    # cross_vec: unit vector perpendicular to heading (strip spacing direction)
    cross_vec = np.array([math.cos(heading_rad), -math.sin(heading_rad)])

    # Project all polygon vertices onto the cross-track axis to find width extent
    pts = np.array(corners_utm)
    cross_projections = pts @ cross_vec
    cross_min = cross_projections.min()
    cross_max = cross_projections.max()

    # Project onto heading axis to find length extent (for generating long lines)
    heading_projections = pts @ heading_vec
    heading_min = heading_projections.min()
    heading_max = heading_projections.max()
    # Extend lines well beyond polygon to ensure clean intersection
    line_margin = 50.0  # meters extra beyond polygon edges

    # Generate strip centerlines
    effective_step = cutting_width - overlap
    if effective_step <= 0:
        raise ValueError(
            f"cutting_width ({cutting_width}) must be greater than overlap ({overlap})"
        )

    # Start from cross_min + half a cutting width (so the first strip edge
    # aligns with the polygon edge), step across
    start_offset = cross_min + cutting_width / 2
    end_offset = cross_max - cutting_width / 2

    strips_utm = []
    offset = start_offset
    while offset <= end_offset + 0.001:  # small epsilon for float comparison
        # Line along heading direction at this cross-track offset
        center = cross_vec * offset
        p1 = center + heading_vec * (heading_min - line_margin)
        p2 = center + heading_vec * (heading_max + line_margin)
        line = LineString([p1, p2])

        # Clip to polygon
        clipped = polygon.intersection(line)
        if clipped.is_empty:
            offset += effective_step
            continue

        # Handle MultiLineString (can happen with concave polygons)
        if isinstance(clipped, MultiLineString):
            for segment in clipped.geoms:
                coords = list(segment.coords)
                if len(coords) >= 2:
                    strips_utm.append(coords)
        elif isinstance(clipped, LineString):
            coords = list(clipped.coords)
            if len(coords) >= 2:
                strips_utm.append(coords)

        offset += effective_step

    # Apply boustrophedon ordering (reverse every other strip)
    for i in range(len(strips_utm)):
        if i % 2 == 1:
            strips_utm[i] = list(reversed(strips_utm[i]))

    # Convert back to WGS84
    strips_latlon = []
    for strip in strips_utm:
        strip_wgs = [utm_to_latlon(e, n) for e, n in strip]
        strips_latlon.append(strip_wgs)

    return strips_latlon


def compute_strip_stats(
    corners_latlon: list[tuple[float, float]],
    cutting_width: float,
    overlap: float = 0.0,
) -> dict:
    """Compute statistics about the mowing plan without generating full strips.

    Returns:
        Dict with keys: strip_count, effective_step, polygon_area_m2,
        runway_width_m, runway_length_m, total_mowing_distance_m.
    """
    corners_utm = corners_to_utm(corners_latlon)
    polygon = Polygon(corners_utm)
    if not polygon.is_valid:
        polygon = polygon.convex_hull

    heading_rad = compute_heading_from_polygon(corners_utm)
    heading_vec = np.array([math.sin(heading_rad), math.cos(heading_rad)])
    cross_vec = np.array([math.cos(heading_rad), -math.sin(heading_rad)])

    pts = np.array(corners_utm)
    cross_projections = pts @ cross_vec
    heading_projections = pts @ heading_vec

    runway_width = cross_projections.max() - cross_projections.min()
    runway_length = heading_projections.max() - heading_projections.min()

    effective_step = cutting_width - overlap
    strip_count = max(1, int(math.ceil(
        (runway_width - cutting_width) / effective_step
    )) + 1)

    return {
        "strip_count": strip_count,
        "effective_step_m": effective_step,
        "polygon_area_m2": polygon.area,
        "runway_width_m": runway_width,
        "runway_length_m": runway_length,
        "total_mowing_distance_m": strip_count * runway_length,
        "heading_deg": math.degrees(heading_rad),
    }
