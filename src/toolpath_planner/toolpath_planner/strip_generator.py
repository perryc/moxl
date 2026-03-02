"""Generate mowing toolpath with headland-first pattern.

The pattern follows standard agricultural practice:
1. Perimeter pass — mow around the full rectangle edge
2. Headland passes — two cross-cuts at each short end (turning room)
3. Main strips — parallel end-to-end cuts with U-turns inside headlands

All waypoints are densified (every ~5m) and returned as a single
continuous path for Nav2 to follow.
"""

import math
import numpy as np
from shapely.geometry import Polygon, LineString, MultiLineString

from .coordinate_utils import latlon_to_utm, utm_to_latlon


WAYPOINT_SPACING = 5.0  # meters between waypoints


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

    mid_12 = (pts[0] + pts[1]) / 2
    mid_34 = (pts[2] + pts[3]) / 2
    mid_14 = (pts[0] + pts[3]) / 2
    mid_23 = (pts[1] + pts[2]) / 2

    axis_a = mid_34 - mid_12
    axis_b = mid_23 - mid_14

    if np.linalg.norm(axis_a) >= np.linalg.norm(axis_b):
        heading_vec = axis_a
    else:
        heading_vec = axis_b

    heading_rad = math.atan2(heading_vec[0], heading_vec[1])
    if heading_rad < 0:
        heading_rad += 2 * math.pi

    return heading_rad


def _densify(p0, p1, spacing=WAYPOINT_SPACING):
    """Generate evenly spaced points from p0 to p1."""
    p0, p1 = np.array(p0), np.array(p1)
    seg = p1 - p0
    length = np.linalg.norm(seg)
    n_pts = max(2, int(math.ceil(length / spacing)) + 1)
    return [(float(p0[0] + t / (n_pts - 1) * seg[0]),
             float(p0[1] + t / (n_pts - 1) * seg[1]))
            for t in range(n_pts)]


def _clip_line_to_polygon(p0, p1, polygon):
    """Clip a line segment to a polygon, return list of (e, n) tuples or empty."""
    line = LineString([p0, p1])
    clipped = polygon.intersection(line)
    if clipped.is_empty:
        return []
    if isinstance(clipped, LineString):
        return list(clipped.coords)
    if isinstance(clipped, MultiLineString):
        # Return longest segment
        best = max(clipped.geoms, key=lambda g: g.length)
        return list(best.coords)
    return []


def generate_strips(
    corners_latlon: list[tuple[float, float]],
    cutting_width: float,
    overlap: float = 0.0,
    heading_override_deg: float | None = None,
) -> list[list[tuple[float, float]]]:
    """Generate mowing toolpath with headland-first pattern.

    Pattern:
        1. Perimeter pass (around the full rectangle)
        2. Two headland passes at each short end
        3. Main parallel strips with boustrophedon ordering

    Args:
        corners_latlon: List of 4 (lat, lon) tuples defining the runway polygon.
        cutting_width: Mower cutting width in meters.
        overlap: Overlap between adjacent strips in meters.
        heading_override_deg: Optional heading override in degrees true from north.

    Returns:
        List of strips, where each strip is a list of (lat, lon) waypoints.
        The first strip is the perimeter, followed by headlands, then main strips.
        Strips are densified with waypoints every ~5m.
    """
    corners_utm = corners_to_utm(corners_latlon)
    polygon = Polygon(corners_utm)
    if not polygon.is_valid:
        polygon = polygon.convex_hull

    if heading_override_deg is not None:
        heading_rad = math.radians(heading_override_deg)
    else:
        heading_rad = compute_heading_from_polygon(corners_utm)

    heading_vec = np.array([math.sin(heading_rad), math.cos(heading_rad)])
    cross_vec = np.array([math.cos(heading_rad), -math.sin(heading_rad)])

    pts = np.array(corners_utm)
    cross_proj = pts @ cross_vec
    heading_proj = pts @ heading_vec
    cross_min, cross_max = cross_proj.min(), cross_proj.max()
    heading_min, heading_max = heading_proj.min(), heading_proj.max()

    effective_step = cutting_width - overlap
    if effective_step <= 0:
        raise ValueError(
            f"cutting_width ({cutting_width}) must be greater than overlap ({overlap})"
        )

    # Headland depth: 2 passes at each end
    headland_depth = 2 * effective_step

    all_strips = []

    # ── 1. PERIMETER PASS ────────────────────────────────────────
    # Inset the polygon by half a cutting width so the mower edge
    # aligns with the boundary, then trace the perimeter
    inset = polygon.buffer(-cutting_width / 2)
    if not inset.is_empty and inset.geom_type == 'Polygon':
        perim_coords = list(inset.exterior.coords)
        # Densify the perimeter
        dense_perim = []
        for i in range(len(perim_coords) - 1):
            pts_seg = _densify(perim_coords[i], perim_coords[i + 1])
            if dense_perim:
                dense_perim.extend(pts_seg[1:])  # skip duplicate junction point
            else:
                dense_perim.extend(pts_seg)
        all_strips.append(dense_perim)

    # ── 2. HEADLAND PASSES ───────────────────────────────────────
    # Two cross-cuts at each short end, just inside the perimeter
    line_margin = 50.0

    for end_sign, end_label in [(1, "far"), (-1, "near")]:
        # "far" end = heading_max, "near" end = heading_min
        if end_sign == 1:
            h_base = heading_max
        else:
            h_base = heading_min

        for pass_num in range(2):
            # Offset inward from the end
            h_offset = h_base - end_sign * (cutting_width / 2 + pass_num * effective_step)
            # Cross-track line at this heading position
            p0 = heading_vec * h_offset + cross_vec * (cross_min - line_margin)
            p1 = heading_vec * h_offset + cross_vec * (cross_max + line_margin)
            coords = _clip_line_to_polygon(p0, p1, polygon)
            if len(coords) >= 2:
                dense = _densify(coords[0], coords[-1])
                all_strips.append(dense)

    # Reverse every other headland strip for continuous path
    for i in range(1, len(all_strips)):
        if i % 2 == 0:
            all_strips[i] = list(reversed(all_strips[i]))

    # ── 3. MAIN PARALLEL STRIPS ──────────────────────────────────
    # Run between the headlands, stepping across the width
    main_heading_min = heading_min + headland_depth
    main_heading_max = heading_max - headland_depth

    start_offset = cross_min + cutting_width / 2
    end_offset = cross_max - cutting_width / 2

    main_strips = []
    offset = start_offset
    while offset <= end_offset + 0.001:
        center = cross_vec * offset
        p0 = center + heading_vec * (main_heading_min - line_margin)
        p1 = center + heading_vec * (main_heading_max + line_margin)

        # Clip to the polygon (strips still bounded by runway edges)
        line = LineString([p0, p1])
        clipped = polygon.intersection(line)

        coords = []
        if isinstance(clipped, LineString) and not clipped.is_empty:
            coords = list(clipped.coords)
        elif isinstance(clipped, MultiLineString):
            best = max(clipped.geoms, key=lambda g: g.length)
            coords = list(best.coords)

        if len(coords) >= 2:
            # Shorten to headland boundaries
            c0, c1 = np.array(coords[0]), np.array(coords[-1])
            h0 = float(c0 @ heading_vec)
            h1 = float(c1 @ heading_vec)
            # Clamp to headland-inset range
            if h0 < main_heading_min:
                t = (main_heading_min - h0) / (h1 - h0) if h1 != h0 else 0
                c0 = c0 + t * (c1 - c0)
            if h1 > main_heading_max:
                t = (main_heading_max - h0) / (h1 - h0) if h1 != h0 else 1
                c1 = c0 + t * (np.array(coords[-1]) - np.array(coords[0]))

            dense = _densify(tuple(c0), tuple(c1))
            main_strips.append(dense)

        offset += effective_step

    # Boustrophedon ordering for main strips
    for i in range(len(main_strips)):
        if i % 2 == 1:
            main_strips[i] = list(reversed(main_strips[i]))

    all_strips.extend(main_strips)

    # Convert back to WGS84
    strips_latlon = []
    for strip in all_strips:
        strip_wgs = [utm_to_latlon(e, n) for e, n in strip]
        strips_latlon.append(strip_wgs)

    return strips_latlon


def compute_strip_stats(
    corners_latlon: list[tuple[float, float]],
    cutting_width: float,
    overlap: float = 0.0,
) -> dict:
    """Compute statistics about the mowing plan without generating full strips."""
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
