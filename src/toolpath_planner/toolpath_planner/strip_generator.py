"""Generate mowing toolpath as a CCW inward spiral.

The mower discharges grass to the right, so a counter-clockwise spiral
keeps the discharge side facing inward toward already-cut grass.

Algorithm:
1. Start at the outermost ring (polygon buffered inward by half a cutting width)
2. Progressively buffer inward by the effective step (cutting_width − overlap)
3. Each ring is oriented CCW and rotated so its start vertex is nearest the
   previous ring's end — producing a continuous path with short transitions.

All waypoints are densified (every ~5 m) and returned as a single
continuous path for Nav2 to follow.
"""

import math
import numpy as np
from shapely.geometry import Polygon, LineString

from .coordinate_utils import latlon_to_utm, utm_to_latlon


WAYPOINT_SPACING = 1.5  # meters between waypoints (dense for smooth controller tracking)


def corners_to_utm(corners_latlon: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Convert a list of (lat, lon) corner points to UTM (easting, northing)."""
    return [latlon_to_utm(lat, lon) for lat, lon in corners_latlon]


def _densify(p0, p1, spacing=WAYPOINT_SPACING):
    """Generate evenly spaced points from p0 to p1."""
    p0, p1 = np.array(p0), np.array(p1)
    seg = p1 - p0
    length = np.linalg.norm(seg)
    n_pts = max(2, int(math.ceil(length / spacing)) + 1)
    return [(float(p0[0] + t / (n_pts - 1) * seg[0]),
             float(p0[1] + t / (n_pts - 1) * seg[1]))
            for t in range(n_pts)]


def _ensure_ccw(coords: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Ensure a closed ring is ordered counter-clockwise (shoelace test).

    If the signed area is negative (CW), the ring is reversed.
    The returned ring is always closed (first == last).
    """
    # Drop closing vertex for the shoelace sum
    ring = coords if coords[0] != coords[-1] else coords[:-1]
    signed_area = sum(
        x0 * y1 - x1 * y0
        for (x0, y0), (x1, y1) in zip(ring, ring[1:] + ring[:1])
    )
    if signed_area < 0:
        ring = list(reversed(ring))
    # Re-close the ring
    return ring + [ring[0]]


def _rotate_ring_to_nearest(
    coords: list[tuple[float, float]],
    target: tuple[float, float],
) -> list[tuple[float, float]]:
    """Rotate a closed ring so the point nearest *target* comes first.

    The ring must be closed (first == last). The returned ring is also closed.
    """
    # Work with the open ring (drop closing duplicate)
    ring = coords[:-1]
    tx, ty = target
    best_idx = 0
    best_dist = float("inf")
    for i, (x, y) in enumerate(ring):
        d = (x - tx) ** 2 + (y - ty) ** 2
        if d < best_dist:
            best_dist = d
            best_idx = i
    rotated = ring[best_idx:] + ring[:best_idx]
    return rotated + [rotated[0]]


def _midpoint_of_longest_edge(polygon: Polygon) -> tuple[float, float]:
    """Return the midpoint of the longest exterior edge of a polygon.

    For a long thin runway polygon, this is the midpoint of one of the
    two long sides — the natural place for a spiral step-in transition.
    """
    coords = list(polygon.exterior.coords)
    best_len = 0.0
    best_mid = (coords[0][0], coords[0][1])
    for i in range(len(coords) - 1):
        p0, p1 = coords[i], coords[i + 1]
        length = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        if length > best_len:
            best_len = length
            best_mid = ((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2)
    return best_mid


def _centerline_pass(
    polygon: Polygon,
    prev_end: tuple[float, float] | None,
) -> list[tuple[float, float]]:
    """Extract an open centerline pass through a thin polygon.

    For rings too narrow for the mower to complete as a closed loop,
    this computes the longest axis of the polygon and returns a densified
    straight-line pass down the middle — no turnaround at the ends.
    The pass direction is chosen so the start is nearest to prev_end.
    """
    # Use the longest pair of edges on the exterior as the axis.
    # For a long thin polygon, the two longest edges are the sides.
    coords = list(polygon.exterior.coords)
    segments = []
    for i in range(len(coords) - 1):
        p0, p1 = coords[i], coords[i + 1]
        length = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        mid = ((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2)
        segments.append((length, i, mid, p0, p1))
    segments.sort(key=lambda s: s[0], reverse=True)

    if len(segments) < 2:
        return []

    # The two longest edges are the "sides" of the thin strip.
    # Average their endpoints to get the centerline.
    _, _, _, a0, a1 = segments[0]
    _, _, _, b0, b1 = segments[1]

    # Centerline endpoints: midpoints of the short ends.
    # For a thin polygon, the start of the centerline is the midpoint
    # between the start of the two long edges, and similarly for the end.
    # Simpler: use the centroid of each short end of the polygon's
    # minimum rotated rectangle.
    rect = polygon.minimum_rotated_rectangle
    rc = list(rect.exterior.coords)
    # Rectangle has 5 coords (closed), find the two short edges
    edges = []
    for i in range(4):
        p0, p1 = rc[i], rc[i + 1]
        length = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        mid = ((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2)
        edges.append((length, mid))
    edges.sort(key=lambda e: e[0])
    # Two shortest edges are the ends; their midpoints define the centerline
    end1 = edges[0][1]
    end2 = edges[1][1]

    # Orient so start is nearest to prev_end
    if prev_end is not None:
        d1 = (end1[0] - prev_end[0]) ** 2 + (end1[1] - prev_end[1]) ** 2
        d2 = (end2[0] - prev_end[0]) ** 2 + (end2[1] - prev_end[1]) ** 2
        if d2 < d1:
            end1, end2 = end2, end1

    return _densify(end1, end2)


def generate_strips(
    corners_latlon: list[tuple[float, float]],
    cutting_width: float,
    overlap: float = 0.0,
    start_latlon: tuple[float, float] | None = None,
) -> list[list[tuple[float, float]]]:
    """Generate a CCW inward-spiral mowing toolpath.

    Args:
        corners_latlon: List of (lat, lon) tuples defining the mowing polygon.
        cutting_width: Mower cutting width in meters.
        overlap: Overlap between adjacent passes in meters.
        start_latlon: Optional (lat, lon) for preferred start position.
            The outermost ring is rotated so the nearest vertex to this
            point comes first.

    Returns:
        List of rings, where each ring is a list of (lat, lon) waypoints.
        Rings proceed from outermost to innermost, all oriented CCW.
        Waypoints are densified with points every ~5 m.
    """
    corners_utm = corners_to_utm(corners_latlon)
    polygon = Polygon(corners_utm)
    if not polygon.is_valid:
        polygon = polygon.convex_hull

    effective_step = cutting_width - overlap
    if effective_step <= 0:
        raise ValueError(
            f"cutting_width ({cutting_width}) must be greater than overlap ({overlap})"
        )

    # Convert start position to UTM if provided
    start_utm = None
    if start_latlon is not None:
        start_utm = latlon_to_utm(*start_latlon)

    all_rings: list[list[tuple[float, float]]] = []
    prev_end: tuple[float, float] | None = start_utm

    ring_index = 0
    while True:
        inset = cutting_width / 2 + ring_index * effective_step
        shrunk = polygon.buffer(-inset)

        if shrunk.is_empty or shrunk.area < 0.1:
            break

        # If buffer returned a MultiPolygon, take the largest piece
        if shrunk.geom_type == "MultiPolygon":
            shrunk = max(shrunk.geoms, key=lambda g: g.area)

        if shrunk.geom_type != "Polygon":
            break

        # If the ring is too narrow to complete as a closed loop (corners
        # would have near-zero turning radius), replace it with an open
        # centerline pass — drive straight down the middle of the remaining
        # strip without trying to turn around at the narrow ends.
        if shrunk.buffer(-effective_step / 2).is_empty:
            centerline = _centerline_pass(shrunk, prev_end)
            if centerline:
                all_rings.append(centerline)
            break

        coords = list(shrunk.exterior.coords)
        coords = _ensure_ccw(coords)

        # Densify the ring BEFORE rotating so nearest-point search
        # works along edges, not just at polygon vertices.
        dense_ring: list[tuple[float, float]] = []
        for i in range(len(coords) - 1):
            pts_seg = _densify(coords[i], coords[i + 1])
            if dense_ring:
                dense_ring.extend(pts_seg[1:])
            else:
                dense_ring.extend(pts_seg)
        dense_closed = dense_ring + [dense_ring[0]]

        # Choose rotation target:
        #   Ring 0 — nearest point on ring to start position (park approach)
        #   Ring 1+ — midpoint of longest edge (step-in at midline)
        if ring_index == 0 and start_utm is not None:
            dense_closed = _rotate_ring_to_nearest(dense_closed, start_utm)
        elif ring_index > 0:
            midline = _midpoint_of_longest_edge(shrunk)
            dense_closed = _rotate_ring_to_nearest(dense_closed, midline)
        elif prev_end is not None:
            dense_closed = _rotate_ring_to_nearest(dense_closed, prev_end)

        # Strip the closing vertex for storage
        dense_ring = dense_closed[:-1]

        # Add densified transit from previous ring end to this ring start
        # so the Nav2 controller always has nearby waypoints to follow.
        if prev_end is not None and dense_ring:
            transit_dist = math.hypot(
                dense_ring[0][0] - prev_end[0],
                dense_ring[0][1] - prev_end[1],
            )
            if transit_dist > WAYPOINT_SPACING * 2:
                transit = _densify(prev_end, dense_ring[0])
                # Drop first (== prev_end) and last (== ring start) to avoid dupes
                all_rings.append(transit[1:-1])

        prev_end = dense_ring[-1]
        all_rings.append(dense_ring)
        ring_index += 1

    # Convert back to WGS84
    rings_latlon = []
    for ring in all_rings:
        ring_wgs = [utm_to_latlon(e, n) for e, n in ring]
        rings_latlon.append(ring_wgs)

    return rings_latlon


def compute_strip_stats(
    corners_latlon: list[tuple[float, float]],
    cutting_width: float,
    overlap: float = 0.0,
) -> dict:
    """Compute statistics about the spiral mowing plan."""
    corners_utm = corners_to_utm(corners_latlon)
    polygon = Polygon(corners_utm)
    if not polygon.is_valid:
        polygon = polygon.convex_hull

    effective_step = cutting_width - overlap

    ring_count = 0
    total_perimeter = 0.0

    ring_index = 0
    while True:
        inset = cutting_width / 2 + ring_index * effective_step
        shrunk = polygon.buffer(-inset)

        if shrunk.is_empty or shrunk.area < 0.1:
            break

        if shrunk.geom_type == "MultiPolygon":
            shrunk = max(shrunk.geoms, key=lambda g: g.area)

        if shrunk.geom_type != "Polygon":
            break

        # Same minimum-width check as generate_strips — thin ring
        # becomes a centerline pass instead of a full ring
        if shrunk.buffer(-effective_step / 2).is_empty:
            rect = shrunk.minimum_rotated_rectangle
            rc = list(rect.exterior.coords)
            edges = []
            for i in range(4):
                p0, p1 = rc[i], rc[i + 1]
                length = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
                edges.append(length)
            edges.sort()
            # Centerline length ≈ longest edge of bounding rectangle
            ring_count += 1
            total_perimeter += edges[-1]
            break

        ring_count += 1
        total_perimeter += shrunk.exterior.length
        ring_index += 1

    return {
        "ring_count": ring_count,
        "effective_step_m": effective_step,
        "polygon_area_m2": polygon.area,
        "total_mowing_distance_m": total_perimeter,
    }
