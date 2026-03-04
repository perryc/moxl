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
from shapely.geometry import Polygon

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
    """Rotate a closed ring so the vertex nearest *target* comes first.

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

        coords = list(shrunk.exterior.coords)
        coords = _ensure_ccw(coords)

        # Rotate so nearest vertex to previous ring end comes first
        if prev_end is not None:
            coords = _rotate_ring_to_nearest(coords, prev_end)

        # Densify the ring
        dense_ring: list[tuple[float, float]] = []
        for i in range(len(coords) - 1):
            pts_seg = _densify(coords[i], coords[i + 1])
            if dense_ring:
                dense_ring.extend(pts_seg[1:])
            else:
                dense_ring.extend(pts_seg)

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

        ring_count += 1
        total_perimeter += shrunk.exterior.length
        ring_index += 1

    return {
        "ring_count": ring_count,
        "effective_step_m": effective_step,
        "polygon_area_m2": polygon.area,
        "total_mowing_distance_m": total_perimeter,
    }
