"""Tests for CCW inward-spiral mowing toolpath generation using actual CDS2 runway data."""

import math
import pytest
from shapely.geometry import Polygon, Point

from toolpath_planner.strip_generator import (
    corners_to_utm,
    generate_strips,
    compute_strip_stats,
)
from toolpath_planner.coordinate_utils import latlon_to_utm


# CDS2 Runway 11/29 RTK-surveyed corners (0.02m accuracy)
CDS2_CORNERS = [
    (50.6361219, -105.03202723),   # C1 NW
    (50.63603243, -105.03175091),  # C2 NE
    (50.63285006, -105.03165067),  # C3 SE
    (50.63288086, -105.03194065),  # C4 SW
]

CUTTING_WIDTH = 1.52   # meters (~5ft Swisher deck)
OVERLAP = 0.15         # meters


class TestCornersToUtm:
    def test_returns_correct_count(self):
        utm = corners_to_utm(CDS2_CORNERS)
        assert len(utm) == 4

    def test_returns_tuples(self):
        utm = corners_to_utm(CDS2_CORNERS)
        for pt in utm:
            assert len(pt) == 2
            assert isinstance(pt[0], float)
            assert isinstance(pt[1], float)


class TestGenerateStrips:
    def test_generates_reasonable_strip_count(self):
        """Polygon ~24m wide with 1.37m effective step → ~8-9 rings."""
        strips = generate_strips(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP)
        assert 5 <= len(strips) <= 12, \
            f"Got {len(strips)} rings (expected 5-12 for spiral)"

    def test_strips_not_empty(self):
        strips = generate_strips(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP)
        for i, strip in enumerate(strips):
            assert len(strip) >= 2, f"Ring {i} has fewer than 2 waypoints"

    def test_strips_are_latlon(self):
        """All waypoints should be valid lat/lon coordinates."""
        strips = generate_strips(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP)
        for strip in strips:
            for lat, lon in strip:
                assert 50.0 < lat < 51.0, f"Lat {lat} out of range"
                assert -106.0 < lon < -104.0, f"Lon {lon} out of range"

    def test_strips_within_polygon(self):
        """All strip waypoints should be within (or very near) the runway polygon."""
        strips = generate_strips(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP)
        utm_corners = corners_to_utm(CDS2_CORNERS)
        polygon = Polygon(utm_corners)
        if not polygon.is_valid:
            polygon = polygon.convex_hull

        # Buffer polygon by 1m to account for floating point at edges
        buffered = polygon.buffer(1.0)

        for i, strip in enumerate(strips):
            for lat, lon in strip:
                e, n = latlon_to_utm(lat, lon)
                pt = Point(e, n)
                assert buffered.contains(pt), \
                    f"Ring {i} waypoint ({lat}, {lon}) -> UTM ({e}, {n}) outside polygon"

    def test_spiral_ccw_direction(self):
        """Each ring should be ordered counter-clockwise (positive shoelace area)."""
        strips = generate_strips(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP)
        for i, strip in enumerate(strips):
            # Convert to UTM for area calculation
            utm_pts = [latlon_to_utm(lat, lon) for lat, lon in strip]
            # Shoelace signed area
            signed_area = sum(
                x0 * y1 - x1 * y0
                for (x0, y0), (x1, y1) in zip(utm_pts, utm_pts[1:] + utm_pts[:1])
            )
            assert signed_area >= 0, \
                f"Ring {i} has negative shoelace area ({signed_area:.1f}), expected CCW"

    def test_start_position(self):
        """Providing start_latlon should rotate outermost ring start vertex."""
        # Use the NE corner as start position
        start = CDS2_CORNERS[1]  # NE corner
        strips_default = generate_strips(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP)
        strips_ne = generate_strips(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP, start_latlon=start)

        # The outermost ring should start at a different waypoint
        assert strips_default[0][0] != strips_ne[0][0], \
            "start_latlon should rotate the outermost ring start"

    def test_overlap_zero(self):
        """With zero overlap, should get fewer rings."""
        strips_overlap = generate_strips(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP)
        strips_no_overlap = generate_strips(CDS2_CORNERS, CUTTING_WIDTH, 0.0)
        # More overlap means more rings (smaller effective step)
        assert len(strips_overlap) >= len(strips_no_overlap)

    def test_invalid_overlap_raises(self):
        """Overlap >= cutting_width should raise ValueError."""
        with pytest.raises(ValueError):
            generate_strips(CDS2_CORNERS, CUTTING_WIDTH, CUTTING_WIDTH)

    def test_rings_are_nested(self):
        """Inner ring centroid should lie inside the outer ring polygon."""
        strips = generate_strips(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP)
        if len(strips) < 2:
            pytest.skip("Need at least 2 rings")

        for i in range(len(strips) - 1):
            outer_utm = [latlon_to_utm(lat, lon) for lat, lon in strips[i]]
            inner_utm = [latlon_to_utm(lat, lon) for lat, lon in strips[i + 1]]

            outer_poly = Polygon(outer_utm)
            if not outer_poly.is_valid:
                outer_poly = outer_poly.convex_hull

            inner_centroid = Polygon(inner_utm).centroid
            assert outer_poly.buffer(1.0).contains(inner_centroid), \
                f"Ring {i+1} centroid is not inside ring {i}"

    def test_ring_connection_proximity(self):
        """Each ring should start within 3x effective_step of the previous ring's end."""
        strips = generate_strips(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP)
        effective_step = CUTTING_WIDTH - OVERLAP

        for i in range(1, len(strips)):
            prev_end = latlon_to_utm(*strips[i - 1][-1])
            curr_start = latlon_to_utm(*strips[i][0])
            dist = math.sqrt(
                (prev_end[0] - curr_start[0]) ** 2 +
                (prev_end[1] - curr_start[1]) ** 2
            )
            assert dist < 3 * effective_step, \
                f"Ring {i} start is {dist:.1f}m from ring {i-1} end (limit {3*effective_step:.1f}m)"


class TestComputeStripStats:
    def test_returns_expected_keys(self):
        stats = compute_strip_stats(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP)
        expected_keys = {
            "ring_count", "effective_step_m", "polygon_area_m2",
            "total_mowing_distance_m",
        }
        assert set(stats.keys()) == expected_keys

    def test_ring_count_matches_generate(self):
        stats = compute_strip_stats(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP)
        strips = generate_strips(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP)
        assert stats["ring_count"] == len(strips)

    def test_effective_step(self):
        stats = compute_strip_stats(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP)
        assert abs(stats["effective_step_m"] - (CUTTING_WIDTH - OVERLAP)) < 0.001

    def test_polygon_area_reasonable(self):
        """Surveyed section area should be in the right ballpark.
        The 4 corners span roughly 360m x 24m ≈ 8,640 m²."""
        stats = compute_strip_stats(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP)
        assert 5_000 < stats["polygon_area_m2"] < 15_000
