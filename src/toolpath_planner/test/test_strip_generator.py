"""Tests for mowing strip generation using actual CDS2 runway data."""

import math
import pytest
from shapely.geometry import Polygon, Point

from toolpath_planner.strip_generator import (
    corners_to_utm,
    compute_heading_from_polygon,
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


class TestComputeHeading:
    def test_heading_roughly_south(self):
        """CDS2 Runway 11/29 runs roughly NNW-SSE (~170-175° from north).
        The surveyed corners define a polygon elongated in that direction."""
        utm = corners_to_utm(CDS2_CORNERS)
        heading_rad = compute_heading_from_polygon(utm)
        heading_deg = math.degrees(heading_rad)

        # The polygon long axis should be roughly 170-190° (southward)
        # or 350-10° (northward) — heading is ambiguous by 180°
        # Accept either direction
        assert (160 < heading_deg < 200) or (340 < heading_deg or heading_deg < 20), \
            f"Heading {heading_deg:.1f}° — expected roughly N-S for this polygon section"

    def test_heading_is_normalized(self):
        utm = corners_to_utm(CDS2_CORNERS)
        heading_rad = compute_heading_from_polygon(utm)
        assert 0 <= heading_rad < 2 * math.pi


class TestGenerateStrips:
    def test_generates_reasonable_strip_count(self):
        """78ft (~23.8m) wide runway with 1.52m cuts and 0.15m overlap.
        Effective step = 1.37m. Expected: ~17 strips."""
        strips = generate_strips(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP)
        assert 12 <= len(strips) <= 25, \
            f"Got {len(strips)} strips (expected ~17 for 23.8m / 1.37m)"

    def test_strips_not_empty(self):
        strips = generate_strips(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP)
        for i, strip in enumerate(strips):
            assert len(strip) >= 2, f"Strip {i} has fewer than 2 waypoints"

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
                    f"Strip {i} waypoint ({lat}, {lon}) -> UTM ({e}, {n}) outside polygon"

    def test_boustrophedon_ordering(self):
        """Odd-indexed strips should be reversed relative to even-indexed ones.
        Check that strip 0 and strip 1 start at opposite ends."""
        strips = generate_strips(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP)
        if len(strips) < 2:
            pytest.skip("Need at least 2 strips")

        # Strip 0 first point and strip 1 first point should be
        # at opposite ends of the runway (different latitudes for a N-S runway)
        s0_start_lat = strips[0][0][0]
        s0_end_lat = strips[0][-1][0]
        s1_start_lat = strips[1][0][0]

        # The start of strip 1 should be closer to the end of strip 0
        dist_to_start = abs(s1_start_lat - s0_start_lat)
        dist_to_end = abs(s1_start_lat - s0_end_lat)
        assert dist_to_end < dist_to_start, \
            "Boustrophedon: strip 1 should start near where strip 0 ends"

    def test_heading_override(self):
        """Manual heading override should produce strips in a different direction."""
        strips_auto = generate_strips(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP)
        strips_90 = generate_strips(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP, heading_override_deg=90.0)

        # Different heading should produce different strip count
        # (cutting across the polygon at a different angle)
        # At minimum, the waypoints should be different
        assert strips_auto[0][0] != strips_90[0][0], \
            "Heading override should produce different waypoints"

    def test_overlap_zero(self):
        """With zero overlap, should get slightly fewer strips."""
        strips_overlap = generate_strips(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP)
        strips_no_overlap = generate_strips(CDS2_CORNERS, CUTTING_WIDTH, 0.0)
        # More overlap means more strips
        assert len(strips_overlap) >= len(strips_no_overlap)

    def test_invalid_overlap_raises(self):
        """Overlap >= cutting_width should raise ValueError."""
        with pytest.raises(ValueError):
            generate_strips(CDS2_CORNERS, CUTTING_WIDTH, CUTTING_WIDTH)


class TestComputeStripStats:
    def test_returns_expected_keys(self):
        stats = compute_strip_stats(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP)
        expected_keys = {
            "strip_count", "effective_step_m", "polygon_area_m2",
            "runway_width_m", "runway_length_m", "total_mowing_distance_m",
            "heading_deg",
        }
        assert set(stats.keys()) == expected_keys

    def test_strip_count_matches_generate(self):
        stats = compute_strip_stats(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP)
        strips = generate_strips(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP)
        # Stats estimate may differ slightly from actual generation (clipping effects)
        assert abs(stats["strip_count"] - len(strips)) <= 2

    def test_effective_step(self):
        stats = compute_strip_stats(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP)
        assert abs(stats["effective_step_m"] - (CUTTING_WIDTH - OVERLAP)) < 0.001

    def test_polygon_area_reasonable(self):
        """Surveyed section area should be in the right ballpark.
        The 4 corners span roughly 360m x 24m ≈ 8,640 m²."""
        stats = compute_strip_stats(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP)
        assert 5_000 < stats["polygon_area_m2"] < 15_000

    def test_runway_dimensions(self):
        stats = compute_strip_stats(CDS2_CORNERS, CUTTING_WIDTH, OVERLAP)
        # Width ~24m (78ft), length ~360m (surveyed section, not full 2246ft)
        assert 15 < stats["runway_width_m"] < 35
        assert 300 < stats["runway_length_m"] < 400
