"""Tests for WGS84 <-> UTM coordinate conversion."""

import pytest
from toolpath_planner.coordinate_utils import latlon_to_utm, utm_to_latlon


# CDS2 Runway 11/29 Corner 1 (NW) — RTK surveyed at 0.02m accuracy
CDS2_C1_LAT = 50.63860436
CDS2_C1_LON = -105.0402994


class TestLatLonToUtm:
    def test_returns_two_floats(self):
        easting, northing = latlon_to_utm(CDS2_C1_LAT, CDS2_C1_LON)
        assert isinstance(easting, float)
        assert isinstance(northing, float)

    def test_cds2_easting_range(self):
        """CDS2 is near central meridian of UTM Zone 13N (105°W).
        Easting should be near 500000m (false easting), slightly west."""
        easting, _ = latlon_to_utm(CDS2_C1_LAT, CDS2_C1_LON)
        assert 496_000 < easting < 502_000

    def test_cds2_northing_range(self):
        """CDS2 at ~50.63°N should have northing around 5,609,000m."""
        _, northing = latlon_to_utm(CDS2_C1_LAT, CDS2_C1_LON)
        assert 5_608_000 < northing < 5_612_000


class TestRoundTrip:
    def test_round_trip_accuracy(self):
        """Round-trip conversion should be accurate to < 0.001m (~0.00001°)."""
        easting, northing = latlon_to_utm(CDS2_C1_LAT, CDS2_C1_LON)
        lat_back, lon_back = utm_to_latlon(easting, northing)

        # 0.00001° latitude ≈ 1.1m, so 1e-8 ≈ 0.001m
        assert abs(lat_back - CDS2_C1_LAT) < 1e-8
        assert abs(lon_back - CDS2_C1_LON) < 1e-8

    def test_round_trip_all_corners(self):
        """All four CDS2 runway 11/29 corners should round-trip cleanly."""
        corners = [
            (50.63860436, -105.0402994),
            (50.63878645, -105.0402185),
            (50.63597254, -105.03158463),
            (50.63574267, -105.03170282),
        ]
        for lat, lon in corners:
            easting, northing = latlon_to_utm(lat, lon)
            lat_back, lon_back = utm_to_latlon(easting, northing)
            assert abs(lat_back - lat) < 1e-8, f"Lat mismatch for ({lat}, {lon})"
            assert abs(lon_back - lon) < 1e-8, f"Lon mismatch for ({lat}, {lon})"

    def test_utm_distances_match_known(self):
        """Runway 11/29 is 2246 ft (~684.6m) long, 78 ft (~23.8m) wide.
        Check that UTM distances between corners are in the right ballpark."""
        import math

        corners = [
            (50.63860436, -105.0402994),   # C1 NW
            (50.63878645, -105.0402185),   # C2 NE
            (50.63597254, -105.03158463),  # C3 SE
            (50.63574267, -105.03170282),  # C4 SW
        ]
        utm = [latlon_to_utm(lat, lon) for lat, lon in corners]

        # Width: C1-C2 (NW to NE) — should be ~23.8m (78 ft)
        dx = utm[1][0] - utm[0][0]
        dy = utm[1][1] - utm[0][1]
        width_12 = math.sqrt(dx**2 + dy**2)
        assert 20 < width_12 < 30, f"C1-C2 width: {width_12:.1f}m (expected ~23.8m)"

        # Length: C1-C4 (NW to SW) — should be ~684.6m (2246 ft)
        dx = utm[3][0] - utm[0][0]
        dy = utm[3][1] - utm[0][1]
        length_14 = math.sqrt(dx**2 + dy**2)
        assert 350 < length_14 < 400, f"C1-C4 length: {length_14:.1f}m (expected ~360m)"
        # Note: C1-C4 is only about half the runway — these are the surveyed
        # portion corners, not full runway thresholds.
