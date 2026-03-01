"""Tests for polygon_loader with actual CDS2 data."""

import json
import tempfile
import os
import pytest

from toolpath_planner.polygon_loader import (
    load_airport,
    get_runway,
    get_corner_points,
    get_corner_accuracy,
)


@pytest.fixture
def cds2_json_path():
    """Path to the CDS2 test data file."""
    # Look for the actual config file in the repo
    repo_path = os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "config", "airstrips", "CDS2.json"
    )
    if os.path.exists(repo_path):
        yield repo_path
        return

    # Fallback: create a minimal test fixture
    data = {
        "icao_code": "CDS2",
        "name": "Disley",
        "runways": [
            {
                "designation": "11/29",
                "survey_metadata": {
                    "corner_points": [
                        {"corner": "CORNER_1", "latitude": 50.6361219, "longitude": -105.03202723, "accuracy_m": 0.02},
                        {"corner": "CORNER_2", "latitude": 50.63603243, "longitude": -105.03175091, "accuracy_m": 0.02},
                        {"corner": "CORNER_3", "latitude": 50.63285006, "longitude": -105.03165067, "accuracy_m": 0.02},
                        {"corner": "CORNER_4", "latitude": 50.63288086, "longitude": -105.03194065, "accuracy_m": 0.02},
                    ]
                },
            }
        ],
    }
    fd, path = tempfile.mkstemp(suffix=".json")
    with os.fdopen(fd, "w") as f:
        json.dump(data, f)
    yield path
    os.unlink(path)


class TestLoadAirport:
    def test_loads_cds2(self, cds2_json_path):
        airport = load_airport(cds2_json_path)
        assert airport["icao_code"] == "CDS2"

    def test_has_runways(self, cds2_json_path):
        airport = load_airport(cds2_json_path)
        assert len(airport["runways"]) >= 1

    def test_missing_file_raises(self):
        with pytest.raises(FileNotFoundError):
            load_airport("/nonexistent/path.json")


class TestGetRunway:
    def test_finds_runway_11_29(self, cds2_json_path):
        airport = load_airport(cds2_json_path)
        runway = get_runway(airport, "11/29")
        assert runway["designation"] == "11/29"

    def test_missing_runway_raises(self, cds2_json_path):
        airport = load_airport(cds2_json_path)
        with pytest.raises(ValueError, match="not found"):
            get_runway(airport, "99/99")


class TestGetCornerPoints:
    def test_returns_four_corners(self, cds2_json_path):
        airport = load_airport(cds2_json_path)
        runway = get_runway(airport, "11/29")
        corners = get_corner_points(runway)
        assert len(corners) == 4

    def test_corners_are_lat_lon_tuples(self, cds2_json_path):
        airport = load_airport(cds2_json_path)
        runway = get_runway(airport, "11/29")
        corners = get_corner_points(runway)
        for lat, lon in corners:
            assert 50.0 < lat < 51.0
            assert -106.0 < lon < -104.0


class TestGetCornerAccuracy:
    def test_returns_accuracy_values(self, cds2_json_path):
        airport = load_airport(cds2_json_path)
        runway = get_runway(airport, "11/29")
        accuracy = get_corner_accuracy(runway)
        assert len(accuracy) == 4
        # All 11/29 corners are RTK at 0.02m
        for acc in accuracy:
            assert acc == pytest.approx(0.02, abs=0.01)
