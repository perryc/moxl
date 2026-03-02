"""Load runway polygon data from ppa_airports JSON format."""

import json
from pathlib import Path


def load_airport(json_path: str | Path) -> dict:
    """Load airport JSON data from a ppa_airports file.

    Args:
        json_path: Path to the airport JSON file (e.g., CDS2.json).

    Returns:
        Parsed airport dict.
    """
    with open(json_path) as f:
        return json.load(f)


def get_runway(airport: dict, designation: str) -> dict:
    """Get a runway by its designation (e.g., '11/29').

    Args:
        airport: Airport dict from load_airport().
        designation: Runway designation string.

    Returns:
        Runway dict.

    Raises:
        ValueError: If runway not found.
    """
    for rwy in airport.get("runways", []):
        if rwy["designation"] == designation:
            return rwy
    available = [r["designation"] for r in airport.get("runways", [])]
    raise ValueError(
        f"Runway '{designation}' not found in {airport.get('icao_code', '?')}. "
        f"Available: {available}"
    )


def get_corner_points(runway: dict) -> list[tuple[float, float]]:
    """Extract the RTK-surveyed corner points from a runway.

    Returns corners as a list of (latitude, longitude) tuples,
    in the order they appear in the survey data.

    Args:
        runway: Runway dict from get_runway().

    Returns:
        List of (lat, lon) tuples for each corner.

    Raises:
        ValueError: If no survey corner points found.
    """
    metadata = runway.get("survey_metadata", {})
    corners = metadata.get("corner_points", [])
    if not corners:
        raise ValueError(
            f"No corner_points in survey_metadata for runway {runway['designation']}"
        )

    return [(c["latitude"], c["longitude"]) for c in corners]


def get_clear_zones(runway: dict) -> list[dict]:
    """Extract clear zone polygons from a runway.

    Each clear zone has a name and a list of (lat, lon) vertices.

    Args:
        runway: Runway dict from get_runway().

    Returns:
        List of dicts with 'name' and 'vertices' keys.
        Vertices are list of (lat, lon) tuples.
        Returns empty list if no clear zones defined.
    """
    zones = runway.get("clear_zones", [])
    result = []
    for zone in zones:
        vertices = [(v["latitude"], v["longitude"]) for v in zone["vertices"]]
        centroid_lat = sum(v[0] for v in vertices) / len(vertices)
        centroid_lon = sum(v[1] for v in vertices) / len(vertices)
        result.append({
            "name": zone["name"],
            "vertices": vertices,
            "centroid": (centroid_lat, centroid_lon),
        })
    return result


def get_corner_accuracy(runway: dict) -> list[float]:
    """Get the GPS accuracy (in meters) of each corner point.

    Returns:
        List of accuracy values in meters, same order as get_corner_points().
    """
    metadata = runway.get("survey_metadata", {})
    corners = metadata.get("corner_points", [])
    return [c.get("accuracy_m", float("inf")) for c in corners]
