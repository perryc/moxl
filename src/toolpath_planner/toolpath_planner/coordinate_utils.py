"""WGS84 <-> UTM coordinate conversion utilities using pyproj."""

from pyproj import Transformer


def make_transformers(utm_zone: int = 13, hemisphere: str = "north"):
    """Create WGS84 <-> UTM transformers for a given zone.

    Args:
        utm_zone: UTM zone number (CDS2 is in zone 13)
        hemisphere: "north" or "south"

    Returns:
        Tuple of (wgs84_to_utm, utm_to_wgs84) Transformer objects.
    """
    epsg_utm = 32600 + utm_zone if hemisphere == "north" else 32700 + utm_zone
    wgs84_to_utm = Transformer.from_crs(
        "EPSG:4326", f"EPSG:{epsg_utm}", always_xy=True
    )
    utm_to_wgs84 = Transformer.from_crs(
        f"EPSG:{epsg_utm}", "EPSG:4326", always_xy=True
    )
    return wgs84_to_utm, utm_to_wgs84


# Default transformers for CDS2 (UTM Zone 13N, EPSG:32613)
_wgs84_to_utm, _utm_to_wgs84 = make_transformers(13, "north")


def latlon_to_utm(lat: float, lon: float) -> tuple[float, float]:
    """Convert WGS84 lat/lon to UTM easting/northing (Zone 13N).

    Note: pyproj with always_xy=True expects (lon, lat) order.

    Returns:
        (easting, northing) in meters.
    """
    easting, northing = _wgs84_to_utm.transform(lon, lat)
    return easting, northing


def utm_to_latlon(easting: float, northing: float) -> tuple[float, float]:
    """Convert UTM easting/northing (Zone 13N) to WGS84 lat/lon.

    Returns:
        (latitude, longitude) in degrees.
    """
    lon, lat = _utm_to_wgs84.transform(easting, northing)
    return lat, lon
