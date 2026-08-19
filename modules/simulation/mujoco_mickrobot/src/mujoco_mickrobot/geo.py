"""Local ENU to WGS84 conversion for simulated GNSS."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


WGS84_A = 6_378_137.0
WGS84_E2 = 6.69437999014e-3


class GeoError(ValueError):
    """Raised when a geographic input is outside the supported domain."""


@dataclass(frozen=True)
class Wgs84Origin:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float


def enu_to_wgs84(enu_m: Sequence[float], origin: Wgs84Origin) -> tuple[float, float, float]:
    """Apply a WGS84 local-tangent first-order conversion."""
    if len(enu_m) != 3:
        raise GeoError("ENU position must contain three values")
    values = (float(enu_m[0]), float(enu_m[1]), float(enu_m[2]), origin.latitude_deg, origin.longitude_deg, origin.altitude_m)
    if not all(math.isfinite(value) for value in values):
        raise GeoError("ENU position and WGS84 origin must be finite")
    if not -90.0 <= origin.latitude_deg <= 90.0:
        raise GeoError("origin latitude must be within [-90, 90]")
    if not -180.0 <= origin.longitude_deg <= 180.0:
        raise GeoError("origin longitude must be within [-180, 180]")
    latitude_rad = math.radians(origin.latitude_deg)
    cosine = math.cos(latitude_rad)
    if abs(cosine) < 1e-12:
        raise GeoError("local ENU longitude is undefined at the poles")
    sine = math.sin(latitude_rad)
    denominator = math.sqrt(1.0 - WGS84_E2 * sine * sine)
    prime_vertical = WGS84_A / denominator
    meridional = WGS84_A * (1.0 - WGS84_E2) / denominator**3
    east, north, up = values[:3]
    latitude = origin.latitude_deg + math.degrees(north / (meridional + origin.altitude_m))
    longitude = origin.longitude_deg + math.degrees(east / ((prime_vertical + origin.altitude_m) * cosine))
    return latitude, longitude, origin.altitude_m + up
