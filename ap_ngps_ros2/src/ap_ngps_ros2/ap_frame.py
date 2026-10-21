"""ArduPilot's own local frame, exactly as it computes it.

/ap/tf (external navigation) must be NED metres from the EKF ORIGIN: ArduPilot does
not re-offset it (AP_VisualOdom's position alignment is a no-op in this firmware) and
aligns only its yaw, once, to the first sample. /ap/pose/filtered is relative to HOME,
which ArduPilot moves when it arms (15 m from the origin on flight 3, 5.24 m in
height), so it must never be used as the output frame.

The scaling is AP_Common/Location's (LATLON_TO_M, cosine of the mid latitude), so a
position computed here is the one ArduPilot means by the same lat/lon.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

LATLON_TO_M = 0.011131884502145034 * 1e7   # metres per degree (AP_Math/definitions.h)


def _lon_scale(lat_deg: float) -> float:
    return max(math.cos(math.radians(lat_deg)), 0.01)


def ll_to_en(origin: Tuple[float, ...], lat: float, lon: float) -> Tuple[float, float]:
    """(east, north) metres of lat/lon from origin (lat, lon[, alt]); Location::get_distance_NE."""
    la0, lo0 = origin[0], origin[1]
    dlon = (lon - lo0 + 180.0) % 360.0 - 180.0
    return (dlon * LATLON_TO_M * _lon_scale(0.5 * (lat + la0)), (lat - la0) * LATLON_TO_M)


def en_to_ll(origin: Tuple[float, ...], east: float, north: float) -> Tuple[float, float]:
    """Inverse of ll_to_en (Location::offset)."""
    la0, lo0 = origin[0], origin[1]
    dlat = north / LATLON_TO_M
    lon = lo0 + east / (LATLON_TO_M * _lon_scale(la0 + 0.5 * dlat))
    return la0 + dlat, (lon + 180.0) % 360.0 - 180.0


def valid_ll(lat: float, lon: float) -> bool:
    """AP_DDS leaves lat/lon at 0 when the EKF has no location or origin yet."""
    return math.isfinite(lat) and math.isfinite(lon) and not (lat == 0.0 and lon == 0.0)


def yaw_enu_deg(q) -> float:
    """ENU yaw (counter-clockwise from east) of a geometry_msgs Quaternion-like object."""
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def ekf_en_up(origin: Optional[Tuple[float, float, float]], lat: float, lon: float, alt: float):
    """The EKF's position from /ap/geopose/filtered as (east, north, up) from the origin."""
    if origin is None or not valid_ll(lat, lon):
        return None
    e, n = ll_to_en(origin, lat, lon)
    return e, n, alt - origin[2]
