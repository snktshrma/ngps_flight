from typing import Tuple

import numpy as np
from pyproj import Transformer

_WGS84_TO_ECEF = Transformer.from_crs('EPSG:4326', 'EPSG:4978', always_xy=True)
_ECEF_TO_WGS84 = Transformer.from_crs('EPSG:4978', 'EPSG:4326', always_xy=True)


def wgs84_to_ecef(lon: float, lat: float, alt: float) -> Tuple[float, float, float]:
    x, y, z = _WGS84_TO_ECEF.transform(lon, lat, alt)
    return float(x), float(y), float(z)


def ecef_to_wgs84(x: float, y: float, z: float) -> Tuple[float, float, float]:
    lon, lat, alt = _ECEF_TO_WGS84.transform(x, y, z)
    return float(lon), float(lat), float(alt)


def enu_to_ecef_matrix(lon: float, lat: float) -> np.ndarray:
    lon, lat = np.radians(lon), np.radians(lat)
    slat, clat = np.sin(lat), np.cos(lat)
    slon, clon = np.sin(lon), np.cos(lon)
    R = np.array([
        [-slon, -slat * clon, clat * clon],
        [clon, -slat * slon, clat * slon],
        [0, clat, slat],
    ])
    return R


def pixel_to_geodetic(
    pixel_x: float,
    pixel_y: float,
    image_width: int,
    image_height: int,
    min_lon: float,
    min_lat: float,
    max_lon: float,
    max_lat: float,
) -> Tuple[float, float]:
    norm_x = pixel_x / image_width if image_width > 0 else 0.0
    norm_y = pixel_y / image_height if image_height > 0 else 0.0
    norm_x = np.clip(norm_x, 0.0, 1.0)
    norm_y = np.clip(norm_y, 0.0, 1.0)
    lon = min_lon + norm_x * (max_lon - min_lon)
    lat = max_lat - norm_y * (max_lat - min_lat)
    return lon, lat


def geodetic_to_pixel(
    lon: float,
    lat: float,
    image_width: int,
    image_height: int,
    min_lon: float,
    min_lat: float,
    max_lon: float,
    max_lat: float,
) -> Tuple[float, float]:
    norm_x = (lon - min_lon) / (max_lon - min_lon) if (max_lon - min_lon) > 0 else 0.0
    norm_y = (max_lat - lat) / (max_lat - min_lat) if (max_lat - min_lat) > 0 else 0.0
    norm_x = np.clip(norm_x, 0.0, 1.0)
    norm_y = np.clip(norm_y, 0.0, 1.0)
    pixel_x = norm_x * image_width
    pixel_y = norm_y * image_height
    return pixel_x, pixel_y


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    R = 6371000
    lat1_rad, lon1_rad = np.radians(lat1), np.radians(lon1)
    lat2_rad, lon2_rad = np.radians(lat2), np.radians(lon2)
    delta_lat = lat2_rad - lat1_rad
    delta_lon = lon2_rad - lon1_rad
    a = (
        np.sin(delta_lat / 2) ** 2
        + np.cos(lat1_rad) * np.cos(lat2_rad) * np.sin(delta_lon / 2) ** 2
    )
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    return R * c


def meters_per_degree_lat(lat: float) -> float:
    return 111045.0


def meters_per_degree_lon(lat: float) -> float:
    return 111045.0 * np.cos(np.radians(lat))
