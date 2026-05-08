"""Geodetic calculations and coordinate transformations"""

import math
from config.settings import EARTH_RADIUS


def lat_lon_to_xy(lat: float, lon: float, origin_lat: float, origin_lon: float) -> dict:
    """
    Convert lat/lon to metric coordinates (X, Y) relative to origin.
    Uses equirectangular approximation.
    
    Args:
        lat: Latitude in degrees
        lon: Longitude in degrees
        origin_lat: Origin latitude in degrees
        origin_lon: Origin longitude in degrees
    
    Returns:
        {'x': x_meters, 'y': y_meters}
    """
    lat_rad = lat * math.pi / 180
    lon_rad = lon * math.pi / 180
    origin_lat_rad = origin_lat * math.pi / 180
    origin_lon_rad = origin_lon * math.pi / 180
    
    x = (lon_rad - origin_lon_rad) * EARTH_RADIUS * math.cos(origin_lat_rad)
    y = (lat_rad - origin_lat_rad) * EARTH_RADIUS
    
    return {'x': x, 'y': y}


def xy_to_lat_lon(x: float, y: float, origin_lat: float, origin_lon: float) -> dict:
    """
    Convert metric coordinates (X, Y) to lat/lon relative to origin.
    
    Args:
        x: X coordinate in meters
        y: Y coordinate in meters
        origin_lat: Origin latitude in degrees
        origin_lon: Origin longitude in degrees
    
    Returns:
        {'lat': latitude, 'lon': longitude}
    """
    origin_lat_rad = origin_lat * math.pi / 180
    origin_lon_rad = origin_lon * math.pi / 180
    
    lat = origin_lat + (y / EARTH_RADIUS) * 180 / math.pi
    lon = origin_lon + (x / (EARTH_RADIUS * math.cos(origin_lat_rad))) * 180 / math.pi
    
    return {'lat': lat, 'lon': lon}


def distance_2d(x1: float, y1: float, x2: float, y2: float) -> float:
    """
    Euclidean distance between two points.
    """
    return math.hypot(x2 - x1, y2 - y1)


def bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate bearing (azimuth) from point 1 to point 2 in degrees.
    """
    lat1_rad = lat1 * math.pi / 180
    lat2_rad = lat2 * math.pi / 180
    dlon_rad = (lon2 - lon1) * math.pi / 180
    
    y = math.sin(dlon_rad) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon_rad)
    
    bearing_rad = math.atan2(y, x)
    bearing_deg = bearing_rad * 180 / math.pi
    
    return (bearing_deg + 360) % 360
