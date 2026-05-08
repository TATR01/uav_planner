"""Polygon geometry and collision detection"""

import math
from typing import List, Tuple, Dict, Optional


class Point:
    """2D point"""
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
    
    def distance_to(self, other: 'Point') -> float:
        return math.hypot(other.x - self.x, other.y - self.y)
    
    def __repr__(self):
        return f'Point({self.x:.2f}, {self.y:.2f})'


def polygon_area(points: List[Point]) -> float:
    """
    Calculate polygon area using Shoelace formula.
    """
    if len(points) < 3:
        return 0.0
    
    area = 0.0
    n = len(points)
    
    for i in range(n):
        j = (i + 1) % n
        area += points[i].x * points[j].y
        area -= points[j].x * points[i].y
    
    return abs(area) / 2.0


def bounding_box(points: List[Point]) -> Dict[str, float]:
    """
    Get bounding box of polygon.
    
    Returns:
        {'min_x', 'max_x', 'min_y', 'max_y'}
    """
    if not points:
        return {'min_x': 0, 'max_x': 0, 'min_y': 0, 'max_y': 0}
    
    xs = [p.x for p in points]
    ys = [p.y for p in points]
    
    return {
        'min_x': min(xs),
        'max_x': max(xs),
        'min_y': min(ys),
        'max_y': max(ys),
    }


def point_in_polygon(point: Point, polygon: List[Point]) -> bool:
    """
    Ray casting algorithm for point-in-polygon test.
    """
    if len(polygon) < 3:
        return False
    
    inside = False
    j = len(polygon) - 1
    
    for i in range(len(polygon)):
        xi, yi = polygon[i].x, polygon[i].y
        xj, yj = polygon[j].x, polygon[j].y
        
        if ((yi > point.y) != (yj > point.y) and
            point.x < (xj - xi) * (point.y - yi) / (yj - yi) + xi):
            inside = not inside
        
        j = i
    
    return inside


def segment_intersection(p1: Point, p2: Point, p3: Point, p4: Point) -> Optional[Point]:
    """
    Find intersection point of two line segments p1-p2 and p3-p4.
    Returns None if segments don't intersect.
    """
    x1, y1 = p1.x, p1.y
    x2, y2 = p2.x, p2.y
    x3, y3 = p3.x, p3.y
    x4, y4 = p4.x, p4.y
    
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    
    if abs(denom) < 1e-12:
        return None  # Parallel or collinear
    
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom
    
    if 0 <= t <= 1 and 0 <= u <= 1:
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)
        return Point(x, y)
    
    return None


def circle_buffer(center: Point, radius: float, n_points: int = 16) -> List[Point]:
    """
    Create circular buffer polygon around a point.
    """
    points = []
    for i in range(n_points):
        angle = 2 * math.pi * i / n_points
        x = center.x + radius * math.cos(angle)
        y = center.y + radius * math.sin(angle)
        points.append(Point(x, y))
    return points


def line_buffer(p1: Point, p2: Point, radius: float, n_points: int = 8) -> List[Point]:
    """
    Create buffer polygon around a line segment (rounded rectangle).
    """
    dx = p2.x - p1.x
    dy = p2.y - p1.y
    length = math.hypot(dx, dy) or 1
    
    # Perpendicular vectors
    nx = -dy / length * radius
    ny = dx / length * radius
    
    points = []
    
    # Semicircle at p1
    for i in range(n_points + 1):
        angle = math.atan2(dy, dx) + math.pi / 2 + math.pi * i / n_points
        x = p1.x + math.cos(angle) * radius
        y = p1.y + math.sin(angle) * radius
        points.append(Point(x, y))
    
    # Semicircle at p2
    for i in range(n_points + 1):
        angle = math.atan2(dy, dx) - math.pi / 2 + math.pi * i / n_points
        x = p2.x + math.cos(angle) * radius
        y = p2.y + math.sin(angle) * radius
        points.append(Point(x, y))
    
    return points


def minkowski_expand(polygon: List[Point], radius: float) -> List[Point]:
    """
    Expand polygon by radius using outset normals (simplified Minkowski sum).
    """
    if len(polygon) < 3:
        return polygon
    
    expanded = []
    n = len(polygon)
    
    for i in range(n):
        prev = polygon[(i - 1) % n]
        cur = polygon[i]
        next_p = polygon[(i + 1) % n]
        
        # Vector from prev to cur
        dx1 = cur.x - prev.x
        dy1 = cur.y - prev.y
        len1 = math.hypot(dx1, dy1) or 1
        
        # Vector from cur to next
        dx2 = next_p.x - cur.x
        dy2 = next_p.y - cur.y
        len2 = math.hypot(dx2, dy2) or 1
        
        # Outward normals
        nx1 = -dy1 / len1
        ny1 = dx1 / len1
        nx2 = -dy2 / len2
        ny2 = dx2 / len2
        
        # Average normal
        nx = (nx1 + nx2) / 2
        ny = (ny1 + ny2) / 2
        nl = math.hypot(nx, ny) or 1
        
        x = cur.x + (nx / nl) * radius
        y = cur.y + (ny / nl) * radius
        expanded.append(Point(x, y))
    
    return expanded
