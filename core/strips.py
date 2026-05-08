"""Strip calculation and management"""

import math
from typing import List, Dict, Tuple
from core.geometry import Point, bounding_box, point_in_polygon, segment_intersection


class Strip:
    """Flight strip (scanning line)"""
    def __init__(self, x: float):
        self.x = x
        self.segments = []  # List of (y0, y1) tuples (intersections with field)
    
    def add_segment(self, y0: float, y1: float):
        if y1 > y0:
            self.segments.append((y0, y1))
    
    def points_up(self) -> List[Point]:
        """Get waypoints along strip going up"""
        points = []
        for y0, y1 in self.segments:
            points.append(Point(self.x, y0))
            points.append(Point(self.x, y1))
        return points
    
    def points_down(self) -> List[Point]:
        """Get waypoints along strip going down"""
        points = []
        for y0, y1 in reversed(self.segments):
            points.append(Point(self.x, y1))
            points.append(Point(self.x, y0))
        return points
    
    def points(self, direction_up: bool = True) -> List[Point]:
        return self.points_up() if direction_up else self.points_down()
    
    def length(self) -> float:
        """Total length of all segments"""
        return sum(y1 - y0 for y0, y1 in self.segments)


def calculate_strip_width(altitude: float, fov: float, overlap_side: float) -> Dict[str, float]:
    """
    Calculate strip width and step based on camera parameters.
    
    Args:
        altitude: Flight altitude in meters
        fov: Camera field of view in degrees
        overlap_side: Side overlap ratio (0.0-1.0)
    
    Returns:
        {'width': strip_width, 'step': spacing_between_strips}
    """
    # Ground coverage width = 2 * altitude * tan(fov/2)
    fov_rad = fov * math.pi / 180
    width = 2 * altitude * math.tan(fov_rad / 2)
    
    # Step between strip centers
    step = width * (1 - overlap_side)
    
    return {'width': width, 'step': step}


def clip_strip_to_polygon(x: float, y_min: float, y_max: float, polygon: List[Point]) -> List[Tuple[float, float]]:
    """
    Find intersections of vertical line x=[const] with polygon edges
    and segments where the line is inside the polygon.
    
    Returns list of (y0, y1) segments.
    """
    # Check intersections with polygon edges
    intersections = []
    p_min = Point(x, y_min - 1)
    p_max = Point(x, y_max + 1)
    
    for i in range(len(polygon)):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % len(polygon)]
        intersection = segment_intersection(p_min, p_max, p1, p2)
        if intersection:
            intersections.append(intersection.y)
    
    # Check endpoints
    if point_in_polygon(Point(x, y_min), polygon):
        intersections.append(y_min)
    if point_in_polygon(Point(x, y_max), polygon):
        intersections.append(y_max)
    
    if len(intersections) < 2:
        return []
    
    intersections.sort()
    
    # Remove duplicates
    unique = [intersections[0]]
    for y in intersections[1:]:
        if abs(y - unique[-1]) > 1e-6:
            unique.append(y)
    
    # Find segments between consecutive intersections that are inside polygon
    segments = []
    for i in range(len(unique) - 1):
        y0, y1 = unique[i], unique[i + 1]
        mid_y = (y0 + y1) / 2
        
        if point_in_polygon(Point(x, mid_y), polygon):
            segments.append((y0, y1))
    
    return segments


def subtract_buffers_from_segment(x: float, y0: float, y1: float, buffer_polygons: List[List[Point]]) -> List[Tuple[float, float]]:
    """
    Subtract buffer zones (obstacles) from a strip segment.
    
    Args:
        x: X coordinate of strip
        y0, y1: Segment boundaries
        buffer_polygons: List of obstacle buffer polygons
    
    Returns:
        List of (y0, y1) segments after subtracting buffers
    """
    if not buffer_polygons:
        return [(y0, y1)] if y1 > y0 else []
    
    # Collect all buffer intersections with this segment
    cuts = []
    for buffer_poly in buffer_polygons:
        segments = clip_strip_to_polygon(x, y0, y1, buffer_poly)
        cuts.extend(segments)
    
    if not cuts:
        return [(y0, y1)] if y1 > y0 else []
    
    # Merge overlapping cuts
    cuts.sort()
    merged = [cuts[0]]
    for cut_y0, cut_y1 in cuts[1:]:
        if cut_y0 <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], cut_y1))
        else:
            merged.append((cut_y0, cut_y1))
    
    # Subtract merged cuts from [y0, y1]
    result = []
    current = y0
    
    for cut_y0, cut_y1 in merged:
        if cut_y0 > current + 1e-6:
            result.append((current, min(cut_y0, y1)))
        current = max(current, cut_y1)
        if current >= y1:
            break
    
    if current < y1 - 1e-6:
        result.append((current, y1))
    
    return [s for s in result if s[1] - s[0] > 1.0]


def build_strips(polygon: List[Point], width: float, step: float, buffer_polygons: List[List[Point]] = None) -> List[Strip]:
    """
    Build strips across polygon, accounting for obstacles.
    
    Args:
        polygon: Field boundary polygon
        width: Strip width
        step: Strip spacing
        buffer_polygons: List of obstacle buffer polygons
    
    Returns:
        List of Strip objects
    """
    if not buffer_polygons:
        buffer_polygons = []
    
    bbox = bounding_box(polygon)
    min_x = bbox['min_x']
    max_x = bbox['max_x']
    min_y = bbox['min_y']
    max_y = bbox['max_y']
    
    strips = []
    x = min_x + width / 2
    
    while x <= max_x + width / 2:
        # Clip strip to polygon
        raw_segments = clip_strip_to_polygon(x, min_y, max_y, polygon)
        
        # Subtract buffer zones
        final_segments = []
        for seg_y0, seg_y1 in raw_segments:
            sub = subtract_buffers_from_segment(x, seg_y0, seg_y1, buffer_polygons)
            final_segments.extend(sub)
        
        if final_segments:
            strip = Strip(x)
            for y0, y1 in final_segments:
                strip.add_segment(y0, y1)
            strips.append(strip)
        
        x += step
    
    return strips


def route_length(order: List[int], strips: List[Strip]) -> float:
    """
    Calculate total route length for given strip order.
    """
    if not order or not strips:
        return 0.0
    
    total = 0.0
    prev_point = None
    
    for i, strip_idx in enumerate(order):
        strip = strips[strip_idx]
        points = strip.points(direction_up=(i % 2 == 0))
        
        if not points:
            continue
        
        # Add transition distance from previous strip
        if prev_point:
            total += prev_point.distance_to(points[0])
        
        # Add strip length
        for j in range(len(points) - 1):
            total += points[j].distance_to(points[j + 1])
        
        prev_point = points[-1]
    
    return total
