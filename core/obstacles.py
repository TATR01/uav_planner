"""Obstacle management and buffer zones"""

from typing import List, Dict, Any
from dataclasses import dataclass, field
from enum import Enum
from core.geometry import Point, circle_buffer, line_buffer, minkowski_expand


class ObstacleType(Enum):
    """Types of obstacles"""
    NFZ = "nfz"  # No-Fly Zone (polygon)
    TREE = "tree"  # Single point
    POWER = "power"  # Power line (line segment)
    BUILDING = "building"  # Building (polygon)


@dataclass
class Obstacle:
    """Obstacle definition"""
    type: ObstacleType
    points: List[Point] = field(default_factory=list)
    radius: float = 30.0  # Buffer radius in meters
    label: str = ""
    xy_points: List[Point] = field(default_factory=list)  # Projected to metric coords


@dataclass
class BufferZone:
    """Buffer zone polygon around an obstacle"""
    polygon: List[Point]
    obstacle_type: ObstacleType
    obstacle: Obstacle


class ObstacleManager:
    """Manages obstacles and buffer zones"""
    
    def __init__(self):
        self.obstacles: List[Obstacle] = []
        self.buffer_zones: List[BufferZone] = []
    
    def add_obstacle(self, obs_type: ObstacleType, points: List[Point], radius: float = 30.0, label: str = "") -> Obstacle:
        """
        Add new obstacle.
        """
        obs = Obstacle(
            type=obs_type,
            points=points,
            radius=radius,
            label=label or f"Obs{len(self.obstacles) + 1}"
        )
        self.obstacles.append(obs)
        self._update_buffer(obs)
        return obs
    
    def remove_obstacle(self, index: int):
        """
        Remove obstacle by index.
        """
        if 0 <= index < len(self.obstacles):
            self.obstacles.pop(index)
            self._rebuild_buffers()
    
    def clear(self):
        """
        Remove all obstacles.
        """
        self.obstacles.clear()
        self.buffer_zones.clear()
    
    def _update_buffer(self, obstacle: Obstacle):
        """
        Create buffer zone for single obstacle.
        """
        if not obstacle.xy_points:
            return
        
        buffer_poly = None
        
        if obstacle.type == ObstacleType.TREE:
            # Circle around point
            buffer_poly = circle_buffer(obstacle.xy_points[0], obstacle.radius)
        
        elif obstacle.type == ObstacleType.NFZ or obstacle.type == ObstacleType.BUILDING:
            # Expand polygon
            buffer_poly = minkowski_expand(obstacle.xy_points, obstacle.radius)
        
        elif obstacle.type == ObstacleType.POWER:
            # Line buffer
            if len(obstacle.xy_points) >= 2:
                buffer_poly = line_buffer(
                    obstacle.xy_points[0],
                    obstacle.xy_points[1],
                    obstacle.radius
                )
        
        if buffer_poly:
            self.buffer_zones.append(BufferZone(
                polygon=buffer_poly,
                obstacle_type=obstacle.type,
                obstacle=obstacle
            ))
    
    def _rebuild_buffers(self):
        """
        Rebuild all buffer zones.
        """
        self.buffer_zones.clear()
        for obs in self.obstacles:
            self._update_buffer(obs)
    
    def get_buffer_polygons(self) -> List[List[Point]]:
        """
        Get all buffer polygons as lists of points.
        """
        return [bz.polygon for bz in self.buffer_zones]
