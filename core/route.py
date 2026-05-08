"""Main route calculation orchestration"""

from typing import List, Dict, Any
from core.geodesy import lat_lon_to_xy, xy_to_lat_lon
from core.geometry import Point
from core.strips import Strip, build_strips, calculate_strip_width
from core.obstacles import ObstacleManager
from core.algorithms import algo_dp, algo_vrp, algo_ga, algo_aco


class RouteCalculator:
    """Calculates UAV routes"""
    
    def __init__(self):
        self.field_points = []  # lat/lon
        self.field_xy = []      # metric
        self.obstacle_manager = ObstacleManager()
        self.strips = []
        self.routes = []
        self.waypoints = []
    
    def set_field(self, lat_lon_points: List[tuple]):
        """Set field boundary points."""
        self.field_points = [Point(lat, lon) if not isinstance(lat, Point) else lat 
                            for lat, lon in lat_lon_points]
        self._project_field()
    
    def _project_field(self):
        """Project field to metric coordinates."""
        if not self.field_points:
            self.field_xy = []
            return
        
        origin = self.field_points[0]
        origin_lat = origin.x if isinstance(origin.x, float) else origin.x
        origin_lon = origin.y if isinstance(origin.y, float) else origin.y
        
        self.field_xy = []
        for pt in self.field_points:
            if isinstance(pt, Point):
                lat, lon = pt.x, pt.y
            else:
                lat, lon = pt
            
            xy = lat_lon_to_xy(lat, lon, origin_lat, origin_lon)
            self.field_xy.append(Point(xy['x'], xy['y']))
    
    def calculate_route(self, altitude: float, speed: float, fov: float, battery: float,
                       overlap_forward: float, overlap_side: float, algorithm: str = 'dp',
                       **algo_params) -> Dict[str, Any]:
        """Calculate optimal route."""
        
        if len(self.field_xy) < 3:
            return {'error': 'Need at least 3 field points'}
        
        # Project obstacles
        origin = self.field_points[0]
        origin_lat = origin.x if isinstance(origin.x, float) else origin.x
        origin_lon = origin.y if isinstance(origin.y, float) else origin.y
        
        for obs in self.obstacle_manager.obstacles:
            obs.xy_points = []
            for pt in obs.points:
                if isinstance(pt, Point):
                    lat, lon = pt.x, pt.y
                else:
                    lat, lon = pt
                xy = lat_lon_to_xy(lat, lon, origin_lat, origin_lon)
                obs.xy_points.append(Point(xy['x'], xy['y']))
        
        # Rebuild buffers
        self.obstacle_manager._rebuild_buffers()
        
        # Calculate strips
        strip_info = calculate_strip_width(altitude, fov, overlap_side)
        buffer_polygons = self.obstacle_manager.get_buffer_polygons()
        self.strips = build_strips(self.field_xy, strip_info['width'], strip_info['step'], buffer_polygons)
        
        if not self.strips:
            return {'error': 'Cannot build strips'}
        
        # Run optimization
        if algorithm == 'dp':
            result = algo_dp(self.strips, buffer_polygons)
        elif algorithm == 'vrp':
            k = algo_params.get('num_routes', 2)
            result = algo_vrp(self.strips, buffer_polygons, k)
        elif algorithm == 'ga':
            pop = algo_params.get('population', 60)
            gen = algo_params.get('generations', 150)
            result = algo_ga(self.strips, buffer_polygons, pop, gen)
        elif algorithm == 'aco':
            ants = algo_params.get('num_ants', 25)
            iters = algo_params.get('num_iterations', 80)
            result = algo_aco(self.strips, buffer_polygons, ants, iters)
        else:
            result = algo_dp(self.strips, buffer_polygons)
        
        self.routes = result['orders']
        
        # Generate waypoints
        self._generate_waypoints(altitude, origin_lat, origin_lon)
        
        return {
            'success': True,
            'num_strips': len(self.strips),
            'num_routes': len(self.routes),
            'num_waypoints': len(self.waypoints),
            'strip_width': strip_info['width'],
            'info': result['info']
        }
    
    def _generate_waypoints(self, altitude: float, origin_lat: float, origin_lon: float):
        """Generate waypoints from routes."""
        self.waypoints = []
        
        for route_idx, route in enumerate(self.routes):
            for strip_pos, strip_idx in enumerate(route):
                strip = self.strips[strip_idx]
                points = strip.points(direction_up=(strip_pos % 2 == 0))
                
                for point in points:
                    ll = xy_to_lat_lon(point.x, point.y, origin_lat, origin_lon)
                    self.waypoints.append({
                        'lat': ll['lat'],
                        'lon': ll['lon'],
                        'altitude': altitude,
                        'x': point.x,
                        'y': point.y,
                        'strip_idx': strip_idx,
                        'route_idx': route_idx
                    })
