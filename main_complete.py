#!/usr/bin/env python3
"""UAV Route Planner v3 - Complete Implementation"""

import sys
import math
import random
import numpy as np
from typing import List, Dict, Tuple, Any
from dataclasses import dataclass
from enum import Enum

try:
    from PyQt6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QSplitter,
        QPushButton, QLabel, QSpinBox, QDoubleSpinBox, QComboBox, QListWidget,
        QListWidgetItem, QTabWidget, QTableWidget, QTableWidgetItem, QFrame,
        QGroupBox, QFormLayout, QCheckBox, QProgressBar, QMessageBox, QFileDialog,
        QDialog, QDialogButtonBox
    )
    from PyQt6.QtCore import Qt, QTimer, QThread, pyqtSignal, QSize
    from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QFont, QPolygonF, QPointF
    from PyQt6.QtWidgets import QSizePolicy
except ImportError:
    print("PyQt6 not installed. Install with: pip install PyQt6")
    sys.exit(1)

# ==================== CONSTANTS ====================
EARTH_RADIUS = 6371000
DEFAULT_FIELD = [(54.271751, 48.550465), (54.265295, 48.551538), (54.266601, 48.565957), (54.272982, 48.564756)]
BUFFER_PENALTY = 5.0
NDVI_WIDTH = 180
NDVI_HEIGHT = 180

# ==================== GEOMETRY ====================
class Point:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
    
    def distance_to(self, other: 'Point') -> float:
        return math.hypot(other.x - self.x, other.y - self.y)

def polygon_area(points: List[Point]) -> float:
    if len(points) < 3:
        return 0.0
    area = 0.0
    n = len(points)
    for i in range(n):
        j = (i + 1) % n
        area += points[i].x * points[j].y - points[j].x * points[i].y
    return abs(area) / 2.0

def bounding_box(points: List[Point]) -> Dict[str, float]:
    if not points:
        return {'min_x': 0, 'max_x': 0, 'min_y': 0, 'max_y': 0}
    xs = [p.x for p in points]
    ys = [p.y for p in points]
    return {'min_x': min(xs), 'max_x': max(xs), 'min_y': min(ys), 'max_y': max(ys)}

def point_in_polygon(point: Point, polygon: List[Point]) -> bool:
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

def segment_intersection(p1: Point, p2: Point, p3: Point, p4: Point):
    x1, y1 = p1.x, p1.y
    x2, y2 = p2.x, p2.y
    x3, y3 = p3.x, p3.y
    x4, y4 = p4.x, p4.y
    
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 1e-12:
        return None
    
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom
    
    if 0 <= t <= 1 and 0 <= u <= 1:
        return Point(x1 + t * (x2 - x1), y1 + t * (y2 - y1))
    return None

def circle_buffer(center: Point, radius: float, n: int = 16) -> List[Point]:
    return [Point(center.x + radius * math.cos(2 * math.pi * i / n),
                  center.y + radius * math.sin(2 * math.pi * i / n)) for i in range(n)]

def line_buffer(p1: Point, p2: Point, radius: float, n: int = 8) -> List[Point]:
    dx, dy = p2.x - p1.x, p2.y - p1.y
    length = math.hypot(dx, dy) or 1
    nx, ny = -dy / length * radius, dx / length * radius
    
    points = []
    for i in range(n + 1):
        angle = math.atan2(dy, dx) + math.pi / 2 + math.pi * i / n
        points.append(Point(p1.x + math.cos(angle) * radius, p1.y + math.sin(angle) * radius))
    
    for i in range(n + 1):
        angle = math.atan2(dy, dx) - math.pi / 2 + math.pi * i / n
        points.append(Point(p2.x + math.cos(angle) * radius, p2.y + math.sin(angle) * radius))
    
    return points

def minkowski_expand(polygon: List[Point], radius: float) -> List[Point]:
    if len(polygon) < 3:
        return polygon
    expanded = []
    n = len(polygon)
    for i in range(n):
        prev = polygon[(i - 1) % n]
        cur = polygon[i]
        next_p = polygon[(i + 1) % n]
        
        dx1, dy1 = cur.x - prev.x, cur.y - prev.y
        len1 = math.hypot(dx1, dy1) or 1
        dx2, dy2 = next_p.x - cur.x, next_p.y - cur.y
        len2 = math.hypot(dx2, dy2) or 1
        
        nx1, ny1 = -dy1 / len1, dx1 / len1
        nx2, ny2 = -dy2 / len2, dx2 / len2
        nx, ny = (nx1 + nx2) / 2, (ny1 + ny2) / 2
        nl = math.hypot(nx, ny) or 1
        
        expanded.append(Point(cur.x + (nx / nl) * radius, cur.y + (ny / nl) * radius))
    return expanded

# ==================== GEODESY ====================
def lat_lon_to_xy(lat: float, lon: float, origin_lat: float, origin_lon: float) -> Dict:
    lat_rad = lat * math.pi / 180
    lon_rad = lon * math.pi / 180
    origin_lat_rad = origin_lat * math.pi / 180
    origin_lon_rad = origin_lon * math.pi / 180
    
    x = (lon_rad - origin_lon_rad) * EARTH_RADIUS * math.cos(origin_lat_rad)
    y = (lat_rad - origin_lat_rad) * EARTH_RADIUS
    return {'x': x, 'y': y}

def xy_to_lat_lon(x: float, y: float, origin_lat: float, origin_lon: float) -> Dict:
    origin_lat_rad = origin_lat * math.pi / 180
    origin_lon_rad = origin_lon * math.pi / 180
    
    lat = origin_lat + (y / EARTH_RADIUS) * 180 / math.pi
    lon = origin_lon + (x / (EARTH_RADIUS * math.cos(origin_lat_rad))) * 180 / math.pi
    return {'lat': lat, 'lon': lon}

# ==================== STRIPS ====================
class Strip:
    def __init__(self, x: float):
        self.x = x
        self.segments = []
    
    def add_segment(self, y0: float, y1: float):
        if y1 > y0:
            self.segments.append((y0, y1))
    
    def points(self, direction_up: bool = True) -> List[Point]:
        points = []
        segs = self.segments if direction_up else list(reversed(self.segments))
        for y0, y1 in segs:
            if direction_up:
                points.append(Point(self.x, y0))
                points.append(Point(self.x, y1))
            else:
                points.append(Point(self.x, y1))
                points.append(Point(self.x, y0))
        return points
    
    def length(self) -> float:
        return sum(y1 - y0 for y0, y1 in self.segments)

def calculate_strip_width(altitude: float, fov: float, overlap_side: float) -> Dict:
    fov_rad = fov * math.pi / 180
    width = 2 * altitude * math.tan(fov_rad / 2)
    step = width * (1 - overlap_side)
    return {'width': width, 'step': step}

def clip_strip_to_polygon(x: float, y_min: float, y_max: float, polygon: List[Point]) -> List[Tuple]:
    intersections = []
    p_min = Point(x, y_min - 1)
    p_max = Point(x, y_max + 1)
    
    for i in range(len(polygon)):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % len(polygon)]
        intersection = segment_intersection(p_min, p_max, p1, p2)
        if intersection:
            intersections.append(intersection.y)
    
    if point_in_polygon(Point(x, y_min), polygon):
        intersections.append(y_min)
    if point_in_polygon(Point(x, y_max), polygon):
        intersections.append(y_max)
    
    if len(intersections) < 2:
        return []
    
    intersections.sort()
    unique = [intersections[0]]
    for y in intersections[1:]:
        if abs(y - unique[-1]) > 1e-6:
            unique.append(y)
    
    segments = []
    for i in range(len(unique) - 1):
        y0, y1 = unique[i], unique[i + 1]
        if point_in_polygon(Point(x, (y0 + y1) / 2), polygon):
            segments.append((y0, y1))
    
    return segments

def subtract_buffers_from_segment(x: float, y0: float, y1: float, buffer_polygons: List) -> List:
    if not buffer_polygons:
        return [(y0, y1)] if y1 > y0 else []
    
    cuts = []
    for buffer_poly in buffer_polygons:
        segments = clip_strip_to_polygon(x, y0, y1, buffer_poly)
        cuts.extend(segments)
    
    if not cuts:
        return [(y0, y1)] if y1 > y0 else []
    
    cuts.sort()
    merged = [cuts[0]]
    for cut_y0, cut_y1 in cuts[1:]:
        if cut_y0 <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], cut_y1))
        else:
            merged.append((cut_y0, cut_y1))
    
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

def build_strips(polygon: List[Point], width: float, step: float, buffer_polygons: List = None) -> List[Strip]:
    if not buffer_polygons:
        buffer_polygons = []
    
    bbox = bounding_box(polygon)
    min_x, max_x = bbox['min_x'], bbox['max_x']
    min_y, max_y = bbox['min_y'], bbox['max_y']
    
    strips = []
    x = min_x + width / 2
    
    while x <= max_x + width / 2:
        raw_segments = clip_strip_to_polygon(x, min_y, max_y, polygon)
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

# ==================== ALGORITHMS ====================
def route_length(order: List[int], strips: List[Strip]) -> float:
    if not order or not strips:
        return 0.0
    total = 0.0
    prev_point = None
    for i, strip_idx in enumerate(order):
        points = strips[strip_idx].points(direction_up=(i % 2 == 0))
        if prev_point:
            total += prev_point.distance_to(points[0])
        for j in range(len(points) - 1):
            total += points[j].distance_to(points[j + 1])
        if points:
            prev_point = points[-1]
    return total

def algo_dp(strips: List[Strip], buffer_polygons: List) -> Dict:
    n = len(strips)
    if n <= 1:
        return {'orders': [list(range(n))], 'info': 'Trivial'}
    
    visited = set()
    order = [0]
    visited.add(0)
    
    while len(order) < n:
        order.append(min((j for j in range(n) if j not in visited), 
                        key=lambda j: route_length([*order, j], strips), default=-1))
        if order[-1] >= 0:
            visited.add(order[-1])
        else:
            break
    
    best_length = route_length(order, strips)
    improved = True
    iterations = 0
    
    while improved and iterations < 300:
        improved = False
        iterations += 1
        for i in range(n - 1):
            for j in range(i + 2, n):
                new_order = order[:i+1] + order[i+1:j+1][::-1] + order[j+1:]
                new_length = route_length(new_order, strips)
                if new_length < best_length - 1e-6:
                    order = new_order
                    best_length = new_length
                    improved = True
                    break
            if improved:
                break
    
    return {'orders': [order], 'info': f'DP/TSP 2-opt, {iterations} iterations'}

def algo_vrp(strips: List[Strip], buffer_polygons: List, k: int = 2) -> Dict:
    n = len(strips)
    if n <= 1:
        return {'orders': [list(range(n))], 'info': 'VRP k=1'}
    
    k = min(k, n)
    indices = sorted(range(n), key=lambda i: strips[i].x)
    orders = []
    chunk_size = math.ceil(n / k)
    
    for chunk_idx in range(k):
        chunk = indices[chunk_idx * chunk_size:(chunk_idx + 1) * chunk_size]
        if not chunk:
            continue
        order = [chunk[0]]
        for _ in range(1, len(chunk)):
            next_idx = min((j for j in chunk if j not in order),
                          key=lambda j: route_length([*order, j], strips), default=-1)
            if next_idx >= 0:
                order.append(next_idx)
        orders.append(order)
    
    return {'orders': orders, 'info': f'VRP: {len(orders)} routes'}

def algo_ga(strips: List[Strip], buffer_polygons: List, population: int = 60, generations: int = 150) -> Dict:
    n = len(strips)
    if n <= 1:
        return {'orders': [[0]], 'info': ''}
    
    def random_ind():
        ind = list(range(n))
        random.shuffle(ind)
        return ind
    
    pop = [random_ind() for _ in range(population)]
    best = min(pop, key=lambda x: route_length(x, strips))[:]
    
    for gen in range(generations):
        pop.sort(key=lambda x: route_length(x, strips))
        next_pop = [p[:] for p in pop[:max(2, int(population * 0.1))]]
        
        while len(next_pop) < population:
            p1, p2 = random.choices(pop[:min(15, population)], k=2)
            child = p1[:]
            if random.random() < 0.5:
                i, j = random.randint(0, n-1), random.randint(0, n-1)
                child[i], child[j] = child[j], child[i]
            next_pop.append(child)
        
        pop = next_pop[:population]
        candidate = min(pop, key=lambda x: route_length(x, strips))
        if route_length(candidate, strips) < route_length(best, strips):
            best = candidate[:]
    
    return {'orders': [best], 'info': f'GA: pop={population}, gen={generations}'}

def algo_aco(strips: List[Strip], buffer_polygons: List, num_ants: int = 25, num_iterations: int = 80) -> Dict:
    n = len(strips)
    if n <= 1:
        return {'orders': [[0]], 'info': ''}
    
    pheromones = [[1.0] * n for _ in range(n)]
    best_route = list(range(n))
    best_length = route_length(best_route, strips)
    
    for it in range(num_iterations):
        for ant in range(num_ants):
            visited = [False] * n
            route = [0]
            visited[0] = True
            
            for _ in range(n - 1):
                current = route[-1]
                unvisited = [j for j in range(n) if not visited[j]]
                if not unvisited:
                    break
                
                probs = [(j, pheromones[current][j] ** 1.2 * (1.0 / route_length(route + [j], strips)) ** 2.5) 
                        for j in unvisited]
                total_p = sum(p for _, p in probs)
                
                r = random.random() * total_p
                acc = 0
                for j, p in probs:
                    acc += p
                    if acc >= r:
                        route.append(j)
                        visited[j] = True
                        break
            
            length = route_length(route, strips)
            if length < best_length:
                best_length = length
                best_route = route[:]
        
        for i in range(n):
            for j in range(n):
                pheromones[i][j] *= 0.6
        
        for k in range(len(best_route) - 1):
            pheromones[best_route[k]][best_route[k+1]] += 1000 / best_length
    
    return {'orders': [best_route], 'info': f'ACO: {num_ants} ants, {num_iterations} iter'}

# ==================== VISUALIZATION ====================
def gen_ndvi(w: int = NDVI_WIDTH, h: int = NDVI_HEIGHT) -> np.ndarray:
    ndvi = np.zeros((h, w), dtype=np.float32)
    for r in range(h):
        for c in range(w):
            nx, ny = c / w, r / h
            v = 0.45 + 0.28 * np.sin(nx * 5 + 1.1) * np.cos(ny * 4.5 + 0.7)
            v += 0.12 * np.sin(nx * 11 + ny * 9) - 0.1 * np.power(abs(nx - 0.5), 1.5)
            ndvi[r, c] = np.clip(v, -1, 0.9)
    return ndvi

def gen_dem(w: int = NDVI_WIDTH, h: int = NDVI_HEIGHT) -> np.ndarray:
    dem = np.zeros((h, w), dtype=np.float32)
    for r in range(h):
        for c in range(w):
            nx, ny = c / w, r / h
            dem[r, c] = 120 + 22 * np.sin(nx * 4 + 0.5) * np.cos(ny * 3 + 1) + 9 * np.sin(nx * 8 + ny * 7)
    return dem

def ndvi_to_rgb(v: float) -> Tuple[int, int, int]:
    if np.isnan(v):
        return (200, 195, 185)
    t = np.clip((v + 0.2) / 1.1, 0, 1)
    if t < 0.25:
        f = t * 4
        return (int(95 + 105*f), int(68 + 100*f), 51)
    elif t < 0.5:
        f = (t - 0.25) * 4
        return (int(200 - 80*f), int(168 + 50*f), int(51 + 29*f))
    elif t < 0.75:
        f = (t - 0.5) * 4
        return (int(120 - 52*f), int(218 - 20*f), int(80 - 30*f))
    else:
        f = (t - 0.75) * 4
        return (int(68 - 51*f), int(198 - 96*f), int(50 - 33*f))

def dem_to_rgb(v: float, vmin: float, vmax: float) -> Tuple[int, int, int]:
    t = (v - vmin) / (vmax - vmin + 1e-9) if vmax > vmin else 0
    st = [(40, 80, 40), (60, 120, 50), (120, 160, 60), (170, 150, 90), (200, 185, 155), (230, 225, 210)]
    i = int(np.clip(t * (len(st) - 1), 0, len(st) - 2))
    f = t * (len(st) - 1) - i
    a, b = st[i], st[i + 1]
    return (int(a[0] + (b[0] - a[0]) * f), int(a[1] + (b[1] - a[1]) * f), int(a[2] + (b[2] - a[2]) * f))

# ==================== CANVAS ====================
class MapCanvas(QWidget):
    def __init__(self):
        super().__init__()
        self.field_xy = []
        self.strips = []
        self.routes = []
        self.obstacles = []
        self.ndvi = None
        self.dem = None
        self.show_strips = True
        self.show_route = True
        self.show_buffer = True
        self.show_obs = True
        self.show_ndvi = False
        self.show_dem = False
        
        self.cam_x = 0
        self.cam_y = 0
        self.cam_scale = 1.0
        self.dragging = False
        self.drag_start = (0, 0)
        self.cam_start = (0, 0)
        
        self.setMouseTracking(True)
        self.setStyleSheet('background-color: #f0f0e8;')
    
    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.dragging = True
            self.drag_start = (event.x(), event.y())
            self.cam_start = (self.cam_x, self.cam_y)
    
    def mouseMoveEvent(self, event):
        if self.dragging:
            dx = event.x() - self.drag_start[0]
            dy = event.y() - self.drag_start[1]
            self.cam_x = self.cam_start[0] + dx
            self.cam_y = self.cam_start[1] + dy
            self.update()
    
    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.dragging = False
    
    def wheelEvent(self, event):
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        mx, my = event.x(), event.y()
        self.cam_x = (self.cam_x - mx) * factor + mx
        self.cam_y = (self.cam_y - my) * factor + my
        self.cam_scale *= factor
        self.update()
    
    def w2s(self, x: float, y: float) -> Tuple[int, int]:
        return (int(self.cam_x + x * self.cam_scale), int(self.cam_y - y * self.cam_scale))
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        w, h = self.width(), self.height()
        painter.fillRect(0, 0, w, h, QColor(240, 240, 232))
        
        if not self.field_xy:
            painter.setPen(QPen(QColor(136, 136, 136)))
            painter.setFont(QFont('Arial', 12))
            painter.drawText(w // 2 - 150, h // 2, 300, 30, Qt.AlignmentFlag.AlignCenter, 
                           'Add field points to display map')
            return
        
        # Initialize camera
        if self.cam_scale == 1.0:
            self._init_camera(w, h)
        
        # Draw NDVI background
        if self.show_ndvi and self.ndvi is not None:
            self._draw_ndvi(painter, w, h)
        
        # Draw DEM contours
        if self.show_dem and self.dem is not None:
            self._draw_dem(painter, w, h)
        
        # Draw buffer zones
        if self.show_buffer and self.obstacles:
            self._draw_buffers(painter)
        
        # Draw strips
        if self.show_strips and self.strips:
            self._draw_strips(painter)
        
        # Draw route
        if self.show_route and self.routes:
            self._draw_route(painter)
        
        # Draw obstacles
        if self.show_obs and self.obstacles:
            self._draw_obstacles(painter)
        
        # Draw field
        self._draw_field(painter)
    
    def _init_camera(self, w: int, h: int):
        if not self.field_xy:
            return
        xs = [p.x for p in self.field_xy]
        ys = [p.y for p in self.field_xy]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        rx, ry = (max_x - min_x) or 200, (max_y - min_y) or 200
        scale = min((w - 80) / rx, (h - 80) / ry)
        self.cam_scale = scale
        self.cam_x = w / 2 - (min_x + rx / 2) * scale
        self.cam_y = h / 2 + (min_y + ry / 2) * scale
    
    def _draw_field(self, painter: QPainter):
        if len(self.field_xy) >= 2:
            painter.setPen(QPen(QColor(0, 0, 139), 2))
            points = [QPointF(*self.w2s(p.x, p.y)) for p in self.field_xy]
            if len(self.field_xy) >= 3:
                painter.drawPolygon(QPolygonF(points))
            else:
                for i in range(len(points) - 1):
                    painter.drawLine(points[i], points[i + 1])
            
            painter.setBrush(QBrush(QColor(0, 170, 0)))
            for i, p in enumerate(self.field_xy):
                sx, sy = self.w2s(p.x, p.y)
                painter.drawEllipse(sx - 4, sy - 4, 8, 8)
                painter.setPen(QPen(QColor(0, 0, 0)))
                painter.setFont(QFont('Arial', 9))
                painter.drawText(sx + 8, sy - 3, f'P{i+1}')
    
    def _draw_strips(self, painter: QPainter):
        painter.setPen(QPen(QColor(100, 150, 200), 1))
        for strip in self.strips:
            for y0, y1 in strip.segments:
                sx0, sy0 = self.w2s(strip.x, y0)
                sx1, sy1 = self.w2s(strip.x, y1)
                painter.drawLine(sx0, sy0, sx1, sy1)
    
    def _draw_route(self, painter: QPainter):
        colors = [QColor(0, 0, 200), QColor(200, 0, 0), QColor(0, 150, 0), QColor(200, 100, 0)]
        for route_idx, route in enumerate(self.routes):
            color = colors[route_idx % len(colors)]
            painter.setPen(QPen(color, 2))
            
            prev_pt = None
            for i, strip_idx in enumerate(route):
                strip = self.strips[strip_idx]
                points = strip.points(direction_up=(i % 2 == 0))
                
                for pt in points:
                    sx, sy = self.w2s(pt.x, pt.y)
                    if prev_pt:
                        painter.drawLine(prev_pt, QPointF(sx, sy))
                    painter.drawEllipse(sx - 2, sy - 2, 4, 4)
                    prev_pt = QPointF(sx, sy)
    
    def _draw_obstacles(self, painter: QPainter):
        for obs in self.obstacles:
            if obs['type'] == 'tree':
                sx, sy = self.w2s(obs['points'][0].x, obs['points'][0].y)
                painter.setBrush(QBrush(QColor(0, 140, 0)))
                painter.drawEllipse(sx - 5, sy - 5, 10, 10)
            elif obs['type'] == 'power' and len(obs['points']) >= 2:
                sx1, sy1 = self.w2s(obs['points'][0].x, obs['points'][0].y)
                sx2, sy2 = self.w2s(obs['points'][1].x, obs['points'][1].y)
                painter.setPen(QPen(QColor(180, 100, 0), 2))
                painter.drawLine(sx1, sy1, sx2, sy2)
            elif obs['type'] in ['nfz', 'building']:
                painter.setPen(QPen(QColor(220, 0, 0), 1))
                points = [QPointF(*self.w2s(p.x, p.y)) for p in obs['points']]
                painter.drawPolygon(QPolygonF(points))
    
    def _draw_buffers(self, painter: QPainter):
        for obs in self.obstacles:
            if 'buffer' in obs:
                painter.setPen(QPen(QColor(200, 0, 0), 1))
                points = [QPointF(*self.w2s(p.x, p.y)) for p in obs['buffer']]
                if len(points) > 2:
                    painter.drawPolygon(QPolygonF(points))
    
    def _draw_ndvi(self, painter: QPainter, w: int, h: int):
        if self.ndvi is None:
            return
        bb = bounding_box(self.field_xy)
        min_x, max_x = bb['min_x'], bb['max_x']
        min_y, max_y = bb['min_y'], bb['max_y']
        rx, ry = (max_x - min_x) or 1, (max_y - min_y) or 1
        
        for r in range(0, NDVI_HEIGHT, 2):
            for c in range(0, NDVI_WIDTH, 2):
                wx = min_x + (c / NDVI_WIDTH) * rx
                wy = max_y - (r / NDVI_HEIGHT) * ry
                if point_in_polygon(Point(wx, wy), self.field_xy):
                    rgb = ndvi_to_rgb(self.ndvi[r, c])
                    sx, sy = self.w2s(wx, wy)
                    painter.fillRect(int(sx), int(sy), max(1, int(2 * self.cam_scale)), 
                                    max(1, int(2 * self.cam_scale)), 
                                    QColor(rgb[0], rgb[1], rgb[2], 180))
    
    def _draw_dem(self, painter: QPainter, w: int, h: int):
        if self.dem is None:
            return
        vmin, vmax = self.dem.min(), self.dem.max()
        bb = bounding_box(self.field_xy)
        min_x, max_x = bb['min_x'], bb['max_x']
        min_y, max_y = bb['min_y'], bb['max_y']
        rx, ry = (max_x - min_x) or 1, (max_y - min_y) or 1
        
        for r in range(1, NDVI_HEIGHT - 1, 2):
            for c in range(NDVI_WIDTH - 1):
                wx = min_x + (c / NDVI_WIDTH) * rx
                v1 = self.dem[r, c]
                v2 = self.dem[r, c + 1]
                for lv in range(1, 8):
                    thresh = vmin + (vmax - vmin) * lv / 8
                    if (v1 - thresh) * (v2 - thresh) <= 0:
                        t = (thresh - v1) / (v2 - v1 + 1e-9)
                        wx_int = wx + (min(c + 1, NDVI_WIDTH - 1) - c) / NDVI_WIDTH * rx * t
                        wy_int = max_y - (r / NDVI_HEIGHT) * ry
                        sx, sy = self.w2s(wx_int, wy_int)
                        painter.setPen(QPen(QColor(255, 255, 255, 100), 0.5))
                        painter.drawLine(int(sx), int(sy - 2), int(sx), int(sy + 2))

# ==================== MAIN WINDOW ====================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('UAV Route Planner v3')
        self.setGeometry(100, 100, 1400, 800)
        
        self.field_pts = []
        self.field_xy = []
        self.obstacles = []
        self.strips = []
        self.routes = []
        self.waypoints = []
        self.ndvi = gen_ndvi()
        self.dem = gen_dem()
        self.selected_pt_idx = -1
        self.selected_obs_idx = -1
        
        self._setup_ui()
        self._load_default_field()
    
    def _setup_ui(self):
        central = QWidget()
        main_layout = QHBoxLayout()
        
        # LEFT PANEL
        left_layout = QVBoxLayout()
        left_layout.setContentsMargins(5, 5, 5, 5)
        left_layout.setSpacing(8)
        
        # Field section
        field_group = QGroupBox('Field Points (lat, lon)')
        field_layout = QVBoxLayout()
        self.field_list = QListWidget()
        self.field_list.itemSelectionChanged.connect(self._on_field_selection)
        field_layout.addWidget(self.field_list)
        
        field_btn_layout = QHBoxLayout()
        add_btn = QPushButton('Load Default')
        add_btn.clicked.connect(self._load_default_field)
        clear_btn = QPushButton('Clear')
        clear_btn.clicked.connect(self._clear_field)
        field_btn_layout.addWidget(add_btn)
        field_btn_layout.addWidget(clear_btn)
        field_layout.addLayout(field_btn_layout)
        field_group.setLayout(field_layout)
        left_layout.addWidget(field_group)
        
        # Obstacles section
        obs_group = QGroupBox('Obstacles')
        obs_layout = QVBoxLayout()
        self.obs_list = QListWidget()
        obs_layout.addWidget(self.obs_list)
        
        obs_type_layout = QHBoxLayout()
        self.obs_type_combo = QComboBox()
        self.obs_type_combo.addItems(['🚫 No-Fly Zone', '🌳 Tree', '⚡ Power Line', '🏢 Building'])
        obs_type_layout.addWidget(QLabel('Type:'))
        obs_type_layout.addWidget(self.obs_type_combo)
        obs_layout.addLayout(obs_type_layout)
        
        radius_layout = QHBoxLayout()
        self.radius_spin = QSpinBox()
        self.radius_spin.setValue(30)
        self.radius_spin.setRange(5, 500)
        radius_layout.addWidget(QLabel('Radius (m):'))
        radius_layout.addWidget(self.radius_spin)
        obs_layout.addLayout(radius_layout)
        
        obs_btn_layout = QHBoxLayout()
        add_obs_btn = QPushButton('+ Add')
        del_obs_btn = QPushButton('Delete')
        clear_obs_btn = QPushButton('Clear All')
        add_obs_btn.clicked.connect(self._add_obstacle)
        del_obs_btn.clicked.connect(self._delete_obstacle)
        clear_obs_btn.clicked.connect(self._clear_obstacles)
        obs_btn_layout.addWidget(add_obs_btn)
        obs_btn_layout.addWidget(del_obs_btn)
        obs_btn_layout.addWidget(clear_obs_btn)
        obs_layout.addLayout(obs_btn_layout)
        obs_group.setLayout(obs_layout)
        left_layout.addWidget(obs_group)
        
        # UAV Parameters
        uav_group = QGroupBox('UAV Parameters')
        uav_layout = QFormLayout()
        
        self.alt_spin = QDoubleSpinBox()
        self.alt_spin.setValue(80)
        self.alt_spin.setRange(10, 500)
        uav_layout.addRow('Altitude (m):', self.alt_spin)
        
        self.spd_spin = QDoubleSpinBox()
        self.spd_spin.setValue(12)
        self.spd_spin.setRange(1, 50)
        uav_layout.addRow('Speed (m/s):', self.spd_spin)
        
        self.fov_spin = QDoubleSpinBox()
        self.fov_spin.setValue(82)
        self.fov_spin.setRange(20, 120)
        uav_layout.addRow('FOV (°):', self.fov_spin)
        
        self.bat_spin = QDoubleSpinBox()
        self.bat_spin.setValue(45)
        self.bat_spin.setRange(5, 200)
        uav_layout.addRow('Battery (min):', self.bat_spin)
        
        self.ol_f_spin = QDoubleSpinBox()
        self.ol_f_spin.setValue(0.75)
        self.ol_f_spin.setRange(0.5, 0.9)
        self.ol_f_spin.setSingleStep(0.05)
        uav_layout.addRow('Overlap Forward:', self.ol_f_spin)
        
        self.ol_s_spin = QDoubleSpinBox()
        self.ol_s_spin.setValue(0.65)
        self.ol_s_spin.setRange(0.4, 0.85)
        self.ol_s_spin.setSingleStep(0.05)
        uav_layout.addRow('Overlap Side:', self.ol_s_spin)
        
        uav_group.setLayout(uav_layout)
        left_layout.addWidget(uav_group)
        
        # Algorithm
        algo_group = QGroupBox('Optimization')
        algo_layout = QVBoxLayout()
        
        self.algo_combo = QComboBox()
        self.algo_combo.addItems(['DP (TSP)', 'VRP', 'GA', 'ACO'])
        algo_layout.addWidget(QLabel('Algorithm:'))
        algo_layout.addWidget(self.algo_combo)
        
        self.ga_pop_spin = QSpinBox()
        self.ga_pop_spin.setValue(60)
        self.ga_pop_spin.setRange(20, 200)
        algo_layout.addWidget(QLabel('GA Population:'))
        algo_layout.addWidget(self.ga_pop_spin)
        
        self.ga_gen_spin = QSpinBox()
        self.ga_gen_spin.setValue(150)
        self.ga_gen_spin.setRange(50, 500)
        algo_layout.addWidget(QLabel('GA Generations:'))
        algo_layout.addWidget(self.ga_gen_spin)
        
        algo_group.setLayout(algo_layout)
        left_layout.addWidget(algo_group)
        
        # Buttons
        btn_layout = QHBoxLayout()
        calc_btn = QPushButton('▶ Calculate')
        calc_btn.clicked.connect(self._calculate)
        calc_btn.setStyleSheet('background-color: #4CAF50; color: white; font-weight: bold;')
        cmp_btn = QPushButton('⚖ Compare All')
        cmp_btn.clicked.connect(self._compare_all)
        btn_layout.addWidget(calc_btn)
        btn_layout.addWidget(cmp_btn)
        left_layout.addLayout(btn_layout)
        
        # Results
        res_group = QGroupBox('Results')
        res_layout = QVBoxLayout()
        self.res_table = QTableWidget()
        self.res_table.setColumnCount(2)
        self.res_table.setHorizontalHeaderLabels(['Parameter', 'Value'])
        self.res_table.setColumnWidth(0, 120)
        self.res_table.setColumnWidth(1, 100)
        
        rows = ['Area', 'Strips', 'Strip Width', 'Distance', 'Time', 'Battery %', 'Flights', 'Calc Time']
        self.res_table.setRowCount(len(rows))
        for i, row in enumerate(rows):
            self.res_table.setItem(i, 0, QTableWidgetItem(row))
            self.res_table.setItem(i, 1, QTableWidgetItem('—'))
        
        res_layout.addWidget(self.res_table)
        res_group.setLayout(res_layout)
        left_layout.addWidget(res_group)
        
        left_layout.addStretch()
        
        left_widget = QWidget()
        left_widget.setLayout(left_layout)
        left_widget.setMaximumWidth(300)
        
        # RIGHT PANEL - TABS
        self.tabs = QTabWidget()
        
        # Map tab
        self.map_canvas = MapCanvas()
        self.tabs.addTab(self.map_canvas, '🗺 Map')
        
        # Waypoints tab
        wp_widget = QWidget()
        wp_layout = QVBoxLayout()
        export_btn = QPushButton('💾 Export CSV')
        export_btn.clicked.connect(self._export_csv)
        wp_layout.addWidget(export_btn)
        self.wp_table = QTableWidget()
        self.wp_table.setColumnCount(8)
        self.wp_table.setHorizontalHeaderLabels(['#', 'Lat', 'Lon', 'Alt', 'X', 'Y', 'Action', 'Route'])
        wp_layout.addWidget(self.wp_table)
        wp_widget.setLayout(wp_layout)
        self.tabs.addTab(wp_widget, '📋 Waypoints')
        
        # Compare tab
        cmp_widget = QWidget()
        cmp_layout = QVBoxLayout()
        self.cmp_table = QTableWidget()
        self.cmp_table.setColumnCount(7)
        self.cmp_table.setHorizontalHeaderLabels(['Algorithm', 'Distance', 'Time', 'Battery %', 'Flights', 'Calc Time', 'Info'])
        cmp_layout.addWidget(self.cmp_table)
        cmp_widget.setLayout(cmp_layout)
        self.tabs.addTab(cmp_widget, '⚖ Compare')
        
        # NDVI tab
        ndvi_widget = QWidget()
        ndvi_layout = QVBoxLayout()
        self.ndvi_canvas = MapCanvas()
        ndvi_layout.addWidget(self.ndvi_canvas)
        ndvi_widget.setLayout(ndvi_layout)
        self.tabs.addTab(ndvi_widget, '🌿 NDVI')
        
        # DEM tab
        dem_widget = QWidget()
        dem_layout = QVBoxLayout()
        self.dem_canvas = MapCanvas()
        dem_layout.addWidget(self.dem_canvas)
        dem_widget.setLayout(dem_layout)
        self.tabs.addTab(dem_widget, '⛰ DEM')
        
        # Assemble
        main_layout.addWidget(left_widget, 0)
        main_layout.addWidget(self.tabs, 1)
        
        central.setLayout(main_layout)
        self.setCentralWidget(central)
    
    def _load_default_field(self):
        self.field_pts = [Point(lat, lon) for lat, lon in DEFAULT_FIELD]
        origin = self.field_pts[0]
        self.field_xy = [Point(lat_lon_to_xy(p.x, p.y, origin.x, origin.y)['x'],
                              lat_lon_to_xy(p.x, p.y, origin.x, origin.y)['y']) for p in self.field_pts]
        self._update_field_list()
        self.map_canvas.field_xy = self.field_xy
        self.map_canvas.update()
    
    def _clear_field(self):
        self.field_pts = []
        self.field_xy = []
        self.field_list.clear()
        self.map_canvas.field_xy = []
        self.map_canvas.update()
    
    def _update_field_list(self):
        self.field_list.clear()
        for i, pt in enumerate(self.field_pts):
            item = QListWidgetItem(f'P{i+1}: ({pt.x:.6f}, {pt.y:.6f})')
            self.field_list.addItem(item)
    
    def _on_field_selection(self):
        items = self.field_list.selectedItems()
        if items:
            self.selected_pt_idx = self.field_list.row(items[0])
    
    def _add_obstacle(self):
        if not self.field_pts:
            QMessageBox.warning(self, 'Error', 'Add field first')
            return
        
        # Dummy obstacle for demo
        type_idx = self.obs_type_combo.currentIndex()
        obs_types = ['nfz', 'tree', 'power', 'building']
        radius = self.radius_spin.value()
        
        obs = {
            'type': obs_types[type_idx],
            'points': [Point(self.field_xy[0].x + random.randint(-50, 50), 
                           self.field_xy[0].y + random.randint(-50, 50))],
            'radius': radius,
            'buffer': []
        }
        
        self.obstacles.append(obs)
        self._update_obs_list()
    
    def _delete_obstacle(self):
        if 0 <= self.selected_obs_idx < len(self.obstacles):
            del self.obstacles[self.selected_obs_idx]
            self._update_obs_list()
    
    def _clear_obstacles(self):
        self.obstacles = []
        self.obs_list.clear()
    
    def _update_obs_list(self):
        self.obs_list.clear()
        for i, obs in enumerate(self.obstacles):
            item = QListWidgetItem(f'Obs{i+1} [{obs["type"]}] r={obs["radius"]}m')
            self.obs_list.addItem(item)
    
    def _calculate(self):
        if len(self.field_xy) < 3:
            QMessageBox.warning(self, 'Error', 'Need at least 3 field points')
            return
        
        import time
        t0 = time.time()
        
        # Build obstacles with buffers
        for obs in self.obstacles:
            if obs['type'] == 'tree':
                obs['buffer'] = circle_buffer(obs['points'][0], obs['radius'])
            elif obs['type'] == 'power' and len(obs['points']) >= 2:
                obs['buffer'] = line_buffer(obs['points'][0], obs['points'][1], obs['radius'])
            elif obs['type'] in ['nfz', 'building']:
                obs['buffer'] = minkowski_expand(obs['points'], obs['radius'])
        
        buffer_polys = [obs['buffer'] for obs in self.obstacles if obs['buffer']]
        
        # Calculate strips
        strip_info = calculate_strip_width(self.alt_spin.value(), self.fov_spin.value(), self.ol_s_spin.value())
        self.strips = build_strips(self.field_xy, strip_info['width'], strip_info['step'], buffer_polys)
        
        if not self.strips:
            QMessageBox.warning(self, 'Error', 'Cannot build strips')
            return
        
        # Run algorithm
        algo = self.algo_combo.currentText().split()[0].lower()
        if algo == 'dp':
            result = algo_dp(self.strips, buffer_polys)
        elif algo == 'vrp':
            result = algo_vrp(self.strips, buffer_polys, 2)
        elif algo == 'ga':
            result = algo_ga(self.strips, buffer_polys, self.ga_pop_spin.value(), self.ga_gen_spin.value())
        elif algo == 'aco':
            result = algo_aco(self.strips, buffer_polys, 25, 80)
        else:
            result = algo_dp(self.strips, buffer_polys)
        
        self.routes = result['orders']
        elapsed = time.time() - t0
        
        # Generate waypoints
        self.waypoints = []
        origin = self.field_pts[0]
        for route_idx, route in enumerate(self.routes):
            for strip_pos, strip_idx in enumerate(route):
                points = self.strips[strip_idx].points(direction_up=(strip_pos % 2 == 0))
                for pt in points:
                    ll = xy_to_lat_lon(pt.x, pt.y, origin.x, origin.y)
                    self.waypoints.append({
                        'lat': ll['lat'], 'lon': ll['lon'], 'alt': int(self.alt_spin.value()),
                        'x': pt.x, 'y': pt.y, 'route': route_idx, 'strip': strip_idx
                    })
        
        # Update results
        area = polygon_area(self.field_xy) / 10000
        total_dist = sum(route_length(route, self.strips) for route in self.routes)
        flight_time = total_dist / self.spd_spin.value() / 60
        bat_use = flight_time / self.bat_spin.value() * 100
        flights = max(1, int(bat_use / 85 + 1))
        
        rows = [
            ('Area', f'{area:.2f} ha'),
            ('Strips', str(len(self.strips))),
            ('Strip Width', f'{strip_info["width"]:.1f} m'),
            ('Distance', f'{int(total_dist)} m'),
            ('Time', f'{flight_time:.1f} min'),
            ('Battery %', f'{int(bat_use)}%'),
            ('Flights', str(flights)),
            ('Calc Time', f'{elapsed:.3f} s')
        ]
        
        for i, (param, val) in enumerate(rows):
            self.res_table.setItem(i, 1, QTableWidgetItem(val))
        
        # Update canvas
        self.map_canvas.field_xy = self.field_xy
        self.map_canvas.strips = self.strips
        self.map_canvas.routes = self.routes
        self.map_canvas.obstacles = self.obstacles
        self.map_canvas.ndvi = self.ndvi
        self.map_canvas.dem = self.dem
        self.map_canvas.show_strips = True
        self.map_canvas.show_route = True
        self.map_canvas.update()
        
        # Update waypoints table
        self._update_waypoints_table()
        
        # Update NDVI/DEM canvas
        self.ndvi_canvas.field_xy = self.field_xy
        self.ndvi_canvas.ndvi = self.ndvi
        self.ndvi_canvas.show_ndvi = True
        self.ndvi_canvas.update()
        
        self.dem_canvas.field_xy = self.field_xy
        self.dem_canvas.dem = self.dem
        self.dem_canvas.show_dem = True
        self.dem_canvas.update()
        
        self.tabs.setCurrentIndex(0)
    
    def _update_waypoints_table(self):
        self.wp_table.setRowCount(len(self.waypoints))
        for i, wp in enumerate(self.waypoints):
            self.wp_table.setItem(i, 0, QTableWidgetItem(str(i + 1)))
            self.wp_table.setItem(i, 1, QTableWidgetItem(f"{wp['lat']:.7f}"))
            self.wp_table.setItem(i, 2, QTableWidgetItem(f"{wp['lon']:.7f}"))
            self.wp_table.setItem(i, 3, QTableWidgetItem(str(wp['alt'])))
            self.wp_table.setItem(i, 4, QTableWidgetItem(f"{wp['x']:.1f}"))
            self.wp_table.setItem(i, 5, QTableWidgetItem(f"{wp['y']:.1f}"))
            self.wp_table.setItem(i, 6, QTableWidgetItem('PHOTO' if (i % 2 == 0) else 'TURN'))
            self.wp_table.setItem(i, 7, QTableWidgetItem(f"M{wp['route'] + 1}"))
    
    def _compare_all(self):
        if len(self.field_xy) < 3:
            QMessageBox.warning(self, 'Error', 'Need at least 3 field points')
            return
        
        import time
        
        # Build obstacles
        for obs in self.obstacles:
            if obs['type'] == 'tree':
                obs['buffer'] = circle_buffer(obs['points'][0], obs['radius'])
            elif obs['type'] == 'power' and len(obs['points']) >= 2:
                obs['buffer'] = line_buffer(obs['points'][0], obs['points'][1], obs['radius'])
            elif obs['type'] in ['nfz', 'building']:
                obs['buffer'] = minkowski_expand(obs['points'], obs['radius'])
        
        buffer_polys = [obs['buffer'] for obs in self.obstacles if obs['buffer']]
        
        # Calculate strips
        strip_info = calculate_strip_width(self.alt_spin.value(), self.fov_spin.value(), self.ol_s_spin.value())
        strips = build_strips(self.field_xy, strip_info['width'], strip_info['step'], buffer_polys)
        
        if not strips:
            QMessageBox.warning(self, 'Error', 'Cannot build strips')
            return
        
        results = []
        for algo_name, algo_func, params in [
            ('DP', algo_dp, (strips, buffer_polys)),
            ('VRP', algo_vrp, (strips, buffer_polys, 2)),
            ('GA', algo_ga, (strips, buffer_polys, self.ga_pop_spin.value(), self.ga_gen_spin.value())),
            ('ACO', algo_aco, (strips, buffer_polys, 25, 80))
        ]:
            t0 = time.time()
            result = algo_func(*params)
            elapsed = time.time() - t0
            
            total_dist = sum(route_length(route, strips) for route in result['orders'])
            flight_time = total_dist / self.spd_spin.value() / 60
            bat_use = flight_time / self.bat_spin.value() * 100
            
            results.append((algo_name, int(total_dist), flight_time, int(bat_use), elapsed))
        
        self.cmp_table.setRowCount(len(results))
        for i, (algo, dist, time_min, bat, calc_time) in enumerate(results):
            self.cmp_table.setItem(i, 0, QTableWidgetItem(algo))
            self.cmp_table.setItem(i, 1, QTableWidgetItem(str(dist)))
            self.cmp_table.setItem(i, 2, QTableWidgetItem(f'{time_min:.1f}'))
            self.cmp_table.setItem(i, 3, QTableWidgetItem(str(bat)))
            self.cmp_table.setItem(i, 4, QTableWidgetItem(str(max(1, int(bat / 85)))))
            self.cmp_table.setItem(i, 5, QTableWidgetItem(f'{calc_time:.3f}'))
        
        self.tabs.setCurrentIndex(2)
    
    def _export_csv(self):
        if not self.waypoints:
            QMessageBox.warning(self, 'Warning', 'No waypoints')
            return
        
        filename, _ = QFileDialog.getSaveFileName(self, 'Save CSV', 'waypoints.csv', 'CSV Files (*.csv)')
        if not filename:
            return
        
        with open(filename, 'w', newline='', encoding='utf-8-sig') as f:
            f.write('#,Lat,Lon,Alt,X,Y,Action,Route\n')
            for i, wp in enumerate(self.waypoints):
                f.write(f"{i+1},{wp['lat']:.7f},{wp['lon']:.7f},{wp['alt']},{wp['x']:.1f},{wp['y']:.1f},PHOTO,M{wp['route']+1}\n")
        
        QMessageBox.information(self, 'Success', f'Exported to {filename}')

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
