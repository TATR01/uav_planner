"""Map canvas for visualization"""

from PyQt6.QtWidgets import QWidget
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QFont
from PyQt6.QtCore import Qt, QPoint


class MapCanvas(QWidget):
    """Canvas for drawing map and route"""
    
    def __init__(self, calculator):
        super().__init__()
        self.calculator = calculator
        self.setStyleSheet('background-color: white;')
    
    def paintEvent(self, event):
        """Paint the canvas."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        # Draw background
        painter.fillRect(self.rect(), QColor(240, 240, 240))
        
        if not self.calculator.field_xy:
            font = QFont('Arial', 12)
            painter.setFont(font)
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, 'Load field data to display map')
            return
        
        # Calculate viewport
        width = self.width()
        height = self.height()
        margin = 50
        
        # Get bounds
        xs = [p.x for p in self.calculator.field_xy]
        ys = [p.y for p in self.calculator.field_xy]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        
        range_x = max_x - min_x or 1
        range_y = max_y - min_y or 1
        
        scale_x = (width - 2 * margin) / range_x
        scale_y = (height - 2 * margin) / range_y
        scale = min(scale_x, scale_y)
        
        offset_x = margin - min_x * scale
        offset_y = margin + max_y * scale
        
        def world_to_screen(x, y):
            sx = x * scale + offset_x
            sy = offset_y - y * scale
            return int(sx), int(sy)
        
        # Draw field polygon
        if len(self.calculator.field_xy) >= 3:
            painter.setPen(QPen(QColor(0, 0, 139), 2))
            points = [QPoint(*world_to_screen(p.x, p.y)) for p in self.calculator.field_xy]
            painter.drawPolygon(points)
            
            # Draw field points
            painter.setBrush(QBrush(QColor(0, 0, 255)))
            for i, p in enumerate(self.calculator.field_xy):
                sx, sy = world_to_screen(p.x, p.y)
                painter.drawEllipse(sx - 4, sy - 4, 8, 8)
                
                # Draw label
                painter.setPen(QPen(QColor(0, 0, 0)))
                painter.drawText(sx + 8, sy - 2, f'P{i+1}')
        
        # Draw strips
        painter.setPen(QPen(QColor(100, 150, 200), 0.5))
        for strip in self.calculator.strips:
            for y0, y1 in strip.segments:
                p1 = QPoint(*world_to_screen(strip.x, y0))
                p2 = QPoint(*world_to_screen(strip.x, y1))
                painter.drawLine(p1, p2)
        
        # Draw routes
        colors = [QColor(0, 0, 200), QColor(200, 0, 0), QColor(0, 150, 0)]
        for route_idx, route in enumerate(self.calculator.routes):
            color = colors[route_idx % len(colors)]
            painter.setPen(QPen(color, 2))
            
            prev_point = None
            for strip_idx in route:
                strip = self.calculator.strips[strip_idx]
                for y0, y1 in strip.segments:
                    p1 = QPoint(*world_to_screen(strip.x, y0))
                    p2 = QPoint(*world_to_screen(strip.x, y1))
                    
                    if prev_point:
                        painter.drawLine(prev_point, p1)
                    
                    painter.drawLine(p1, p2)
                    prev_point = p2
