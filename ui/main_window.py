"""Main application window"""

from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QPushButton, QLabel,
    QSpinBox, QDoubleSpinBox, QComboBox, QListWidget, QListWidgetItem,
    QTabWidget, QTableWidget, QTableWidgetItem, QProgressDialog, QMessageBox
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QIcon
import time

from config.settings import WINDOW_WIDTH, WINDOW_HEIGHT, DEFAULT_FIELD, DEFAULT_ALTITUDE, DEFAULT_SPEED
from core.route import RouteCalculator
from ui.canvas import MapCanvas
from utils.export import export_waypoints_csv


class MainWindow(QMainWindow):
    """Main application window"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle('UAV Route Planner v3')
        self.setGeometry(100, 100, WINDOW_WIDTH, WINDOW_HEIGHT)
        
        self.calculator = RouteCalculator()
        self.progress_dialog = None
        
        # Create main layout
        main_widget = QWidget()
        main_layout = QHBoxLayout()
        
        # Left panel
        left_layout = QVBoxLayout()
        
        # Field points section
        left_layout.addWidget(QLabel('Field Points (lat, lon)'))
        self.field_list = QListWidget()
        left_layout.addWidget(self.field_list)
        
        add_field_btn = QPushButton('Add Default Field')
        add_field_btn.clicked.connect(self.load_default_field)
        left_layout.addWidget(add_field_btn)
        
        # Parameters section
        left_layout.addWidget(QLabel('\nUAV Parameters'))
        
        self.altitude_spin = QDoubleSpinBox()
        self.altitude_spin.setValue(DEFAULT_ALTITUDE)
        self.altitude_spin.setRange(10, 500)
        left_layout.addWidget(QLabel('Altitude (m):'))
        left_layout.addWidget(self.altitude_spin)
        
        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setValue(DEFAULT_SPEED)
        self.speed_spin.setRange(1, 50)
        left_layout.addWidget(QLabel('Speed (m/s):'))
        left_layout.addWidget(self.speed_spin)
        
        # Algorithm selection
        left_layout.addWidget(QLabel('\nAlgorithm'))
        self.algo_combo = QComboBox()
        self.algo_combo.addItems(['DP (TSP)', 'VRP', 'GA', 'ACO'])
        left_layout.addWidget(self.algo_combo)
        
        # Calculate button
        calc_btn = QPushButton('Calculate Route')
        calc_btn.clicked.connect(self.calculate_route)
        left_layout.addWidget(calc_btn)
        
        # Results section
        left_layout.addWidget(QLabel('\nResults'))
        self.results_label = QLabel('No route calculated')
        left_layout.addWidget(self.results_label)
        
        left_layout.addStretch()
        
        left_widget = QWidget()
        left_widget.setLayout(left_layout)
        left_widget.setMaximumWidth(280)
        
        # Right panel - tabs
        self.tabs = QTabWidget()
        
        # Map tab
        self.map_canvas = MapCanvas(self.calculator)
        self.tabs.addTab(self.map_canvas, 'Map')
        
        # Waypoints tab
        self.waypoints_table = QTableWidget()
        self.waypoints_table.setColumnCount(6)
        self.waypoints_table.setHorizontalHeaderLabels(['#', 'Lat', 'Lon', 'Alt', 'X (m)', 'Y (m)'])
        self.tabs.addTab(self.waypoints_table, 'Waypoints')
        
        export_btn = QPushButton('Export CSV')
        export_btn.clicked.connect(self.export_waypoints)
        wp_layout = QVBoxLayout()
        wp_layout.addWidget(export_btn)
        wp_layout.addWidget(self.waypoints_table)
        wp_widget = QWidget()
        wp_widget.setLayout(wp_layout)
        self.tabs.addTab(wp_widget, 'Waypoints')
        
        # Assemble main layout
        main_layout.addWidget(left_widget, 0)
        main_layout.addWidget(self.tabs, 1)
        
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)
    
    def load_default_field(self):
        """Load default field."""
        self.calculator.set_field(DEFAULT_FIELD)
        self.update_field_list()
        self.map_canvas.update()
    
    def update_field_list(self):
        """Update field points list."""
        self.field_list.clear()
        for i, pt in enumerate(self.calculator.field_points):
            item = QListWidgetItem(f'P{i+1}: ({pt.x:.6f}, {pt.y:.6f})')
            self.field_list.addItem(item)
    
    def calculate_route(self):
        """Calculate route."""
        if len(self.calculator.field_points) < 3:
            QMessageBox.warning(self, 'Error', 'Need at least 3 field points')
            return
        
        # Get parameters
        altitude = self.altitude_spin.value()
        speed = self.speed_spin.value()
        algo = self.algo_combo.currentText().split()[0].lower()
        
        # Show progress
        self.progress_dialog = QProgressDialog('Calculating route...', None, 0, 100, self)
        self.progress_dialog.setWindowModality(Qt.WindowModality.WindowModal)
        self.progress_dialog.show()
        
        # Calculate
        start_time = time.time()
        result = self.calculator.calculate_route(
            altitude=altitude,
            speed=speed,
            fov=82,
            battery=45,
            overlap_forward=0.75,
            overlap_side=0.65,
            algorithm=algo
        )
        elapsed = time.time() - start_time
        
        if self.progress_dialog:
            self.progress_dialog.close()
        
        if 'error' in result:
            QMessageBox.critical(self, 'Error', result['error'])
            return
        
        # Update results
        info = f"Strips: {result['num_strips']}\nRoutes: {result['num_routes']}\nWaypoints: {result['num_waypoints']}\nTime: {elapsed:.2f}s"
        self.results_label.setText(info)
        
        # Update waypoints table
        self.update_waypoints_table()
        
        # Update map
        self.map_canvas.update()
    
    def update_waypoints_table(self):
        """Update waypoints table."""
        self.waypoints_table.setRowCount(len(self.calculator.waypoints))
        
        for row, wp in enumerate(self.calculator.waypoints):
            self.waypoints_table.setItem(row, 0, QTableWidgetItem(str(row + 1)))
            self.waypoints_table.setItem(row, 1, QTableWidgetItem(f"{wp['lat']:.7f}"))
            self.waypoints_table.setItem(row, 2, QTableWidgetItem(f"{wp['lon']:.7f}"))
            self.waypoints_table.setItem(row, 3, QTableWidgetItem(str(wp['altitude'])))
            self.waypoints_table.setItem(row, 4, QTableWidgetItem(f"{wp['x']:.2f}"))
            self.waypoints_table.setItem(row, 5, QTableWidgetItem(f"{wp['y']:.2f}"))
    
    def export_waypoints(self):
        """Export waypoints to CSV."""
        if not self.calculator.waypoints:
            QMessageBox.warning(self, 'Warning', 'No waypoints to export')
            return
        
        filename = 'waypoints.csv'
        export_waypoints_csv(self.calculator.waypoints, filename)
        QMessageBox.information(self, 'Success', f'Exported to {filename}')
