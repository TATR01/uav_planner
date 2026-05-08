"""Application settings and constants"""

# Geodesy
EARTH_RADIUS = 6371000  # meters

# Default UAV parameters
DEFAULT_ALTITUDE = 80  # meters
DEFAULT_SPEED = 12  # m/s
DEFAULT_FOV = 82  # degrees
DEFAULT_BATTERY = 45  # minutes
DEFAULT_OVERLAP_FORWARD = 0.75  # 75%
DEFAULT_OVERLAP_SIDE = 0.65  # 65%

# Obstacle parameters
DEFAULT_OBSTACLE_RADIUS = 30  # meters
BUFFER_PENALTY = 5.0  # Cost multiplier for crossing buffer zones

# Algorithm parameters
# DP/TSP
DP_MAX_ITERATIONS = 300
DP_2OPT_THRESHOLD = 1e-6

# GA
GA_DEFAULT_POPULATION = 60
GA_DEFAULT_GENERATIONS = 150
GA_ELITE_RATIO = 0.1
GA_TOURNAMENT_SIZE = 15
GA_MUTATION_INV_RATE = 0.08  # Inversion mutation
GA_MUTATION_SWAP_RATE = 0.10  # Swap mutation

# ACO
ACO_DEFAULT_ANTS = 25
ACO_DEFAULT_ITERATIONS = 80
ACO_RHO = 0.4  # Evaporation rate
ACO_ALPHA = 1.2  # Pheromone weight
ACO_BETA = 2.5  # Heuristic weight
ACO_Q = 1000  # Pheromone amount
ACO_PH_MAX = 5.0
ACO_PH_MIN = 0.01

# VRP
VRP_DEFAULT_ROUTES = 2
VRP_BATTERY_THRESHOLD = 0.85  # Max 85% battery usage per flight

# Visualization
NDVI_RASTER_WIDTH = 180
NDVI_RASTER_HEIGHT = 180

DEM_CONTOUR_LEVELS = 7

# Colors
COLOR_FIELD = '#00008b'  # Dark blue
COLOR_FIELD_FILL = 'rgba(0,0,255,0.03)'
COLOR_STRIP_ODD = 'rgba(100,180,100,0.08)'
COLOR_STRIP_EVEN = 'rgba(100,150,220,0.08)'
COLOR_BUFFER = 'rgba(255,80,80,0.12)'
COLOR_BUFFER_BORDER = 'rgba(200,0,0,0.5)'
COLOR_ROUTE = ['#0000cc', '#cc0000', '#008800', '#cc6600', '#880088']
COLOR_TAKEOFF = '#00aa00'
COLOR_LANDING = '#cc0000'

# Obstacle colors
OBSTACLE_COLORS = {
    'nfz': (220, 0, 0),
    'tree': (0, 140, 0),
    'power': (180, 100, 0),
    'building': (100, 0, 180),
}

# UI
WINDOW_WIDTH = 1400
WINDOW_HEIGHT = 800
LEFT_PANEL_WIDTH = 280
TOOLBAR_HEIGHT = 50
STATUSBAR_HEIGHT = 22

# Default field (demo)
DEFAULT_FIELD = [
    (54.271751, 48.550465),
    (54.265295, 48.551538),
    (54.266601, 48.565957),
    (54.272982, 48.564756),
]

# Progress dialog
PROGRESS_UPDATE_INTERVAL = 100  # ms

# Export
CSV_DELIMITER = ','
CSV_ENCODING = 'utf-8-sig'  # BOM for Excel

# Logging
LOG_LEVEL = 'INFO'
LOG_FORMAT = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
