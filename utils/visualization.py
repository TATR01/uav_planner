"""Visualization utilities"""

import numpy as np


def generate_ndvi(width: int = 180, height: int = 180) -> np.ndarray:
    """Generate synthetic NDVI data."""
    ndvi = np.zeros((height, width), dtype=np.float32)
    
    for r in range(height):
        for c in range(width):
            nx = c / width
            ny = r / height
            
            v = 0.45 + 0.28 * np.sin(nx * 5 + 1.1) * np.cos(ny * 4.5 + 0.7)
            v += 0.12 * np.sin(nx * 11 + ny * 9)
            v -= 0.1 * np.power(np.abs(nx - 0.5), 1.5)
            
            ndvi[r, c] = np.clip(v, -1, 0.9)
    
    return ndvi


def generate_dem(width: int = 180, height: int = 180) -> np.ndarray:
    """Generate synthetic DEM data."""
    dem = np.zeros((height, width), dtype=np.float32)
    
    for r in range(height):
        for c in range(width):
            nx = c / width
            ny = r / height
            
            dem[r, c] = (120 + 22 * np.sin(nx * 4 + 0.5) * np.cos(ny * 3 + 1) +
                        9 * np.sin(nx * 8 + ny * 7) + 7 * np.cos(nx * 2 + ny * 5))
    
    return dem
