"""Export utilities"""

import csv
from typing import List, Dict


def export_waypoints_csv(waypoints: List[Dict], filename: str = 'waypoints.csv'):
    """Export waypoints to CSV file."""
    if not waypoints:
        return
    
    with open(filename, 'w', newline='', encoding='utf-8-sig') as f:
        writer = csv.writer(f)
        writer.writerow(['#', 'Latitude', 'Longitude', 'Altitude (m)', 'X (m)', 'Y (m)', 'Strip', 'Route'])
        
        for i, wp in enumerate(waypoints):
            writer.writerow([
                i + 1,
                f"{wp['lat']:.7f}",
                f"{wp['lon']:.7f}",
                wp['altitude'],
                f"{wp['x']:.2f}",
                f"{wp['y']:.2f}",
                wp['strip_idx'] + 1,
                wp['route_idx'] + 1
            ])
