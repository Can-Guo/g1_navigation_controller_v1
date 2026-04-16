#!/usr/bin/env python3
"""
Shared helper utilities for A* planner scripts.

This module intentionally keeps behavior conservative:
- Preserve existing CSV format used by downstream tooling.
- Return created file path for consistent logging in caller.
"""

import csv
import os
import time
from ament_index_python.packages import get_package_share_directory


def save_world_path_to_csv(world_path):
    """
    Save world-coordinate path points to a timestamped CSV file.

    Args:
        world_path: list[dict], each dict requires keys `x` and `y`.

    Returns:
        str: absolute path of generated CSV file.
    """
    package_path = get_package_share_directory('navigation_controller')

    path_dir = os.path.join(package_path, 'path')
    os.makedirs(path_dir, exist_ok=True)

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(path_dir, f'path_{timestamp}.csv')

    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['x', 'y'])
        for point in world_path:
            writer.writerow([point['x'], point['y']])

    return filename
