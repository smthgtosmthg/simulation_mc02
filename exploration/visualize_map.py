#!/usr/bin/env python3
"""
Visualize the belief map as PNG image and CSV file.

Colors:
  - Gray  = unknown
  - White = free
  - Black = occupied
  - Colored dots = drone positions
"""

import numpy as np


def save_map_csv(grid_map, filepath="/tmp/belief_map.csv"):
    """Save categorized map (0/100/-1) to CSV."""
    np.savetxt(filepath, grid_map, fmt='%d', delimiter=',')


def save_map_png(grid_map, drone_positions=None, grid=None,
                 filepath="/tmp/belief_map.png"):
    """
    Save map as image. Uses PIL if available, otherwise PPM fallback.

    Args:
        grid_map:        2D array (0=free, 100=occupied, -1=unknown)
        drone_positions: dict {drone_id: (world_x, world_y)} or None
        grid:            OccupancyGrid for coordinate conversion
        filepath:        output path

    Returns:
        actual filepath used (may differ if PIL missing)
    """
    rows, cols = grid_map.shape

    # Build RGB image
    img = np.zeros((rows, cols, 3), dtype=np.uint8)
    img[grid_map == -1]  = [128, 128, 128]   # unknown → gray
    img[grid_map == 0]   = [255, 255, 255]   # free    → white
    img[grid_map == 100] = [0,   0,   0]     # occupied→ black

    # Drone markers (3×3 colored dots)
    if drone_positions and grid:
        colors = [(255, 50, 50), (50, 200, 50), (50, 50, 255)]
        for did, (wx, wy) in drone_positions.items():
            r, c = grid.world_to_grid(wx, wy)
            color = colors[did % len(colors)]
            for dr in range(-1, 2):
                for dc in range(-1, 2):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < rows and 0 <= cc < cols:
                        img[rr, cc] = color

    # Flip Y so world-up matches image-up
    img = img[::-1]

    # Try PIL, fallback to raw PPM
    try:
        from PIL import Image
        Image.fromarray(img).save(filepath)
        return filepath
    except ImportError:
        ppm_path = filepath.replace('.png', '.ppm')
        with open(ppm_path, 'wb') as f:
            f.write(f"P6\n{cols} {rows}\n255\n".encode())
            f.write(img.tobytes())
        return ppm_path
