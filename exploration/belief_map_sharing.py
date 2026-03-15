#!/usr/bin/env python3

import numpy as np
from occupancy_grid import OccupancyGrid


class BeliefMap:
    """Shared belief map merging N drone grids."""

    def __init__(self, n_drones, **grid_kwargs):
        self.n_drones = n_drones
        self.grids = [OccupancyGrid(**grid_kwargs) for _ in range(n_drones)]
        self.fused = OccupancyGrid(**grid_kwargs)

    def update_drone(self, drone_id, drone_x, drone_y, scan_points):
        """Feed a new LIDAR scan into one drone's local grid."""
        self.grids[drone_id].update_from_scan(drone_x, drone_y, scan_points)

    def fuse(self):
        """Combine all local grids into the fused belief map."""
        fused_lo = np.zeros_like(self.fused.log_odds)
        for grid in self.grids:
            fused_lo += grid.log_odds
        self.fused.log_odds = np.clip(fused_lo, self.fused.l_min, self.fused.l_max)

    def get_fused_map(self):
        """Fuse then return categorized map (0/100/-1)."""
        self.fuse()
        return self.fused.get_map()

    def get_coverage(self):
        """Fuse then return coverage fraction."""
        self.fuse()
        return self.fused.get_coverage()
