#!/usr/bin/env python3

import numpy as np


class OccupancyGrid:

    def __init__(self, width_m=30.0, height_m=20.0, resolution=0.25,
                 origin_x=-15.0, origin_y=-10.0):
      
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.width_m = width_m
        self.height_m = height_m
        self.cols = int(width_m / resolution)  
        self.rows = int(height_m / resolution)  

        self.log_odds = np.zeros((self.rows, self.cols), dtype=np.float32)

        self.l_free = -0.4
        self.l_occupied = 0.85
        self.l_min = -5.0
        self.l_max = 5.0


    def world_to_grid(self, wx, wy):
        """World (meters) -> grid (row, col)."""
        col = int((wx - self.origin_x) / self.resolution)
        row = int((wy - self.origin_y) / self.resolution)
        return row, col

    def grid_to_world(self, row, col):
        """Grid (row, col) -> world center (x, y)."""
        wx = self.origin_x + (col + 0.5) * self.resolution
        wy = self.origin_y + (row + 0.5) * self.resolution
        return wx, wy

    def in_bounds(self, row, col):
        return 0 <= row < self.rows and 0 <= col < self.cols



    def update_from_scan(self, drone_x, drone_y, scan_points):
       
        r0, c0 = self.world_to_grid(drone_x, drone_y)

        for px, py, is_hit in scan_points:
            r1, c1 = self.world_to_grid(px, py)
            cells = self._bresenham(r0, c0, r1, c1) #retourne les cases traversées par le rayon, du drone à l'obstacle ou au max range

            # tous les cell du depart jusqua arrivé sauf ledernier sont libres
            for r, c in cells[:-1]:
                if self.in_bounds(r, c):
                    self.log_odds[r, c] = np.clip(
                        self.log_odds[r, c] + self.l_free,
                        self.l_min, self.l_max)

            # Endpoint: occupied if hit, free if max-range
            if cells:
                r, c = cells[-1]
                if self.in_bounds(r, c):
                    delta = self.l_occupied if is_hit else self.l_free
                    self.log_odds[r, c] = np.clip(
                        self.log_odds[r, c] + delta,
                        self.l_min, self.l_max)


    def get_map(self):
        """Categorized map: 0=free, 100=occupied, -1=unknown."""
        m = np.full((self.rows, self.cols), -1, dtype=np.int8)
        m[self.log_odds < -0.5] = 0     # free
        m[self.log_odds > 0.5] = 100    # occupied
        return m

    def get_coverage(self):
        """Fraction of cells classified (not unknown)."""
        known = np.sum(np.abs(self.log_odds) > 0.5)
        return float(known) / (self.rows * self.cols)


    @staticmethod
    def _bresenham(r0, c0, r1, c1):
        """Bresenham line: returns list of (row, col) from start to end."""
        cells = []
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r1 > r0 else -1
        sc = 1 if c1 > c0 else -1
        err = dc - dr
        r, c = r0, c0

        while True:
            cells.append((r, c))
            if r == r1 and c == c1:
                break
            e2 = 2 * err
            if e2 > -dr:
                err -= dr
                c += sc
            if e2 < dc:
                err += dc
                r += sr

        return cells
