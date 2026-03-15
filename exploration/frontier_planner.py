#!/usr/bin/env python3
"""
Frontier-based exploration planner.

A *frontier* cell is a FREE cell adjacent to at least one UNKNOWN cell.
Frontiers are grouped into clusters, then each drone is assigned to the
nearest unassigned cluster centroid.
"""

import numpy as np
from collections import deque


def find_frontiers(grid_map):
    """
    Find frontier cells in the categorized map.

    Args:
        grid_map: 2D array (0=free, 100=occupied, -1=unknown)
    Returns:
        list of (row, col) frontier cells
    """
    rows, cols = grid_map.shape
    frontiers = []

    for r in range(1, rows - 1):
        for c in range(1, cols - 1):
            if grid_map[r, c] != 0:          # only free cells
                continue
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                if grid_map[r + dr, c + dc] == -1:
                    frontiers.append((r, c))
                    break

    return frontiers


def cluster_frontiers(frontiers, min_size=3, merge_dist=3):
    """
    Group nearby frontier cells into clusters via BFS.

    Cells within `merge_dist` grid-steps of each other are merged.
    Clusters smaller than `min_size` are discarded.

    Returns:
        list of clusters, each cluster = list of (row, col)
    """
    if not frontiers:
        return []

    frontier_set = set(frontiers)
    visited = set()
    clusters = []

    for f in frontiers:
        if f in visited:
            continue

        cluster = []
        queue = deque([f])
        visited.add(f)

        while queue:
            cr, cc = queue.popleft()
            cluster.append((cr, cc))

            for dr in range(-merge_dist, merge_dist + 1):
                for dc in range(-merge_dist, merge_dist + 1):
                    nb = (cr + dr, cc + dc)
                    if nb in frontier_set and nb not in visited:
                        visited.add(nb)
                        queue.append(nb)

        if len(cluster) >= min_size:
            clusters.append(cluster)

    return clusters


def assign_targets(clusters, drone_positions, grid):
    """
    Greedy assignment: each drone gets the nearest unassigned cluster.

    Args:
        clusters:        list of clusters from cluster_frontiers()
        drone_positions: dict {drone_id: (world_x, world_y)}
        grid:            OccupancyGrid (for coord conversion)

    Returns:
        dict {drone_id: (target_x, target_y)}
    """
    if not clusters:
        return {}

    # Cluster centroids in world coordinates
    centroids = []
    for cluster in clusters:
        mean_r = np.mean([r for r, c in cluster])
        mean_c = np.mean([c for r, c in cluster])
        wx, wy = grid.grid_to_world(mean_r, mean_c)
        centroids.append((wx, wy))

    assignments = {}
    used = set()

    for did in sorted(drone_positions.keys()):
        dx, dy = drone_positions[did]
        best_dist = float('inf')
        best_idx = None

        for ci, (cx, cy) in enumerate(centroids):
            if ci in used:
                continue
            dist = np.hypot(dx - cx, dy - cy)
            if dist < best_dist:
                best_dist = dist
                best_idx = ci

        if best_idx is not None:
            used.add(best_idx)
            assignments[did] = centroids[best_idx]

    return assignments
