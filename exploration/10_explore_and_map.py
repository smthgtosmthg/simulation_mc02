#!/usr/bin/env python3
"""
10_explore_and_map.py — Multi-drone exploration with belief map sharing.

Workflow:
  1. Connect to N drones via pymavlink
  2. Takeoff to exploration altitude
  3. Main loop:
     - Read drone positions (MAVLink)
     - LIDAR scan (geometric raycasting)
     - Update each drone's local occupancy grid
     - Fuse into shared belief map
     - Find frontiers → assign exploration targets
     - Navigate drones to targets
  4. Stop when coverage target reached or timeout
  5. Save belief map (CSV + PNG) and land

Usage:
    python3 exploration/10_explore_and_map.py [--drones 3] [--altitude 2.0]
"""

import argparse
import os
import signal
import sys
import time

# Add exploration/ to Python path for local imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from occupancy_grid import OccupancyGrid
from lidar_processor import LidarManager
from belief_map_sharing import BeliefMap
from frontier_planner import find_frontiers, cluster_frontiers, assign_targets
from drone_controller import DroneController, MODE_GUIDED
from visualize_map import save_map_csv, save_map_png

# ── Graceful shutdown ──────────────────────────────────────────────
running = True

def _signal_handler(sig, frame):
    global running
    running = False
    print("\n  Interruption — landing...")

signal.signal(signal.SIGINT, _signal_handler)


# ── Position CSV (compatible with dashboard/bridge scripts) ────────
POS_CSV = "/tmp/drone_positions.csv"

def write_positions_csv(drones):
    """Write current positions for other scripts to read."""
    with open(POS_CSV, "w") as f:
        for dc in drones:
            x, y, z = dc.position
            f.write(f"{dc.instance},{x:.4f},{y:.4f},{z:.4f}\n")


# ── Main ───────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description='Multi-drone exploration + belief map sharing')
    parser.add_argument('--drones', type=int, default=3,
                        help='Number of drones (default: 3)')
    parser.add_argument('--altitude', type=float, default=2.0,
                        help='Exploration altitude in meters (default: 2.0)')
    parser.add_argument('--max-time', type=int, default=180,
                        help='Max exploration time in seconds (default: 180)')
    parser.add_argument('--coverage-target', type=float, default=0.70,
                        help='Stop at this coverage fraction (default: 0.70)')
    parser.add_argument('--lidar-rays', type=int, default=360,
                        help='Number of LIDAR rays per scan (default: 360)')
    parser.add_argument('--lidar-range', type=float, default=10.0,
                        help='LIDAR max range in meters (default: 10.0)')
    parser.add_argument('--update-hz', type=float, default=2.0,
                        help='Map update frequency (default: 2.0)')
    parser.add_argument('--ekf-wait', type=int, default=15,
                        help='EKF calibration wait time (default: 15)')
    args = parser.parse_args()

    n = args.drones
    W = 70

    print()
    print("=" * W)
    print("  MULTI-DRONE EXPLORATION + BELIEF MAP SHARING")
    print(f"  {n} drones | alt={args.altitude}m | "
          f"target={args.coverage_target*100:.0f}% | "
          f"max={args.max_time}s")
    print(f"  LIDAR: {args.lidar_rays} rays, {args.lidar_range}m range, "
          f"{args.update_hz} Hz")
    print("=" * W)
    print()

    # ── 1. Connect ─────────────────────────────────────────────────
    print(f"[1/5] Connecting to {n} drones...")
    drones = []
    for i in range(n):
        dc = DroneController(i)
        try:
            dc.connect()
        except Exception as e:
            print(f"\n  FAILED: {e}")
            print("  Make sure 06_launch_multi_drones.sh is running.")
            sys.exit(1)
        drones.append(dc)
    print()

    # ── 2. EKF wait ────────────────────────────────────────────────
    print(f"[2/5] Waiting for EKF calibration ({args.ekf_wait}s)...")
    for s in range(args.ekf_wait, 0, -1):
        sys.stdout.write(f"\r  {s}s...")
        sys.stdout.flush()
        time.sleep(1)
    print("\r  EKF ready!      ")
    print()

    # ── 3. Takeoff ─────────────────────────────────────────────────
    print("[3/5] GUIDED + ARM + Takeoff...")
    for i, dc in enumerate(drones):
        dc.set_mode(MODE_GUIDED)
        ok = dc.arm(timeout=15)
        if not ok:
            time.sleep(3)
            ok = dc.arm(timeout=15)
        alt = args.altitude + i * 0.5   # stagger altitudes
        dc.takeoff(alt)
        print(f"  Drone {i}: {'ARMED' if ok else 'FAIL'} -> {alt}m")
        time.sleep(2)

    print("  Climbing...")
    for _ in range(25):
        time.sleep(1)
        all_up = True
        for i, dc in enumerate(drones):
            dc.update_all()
            if dc.position[2] < args.altitude * 0.6:
                all_up = False
        if all_up:
            break
    print("  All drones airborne!")
    print()

    # ── 4. Initialize belief map + LIDAR ───────────────────────────
    print("[4/5] Initializing belief map + real LIDAR readers...")
    belief = BeliefMap(n)
    grid = belief.fused
    print(f"  Grid: {grid.cols}x{grid.rows} cells "
          f"({grid.width_m}x{grid.height_m}m, res={grid.resolution}m)")

    lidar_mgr = LidarManager(n)
    lidar_mgr.start_all()
    print("  Waiting 3s for first scans...")
    time.sleep(3)
    print()

    # ── 5. Exploration loop ────────────────────────────────────────
    print("[5/5] === EXPLORATION START ===")
    print(f"  Target: {args.coverage_target*100:.0f}% coverage | "
          f"Timeout: {args.max_time}s")
    print()

    dt = 1.0 / args.update_hz
    t_start = time.time()
    iteration = 0
    current_targets = {}
    positions = {}

    while running:
        elapsed = time.time() - t_start
        if elapsed > args.max_time:
            print(f"\n  Time limit ({args.max_time}s) reached.")
            break

        # (a) Read positions (Gazebo world frame, fast non-blocking drain)
        for i, dc in enumerate(drones):
            dc.update_all()
            x, y, z = dc.position
            positions[i] = (x, y)

        # (b) Real LIDAR scan → update local grids (Gazebo frame)
        for did, (px, py) in positions.items():
            gz_yaw = drones[did].yaw   # already Gazebo-frame yaw
            scan = lidar_mgr.get_scan(did, px, py, drone_yaw=gz_yaw)
            if scan is not None:
                belief.update_drone(did, px, py, scan)

        # (c) Fuse belief map
        fused_map = belief.get_fused_map()
        coverage = belief.get_coverage()

        # (d) Check coverage target
        if coverage >= args.coverage_target:
            print(f"\n  Coverage target reached: {coverage*100:.1f}%")
            break

        # (e) Frontier planning (every 2 iterations)
        if iteration % 2 == 0:
            frontiers = find_frontiers(fused_map)
            clusters = cluster_frontiers(frontiers)

            # Replan if any drone reached its target or has none
            need_replan = False
            for did in positions:
                if did not in current_targets:
                    need_replan = True
                    break
                tx, ty = current_targets[did]
                if drones[did].distance_to(tx, ty) < 2.0:
                    need_replan = True
                    break

            if need_replan or not current_targets:
                new_targets = assign_targets(clusters, positions, grid)
                if new_targets:
                    current_targets = new_targets

        # (f) Navigate (targets are Gazebo frame → goto_world converts to NED)
        for did, (tx, ty) in current_targets.items():
            if did < len(drones):
                alt = args.altitude + did * 0.5
                drones[did].goto_world(tx, ty, alt)

        # (g) Write positions CSV (for dashboard/bridge)
        write_positions_csv(drones)

        # (h) Print status every ~2s
        if iteration % 4 == 0:
            pos_str = "  ".join(
                f"D{i}({positions[i][0]:+6.1f},{positions[i][1]:+5.1f})"
                for i in sorted(positions.keys()))
            tgt_str = "  ".join(
                f"D{i}->({t[0]:+5.1f},{t[1]:+5.1f})"
                for i, t in sorted(current_targets.items()))
            print(f"  [{elapsed:5.1f}s] cov={coverage*100:5.1f}% | {pos_str}")
            if tgt_str:
                print(f"           targets: {tgt_str}")

        # (i) Save map every ~5s
        if iteration % 10 == 0:
            save_map_csv(fused_map)
            save_map_png(fused_map, positions, grid=grid)

        iteration += 1
        time.sleep(dt)

    # ── Final save ─────────────────────────────────────────────────
    fused_map = belief.get_fused_map()
    coverage = belief.get_coverage()
    save_map_csv(fused_map)
    img_path = save_map_png(fused_map, positions, grid=grid)

    # Exploration log
    with open("/tmp/drone_exploration_log.csv", "w") as f:
        f.write("drone,x,y,z,coverage\n")
        for dc in drones:
            x, y, z = dc.position
            f.write(f"{dc.instance},{x:.3f},{y:.3f},{z:.3f},{coverage:.4f}\n")

    print()
    print(f"  Final coverage: {coverage*100:.1f}%")
    print(f"  Map CSV:   /tmp/belief_map.csv")
    print(f"  Map image: {img_path}")
    print(f"  Log:       /tmp/drone_exploration_log.csv")

    # ── Land ───────────────────────────────────────────────────────
    print()
    print("  Landing...")
    for dc in drones:
        dc.land()

    for s in range(25):
        time.sleep(1)
        all_down = all(
            dc.update_all()[2] < 0.3
            for dc in drones)
        if all_down:
            print("  All landed!")
            break

    lidar_mgr.stop_all()

    for dc in drones:
        dc.close()

    print()
    print("=" * W)
    print("  EXPLORATION COMPLETE")
    print("=" * W)


if __name__ == '__main__':
    main()
