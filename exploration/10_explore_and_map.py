#!/usr/bin/env python3


import argparse
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from occupancy_grid import OccupancyGrid
from lidar_processor import LidarManager
from belief_map_sharing import BeliefMap
from frontier_planner import find_frontiers, cluster_frontiers, assign_targets
from drone_controller import DroneController, MODE_GUIDED
from visualize_map import save_map_csv, save_map_png

running = True

def _signal_handler(sig, frame):
    global running
    running = False
    print("\n  Interruption — landing...")

signal.signal(signal.SIGINT, _signal_handler)


POS_CSV = "/tmp/drone_positions.csv"

def write_positions_csv(drones):
    """Write current positions for other scripts to read."""
    with open(POS_CSV, "w") as f:
        for dc in drones:
            x, y, z = dc.position
            f.write(f"{dc.instance},{x:.4f},{y:.4f},{z:.4f}\n")


# ── Main 
def main():
    parser = argparse.ArgumentParser(
        description='Multi-drone exploration + belief map sharing')
    parser.add_argument('--drones', type=int, default=3,
                        help='Number of drones (default: 3)')
    parser.add_argument('--altitude', type=float, default=2.0,
                        help='Exploration altitude in meters (default: 2.0)')
    parser.add_argument('--max-time', type=int, default=180,
                        help='Max exploration time in seconds (default: 180)')
    parser.add_argument('--coverage-target', type=float, default=0.95,
                        help='Coverage fraction for reference (default: 0.95)')
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

    print(f"[2/5] Waiting for EKF calibration ({args.ekf_wait}s)...")
    for s in range(args.ekf_wait, 0, -1):
        sys.stdout.write(f"\r  {s}s...")
        sys.stdout.flush()
        time.sleep(1)
    print("\r  EKF ready!      ")
    print()

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


    WAYPOINTS = {
        0: [  # Left half — scans all 4 faces of NW + SW shelves
            # South face of shelf_nw (fly at y=3, close to face at y=4.4)
            (-3, 0),       (-3, 3),       (-11, 3),
            # West face of shelf_nw (fly at x=-11, close to face at x=-10)
            (-11, 7),
            # North face of shelf_nw (fly at y=7, close to face at y=5.6)
            (-3, 7),
            # Top-left wall sweep
            (-3, 9),       (-14, 9),
            # Left wall full sweep
            (-14, -9),
            # Bottom-left wall + crate_1 area (crate at -12,-7)
            (-3, -9),
            # South face of shelf_sw (fly at y=-7, close to face at y=-5.6)
            (-3, -7),      (-11, -7),
            # West face of shelf_sw
            (-11, -3),
            # North face of shelf_sw (fly at y=-3, close to face at y=-4.4)
            (-3, -3),
            # Return to center
            (-3, 0),
        ],
        1: [  # Center — corridor sweeps + inner shelf faces from center
            # Center corridor full sweep
            (0, 0),        (0, 9),        (0, -9),
            # Left of center corridor (scan shelf east faces at x=-4)
            (-2, -9),      (-2, 3),
            # Cross between shelves at y=3 (scan inner top row)
            (2, 3),
            # Right of center corridor (scan shelf west faces at x=4)
            (2, -3),
            # Cross between shelves at y=-3 (scan inner bottom row)
            (-2, -3),      (-2, 0),
            # Second center pass shifted right
            (2, 9),        (2, -9),
            # Perimeter sweep for full wall detection
            (-14, -9),     (-14, 9),
            (14, 9),       (14, -9),
            (0, -9),       (0, 0),
        ],
        2: [  # Right half — scans all 4 faces of NE + SE shelves
            # South face of shelf_ne (fly at y=3, close to face at y=4.4)
            (3, 0),        (3, 3),        (11, 3),
            # East face of shelf_ne (fly at x=11, close to face at x=10)
            (11, 7),
            # North face of shelf_ne (fly at y=7, close to face at y=5.6)
            (3, 7),
            # Top-right wall sweep + crate_2 area (crate at 12,7)
            (3, 9),        (14, 9),
            # Right wall full sweep
            (14, -9),
            # Bottom-right wall
            (3, -9),
            # South face of shelf_se (fly at y=-7, close to face at y=-5.6)
            (3, -7),       (11, -7),
            # East face of shelf_se
            (11, -3),
            # North face of shelf_se (fly at y=-3, close to face at y=-4.4)
            (3, -3),
            # Return to center
            (3, 0),
        ],
    }
    wp_index = {i: 0 for i in range(n)}
    WP_REACHED = 1.5   

    for did in range(n):
        wps = WAYPOINTS.get(did, [])
        print(f"  Drone {did}: {len(wps)} waypoints planned")
    print()

    print("[5/5] === EXPLORATION START ===")
    print(f"  Target: {args.coverage_target*100:.0f}% coverage | "
          f"Timeout: {args.max_time}s")
    print()

    dt = 1.0 / args.update_hz
    t_start = time.time()
    iteration = 0
    current_targets = {}
    positions = {}

    for did in range(n):
        if did in WAYPOINTS and WAYPOINTS[did]:
            current_targets[did] = WAYPOINTS[did][0]

    while running:
        elapsed = time.time() - t_start
        if elapsed > args.max_time:
            print(f"\n  Time limit ({args.max_time}s) reached.")
            break

        for i, dc in enumerate(drones):
            dc.update_all()
            x, y, z = dc.position
            positions[i] = (x, y)

        for did, (px, py) in positions.items():
            gz_yaw = drones[did].yaw  
            scan = lidar_mgr.get_scan(did, px, py, drone_yaw=gz_yaw)
            if scan is not None:
                belief.update_drone(did, px, py, scan)

        fused_map = belief.get_fused_map()
        coverage = belief.get_coverage()

     
        all_done = True
        for did in positions:
            if did not in WAYPOINTS:
                continue
            wps = WAYPOINTS[did]
            idx = wp_index[did]
            if idx < len(wps):
                tx, ty = wps[idx]
                if drones[did].distance_to(tx, ty) < WP_REACHED:
                    if idx + 1 < len(wps):
                        wp_index[did] = idx + 1
                        tx, ty = wps[idx + 1]
                current_targets[did] = (tx, ty)
                if wp_index[did] < len(wps) - 1:
                    all_done = False
            else:
                current_targets[did] = wps[-1]

        if all_done and iteration > 10:
            print(f"\n  All waypoints completed!")
            break

        for did, (tx, ty) in current_targets.items():
            if did < len(drones):
                alt = args.altitude + did * 0.5
                drones[did].goto_world(tx, ty, alt)

        write_positions_csv(drones)

        if iteration % 4 == 0:
            pos_str = "  ".join(
                f"D{i}({positions[i][0]:+6.1f},{positions[i][1]:+5.1f})"
                for i in sorted(positions.keys()))
            tgt_str = "  ".join(
                f"D{i}->({t[0]:+5.1f},{t[1]:+5.1f})"
                for i, t in sorted(current_targets.items()))
            wp_str = "  ".join(
                f"D{i}:WP{wp_index.get(i,0)+1}/{len(WAYPOINTS.get(i,[]))}"
                for i in sorted(positions.keys()))
            print(f"  [{elapsed:5.1f}s] cov={coverage*100:5.1f}% | {pos_str}")
            if tgt_str:
                print(f"           targets: {tgt_str}")
            print(f"           waypts:  {wp_str}")

        if iteration % 10 == 0:
            save_map_csv(fused_map)
            save_map_png(fused_map, positions, grid=grid)

        iteration += 1
        time.sleep(dt)

    fused_map = belief.get_fused_map()
    coverage = belief.get_coverage()
    save_map_csv(fused_map)
    img_path = save_map_png(fused_map, positions, grid=grid)

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
