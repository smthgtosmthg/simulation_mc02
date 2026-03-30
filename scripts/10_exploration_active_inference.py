#!/usr/bin/env python3

import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time

def _bootstrap_python_with_pymavlink():
    if os.environ.get("SIM_MC02_PYMAVLINK_BOOTSTRAPPED") == "1":
        return
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(script_dir, "installation", "venv", "bin", "python3"),
        os.path.expanduser(
            "~/ns-allinone-3.40/ns-3.40/contrib/sionna/model/ns3sionna/"
            "sionna-venv/bin/python3"
        ),
    ]
    current_python = os.path.abspath(sys.executable) if sys.executable else ""
    for candidate in candidates:
        candidate = os.path.abspath(candidate)
        if not os.path.isfile(candidate) or not os.access(candidate, os.X_OK):
            continue
        if candidate == current_python:
            continue
        env = os.environ.copy()
        env["SIM_MC02_PYMAVLINK_BOOTSTRAPPED"] = "1"
        os.execve(candidate, [candidate, os.path.abspath(__file__), *sys.argv[1:]], env)

try:
    from pymavlink import mavutil
except ImportError:
    _bootstrap_python_with_pymavlink()
    print("ERREUR: pymavlink non installé.")
    sys.exit(1)

import numpy as np

MODE_GUIDED = 4
MODE_LAND   = 9

WH_X_MIN, WH_X_MAX = -15.0, 15.0  
WH_Y_MIN, WH_Y_MAX = -10.0, 10.0  

GRID_RES   = 0.5        # m par cell
GRID_W     = int((WH_X_MAX - WH_X_MIN) / GRID_RES)   
GRID_H     = int((WH_Y_MAX - WH_Y_MIN) / GRID_RES)  #col
LIDAR_RANGE = 10.0      

#C'est le modèle bayésien du capteur. Plutôt que stocker des probabilités
# directement (qui se multiplient et deviennent très petites),
# on travaille en log-odds (logarithme de la cote).
P_PRIOR = 0.5 #pob dune cellule avant a 50% de chance d'être occupée"
P_OCC   = 0.7          
P_FREE  = 0.3          

def _logodds(p): #convertit une probabilité en log-odds"
    p = np.clip(p, 1e-6, 1 - 1e-6)
    return np.log(p / (1 - p))

def _prob(lo): #convertit des log-odds en probabilité"
    return 1.0 / (1.0 + np.exp(-np.clip(lo, -10, 10)))

L0   = _logodds(P_PRIOR)
L_OCC  = _logodds(P_OCC)
L_FREE = _logodds(P_FREE)

POS_CSV       = "/tmp/drone_positions.csv"
EXPLORE_JSON  = "/tmp/exploration_state.json"

running = True

def signal_handler(sig, frame):
    global running
    running = False
    print("\n  ⛔ Interruption reçue — atterrissage…")


class OccupancyGrid:

    def __init__(self):
        self.lo = np.full((GRID_H, GRID_W), L0, dtype=np.float32)
        self.visited = np.zeros((GRID_H, GRID_W), dtype=bool)

    # Conv de coord
    def w2g(self, wx, wy):
        gx = int((wx - WH_X_MIN) / GRID_RES)
        gy = int((wy - WH_Y_MIN) / GRID_RES)
        return np.clip(gx, 0, GRID_W - 1), np.clip(gy, 0, GRID_H - 1)

    def g2w(self, gx, gy):
        return (gx * GRID_RES + WH_X_MIN + GRID_RES / 2,
                gy * GRID_RES + WH_Y_MIN + GRID_RES / 2)

    def update(self, dx, dy, yaw, ranges, angle_min, angle_max,
               other_positions=None):

        n = len(ranges)
        if n == 0:
            return 0
        angles = np.linspace(angle_min, angle_max, n, endpoint=False)
        new_cells = 0
        for ang, r in zip(angles, ranges):
            world_ang = ang + yaw
            if r < 0.1 or not np.isfinite(r):
                continue
            hit = r < LIDAR_RANGE - 0.1
            end_r = r if hit else LIDAR_RANGE
            ex = dx + end_r * math.cos(world_ang)
            ey = dy + end_r * math.sin(world_ang)
            #  skip if hit point is near another drone
            if hit and other_positions:
                skip = False
                for ox, oy in other_positions:
                    if (ex - ox) ** 2 + (ey - oy) ** 2 < 2.25:  # 1.5m radius
                        skip = True
                        break
                if skip:
                    continue
            new_cells += self._trace(dx, dy, ex, ey, hit)
        return new_cells

    def _trace(self, x0, y0, x1, y1, hit):
        """Bresenham ==> mark free cells + last cell (hit or free)"""
        gx0, gy0 = self.w2g(x0, y0)
        gx1, gy1 = self.w2g(x1, y1)
        cells = self._bresenham(gx0, gy0, gx1, gy1)
        new = 0
        # free cells along the ray (all except last)
        for gx, gy in cells[:-1]:
            if not self.visited[gy, gx]:
                new += 1
            self.lo[gy, gx] += L_FREE - L0
            self.visited[gy, gx] = True
        # end cell
        if cells:
            gx, gy = cells[-1]
            if not self.visited[gy, gx]:
                new += 1
            self.lo[gy, gx] += (L_OCC if hit else L_FREE) - L0
            self.visited[gy, gx] = True
        self.lo = np.clip(self.lo, -5, 5)
        return new

    @staticmethod
    def _bresenham(x0, y0, x1, y1):
        cells = []
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            if 0 <= x0 < GRID_W and 0 <= y0 < GRID_H:
                cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return cells

    def prob(self):
        return _prob(self.lo)

    def explored_pct(self):
        return float(np.mean(self.visited) * 100)

    def entropy(self):
        p = np.clip(self.prob(), 1e-6, 1 - 1e-6)
        return float(-np.mean(p * np.log2(p) + (1 - p) * np.log2(1 - p)))

    def frontiers(self):
        """cellules non visitées qui sont adjacentes à au moins une cellule visitée."""
        exp = self.visited
        frt = []
        for gy in range(1, GRID_H - 1):
            for gx in range(1, GRID_W - 1):
                if not exp[gy, gx]:
                    if exp[gy - 1, gx] or exp[gy + 1, gx] or exp[gy, gx - 1] or exp[gy, gx + 1]:
                        frt.append((gx, gy))
        return frt


class ActiveInferencePlanner:
    def __init__(self, grid: OccupancyGrid, drone_id: int):
        self.grid = grid
        self.id = drone_id
        # compter les cellules inconnues autour du drone
        r = int(LIDAR_RANGE / GRID_RES)
        self._disc = [(ddx, ddy)
                      for ddy in range(-r, r + 1)
                      for ddx in range(-r, r + 1)
                      if ddx * ddx + ddy * ddy <= r * r]

    def select(self, dx, dy, others=None, taken_targets=None):
        """Return (wx, wy) of best waypoint or None if done."""
        frontiers = self.grid.frontiers()
        if not frontiers:
            return None

        # Cluster frontiers into ≤15 candidates 
        candidates = self._sample(frontiers, 15)

        best, best_g = None, float('inf')
        for gx, gy in candidates:
            wx, wy = self.grid.g2w(gx, gy)
            epist = self._epistemic(gx, gy)
            pragm = self._pragmatic(wx, wy, dx, dy, others, taken_targets)
            g = -epist - pragm
            if g < best_g:
                best_g = g
                best = (wx, wy)
        return best

    def _epistemic(self, gx, gy):
        """Count unknown cells inside LiDAR disc centred at (gx, gy)."""
        count = 0
        for ddx, ddy in self._disc:
            nx, ny = gx + ddx, gy + ddy
            if 0 <= nx < GRID_W and 0 <= ny < GRID_H:
                if not self.grid.visited[ny, nx]:
                    count += 1
        return count

    def _pragmatic(self, wx, wy, dx, dy, others, taken_targets=None):
        val = 0.0
        # distance cost
        dist = math.hypot(wx - dx, wy - dy)
        val -= dist * 0.5
        # obstacle avoidance at destination
        gx, gy = self.grid.w2g(wx, wy)
        p = _prob(self.grid.lo[gy, gx])
        if p > 0.65:
            val -= 500
        # PATH collision check: verify cells along the flight path
        gx0, gy0 = self.grid.w2g(dx, dy)
        path_cells = OccupancyGrid._bresenham(gx0, gy0, gx, gy)
        obstacle_on_path = 0
        for px, py in path_cells:
            if 0 <= px < GRID_W and 0 <= py < GRID_H:
                if _prob(self.grid.lo[py, px]) > 0.60:
                    obstacle_on_path += 1
        if obstacle_on_path > 0:
            val -= obstacle_on_path * 100
        # wall margin
        if not (WH_X_MIN + 1.5 < wx < WH_X_MAX - 1.5 and
                WH_Y_MIN + 1.5 < wy < WH_Y_MAX - 1.5):
            val -= 80
        # inter-drone coordination: penalise proximity to other drones
        if others:
            for oid, (ox, oy, _) in others.items():
                if oid != self.id:
                    d = math.hypot(wx - ox, wy - oy)
                    if d < 6.0:
                        val -= (6.0 - d) * 50
        # penalise targets already taken by other drones
        if taken_targets:
            for tid, (tx, ty) in taken_targets.items():
                if tid != self.id:
                    d = math.hypot(wx - tx, wy - ty)
                    if d < 3.0:
                        val -= 300
        return val

    @staticmethod
    def _sample(frontiers, n):
        if len(frontiers) <= n:
            return frontiers
        step = max(1, len(frontiers) // n)
        return frontiers[::step][:n]



class GzLidarSubscriber:

    def __init__(self, drone_id):
        self.drone_id = drone_id
        self.latest = None
        self._lock = threading.Lock()
        self._proc = None

    def start(self):
        topic = self._find_topic()
        if not topic:
            print(f"    ⚠  Drone {self.drone_id}: pas de topic LiDAR trouvé")
            return False
        print(f"    Drone {self.drone_id}: LiDAR → {topic}")
        self._proc = subprocess.Popen(
            ["gz", "topic", "-e", "-t", topic],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, bufsize=1,
        )
        t = threading.Thread(target=self._reader, daemon=True)
        t.start()
        return True

    def stop(self):
        if self._proc:
            self._proc.terminate()

    def get(self):
        with self._lock:
            return self.latest

    def _find_topic(self):
        try:
            r = subprocess.run(["gz", "topic", "-l"],
                               capture_output=True, text=True, timeout=10)
            for line in r.stdout.splitlines():
                line = line.strip()
                if (f"drone_{self.drone_id}" in line and
                        "lidar" in line and line.endswith("/scan")):
                    return line
        except Exception:
            pass
        return (f"/world/warehouse/model/drone_{self.drone_id}/"
                f"model/iris_with_ardupilot/link/lidar_link/sensor/lidar/scan")

    _RE_FIELD = re.compile(
        r"^\s*(angle_min|angle_max|count|vertical_angle_min|vertical_angle_max|vertical_count|ranges)\s*:\s*([-\d.eE+inf]+)")

    # Vertical scan parameters (must match SDF: 3 samples, -0.4 to 0 rad)
    VERT_SAMPLES = 3
    VERT_MIN = -0.4
    VERT_MAX = 0.0

    def _reader(self):
        ranges = []
        amin, amax = -math.pi, math.pi
        hcount = 360
        vcount = self.VERT_SAMPLES

        for line in self._proc.stdout:
            line = line.rstrip('\n')

            if line.strip() == '' and ranges:
                projected = self._project_multilayer(ranges, hcount, vcount)
                with self._lock:
                    self.latest = {
                        "ranges": projected,
                        "angle_min": amin,
                        "angle_max": amax,
                    }
                ranges = []
                continue

            m = self._RE_FIELD.match(line)
            if not m:
                continue
            key, val = m.group(1), m.group(2)
            if key == "angle_min":
                amin = float(val)
            elif key == "angle_max":
                amax = float(val)
            elif key == "count":
                hcount = int(float(val))
            elif key == "vertical_count":
                vcount = int(float(val))
            elif key == "ranges":
                try:
                    ranges.append(float(val))
                except ValueError:
                    ranges.append(float('inf'))

    def _project_multilayer(self, raw_ranges, hcount, vcount):
        # Le LiDAR a 3 couches verticales (angles -0.4, -0.2, 0.0 radians ≈ -23°, -11.5°, 0°). 
        # Les données brutes contiennent 1080 distances : 
        # d'abord les 360 ranges de la couche 0, 
        # puis les 360 de la couche 1, puis les 360 de la couche 2.
        if vcount <= 1:
            return raw_ranges 
        total = hcount * vcount
        if len(raw_ranges) < total:
            return raw_ranges[:hcount] if len(raw_ranges) >= hcount else raw_ranges

        vert_angles = np.linspace(self.VERT_MIN, self.VERT_MAX, vcount)
        cos_vert = np.cos(vert_angles)  

        projected = []
        for h in range(hcount):
            best_r = float('inf')
            for v in range(vcount):
                idx = v * hcount + h 
                r = raw_ranges[idx] if idx < len(raw_ranges) else float('inf')
                if np.isfinite(r) and r > 0.08:
                    r_horiz = r * cos_vert[v]  
                    if r_horiz < best_r:
                        best_r = r_horiz
            projected.append(best_r)
        return projected


def connect_drone(instance, timeout=30):
    port = 5760 + instance * 10
    addr = f"tcp:127.0.0.1:{port}"
    print(f"    Drone {instance}: {addr}…", end=" ", flush=True)
    try:
        conn = mavutil.mavlink_connection(addr, source_system=255)
        conn.wait_heartbeat(timeout=timeout)
        print(f"OK (sys={conn.target_system})")
        return conn
    except Exception as e:
        print(f"ERREUR: {e}")
        return None


def set_mode(conn, mode, timeout=10):
    conn.mav.set_mode_send(
        conn.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode)
    t0 = time.time()
    while time.time() - t0 < timeout:
        hb = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb and hb.custom_mode == mode:
            return True
    return False


def arm_drone(conn, timeout=30):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)
    t0 = time.time()
    while time.time() - t0 < timeout:
        hb = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return True
    return False


def takeoff(conn, alt):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, alt)


def goto_local(conn, x, y, z):
    conn.mav.set_position_target_local_ned_send(
        0, conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000_1111_1111_1000,
        x, y, -z,
        0, 0, 0, 0, 0, 0, 0, 0)


def get_position(conn, timeout=1):
    # Drain buffer to get the LATEST position, not a stale queued one
    latest = None
    while True:
        msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg is None:
            break
        latest = msg
    if latest is None:
        latest = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=timeout)
    if latest:
        return (latest.x, latest.y, -latest.z)
    return None


def get_yaw(conn, timeout=1):
    # Drain buffer to get the LATEST yaw, not a stale queued one
    latest = None
    while True:
        msg = conn.recv_match(type='ATTITUDE', blocking=False)
        if msg is None:
            break
        latest = msg
    if latest is None:
        latest = conn.recv_match(type='ATTITUDE', blocking=True, timeout=timeout)
    if latest:
        return latest.yaw
    return 0.0


def request_streams(conn, rate=4):
    conn.mav.request_data_stream_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION, rate, 1)
    conn.mav.request_data_stream_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, rate, 1)


def wait_alt(conn, target, tol=1.0, timeout=30):
    t0 = time.time()
    while time.time() - t0 < timeout:
        pos = get_position(conn)
        if pos and abs(pos[2] - target) < tol:
            return True
        time.sleep(0.5)
    return False


def write_positions(connections):
    lines = []
    for i, conn in enumerate(connections):
        msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=0.5)
        if msg:
            lines.append(f"{i},{msg.x:.4f},{msg.y:.4f},{-msg.z:.4f}")
    with open(POS_CSV, "w") as f:
        f.write("\n".join(lines) + "\n")


def write_exploration_state(step, global_grid, drones_info, trajectories,
                            frontiers, event_log, step_history,
                            altitude_history, speed_history,
                            pct_history, entropy_history,
                            inter_drone_dist_history):
    """Write JSON snapshot consumed by the dashboard."""
    prob = global_grid.prob()
    flat = [round(float(v), 2) for v in prob.flatten()]
    visited_flat = [int(v) for v in global_grid.visited.flatten()]

    fr_sample = frontiers[:200] if len(frontiers) > 200 else frontiers

    state = {
        "step": step,
        "timestamp": time.strftime("%H:%M:%S"),
        "explored_pct": round(global_grid.explored_pct(), 1),
        "entropy": round(global_grid.entropy(), 4),
        "total_cells": GRID_W * GRID_H,
        "explored_cells": int(np.sum(global_grid.visited)),
        "frontier_count": len(frontiers),
        "drones": drones_info,
        "grid": {
            "width": GRID_W,
            "height": GRID_H,
            "resolution": GRID_RES,
            "origin_x": WH_X_MIN,
            "origin_y": WH_Y_MIN,
            "data": flat,
            "visited": visited_flat,
        },
        "frontiers": fr_sample,
        "trajectories": {str(k): v for k, v in trajectories.items()},
        "events": event_log[-100:],
        "step_history": step_history[-150:],
        "altitude_history": {str(k): v for k, v in altitude_history.items()},
        "speed_history": {str(k): v for k, v in speed_history.items()},
        "pct_history": pct_history,
        "entropy_history": entropy_history,
        "inter_drone_dist_history": inter_drone_dist_history[-150:],
    }
    tmp = EXPLORE_JSON + ".tmp"
    with open(tmp, "w") as f:
        json.dump(state, f, separators=(',', ':'))
    os.replace(tmp, EXPLORE_JSON)



def main():
    parser = argparse.ArgumentParser(
        description="Exploration Active Inference — Multi-Drone Warehouse")
    parser.add_argument("--drones", type=int, default=3)
    parser.add_argument("--altitude", type=float, default=4.0,
                        help="Flight altitude for exploration (default 4.0 m — above 3m shelves)")
    parser.add_argument("--ekf-wait", type=int, default=20)
    parser.add_argument("--max-steps", type=int, default=150,
                        help="Max exploration steps before landing")
    parser.add_argument("--step-time", type=float, default=3.0,
                        help="Seconds between planning steps")
    args = parser.parse_args()
    n = min(args.drones, 3)

    signal.signal(signal.SIGINT, signal_handler)

    W = 70
    print()
    print("┌" + "─" * (W - 2) + "┐")
    print("│" + "  10 — EXPLORATION ACTIVE INFERENCE".ljust(W - 2) + "│")
    print("│" + f"  {n} drones · alt {args.altitude}m · LiDAR 360° 10m".ljust(W - 2) + "│")
    print("│" + f"  Grille {GRID_W}×{GRID_H} ({GRID_RES}m/cell) · max {args.max_steps} steps".ljust(W - 2) + "│")
    print("└" + "─" * (W - 2) + "┘")
    print()

    print("[1/6] Connexion aux drones…")
    conns = []
    for i in range(n):
        c = connect_drone(i)
        if c is None:
            print(f"\nERREUR: Drone {i} inaccessible. Relance 06_launch_multi_drones.sh.")
            sys.exit(1)
        conns.append(c)
    print()
    for c in conns:
        request_streams(c)

    print(f"[2/6] Calibration EKF ({args.ekf_wait}s)…")
    for s in range(args.ekf_wait, 0, -1):
        sys.stdout.write(f"\r    {s}s…")
        sys.stdout.flush()
        time.sleep(1)
    print("\r    EKF prêt !\n")

    print("[3/6] GUIDED + Armement…")
    for i, c in enumerate(conns):
        set_mode(c, MODE_GUIDED)
        ok = arm_drone(c, timeout=15)
        if not ok:
            time.sleep(5)
            ok = arm_drone(c, timeout=15)
        print(f"    Drone {i}: {'ARMED ✓' if ok else 'ÉCHEC ✗'}")
    print()

    print(f"[4/6] Décollage → {args.altitude}m…")
    for i, c in enumerate(conns):
        takeoff(c, args.altitude)
        time.sleep(1.5)
    for i, c in enumerate(conns):
        ok = wait_alt(c, args.altitude, tol=1.0, timeout=60)
        if not ok:
            print(f"    ⚠ Drone {i}: altitude non atteinte, on continue…")
    print("    Tous en vol !\n")

    print("[5/6] Connexion aux capteurs LiDAR…")
    lidars = []
    for i in range(n):
        sub = GzLidarSubscriber(i)
        ok = sub.start()
        lidars.append(sub)
    time.sleep(3)  # let first scans arrive
    for i, sub in enumerate(lidars):
        scan = sub.get()
        rays = len(scan["ranges"]) if scan else 0
        print(f"    Drone {i}: {'%d rayons ✓' % rays if rays else 'en attente…'}")
    print()

    print("[6/6] ═══ DÉBUT EXPLORATION ACTIVE INFERENCE ═══\n")

    local_grids = [OccupancyGrid() for _ in range(n)]
    global_grid = OccupancyGrid()
    planners = [ActiveInferencePlanner(global_grid, i) for i in range(n)]

    trajectories = {i: [] for i in range(n)}
    distances = [0.0] * n
    cells_by_drone = [0] * n
    prev_pos = [None] * n
    targets = [None] * n
    drone_status = ["flying"] * n       # "flying" | "crashed" | "recovering" | "landed"
    drone_speeds = [0.0] * n
    event_log = []                       
    step_history = []                    
    altitude_history = {i: [] for i in range(n)}
    speed_history = {i: [] for i in range(n)}
    pct_history = []
    entropy_history = []
    inter_drone_dist_history = []

    def log_event(step_num, drone_id, etype, msg):
        entry = {
            "step": step_num,
            "time": time.strftime("%H:%M:%S"),
            "drone": drone_id,
            "type": etype,     # "info" | "warning" | "crash" | "recovery" | "target"
            "message": msg,
        }
        event_log.append(entry)
        if len(event_log) > 500:
            event_log.pop(0)

    for i in range(n):
        log_event(0, i, "info", f"Drone {i} prêt — altitude cible {args.altitude}m")

    for step in range(1, args.max_steps + 1):
        if not running:
            break
        t0 = time.time()

        # ── A. Positions  ────────────────────────────────
        positions = {}
        yaws = {}
        for i, c in enumerate(conns):
            pos = get_position(c)
            yaw = get_yaw(c)
            if pos:
                positions[i] = pos
                yaws[i] = yaw if yaw else 0.0

        # ── B. LiDAR → update local grids ─────────────────────
        for i in range(n):
            if i not in positions:
                continue
            scan = lidars[i].get()
            if not scan:
                continue
            x, y, z = positions[i]
            other_pos = [(positions[j][0], positions[j][1])
                         for j in range(n) if j != i and j in positions]
            new = local_grids[i].update(
                x, y, yaws[i],
                scan["ranges"], scan["angle_min"], scan["angle_max"],
                other_positions=other_pos)
            cells_by_drone[i] += new

        # ── C. Fuse all local grids → global ──────────────────
        combined = np.full((GRID_H, GRID_W), L0, dtype=np.float32)
        combined_vis = np.zeros((GRID_H, GRID_W), dtype=bool)
        for i in range(n):
            combined += local_grids[i].lo - L0
            combined_vis |= local_grids[i].visited
        global_grid.lo = np.clip(combined, -5, 5)
        global_grid.visited = combined_vis

        # ── D. Active Inference → next waypoint ───────────────
        for i in range(n):
            if i not in positions:
                continue
            x, y, z = positions[i]
            if z < 1.0:
                # Drone crashed — attempt recovery: re-arm + re-takeoff
                if drone_status[i] != "crashed" and drone_status[i] != "recovering":
                    log_event(step, i, "crash", f"CRASH z={z:.2f}m — tentative re-arm + takeoff")
                    print(f"    ⚠ Drone {i} CRASH DÉTECTÉ (z={z:.2f}m) → re-arm + takeoff")
                    set_mode(conns[i], MODE_GUIDED)
                    arm_drone(conns[i], timeout=5)
                    takeoff(conns[i], args.altitude)
                    drone_status[i] = "recovering"
                    log_event(step, i, "recovery", f"Commande de récupération envoyée")
                elif drone_status[i] == "recovering":
                    # Still on ground after recovery attempt — resend
                    takeoff(conns[i], args.altitude)
                else:
                    drone_status[i] = "crashed"
            elif z < args.altitude - 1.5:
                # Altitude dropping — re-send altitude command
                if drone_status[i] == "recovering" and z > 2.0:
                    drone_status[i] = "flying"
                    log_event(step, i, "info", f"Récupéré — altitude z={z:.2f}m")
                elif drone_status[i] == "flying":
                    log_event(step, i, "warning", f"Altitude basse z={z:.2f}m — correction")
                print(f"    ⚠ Drone {i} altitude basse (z={z:.2f}m) → correction")
                goto_local(conns[i], x, y, args.altitude)
            else:
                if drone_status[i] == "recovering":
                    drone_status[i] = "flying"
                    log_event(step, i, "info", f"Récupéré — altitude z={z:.2f}m")

        frontiers = global_grid.frontiers()
        # Sequential target assignment to avoid convergence
        assigned_targets = {}
        for i in range(n):
            if i not in positions:
                continue
            if drone_status[i] in ("crashed", "recovering"):
                continue
            x, y, z = positions[i]
            wp = planners[i].select(x, y, positions, taken_targets=assigned_targets)
            if wp:
                old_target = targets[i]
                targets[i] = wp
                assigned_targets[i] = wp
                goto_local(conns[i], wp[0], wp[1], args.altitude)
                if old_target is None or math.hypot(wp[0] - old_target[0], wp[1] - old_target[1]) > 1.0:
                    log_event(step, i, "target", f"Nouvelle cible ({wp[0]:.1f}, {wp[1]:.1f})")

        # ── E. Track trajectory + distance + speed ──────────
        step_time_so_far = time.time() - t0
        for i in range(n):
            if i not in positions:
                continue
            x, y, z = positions[i]
            trajectories[i].append([round(x, 2), round(y, 2)])
            if len(trajectories[i]) > 500:
                trajectories[i] = trajectories[i][-500:]
            if prev_pos[i]:
                dd = math.hypot(x - prev_pos[i][0], y - prev_pos[i][1])
                distances[i] += dd
                actual_dt = max(step_time_so_far, 0.5)
                drone_speeds[i] = round(dd / actual_dt, 2)
            else:
                drone_speeds[i] = 0.0
            prev_pos[i] = (x, y)
            # Track altitude & speed history
            altitude_history[i].append(round(z, 2))
            speed_history[i].append(drone_speeds[i])
            if len(altitude_history[i]) > 300:
                altitude_history[i].pop(0)
            if len(speed_history[i]) > 300:
                speed_history[i].pop(0)

        # Compute inter-drone distances
        inter_dists = {}
        drone_ids = sorted(positions.keys())
        for ii in range(len(drone_ids)):
            for jj in range(ii + 1, len(drone_ids)):
                a, b = drone_ids[ii], drone_ids[jj]
                xa, ya, _ = positions[a]
                xb, yb, _ = positions[b]
                inter_dists[f"{a}-{b}"] = round(math.hypot(xa - xb, ya - yb), 2)
        inter_drone_dist_history.append(inter_dists)
        if len(inter_drone_dist_history) > 300:
            inter_drone_dist_history.pop(0)

        # Track global metrics history
        pct = global_grid.explored_pct()
        ent = global_grid.entropy()
        pct_history.append(round(pct, 1))
        entropy_history.append(round(ent, 4))
        if len(pct_history) > 300:
            pct_history.pop(0)
        if len(entropy_history) > 300:
            entropy_history.pop(0)

        # ── F. Write state for dashboard ──────────────────────
        write_positions(conns)
        drones_info = []
        for i in range(n):
            pos = positions.get(i, (0, 0, 0))
            di = {
                "id": i,
                "x": round(pos[0], 2), "y": round(pos[1], 2), "z": round(pos[2], 2),
                "yaw": round(yaws.get(i, 0), 2),
                "cells_discovered": cells_by_drone[i],
                "distance_traveled": round(distances[i], 1),
                "speed": drone_speeds[i],
                "status": drone_status[i],
                "target_x": round(targets[i][0], 1) if targets[i] else None,
                "target_y": round(targets[i][1], 1) if targets[i] else None,
            }
            drones_info.append(di)

        # Build step summary for timeline
        step_summary = {
            "step": step,
            "time": time.strftime("%H:%M:%S"),
            "pct": round(pct, 1),
            "entropy": round(ent, 4),
            "frontiers": len(frontiers),
            "drones": [
                {
                    "id": i,
                    "x": round(positions[i][0], 2) if i in positions else None,
                    "y": round(positions[i][1], 2) if i in positions else None,
                    "z": round(positions[i][2], 2) if i in positions else None,
                    "status": drone_status[i],
                    "speed": drone_speeds[i],
                    "target": [round(targets[i][0], 1), round(targets[i][1], 1)] if targets[i] else None,
                }
                for i in range(n)
            ],
            "inter_dists": inter_dists,
        }
        step_history.append(step_summary)
        if len(step_history) > 300:
            step_history.pop(0)

        write_exploration_state(step, global_grid, drones_info,
                                trajectories, frontiers,
                                event_log, step_history,
                                altitude_history, speed_history,
                                pct_history, entropy_history,
                                inter_drone_dist_history)

        # ── G. Console output ─────────────────────────────────
        elapsed = time.time() - t0
        pos_str = "  ".join(
            f"D{i}({positions[i][0]:+6.1f},{positions[i][1]:+6.1f})"
            if i in positions else f"D{i}(  N/A  )"
            for i in range(n))
        tgt_str = "  ".join(
            f"→({targets[i][0]:+5.1f},{targets[i][1]:+5.1f})"
            if targets[i] else "→(done)"
            for i in range(n))
        bar_len = 30
        filled = int(pct / 100 * bar_len)
        bar = "█" * filled + "░" * (bar_len - filled)
        print(f"  Step {step:3d} │ [{bar}] {pct:5.1f}% │ H={ent:.3f} │ "
              f"F={len(frontiers):4d} │ {elapsed:.1f}s")
        print(f"          │ {pos_str}")
        print(f"          │ {tgt_str}")

        # ── H. Check completion ───────────────────────────────
        if pct >= 95:
            print(f"\n  ✅ EXPLORATION TERMINÉE — {pct:.1f}% couvert en {step} steps")
            break

        # Wait until next planning cycle, RE-SENDING goto_local every 0.5s
        # ArduPilot requires repeated position commands to maintain flight
        while running:
            remaining = args.step_time - (time.time() - t0)
            if remaining <= 0:
                break
            # Re-send current targets to all flying drones
            for i in range(n):
                if targets[i] and drone_status[i] == "flying":
                    goto_local(conns[i], targets[i][0], targets[i][1], args.altitude)
            time.sleep(min(0.5, remaining))

    # ── Atterrissage ──────────────────────────────────────────
    print("\n  ATTERRISSAGE…")
    for i, c in enumerate(conns):
        set_mode(c, MODE_LAND)
        print(f"    Drone {i}: LAND")

    for s in range(30):
        time.sleep(1)
        write_positions(conns)
        all_down = all(
            (lambda p: p is not None and p[2] < 0.3)(get_position(c))
            for c in conns)
        if all_down:
            print("    Tous posés ✓")
            break

    # cleanup
    for sub in lidars:
        sub.stop()

    print()
    print("=" * W)
    print("  EXPLORATION TERMINÉE !")
    pct = global_grid.explored_pct()
    print(f"  Couverture finale : {pct:.1f}%")
    for i in range(n):
        print(f"  Drone {i}: {cells_by_drone[i]} cellules, "
              f"{distances[i]:.1f}m parcourus")
    print("=" * W)

    for c in conns:
        try:
            c.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
