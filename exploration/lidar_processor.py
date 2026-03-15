#!/usr/bin/env python3


import math
import subprocess
import threading
import time


class LidarReader:

    def __init__(self, drone_id):
        self.drone_id = drone_id
        self.topic = f"/drone_{drone_id}/lidar"
        self._polar = None       
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def get_polar(self):
        """Return latest polar scan or None."""
        with self._lock:
            return self._polar


    def _read_loop(self):
        while self._running:
            try:
                scan = self._read_one_scan()
                if scan is not None:
                    with self._lock:
                        self._polar = scan
            except Exception:
                pass
            time.sleep(0.15)

    def _read_one_scan(self):
        """Read one scan via `gz topic -e -n 1 -t <topic>`."""
        try:
            result = subprocess.run(
                ["gz", "topic", "-e", "-n", "1", "-t", self.topic],
                capture_output=True, text=True, timeout=3.0
            )
            if result.returncode != 0 or not result.stdout.strip():
                return None
            return _parse_gz_scan(result.stdout)
        except (subprocess.TimeoutExpired, FileNotFoundError):
            return None



def _parse_gz_scan(raw_text):
    """
    Parse `gz topic -e`  text output for LaserScan.


    Returns: list of (range, angle, is_hit)==> points polaires
    """
    angle_min = None
    angle_step = None
    range_max = 10.0
    ranges = []

    for line in raw_text.strip().split('\n'):
        line = line.strip()
        if line.startswith("angle_min:"):
            angle_min = float(line.split(":", 1)[1])
        elif line.startswith("angle_step:"):
            angle_step = float(line.split(":", 1)[1])
        elif line.startswith("range_max:"):
            range_max = float(line.split(":", 1)[1])
        elif line.startswith("ranges:"):
            val = line.split(":", 1)[1].strip()
            try:
                ranges.append(float(val))
            except ValueError:
                ranges.append(float('inf'))

    if angle_min is None or angle_step is None or not ranges:
        return None

    polar = []
    for i, r in enumerate(ranges):
        angle = angle_min + i * angle_step
        is_hit = r < (range_max - 0.05) and not math.isinf(r)
        clamped_r = r if is_hit else range_max
        polar.append((clamped_r, angle, is_hit))
    return polar



def polar_to_cartesian(polar_scan, drone_x, drone_y, drone_yaw=0.0):
    """
    Convert polar (range, angle, is_hit) to world-frame (hit_x, hit_y, is_hit).
    """
    points = []
    for r, angle, is_hit in polar_scan:
        world_angle = angle + drone_yaw
        hx = drone_x + r * math.cos(world_angle)
        hy = drone_y + r * math.sin(world_angle)
        points.append((hx, hy, is_hit))
    return points


# ── manager ────────────────────────────────────────────────────────

class LidarManager:
    """Manages LIDAR readers for N drones."""

    def __init__(self, n_drones):
        self.readers = {i: LidarReader(i) for i in range(n_drones)}

    def start_all(self):
        for r in self.readers.values():
            r.start()
        print(f"  LIDAR readers started for {len(self.readers)} drones"
              f" (topics: {', '.join(r.topic for r in self.readers.values())})")

    def stop_all(self):
        for r in self.readers.values():
            r.stop()

    def get_scan(self, drone_id, drone_x, drone_y, drone_yaw=0.0):
        """
        Get latest scan as list of (hit_x, hit_y, is_hit).
        Returns None if no scan available yet.
        """
        reader = self.readers.get(drone_id)
        if reader is None:
            return None
        polar = reader.get_polar()
        if polar is None:
            return None
        return polar_to_cartesian(polar, drone_x, drone_y, drone_yaw)
