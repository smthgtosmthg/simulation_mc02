#!/usr/bin/env python3
"""
Pymavlink drone controller — thin wrapper for exploration.

Handles: connect, mode, arm, takeoff, goto, position reading, land.
"""

import time
import math
from pymavlink import mavutil

MODE_GUIDED = 4
MODE_LAND = 9


def ned_to_world(nx, ny, nz=0):
    """NED position → Gazebo world position."""
    return (ny, nx, nz)

def world_to_ned(gx, gy, gz=0):
    """Gazebo world position → NED position."""
    return (gy, gx, gz)

def ned_yaw_to_world(ned_yaw):
    """NED yaw → Gazebo yaw. NED yaw=0 is North = Gazebo Y+."""
    return math.pi / 2.0 - ned_yaw


class DroneController:
    """Controls one ArduPilot SITL drone instance."""

    def __init__(self, instance):
        self.instance = instance
        self.port = 5760 + instance * 10
        self.addr = f"tcp:127.0.0.1:{self.port}"
        self.conn = None
        self.ned_pos = (0.0, 0.0, 0.0)    # raw NED (x, y, z-up)
        self.position = (0.0, 0.0, 0.0)   # (x, y, z) — z positive up
        self.ned_yaw = 0.0                 # NED yaw in radians
        self.yaw = 0.0                     # Gazebo yaw in radians

    def connect(self, timeout=30):
        print(f"  Drone {self.instance}: {self.addr}...", end=" ", flush=True)
        self.conn = mavutil.mavlink_connection(self.addr, source_system=255)
        self.conn.wait_heartbeat(timeout=timeout)
        print(f"OK (sysid={self.conn.target_system})")
        # Request position + attitude streams at 4 Hz
        self.conn.mav.request_data_stream_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION, 4, 1)
        self.conn.mav.request_data_stream_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 4, 1)

    def set_mode(self, mode_id, timeout=10):
        self.conn.mav.set_mode_send(
            self.conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
        t0 = time.time()
        while time.time() - t0 < timeout:
            hb = self.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if hb and hb.custom_mode == mode_id:
                return True
        return False

    def arm(self, timeout=30):
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0)
        t0 = time.time()
        while time.time() - t0 < timeout:
            hb = self.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return True
        return False

    def takeoff(self, altitude):
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude)

    def goto_world(self, gx, gy, gz):
        """Navigate to a Gazebo world-frame position."""
        ned_x, ned_y, _ = world_to_ned(gx, gy)
        self.conn.mav.set_position_target_local_ned_send(
            0,
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,   # position only
            ned_x, ned_y, -gz,   # NED: z negative up
            0, 0, 0,
            0, 0, 0,
            0, 0)

    def update_all(self):
        """Read all pending messages, update position + yaw. Non-blocking drain."""
        # Drain all pending messages (non-blocking)
        while True:
            msg = self.conn.recv_match(
                type=['LOCAL_POSITION_NED', 'ATTITUDE'],
                blocking=False)
            if msg is None:
                break
            mtype = msg.get_type()
            if mtype == 'LOCAL_POSITION_NED':
                self.ned_pos = (msg.x, msg.y, -msg.z)
                gx, gy, gz = ned_to_world(msg.x, msg.y, -msg.z)
                self.position = (gx, gy, gz)
            elif mtype == 'ATTITUDE':
                self.ned_yaw = msg.yaw
                self.yaw = ned_yaw_to_world(msg.yaw)

        # If nothing was pending, do one blocking read
        if self.position == (0.0, 0.0, 0.0):
            msg = self.conn.recv_match(type='LOCAL_POSITION_NED',
                                       blocking=True, timeout=1)
            if msg:
                self.ned_pos = (msg.x, msg.y, -msg.z)
                gx, gy, gz = ned_to_world(msg.x, msg.y, -msg.z)
                self.position = (gx, gy, gz)

        return self.position

    def update_position(self):
        """Convenience: calls update_all, returns Gazebo-frame position."""
        return self.update_all()

    def land(self):
        self.set_mode(MODE_LAND)

    def distance_to(self, tx, ty):
        """Distance in Gazebo world frame."""
        x, y, _ = self.position
        return math.hypot(x - tx, y - ty)

    def close(self):
        if self.conn:
            try:
                self.conn.close()
            except Exception:
                pass
