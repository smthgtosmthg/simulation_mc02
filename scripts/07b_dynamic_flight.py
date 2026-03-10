#!/usr/bin/env python3
"""
Script 08b : Vol dynamique multi-drones — Patrouille dans le warehouse

Scénario « Patrouille » :
    Les 3 drones décollent puis suivent des waypoints à travers le warehouse.
    Certains waypoints passent derrière les étagères (NLOS → RSSI faible),
    d'autres rapprochent les drones (meilleur signal), d'autres les éloignent.

    Cela crée des variations réalistes de RSSI et latence pour le live bridge.

Architecture :
    Script 06 (Gazebo+SITL)  →  ce script (vol + écrit /tmp/drone_positions.csv)
                                      ↓
    Script 14 (live bridge) watches CSV → RSSI → render

Usage :
    python3 08b_dynamic_flight.py              # 3 drones, patrouille
    python3 08b_dynamic_flight.py --drones 3 --speed 1.5 --loops 2

Prérequis : 06_launch_multi_drones.sh doit tourner dans un autre terminal.
"""

import argparse
import math
import signal
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("ERREUR: pymavlink non installé.")
    print("Installe avec : pip3 install pymavlink")
    sys.exit(1)

# ============================================================
# Constantes ArduCopter
# ============================================================
MODE_GUIDED = 4
MODE_LAND   = 9

# ============================================================
# Waypoints de patrouille par drone
# ============================================================
# Le warehouse fait ~30x20m, étagères à (±7, ±5), plafond 6m.
# On fait passer les drones à travers des zones variées :
#   - Derrière les étagères (NLOS)
#   - Au centre (LOS, proches)
#   - Aux extrémités (LOS, loin)
#   - À différentes altitudes (au-dessus / en-dessous des étagères 3m)

WAYPOINTS = {
    # Drone 0 : ouest ↔ est, passe derrière étagères — mouvements amples
    0: [
        (-10.0,  5.5,  1.5),  # WP1: Derrière étagère NW, bas → NLOS immédiat
        ( 0.5,   0.5,  5.5),  # WP2: Centre haut → LOS, proche de D1
        (-12.0, -6.0,  1.5),  # WP3: Coin SW bas derrière étagère → NLOS
        ( 10.0,  5.5,  1.5),  # WP4: Traverse tout → derrière étagère NE → NLOS
        ( 0.0,   0.0,  5.8),  # WP5: Plafond centre → LOS max
        (-8.0,  -5.5,  1.5),  # WP6: Derrière étagère SW → NLOS
    ],
    # Drone 1 : zigzag nord-sud, change d'altitude radicalement
    1: [
        ( 0.0,   8.0,  1.5),  # WP1: Nord extrême, bas → NLOS (étagères entre)
        ( 0.0,  -8.0,  5.5),  # WP2: Sud extrême, haut → LOS
        ( 7.5,   5.5,  1.5),  # WP3: Derrière étagère NE → NLOS vs D0
        (-7.5,  -5.5,  5.5),  # WP4: Au-dessus étagère SW → LOS
        ( 1.0,   0.5,  5.5),  # WP5: Centre → LOS, très proche D0 et D2
        ( 7.5,   5.5,  1.0),  # WP6: Retour derrière étagère NE → NLOS
    ],
    # Drone 2 : grands déplacements est-ouest, altitudes variées
    2: [
        ( 12.0,  0.0,  2.0),  # WP1: Extrême est, bas → loin de D0
        (-12.0,  0.0,  5.5),  # WP2: Extrême ouest, haut → 24m traversée !
        ( 8.0,  -5.5,  1.5),  # WP3: Derrière étagère SE → NLOS
        (-0.5,   0.5,  5.5),  # WP4: Centre haut → LOS, collé à D0+D1
        ( 10.0,  5.5,  1.0),  # WP5: Derrière étagère NE bas → NLOS total
        (-8.0,   5.5,  5.5),  # WP6: Au-dessus étagère NW → LOS
    ],
}

POS_CSV = "/tmp/drone_positions.csv"
running = True


def signal_handler(sig, frame):
    global running
    running = False
    print("\n  ⛔ Interruption reçue...")


# ============================================================
# Fonctions drone (identiques au 08 original)
# ============================================================

def connect_drone(instance, timeout=30):
    port = 5760 + instance * 10
    addr = f"tcp:127.0.0.1:{port}"
    print(f"  Drone {instance} : connexion à {addr}...", end=" ", flush=True)
    try:
        conn = mavutil.mavlink_connection(addr, source_system=255)
        conn.wait_heartbeat(timeout=timeout)
        print(f"OK (sysid={conn.target_system})")
        return conn
    except Exception as e:
        print(f"ERREUR : {e}")
        return None


def set_mode(conn, mode_id, timeout=10):
    conn.mav.set_mode_send(
        conn.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    t0 = time.time()
    while time.time() - t0 < timeout:
        hb = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb and hb.custom_mode == mode_id:
            return True
    return False


def arm_drone(conn, timeout=30):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    t0 = time.time()
    while time.time() - t0 < timeout:
        hb = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return True
    return False


def takeoff(conn, altitude):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )


def get_altitude(conn, timeout=2):
    msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=timeout)
    if msg:
        return -msg.z
    return None


def get_position(conn, timeout=1):
    """Retourne (x, y, z) ou None."""
    msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=timeout)
    if msg:
        return (msg.x, msg.y, -msg.z)
    return None


def request_data_streams(conn, rate_hz=4):
    conn.mav.request_data_stream_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        rate_hz, 1
    )


def wait_altitude(conn, target_alt, tolerance=1.0, timeout=30):
    t0 = time.time()
    while time.time() - t0 < timeout:
        alt = get_altitude(conn, timeout=2)
        if alt is not None and abs(alt - target_alt) < tolerance:
            return True
        time.sleep(0.5)
    return False


def goto_local(conn, x, y, z):
    """Envoyer le drone vers une position locale (NED frame)."""
    conn.mav.set_position_target_local_ned_send(
        0,                                          # time_boot_ms
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,                         # type_mask: positions only
        x, y, -z,                                   # NED: z inversé
        0, 0, 0,                                    # velocity
        0, 0, 0,                                    # acceleration
        0, 0                                        # yaw, yaw_rate
    )


def distance_3d(p1, p2):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(p1, p2)))


def write_positions_csv(connections):
    """Écrit les positions actuelles dans le CSV + retourne un résumé."""
    parts = []
    with open(POS_CSV, "w") as pf:
        for i, conn in enumerate(connections):
            msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
            if msg:
                x, y, z = msg.x, msg.y, -msg.z
                pf.write(f"{i},{x:.4f},{y:.4f},{z:.4f}\n")
                parts.append(f"D{i}({x:6.1f},{y:6.1f},{z:4.1f}m)")
            else:
                parts.append(f"D{i}(  N/A  )")
    return "  ".join(parts)


# ============================================================
# Boucle de navigation vers un waypoint
# ============================================================

def navigate_to_waypoint(connections, targets, tolerance=3.0, timeout=6):
    """Envoie tous les drones vers leurs waypoints.
    Timeout COURT (6s) — on enchaîne immédiatement, jamais figé.
    Re-envoie la commande goto toutes les 1s pour garder le mouvement."""
    t0 = time.time()
    while time.time() - t0 < timeout and running:
        # Re-envoyer goto à chaque itération pour maintenir le mouvement
        for i, conn in enumerate(connections):
            if i in targets:
                x, y, z = targets[i]
                goto_local(conn, x, y, z)

        time.sleep(0.5)
        pos_line = write_positions_csv(connections)

        all_reached = True
        for i, conn in enumerate(connections):
            if i not in targets:
                continue
            pos = get_position(conn)
            if pos is None or distance_3d(pos, targets[i]) > tolerance:
                all_reached = False

        elapsed = time.time() - t0
        print(f"    {'✓' if all_reached else '→'} {elapsed:4.0f}s | {pos_line}")

        if all_reached:
            return True

    # Timeout — on passe au WP suivant, le drone est EN MOUVEMENT
    return False


# ============================================================
# Programme principal
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description='Vol dynamique multi-drones — Patrouille warehouse')
    parser.add_argument('--drones', type=int, default=3)
    parser.add_argument('--altitude', type=float, default=4.0,
                        help='Altitude initiale (défaut: 4)')
    parser.add_argument('--alt-step', type=float, default=1.0)
    parser.add_argument('--ekf-wait', type=int, default=20)
    parser.add_argument('--loops', type=int, default=2,
                        help='Nombre de boucles de patrouille (défaut: 2)')
    args = parser.parse_args()

    n_drones = min(args.drones, 3)  # max 3 waypoint sets
    signal.signal(signal.SIGINT, signal_handler)

    W = 70
    print()
    print("┌" + "─" * (W - 2) + "┐")
    print("│" + "  08b — VOL DYNAMIQUE : PATROUILLE WAREHOUSE".ljust(W - 2) + "│")
    print("│" + f"  {n_drones} drones · {args.loops} boucles · {len(WAYPOINTS[0])} waypoints/drone".ljust(W - 2) + "│")
    print("│" + f"  Max 6s/WP · Positions → {POS_CSV}".ljust(W - 2) + "│")
    print("└" + "─" * (W - 2) + "┘")
    print()

    # ── Connexion ──
    print(f"[1/6] Connexion aux {n_drones} drones...")
    connections = []
    for i in range(n_drones):
        conn = connect_drone(i)
        if conn is None:
            print(f"\nERREUR: Drone {i} non accessible.")
            print("Vérifie que 06_launch_multi_drones.sh tourne.")
            sys.exit(1)
        connections.append(conn)
    print(f"  => Tous connectés !\n")

    for conn in connections:
        request_data_streams(conn)

    # ── EKF ──
    print(f"[2/6] Attente calibration EKF ({args.ekf_wait}s)...")
    for s in range(args.ekf_wait, 0, -1):
        sys.stdout.write(f"\r  {s}s...")
        sys.stdout.flush()
        time.sleep(1)
    print("\r  => EKF prêt !\n")

    # ── GUIDED ──
    print("[3/6] Mode GUIDED...")
    for i, conn in enumerate(connections):
        ok = set_mode(conn, MODE_GUIDED)
        print(f"  Drone {i} : {'GUIDED' if ok else 'TIMEOUT'}")
    print()

    # ── ARM ──
    print("[4/6] Armement...")
    for i, conn in enumerate(connections):
        ok = arm_drone(conn, timeout=15)
        print(f"  Drone {i} : {'ARMED' if ok else 'ECHEC'}")
        if not ok:
            time.sleep(5)
            ok = arm_drone(conn, timeout=15)
            print(f"  Drone {i} (retry) : {'ARMED' if ok else 'ECHEC'}")
    print()

    # ── Takeoff ──
    print("[5/6] Décollage...")
    target_alts = []
    for i, conn in enumerate(connections):
        alt = args.altitude + i * args.alt_step
        target_alts.append(alt)
        takeoff(conn, alt)
        print(f"  Drone {i} : takeoff → {alt}m")
        time.sleep(2)

    print("\n  Attente des altitudes cibles...")
    for s in range(30):
        time.sleep(1)
        pos_line = write_positions_csv(connections)
        all_ok = all(
            (lambda a: a is not None and abs(a - target_alts[i]) < 1.0)(get_altitude(c))
            for i, c in enumerate(connections)
        )
        if s % 3 == 0:
            print(f"  ↗ {s}s | {pos_line}")
        if all_ok:
            break
    print()

    # ══════════════════════════════════════════════════════════
    # [6/6] PATROUILLE DYNAMIQUE
    # ══════════════════════════════════════════════════════════
    print("[6/6] ═══ DÉBUT DE LA PATROUILLE ═══")
    print(f"  {args.loops} boucles × {len(WAYPOINTS[0])} waypoints")
    print(f"  Max 6s/WP — mouvement continu, jamais figé !")
    print(f"  Les étagères (3m haut) vont bloquer le signal !")
    print()

    total_wps = len(WAYPOINTS[0])

    for loop in range(args.loops):
        if not running:
            break
        print(f"  ┌── Boucle {loop + 1}/{args.loops} ──────────────────────────")

        for wp_idx in range(total_wps):
            if not running:
                break

            # Construire les cibles pour ce waypoint
            targets = {}
            descriptions = []
            for i in range(n_drones):
                wps = WAYPOINTS.get(i, WAYPOINTS[0])
                wp = wps[wp_idx % len(wps)]
                targets[i] = wp
                descriptions.append(f"D{i}→({wp[0]:+.0f},{wp[1]:+.0f},{wp[2]:.0f}m)")

            print(f"  │")
            print(f"  ├─ WP {wp_idx + 1}/{total_wps}: {' · '.join(descriptions)}")

            # Naviguer (max 6s puis on enchaîne — le drone bouge en continu)
            navigate_to_waypoint(connections, targets, tolerance=3.0, timeout=6)

        print(f"  └── Boucle {loop + 1} terminée")
        print()

    # ══════════════════════════════════════════════════════════
    # Atterrissage
    # ══════════════════════════════════════════════════════════
    print("ATTERRISSAGE...")
    for i, conn in enumerate(connections):
        set_mode(conn, MODE_LAND)
        print(f"  Drone {i} : LAND")

    for s in range(30):
        time.sleep(1)
        pos_line = write_positions_csv(connections)
        if s % 5 == 4:
            print(f"  ↘ {s+1}s | {pos_line}")
        all_landed = all(
            (lambda a: a is not None and a < 0.3)(get_altitude(c))
            for c in connections
        )
        if all_landed:
            print("  ✓ Tous posés !")
            break

    print()
    print("=" * W)
    print("  PATROUILLE TERMINÉE !")
    print("=" * W)

    for conn in connections:
        try:
            conn.close()
        except Exception:
            pass


if __name__ == '__main__':
    main()
