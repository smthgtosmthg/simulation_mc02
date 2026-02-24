#!/usr/bin/env python3
"""
Script 8 : Vol automatique multi-drones — ArduPilot SITL

Usage :
    python3 08_multi_drone_flight.py --drones 3
    python3 08_multi_drone_flight.py --drones 5 --altitude 5 --hover 30
    python3 08_multi_drone_flight.py --drones 3 --alt-step 2

Séquence :
    1. Connexion à N drones (tcp:127.0.0.1:5760+I*10)
    2. Attente de la calibration EKF
    3. Mode GUIDED pour tous les drones
    4. Arm + Takeoff (altitudes décalées pour éviter les collisions)
    5. Hover (affichage des positions)
    6. Atterrissage (mode LAND)

Prérequis : 06_launch_multi_drones.sh doit tourner dans un autre terminal.
"""

import argparse
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

# Modes de vol ArduCopter (custom_mode IDs)
MODE_STABILIZE = 0
MODE_ALT_HOLD = 2
MODE_AUTO = 3
MODE_GUIDED = 4
MODE_LOITER = 5
MODE_RTL = 6
MODE_LAND = 9

MODE_NAMES = {
    0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
    5: "LOITER", 6: "RTL", 9: "LAND"
}


# ============================================================
# Fonctions drone
# ============================================================

def connect_drone(instance, timeout=30):
    """Connexion à une instance ArduPilot SITL."""
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
    """Passer en mode de vol spécifique."""
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
    """Armer les moteurs du drone."""
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,      # confirmation
        1,      # 1 = arm
        0, 0, 0, 0, 0, 0
    )
    # Attendre l'armement
    t0 = time.time()
    while time.time() - t0 < timeout:
        hb = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return True
    return False


def disarm_drone(conn):
    """Désarmer les moteurs du drone."""
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0,  # 0 = disarm
        0, 0, 0, 0, 0, 0
    )


def takeoff(conn, altitude):
    """Envoyer la commande de décollage."""
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,              # confirmation
        0,              # param1 : pitch (ignored for multi)
        0, 0, 0, 0, 0,
        altitude        # param7 : altitude (m)
    )


def get_altitude(conn, timeout=2):
    """Récupère l'altitude actuelle (z-up) du drone."""
    msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=timeout)
    if msg:
        return -msg.z  # NED z-down → z-up
    return None


def get_position_str(conn, timeout=1):
    """Récupère une string formatée de la position."""
    msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=timeout)
    if msg:
        return f"({msg.x:7.2f}, {msg.y:7.2f}, {-msg.z:5.2f}m)"
    return "(  N/A  )"


def wait_altitude(conn, target_alt, tolerance=1.0, timeout=30):
    """Attendre que le drone atteigne une altitude cible."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        alt = get_altitude(conn, timeout=2)
        if alt is not None and abs(alt - target_alt) < tolerance:
            return True
        time.sleep(0.5)
    return False


def request_data_streams(conn, rate_hz=4):
    """Demander les flux de données au drone."""
    conn.mav.request_data_stream_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        rate_hz, 1
    )


# ============================================================
# Programme principal
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description='Vol automatique multi-drones ArduPilot SITL'
    )
    parser.add_argument('--drones', type=int, default=3,
                        help='Nombre de drones (défaut: 3)')
    parser.add_argument('--altitude', type=float, default=4.0,
                        help='Altitude de base en mètres (défaut: 4)')
    parser.add_argument('--alt-step', type=float, default=1.0,
                        help='Écart d\'altitude entre drones (défaut: 1m)')
    parser.add_argument('--hover', type=int, default=15,
                        help='Durée du hover en secondes (défaut: 15)')
    parser.add_argument('--ekf-wait', type=int, default=20,
                        help='Temps d\'attente EKF en secondes (défaut: 20)')
    args = parser.parse_args()

    n_drones = args.drones

    print()
    print("=" * 65)
    print(f"  VOL AUTOMATIQUE — {n_drones} DRONES")
    print(f"  Altitude : {args.altitude}m (base) + {args.alt_step}m/drone")
    print(f"  Hover    : {args.hover}s")
    print("=" * 65)
    print()

    # ============================================================
    # Étape 1 : Connexion
    # ============================================================
    print(f"[1/6] Connexion aux {n_drones} drones...")
    connections = []
    for i in range(n_drones):
        conn = connect_drone(i)
        if conn is None:
            print(f"\nERREUR: Drone {i} non accessible.")
            print("Vérifie que 06_launch_multi_drones.sh tourne.")
            sys.exit(1)
        connections.append(conn)
    print(f"  => Tous les drones connectés !\n")

    # Demander les flux de données
    for conn in connections:
        request_data_streams(conn)

    # ============================================================
    # Étape 2 : Attente EKF
    # ============================================================
    print(f"[2/6] Attente de la calibration EKF ({args.ekf_wait}s)...")
    for s in range(args.ekf_wait, 0, -1):
        sys.stdout.write(f"\r  {s}s restantes...")
        sys.stdout.flush()
        time.sleep(1)
    print("\r  => EKF prêt !              \n")

    # ============================================================
    # Étape 3 : Mode GUIDED
    # ============================================================
    print("[3/6] Passage en mode GUIDED...")
    for i, conn in enumerate(connections):
        success = set_mode(conn, MODE_GUIDED)
        status = "GUIDED" if success else "TIMEOUT (on continue)"
        print(f"  Drone {i} : {status}")
    print()

    # ============================================================
    # Étape 4 : Arm
    # ============================================================
    print("[4/6] Armement des moteurs...")
    for i, conn in enumerate(connections):
        success = arm_drone(conn, timeout=15)
        status = "ARMED" if success else "ECHEC (réessaie dans 5s)"
        print(f"  Drone {i} : {status}")
        if not success:
            time.sleep(5)
            success = arm_drone(conn, timeout=15)
            print(f"  Drone {i} (retry) : {'ARMED' if success else 'ECHEC'}")
    print()

    # ============================================================
    # Étape 5 : Takeoff
    # ============================================================
    print("[5/6] Décollage...")
    target_alts = []
    for i, conn in enumerate(connections):
        alt = args.altitude + i * args.alt_step
        target_alts.append(alt)
        takeoff(conn, alt)
        print(f"  Drone {i} : takeoff → {alt}m")
        time.sleep(2)  # Délai entre décollages

    print()
    print("  Attente des altitudes cibles...")
    for i, conn in enumerate(connections):
        reached = wait_altitude(conn, target_alts[i], tolerance=1.0, timeout=30)
        current = get_altitude(conn)
        current_str = f"{current:.1f}m" if current else "N/A"
        if reached:
            print(f"  Drone {i} : {current_str} / {target_alts[i]}m — OK")
        else:
            print(f"  Drone {i} : {current_str} / {target_alts[i]}m — timeout (continue)")
    print()

    # ============================================================
    # Hover avec affichage des positions
    # ============================================================
    print(f"HOVER pendant {args.hover}s — Positions :")
    print("-" * 65)

    for s in range(args.hover):
        time.sleep(1)
        line = f"  t={s+1:3d}s |"
        for i, conn in enumerate(connections):
            pos_str = get_position_str(conn)
            line += f" D{i}{pos_str}"
        print(line)
    print()

    # ============================================================
    # Étape 6 : Atterrissage
    # ============================================================
    print("[6/6] Atterrissage...")
    for i, conn in enumerate(connections):
        success = set_mode(conn, MODE_LAND)
        status = "LAND" if success else "commande envoyée"
        print(f"  Drone {i} : {status}")
    print()

    print("  Attente de l'atterrissage (30s)...")
    for s in range(30, 0, -5):
        time.sleep(5)
        alts = []
        for conn in connections:
            alt = get_altitude(conn)
            alts.append(f"{alt:.1f}m" if alt else "N/A")
        print(f"  {s-5:2d}s | Altitudes : {', '.join(alts)}")

    print()
    print("=" * 65)
    print("  VOL TERMINÉ AVEC SUCCÈS !")
    print("=" * 65)
    print()
    print("  Résumé :")
    print(f"    Drones   : {n_drones}")
    print(f"    Altitudes: {', '.join(f'{a}m' for a in target_alts)}")
    print(f"    Hover    : {args.hover}s")
    print()

    # Fermer les connexions
    for conn in connections:
        try:
            conn.close()
        except Exception:
            pass


if __name__ == '__main__':
    main()
