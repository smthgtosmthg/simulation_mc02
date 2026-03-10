#!/usr/bin/env python3

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


POS_CSV = "/tmp/drone_positions.csv"


def write_positions_csv(connections):
    """Écrit les positions actuelles de tous les drones dans le CSV.
    Retourne une string résumée des positions pour l'affichage."""
    parts = []
    with open(POS_CSV, "w") as pf:
        for i, conn in enumerate(connections):
            msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
            if msg:
                x, y, z = msg.x, msg.y, -msg.z
                pf.write(f"{i},{x:.4f},{y:.4f},{z:.4f}\n")
                parts.append(f"D{i}({x:7.2f},{y:7.2f},{z:5.2f}m)")
            else:
                parts.append(f"D{i}(  N/A  )")
    return "  ".join(parts)


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
    print(f"  Positions: {POS_CSV} (continu)")
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
    print(f"  Attente des altitudes cibles... (positions → {POS_CSV})")
    t_takeoff = 0
    all_reached = [False] * n_drones
    while not all(all_reached):
        time.sleep(1)
        t_takeoff += 1
        pos_line = write_positions_csv(connections)
        for i, conn in enumerate(connections):
            if not all_reached[i]:
                alt = get_altitude(conn)
                if alt is not None and abs(alt - target_alts[i]) < 1.0:
                    all_reached[i] = True
        done = sum(all_reached)
        print(f"  ↗ t={t_takeoff:3d}s | {done}/{n_drones} at target | {pos_line}")
        if t_takeoff >= 60:  # safety timeout
            print("  ⚠ Timeout décollage (60s), on continue")
            break
    print()

    # ============================================================
    # Hover avec affichage des positions
    # ============================================================
    print(f"HOVER pendant {args.hover}s :")
    print("-" * 65)

    for s in range(args.hover):
        time.sleep(1)
        pos_line = write_positions_csv(connections)
        print(f"  ● t={s+1:3d}s | {pos_line}")
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
    for s in range(30):
        time.sleep(1)
        pos_line = write_positions_csv(connections)
        if s % 5 == 4:  # afficher toutes les 5s
            print(f"  ↘ t={s+1:3d}s | {pos_line}")
        # Vérifier si tous posés (alt < 0.3m)
        all_landed = True
        for conn in connections:
            alt = get_altitude(conn)
            if alt is None or alt > 0.3:
                all_landed = False
        if all_landed:
            write_positions_csv(connections)
            print(f"  ✓ Tous les drones posés !")
            break

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
