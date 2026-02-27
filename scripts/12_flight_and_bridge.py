#!/usr/bin/env python3
"""
Script 12 : Vol multi-drones + Bridge NS-3 — combiné

Ce script résout le problème de connexion simultanée en fusionnant
08_multi_drone_flight.py et 11_ns3_bridge.py dans un seul processus.

Les drones sont connectés UNE SEULE FOIS, puis deux threads tournent
en parallèle :
  - Thread FLIGHT : arm → takeoff → hover → land
  - Thread BRIDGE : lecture positions → RSSI/latence → CSV

Usage :
    python3 12_flight_and_bridge.py --drones 3
    python3 12_flight_and_bridge.py --drones 3 --altitude 5 --hover 30
    python3 12_flight_and_bridge.py --drones 3 --alt-step 2 --hover 60

Prérequis : 06_launch_multi_drones.sh doit tourner dans un autre terminal.
"""

import argparse
import csv
import os
import signal
import subprocess
import sys
import threading
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("ERREUR: pymavlink non installé.")
    print("pip3 install pymavlink")
    sys.exit(1)


# ============================================================
# Constantes ArduCopter
# ============================================================

MODE_GUIDED    = 4
MODE_LAND      = 9


# ============================================================
# Configuration fichiers
# ============================================================

POS_FILE    = "/tmp/drone_positions.csv"
NS3_OUTPUT  = "/tmp/ns3_output.csv"
WORKSPACE   = os.path.expanduser("~/simulation_mc02")
UNIFIED_LOG = os.path.join(WORKSPACE, "comm_metrics.csv")
NS3_DIR     = os.path.expanduser("~/ns-allinone-3.40/ns-3.40")


# ============================================================
# État partagé (thread-safe)
# ============================================================

class SharedState:
    """État partagé entre le thread flight et le thread bridge."""
    def __init__(self, n_drones):
        self.n_drones = n_drones
        self.lock = threading.Lock()
        # Positions mises à jour par le bridge
        self.positions = [None] * n_drones
        # Flag global : les deux threads vérifient ceci
        self.running = True
        # Le flight indique quand les drones sont en l'air
        self.drones_airborne = threading.Event()
        # Le flight indique quand le vol est terminé
        self.flight_done = threading.Event()


# ============================================================
# Fonctions drone (thread-safe avec lock par connexion)
# ============================================================

def connect_drone(instance, timeout=30):
    port = 5760 + instance * 10
    addr = f"tcp:127.0.0.1:{port}"
    print(f"  Drone {instance} : connexion à {addr}...", end=" ", flush=True)
    try:
        conn = mavutil.mavlink_connection(addr, source_system=255)
        conn.wait_heartbeat(timeout=timeout)
        print("OK")
        return conn
    except Exception as e:
        print(f"ERREUR : {e}")
        return None


def set_mode(conn, lock, mode_id, timeout=10):
    with lock:
        conn.mav.set_mode_send(
            conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
    t0 = time.time()
    while time.time() - t0 < timeout:
        with lock:
            hb = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb and hb.custom_mode == mode_id:
            return True
    return False


def arm_drone(conn, lock, timeout=30):
    with lock:
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
    t0 = time.time()
    while time.time() - t0 < timeout:
        with lock:
            hb = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return True
    return False


def takeoff(conn, lock, altitude):
    with lock:
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0,
            altitude
        )


def get_altitude(conn, lock, timeout=2):
    with lock:
        msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=timeout)
    if msg:
        return -msg.z
    return None


def get_local_position(conn, lock, timeout=2):
    with lock:
        msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=timeout)
    if msg:
        return {'x': msg.x, 'y': msg.y, 'z': -msg.z}
    return None


def request_data_streams(conn, lock, rate_hz=4):
    with lock:
        conn.mav.request_data_stream_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION, rate_hz, 1
        )


def wait_altitude(conn, lock, target_alt, tolerance=1.0, timeout=30):
    t0 = time.time()
    while time.time() - t0 < timeout:
        alt = get_altitude(conn, lock, timeout=2)
        if alt is not None and abs(alt - target_alt) < tolerance:
            return True
        time.sleep(0.5)
    return False


# ============================================================
# Écriture / Lecture CSV
# ============================================================

def write_positions_csv(positions):
    with open(POS_FILE, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['drone_id', 'x', 'y', 'z'])
        for i, pos in enumerate(positions):
            if pos:
                writer.writerow([i, f"{pos['x']:.4f}", f"{pos['y']:.4f}", f"{pos['z']:.4f}"])
            else:
                writer.writerow([i, 0, 0, 0])


def read_ns3_output():
    results = []
    if not os.path.exists(NS3_OUTPUT):
        return results
    try:
        with open(NS3_OUTPUT, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                results.append(row)
    except Exception:
        pass
    return results


# ============================================================
# THREAD 1 : Flight (arm → takeoff → hover → land)
# ============================================================

def flight_thread(connections, locks, state, args):
    """
    Exécute la séquence de vol complète.
    Le bridge tourne en parallèle et lit les positions.
    """
    n = state.n_drones

    try:
        # --- EKF wait ---
        print(f"\n[1/5] Calibration EKF ({args.ekf_wait}s)...", end=" ", flush=True)
        for s in range(args.ekf_wait, 0, -1):
            if not state.running:
                return
            time.sleep(1)
        print("OK")

        # --- GUIDED mode ---
        print("\n[2/5] Mode GUIDED...")
        for i, conn in enumerate(connections):
            if not state.running:
                return
            success = set_mode(conn, locks[i], MODE_GUIDED)
            print(f"  Drone {i} : {'GUIDED' if success else 'TIMEOUT'}")

        # --- ARM ---
        print("\n[3/5] Armement...")
        for i, conn in enumerate(connections):
            if not state.running:
                return
            success = arm_drone(conn, locks[i], timeout=15)
            if not success:
                time.sleep(5)
                success = arm_drone(conn, locks[i], timeout=15)
            print(f"  Drone {i} : {'ARMED' if success else 'ECHEC'}")

        # --- TAKEOFF ---
        target_alts = [args.altitude + i * args.alt_step for i in range(n)]
        alts_str = ', '.join(f"{a}m" for a in target_alts)
        print(f"\n[4/5] Décollage → [{alts_str}]...")
        for i, conn in enumerate(connections):
            if not state.running:
                return
            takeoff(conn, locks[i], target_alts[i])
            time.sleep(2)

        print("  Attente altitudes cibles...")
        for i, conn in enumerate(connections):
            if not state.running:
                return
            reached = wait_altitude(conn, locks[i], target_alts[i], tolerance=1.0, timeout=30)
            current = get_altitude(conn, locks[i])
            current_str = f"{current:.1f}m" if current else "N/A"
            print(f"  Drone {i} : {current_str} / {target_alts[i]}m {'OK' if reached else '(timeout)'}")

        # Signal : drones en l'air
        state.drones_airborne.set()
        print(f"\n[5/5] Hover {args.hover}s...")

        # --- HOVER (affichage toutes les 5s) ---
        for s in range(args.hover):
            if not state.running:
                return
            time.sleep(1)
            if (s + 1) % 5 == 0 or s + 1 == args.hover:
                line = f"  t={s+1:3d}s |"
                for i, conn in enumerate(connections):
                    alt = get_altitude(conn, locks[i], timeout=1)
                    alt_str = f"{alt:5.1f}m" if alt else " N/A "
                    line += f" D{i}={alt_str}"
                print(line)

        # --- LAND ---
        print("\n[LAND] Atterrissage...")
        for i, conn in enumerate(connections):
            set_mode(conn, locks[i], MODE_LAND)
            print(f"  Drone {i} : LAND")

        print("  Descente en cours...")
        for s in range(30, 0, -10):
            if not state.running:
                return
            time.sleep(10)
            alts = []
            for i, conn in enumerate(connections):
                alt = get_altitude(conn, locks[i])
                alts.append(f"{alt:.1f}m" if alt else "N/A")
            print(f"  Altitudes : {', '.join(alts)}")

        print("\n  Vol terminé !")
        print(f"  Drones : {n} | Altitudes : [{alts_str}] | Hover : {args.hover}s")

    except Exception as e:
        print(f"\n  [FLIGHT] ERREUR: {e}")
    finally:
        state.flight_done.set()


# ============================================================
# THREAD 2 : Bridge (positions → RSSI/latence → CSV)
# ============================================================

def bridge_thread(connections, locks, state, args, ns3_process):
    """
    Lit les positions des drones en boucle et récupère les métriques NS-3.
    Tourne pendant toute la durée du vol.
    """
    n = state.n_drones
    interval = 1.0 / args.rate

    # --- Préparer le log unifié ---
    csv_file = open(UNIFIED_LOG, 'w', newline='')
    writer = csv.writer(csv_file)
    header = ['timestamp_s']
    for i in range(n):
        header.extend([f'd{i}_x', f'd{i}_y', f'd{i}_z'])
    for i in range(n):
        for j in range(i+1, n):
            header.extend([
                f'd{i}_d{j}_dist_m',
                f'd{i}_d{j}_rssi_dbm',
                f'd{i}_d{j}_latency_ms'
            ])
    writer.writerow(header)

    start_time = time.time()
    sample_count = 0



    try:
        while state.running and not state.flight_done.is_set():
            elapsed = time.time() - start_time

            # --- Lire les positions ---
            positions = []
            for i, conn in enumerate(connections):
                pos = get_local_position(conn, locks[i], timeout=1)
                positions.append(pos)
                # Mettre à jour l'état partagé
                with state.lock:
                    state.positions[i] = pos

            # --- Écrire le CSV partagé pour NS-3 ---
            write_positions_csv(positions)

            # --- Construire la ligne CSV ---
            row = [f"{elapsed:.3f}"]
            for pos in positions:
                if pos:
                    row.extend([f"{pos['x']:.3f}", f"{pos['y']:.3f}", f"{pos['z']:.3f}"])
                else:
                    row.extend(['NaN', 'NaN', 'NaN'])

            # --- Lire les dernières données NS-3 ---
            ns3_data = read_ns3_output()

            # --- Métriques par paire (depuis NS-3 / Sionna uniquement) ---
            for i in range(n):
                for j in range(i+1, n):
                    if positions[i] and positions[j]:
                        # Chercher la dernière entrée NS-3 pour cette paire
                        rssi = None
                        latency = None
                        dist = None
                        for d in reversed(ns3_data):
                            try:
                                di = int(d.get('drone_i', -1))
                                dj = int(d.get('drone_j', -1))
                                if di == i and dj == j:
                                    rssi = float(d['rssi_dbm'])
                                    latency = float(d['latency_ms'])
                                    dist = float(d['distance_m'])
                                    break
                            except (ValueError, KeyError):
                                continue

                        if dist is not None and rssi is not None and latency is not None:
                            row.extend([f"{dist:.3f}", f"{rssi:.1f}", f"{latency:.3f}"])
                        else:
                            # NS-3/Sionna pas encore prêt, on écrit NaN
                            row.extend(['NaN', 'NaN', 'NaN'])
                    else:
                        row.extend(['NaN', 'NaN', 'NaN'])

            writer.writerow(row)
            sample_count += 1

            if sample_count % 10 == 0:
                csv_file.flush()

            time.sleep(interval)

    except Exception as e:
        print(f"\n  [BRIDGE] ERREUR: {e}")
    finally:
        csv_file.close()
        if ns3_process:
            ns3_process.terminate()
            try:
                ns3_process.wait(timeout=5)
            except Exception:
                ns3_process.kill()




# ============================================================
# Programme principal
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description='Vol multi-drones + Bridge NS-3 (combiné)'
    )
    # Flight args
    parser.add_argument('--drones', type=int, default=3,
                        help='Nombre de drones (défaut: 3)')
    parser.add_argument('--altitude', type=float, default=4.0,
                        help='Altitude de base en mètres (défaut: 4)')
    parser.add_argument('--alt-step', type=float, default=1.0,
                        help='Écart d\'altitude entre drones (défaut: 1m)')
    parser.add_argument('--hover', type=int, default=15,
                        help='Durée du hover en secondes (défaut: 15)')
    parser.add_argument('--ekf-wait', type=int, default=20,
                        help='Attente EKF en secondes (défaut: 20)')
    # Bridge args
    parser.add_argument('--rate', type=float, default=2.0,
                        help='Fréquence bridge en Hz (défaut: 2)')
    parser.add_argument('--ns3-sim-time', type=int, default=0,
                        help='Durée simulation NS-3 (0=auto, défaut: auto)')
    parser.add_argument('--sionna-env', type=str, default='simple_room/simple_room.xml',
                        help='Scène Sionna XML (défaut: simple_room/simple_room.xml)')
    parser.add_argument('--sionna-url', type=str, default='tcp://localhost:5555',
                        help='URL du serveur Sionna ZMQ (défaut: tcp://localhost:5555)')
    args = parser.parse_args()

    n_drones = args.drones

    # Auto-calculer le simTime NS-3 pour couvrir tout le vol + marge
    if args.ns3_sim_time <= 0:
        # EKF wait + arm (~20s) + takeoff (~15s) + hover + landing (~35s) + marge
        args.ns3_sim_time = args.ekf_wait + 20 + 15 + args.hover + 35 + 30

    print()
    print("=" * 70)
    print(f"  VOL + BRIDGE COMBINÉ — {n_drones} DRONES")
    print(f"  Altitude : {args.altitude}m + {args.alt_step}m/drone")
    print(f"  Hover    : {args.hover}s")
    print(f"  NS-3     : real-time, simTime={args.ns3_sim_time}s @ {args.rate} Hz")
    print(f"  Canal    : sionna  (NS3-Sionna ray-tracing)")
    print("=" * 70)
    print()

    # ============================================================
    # Connexion UNIQUE aux drones
    # ============================================================
    print(f"[INIT] Connexion aux {n_drones} drones...")
    connections = []
    locks = []  # Un lock par connexion pour le thread-safety
    for i in range(n_drones):
        conn = connect_drone(i)
        if conn is None:
            print(f"\nERREUR: Drone {i} non accessible.")
            print("Vérifie que 06_launch_multi_drones.sh tourne.")
            sys.exit(1)
        connections.append(conn)
        locks.append(threading.Lock())
    print(f"  => {n_drones} drones connectés !\n")

    # Demander les flux de données
    for i, conn in enumerate(connections):
        request_data_streams(conn, locks[i])

    # ============================================================
    # Lancer NS-3 (obligatoire)
    # ============================================================
    ns3_process = None
    print("[INIT] Configuration NS-3 (mode real-time)...")
    ns3_exe = os.path.join(NS3_DIR, "ns3")
    if not os.path.exists(ns3_exe):
        print(f"  ERREUR: NS-3 non trouvé dans {NS3_DIR}")
        print(f"  Installe NS-3 d'abord (scripts 09 + 10).")
        sys.exit(1)

    # Toujours copier le scénario (peut avoir été mis à jour)
    import shutil
    scenario_src = os.path.join(WORKSPACE, "ns3_scenarios", "drone-wifi-scenario.cc")
    scenario_dst = os.path.join(NS3_DIR, "scratch", "drone-wifi-scenario.cc")
    os.makedirs(os.path.dirname(scenario_dst), exist_ok=True)
    shutil.copy2(scenario_src, scenario_dst)
    # Build NS-3
    print(f"  Build NS-3...", end=" ", flush=True)
    build_result = subprocess.run(
        [ns3_exe, "build"], cwd=NS3_DIR,
        capture_output=True, text=True, timeout=180
    )
    if build_result.returncode != 0:
        print(f"ERREUR")
        print(build_result.stderr[-500:] if build_result.stderr else "(pas de sortie)")
        sys.exit(1)
    print(f"OK")

    # Nettoyer l'ancien output NS-3
    if os.path.exists(NS3_OUTPUT):
        os.remove(NS3_OUTPUT)

    # Écrire les positions initiales (pour que NS-3 ait quelque chose à lire)
    write_positions_csv([{'x': 0, 'y': i * 3.0, 'z': 0.19} for i in range(n_drones)])

    # Lancer NS-3 en real-time
    cmd_args = (
        f"scratch/drone-wifi-scenario"
        f" --nDrones={n_drones}"
        f" --posFile={POS_FILE}"
        f" --outFile={NS3_OUTPUT}"
        f" --simTime={args.ns3_sim_time}"
        f" --updateInterval=0.5"
        f" --channelModel=sionna"
        f" --sionnaEnv={args.sionna_env}"
        f" --sionnaUrl={args.sionna_url}"
    )
    cmd = [ns3_exe, "run", cmd_args]

    try:
        ns3_process = subprocess.Popen(
            cmd, cwd=NS3_DIR,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        # Attendre un peu que NS-3 démarre
        time.sleep(2)
        if ns3_process.poll() is not None:
            stderr = ns3_process.stderr.read().decode('utf-8', errors='replace')
            print(f"  ERREUR: NS-3 s'est arrêté immédiatement")
            print(f"  stderr: {stderr[-500:]}")
            sys.exit(1)
        print(f"  NS-3 + Sionna actif ✓")
    except Exception as e:
        print(f"  ERREUR lancement NS-3: {e}")
        sys.exit(1)

    # ============================================================
    # État partagé
    # ============================================================
    state = SharedState(n_drones)

    def signal_handler(sig, frame):
        print("\n\n*** CTRL+C détecté — arrêt en cours... ***\n")
        state.running = False
        state.flight_done.set()

    signal.signal(signal.SIGINT, signal_handler)

    # ============================================================
    # Lancer les deux threads
    # ============================================================
    t_flight = threading.Thread(
        target=flight_thread,
        args=(connections, locks, state, args),
        name="FlightThread",
        daemon=True
    )
    t_bridge = threading.Thread(
        target=bridge_thread,
        args=(connections, locks, state, args, ns3_process),
        name="BridgeThread",
        daemon=True
    )



    t_bridge.start()
    t_flight.start()

    # Attendre que le vol soit terminé
    t_flight.join()

    # Laisser un peu de temps au bridge pour finir
    state.flight_done.set()
    t_bridge.join(timeout=5)

    # ============================================================
    # Nettoyage
    # ============================================================
    state.running = False

    print()
    print("=" * 70)
    print("  TERMINÉ — Vol + Bridge")
    print(f"  Log unifié : {UNIFIED_LOG}")
    print("=" * 70)
    print()

    for conn in connections:
        try:
            conn.close()
        except Exception:
            pass


if __name__ == '__main__':
    main()
