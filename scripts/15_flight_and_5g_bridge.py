#!/usr/bin/env python3
"""
Script 15 : Vol multi-drones + Bridge NS-3 5G NR — combiné

Version 5G du script 12 (flight_and_bridge.py). Fusionne le vol
automatique des drones et le bridge NS-3 5G NR dans un seul processus.

Architecture 5G :
  - 1 gNB (station de base) au centre de l'entrepôt
  - N drones comme UEs connectés à la gNB via 5G NR
  - Métriques collectées : RSRP, latence, distance à la gNB

Usage :
    python3 15_flight_and_5g_bridge.py --drones 3
    python3 15_flight_and_5g_bridge.py --drones 3 --altitude 5 --hover 30
    python3 15_flight_and_5g_bridge.py --drones 3 --frequency 28e9 --numerology 3

Prérequis :
  - 06_launch_multi_drones.sh doit tourner dans un autre terminal
  - NS-3 + 5G-LENA installés (scripts 09 + 13)
"""

import argparse
import csv
import math
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

POS_FILE     = "/tmp/drone_positions.csv"
NS3_OUTPUT   = "/tmp/ns3_5g_output.csv"
WORKSPACE    = os.path.expanduser("~/simulation_mc02")
UNIFIED_LOG  = os.path.join(WORKSPACE, "comm_metrics_5g.csv")
NS3_DIR      = os.path.expanduser("~/ns-allinone-3.40/ns-3.40")


# ============================================================
# État partagé (thread-safe)
# ============================================================

class SharedState:
    """État partagé entre le thread flight et le thread bridge."""
    def __init__(self, n_drones):
        self.n_drones = n_drones
        self.lock = threading.Lock()
        self.positions = [None] * n_drones
        self.running = True
        self.drones_airborne = threading.Event()
        self.flight_done = threading.Event()


# ============================================================
# Modèle de propagation standalone (3GPP InH-Office)
# ============================================================

def compute_rsrp_standalone(pos_drone, pos_gnb, tx_power_dbm=30.0, frequency_ghz=3.5):
    """
    Calcul du RSRP 3GPP TR 38.901 InH-Office LOS simplifié.
    PL_InH_LOS = 32.4 + 17.3 * log10(d_3D) + 20 * log10(fc)
    """
    dx = pos_drone['x'] - pos_gnb['x']
    dy = pos_drone['y'] - pos_gnb['y']
    dz = pos_drone['z'] - pos_gnb['z']
    distance = math.sqrt(dx*dx + dy*dy + dz*dz)

    if distance < 0.1:
        distance = 0.1

    path_loss = 32.4 + 17.3 * math.log10(distance) + 20.0 * math.log10(frequency_ghz)
    rsrp = tx_power_dbm - path_loss
    return rsrp, distance


def compute_latency_5g_standalone(distance, numerology=1):
    """
    Estimation de la latence 5G NR.
    Slot duration = 1ms / 2^μ, latence ≈ 2 slots + propagation + processing
    """
    slot_duration_ms = 1.0 / (2 ** numerology)
    propagation_ms = (distance / 3e8) * 1000.0
    processing_ms = 0.5
    return 2 * slot_duration_ms + propagation_ms + processing_ms


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


def read_ns3_5g_output():
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
    """Séquence de vol complète, identique au script 12."""
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

        state.drones_airborne.set()
        print(f"\n[5/5] Hover {args.hover}s...")

        # --- HOVER ---
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

        print(f"\n  Vol terminé ! ({n} drones, altitudes [{alts_str}], hover {args.hover}s)")

    except Exception as e:
        print(f"\n  [FLIGHT] ERREUR: {e}")
    finally:
        state.flight_done.set()


# ============================================================
# THREAD 2 : Bridge 5G (positions → RSRP/latence → CSV)
# ============================================================

def bridge_thread(connections, locks, state, args, ns3_process):
    """
    Lit les positions des drones et collecte les métriques 5G NR.
    """
    n = state.n_drones
    interval = 1.0 / args.rate
    frequency_ghz = args.frequency / 1e9

    # Position gNB
    gnb_pos = {'x': args.gnb_x, 'y': args.gnb_y, 'z': args.gnb_z}

    # --- Préparer le log unifié ---
    csv_file = open(UNIFIED_LOG, 'w', newline='')
    writer = csv.writer(csv_file)

    # En-tête avec métriques 5G
    header = ['timestamp_s']
    for i in range(n):
        header.extend([f'd{i}_x', f'd{i}_y', f'd{i}_z'])
    for i in range(n):
        header.extend([f'd{i}_dist_gnb_m', f'd{i}_rsrp_dbm'])
    for i in range(n):
        for j in range(i+1, n):
            header.extend([
                f'd{i}_d{j}_dist_m',
                f'd{i}_d{j}_rsrp_pair_dbm',
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
                with state.lock:
                    state.positions[i] = pos

            # --- Écrire le CSV pour NS-3 ---
            write_positions_csv(positions)

            # --- Construire la ligne CSV ---
            row = [f"{elapsed:.3f}"]

            # Positions
            for pos in positions:
                if pos:
                    row.extend([f"{pos['x']:.3f}", f"{pos['y']:.3f}", f"{pos['z']:.3f}"])
                else:
                    row.extend(['NaN', 'NaN', 'NaN'])

            # Métriques par drone (distance et RSRP vers gNB)
            for i in range(n):
                if positions[i]:
                    rsrp, dist_gnb = compute_rsrp_standalone(
                        positions[i], gnb_pos, 30.0, frequency_ghz)
                    row.extend([f"{dist_gnb:.3f}", f"{rsrp:.1f}"])
                else:
                    row.extend(['NaN', 'NaN'])

            # --- Lire les données NS-3 5G ---
            ns3_data = read_ns3_5g_output()

            # --- Métriques par paire ---
            for i in range(n):
                for j in range(i+1, n):
                    if positions[i] and positions[j]:
                        rsrp = None
                        latency = None
                        dist = None

                        # Chercher dans les données NS-3
                        for d in reversed(ns3_data):
                            try:
                                di = int(d.get('drone_i', -1))
                                dj = int(d.get('drone_j', -1))
                                if di == i and dj == j:
                                    rsrp = float(d['rsrp_dbm'])
                                    latency = float(d['latency_ms'])
                                    dist = float(d['distance_m'])
                                    break
                            except (ValueError, KeyError):
                                continue

                        if rsrp is None:
                            # Fallback standalone
                            rsrp_i, _ = compute_rsrp_standalone(
                                positions[i], gnb_pos, 30.0, frequency_ghz)
                            rsrp_j, _ = compute_rsrp_standalone(
                                positions[j], gnb_pos, 30.0, frequency_ghz)
                            rsrp = min(rsrp_i, rsrp_j)

                            dx = positions[i]['x'] - positions[j]['x']
                            dy = positions[i]['y'] - positions[j]['y']
                            dz = positions[i]['z'] - positions[j]['z']
                            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                            latency = compute_latency_5g_standalone(
                                dist, args.numerology)

                        row.extend([f"{dist:.3f}", f"{rsrp:.1f}", f"{latency:.3f}"])
                    else:
                        row.extend(['NaN', 'NaN', 'NaN'])

            writer.writerow(row)
            sample_count += 1

            if sample_count % 10 == 0:
                csv_file.flush()

            time.sleep(interval)

    except Exception as e:
        print(f"\n  [BRIDGE 5G] ERREUR: {e}")
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
        description='Vol multi-drones + Bridge NS-3 5G NR (combiné)'
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
    # Bridge 5G args
    parser.add_argument('--rate', type=float, default=2.0,
                        help='Fréquence bridge en Hz (défaut: 2)')
    parser.add_argument('--ns3-sim-time', type=int, default=0,
                        help='Durée simulation NS-3 (0=auto)')
    parser.add_argument('--frequency', type=float, default=3.5e9,
                        help='Fréquence 5G en Hz (défaut: 3.5e9 = FR1)')
    parser.add_argument('--bandwidth', type=float, default=20e6,
                        help='Bande passante en Hz (défaut: 20e6)')
    parser.add_argument('--numerology', type=int, default=1,
                        help='Numérologie NR μ (défaut: 1 → 30kHz SCS)')
    parser.add_argument('--scenario', type=str, default='InH-OfficeMixed',
                        help='Scénario 3GPP (défaut: InH-OfficeMixed)')
    parser.add_argument('--gnb-x', type=float, default=15.0,
                        help='Position X de la gNB (défaut: 15m)')
    parser.add_argument('--gnb-y', type=float, default=10.0,
                        help='Position Y de la gNB (défaut: 10m)')
    parser.add_argument('--gnb-z', type=float, default=3.0,
                        help='Position Z de la gNB (défaut: 3m)')
    args = parser.parse_args()

    n_drones = args.drones
    frequency_ghz = args.frequency / 1e9

    # Auto-calculer le simTime NS-3
    if args.ns3_sim_time <= 0:
        args.ns3_sim_time = args.ekf_wait + 20 + 15 + args.hover + 35 + 30

    print()
    print("=" * 70)
    print(f"  VOL + BRIDGE 5G NR COMBINÉ — {n_drones} DRONES")
    print(f"  Altitude  : {args.altitude}m + {args.alt_step}m/drone")
    print(f"  Hover     : {args.hover}s")
    print(f"  Fréquence : {frequency_ghz} GHz ({'FR1' if frequency_ghz < 7 else 'FR2 mmWave'})")
    print(f"  Bande     : {args.bandwidth / 1e6} MHz")
    print(f"  Numérologie: μ={args.numerology} (SCS={15 * (2**args.numerology)} kHz)")
    print(f"  Scénario  : {args.scenario}")
    print(f"  gNB       : ({args.gnb_x}, {args.gnb_y}, {args.gnb_z})")
    print(f"  NS-3      : simTime={args.ns3_sim_time}s @ {args.rate} Hz")
    print("=" * 70)
    print()

    # ============================================================
    # Connexion UNIQUE aux drones
    # ============================================================
    print(f"[INIT] Connexion aux {n_drones} drones...")
    connections = []
    locks = []
    for i in range(n_drones):
        conn = connect_drone(i)
        if conn is None:
            print(f"\nERREUR: Drone {i} non accessible.")
            print("Vérifie que 06_launch_multi_drones.sh tourne.")
            sys.exit(1)
        connections.append(conn)
        locks.append(threading.Lock())
    print(f"  => {n_drones} drones connectés !\n")

    for i, conn in enumerate(connections):
        request_data_streams(conn, locks[i])

    # ============================================================
    # Lancer NS-3 5G NR
    # ============================================================
    ns3_process = None
    print("[INIT] Configuration NS-3 5G NR (mode real-time)...")
    ns3_exe = os.path.join(NS3_DIR, "ns3")
    if not os.path.exists(ns3_exe):
        print(f"  ERREUR: NS-3 non trouvé dans {NS3_DIR}")
        print(f"  Installe NS-3 + 5G-LENA d'abord (scripts 09 + 13).")
        sys.exit(1)

    # Copier le scénario 5G
    import shutil
    scenario_src = os.path.join(WORKSPACE, "ns3_scenarios", "drone-5g-scenario.cc")
    scenario_dst = os.path.join(NS3_DIR, "scratch", "drone-5g-scenario.cc")
    os.makedirs(os.path.dirname(scenario_dst), exist_ok=True)
    shutil.copy2(scenario_src, scenario_dst)

    # Build NS-3
    print(f"  Build NS-3 + 5G-LENA...", end=" ", flush=True)
    build_result = subprocess.run(
        [ns3_exe, "build"], cwd=NS3_DIR,
        capture_output=True, text=True, timeout=300
    )
    if build_result.returncode != 0:
        print(f"ERREUR")
        print(build_result.stderr[-500:] if build_result.stderr else "(pas de sortie)")
        sys.exit(1)
    print(f"OK")

    # Nettoyer l'ancien output
    if os.path.exists(NS3_OUTPUT):
        os.remove(NS3_OUTPUT)

    # Positions initiales
    write_positions_csv([{'x': 0, 'y': i * 3.0, 'z': 0.19} for i in range(n_drones)])

    # Lancer NS-3 5G NR en real-time
    cmd_args = (
        f"scratch/drone-5g-scenario"
        f" --nDrones={n_drones}"
        f" --posFile={POS_FILE}"
        f" --outFile={NS3_OUTPUT}"
        f" --simTime={args.ns3_sim_time}"
        f" --updateInterval=0.5"
        f" --frequency={args.frequency}"
        f" --bandwidth={args.bandwidth}"
        f" --numerology={args.numerology}"
        f" --scenario={args.scenario}"
        f" --gnbX={args.gnb_x}"
        f" --gnbY={args.gnb_y}"
        f" --gnbZ={args.gnb_z}"
    )
    cmd = [ns3_exe, "run", cmd_args]

    try:
        ns3_process = subprocess.Popen(
            cmd, cwd=NS3_DIR,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        time.sleep(3)
        if ns3_process.poll() is not None:
            stderr = ns3_process.stderr.read().decode('utf-8', errors='replace')
            print(f"  ERREUR: NS-3 5G NR s'est arrêté immédiatement")
            print(f"  stderr: {stderr[-500:]}")
            sys.exit(1)
        print(f"  NS-3 5G NR actif ✓")
    except Exception as e:
        print(f"  ERREUR lancement NS-3 5G: {e}")
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
        name="Bridge5GThread",
        daemon=True
    )

    t_bridge.start()
    t_flight.start()

    # Attendre la fin du vol
    t_flight.join()

    state.flight_done.set()
    t_bridge.join(timeout=5)

    # ============================================================
    # Nettoyage
    # ============================================================
    state.running = False

    print()
    print("=" * 70)
    print("  TERMINÉ — Vol + Bridge 5G NR")
    print(f"  Log unifié : {UNIFIED_LOG}")
    print(f"  Technologie: 5G NR ({frequency_ghz} GHz, μ={args.numerology})")
    print("=" * 70)
    print()

    for conn in connections:
        try:
            conn.close()
        except Exception:
            pass


if __name__ == '__main__':
    main()
