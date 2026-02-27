#!/usr/bin/env python3
"""
Script 14 : Bridge ArduPilot SITL ↔ NS-3 5G NR

Version 5G du script 11 (ns3_bridge.py). Fait le lien entre :
  - ArduPilot SITL (simulation physique des drones)
  - NS-3 + 5G-LENA (simulation réseau 5G NR)

Architecture 5G :
  - 1 gNB (station de base) au centre de l'entrepôt
  - N drones comme UEs connectés à la gNB
  - Communication inter-drones via : UE → gNB → EPC → gNB → UE
  - Métriques : RSRP (dBm), latence (ms), distance (m)

Différences avec le mode WiFi (script 11) :
  - Utilise drone-5g-scenario.cc au lieu de drone-wifi-scenario.cc
  - Métriques 5G : RSRP, distance à la gNB
  - Modèle de canal 3GPP TR 38.901 (InH, UMi, UMa, RMa)
  - Latence plus faible (~1ms TTI)

Usage :
    python3 14_ns3_5g_bridge.py --drones 3
    python3 14_ns3_5g_bridge.py --drones 3 --duration 60 --frequency 3.5e9
    python3 14_ns3_5g_bridge.py --drones 3 --no-ns3   (mode standalone)

Prérequis :
  - 06_launch_multi_drones.sh tourne (drones actifs)
  - NS-3 + 5G-LENA installés (scripts 09 + 13)
"""

import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import time
from datetime import datetime

try:
    from pymavlink import mavutil
except ImportError:
    print("ERREUR: pymavlink non installé.")
    print("pip3 install pymavlink")
    sys.exit(1)


# ============================================================
# Configuration
# ============================================================

POS_FILE     = "/tmp/drone_positions.csv"
NS3_OUTPUT   = "/tmp/ns3_5g_output.csv"
WORKSPACE    = os.path.expanduser("~/simulation_mc02")
UNIFIED_LOG  = os.path.join(WORKSPACE, "comm_metrics_5g.csv")

NS3_DIR      = os.path.expanduser("~/ns-allinone-3.40/ns-3.40")
NS3_SCENARIO = "scratch/drone-5g-scenario"


# ============================================================
# Modèle de propagation standalone (3GPP InH-Office)
# Utilisé quand NS-3 n'est pas disponible
# ============================================================

def compute_rsrp_standalone(pos_drone, pos_gnb, tx_power_dbm=30.0, frequency_ghz=3.5):
    """
    Calcul du RSRP entre un drone et la gNB sans NS-3.
    Modèle 3GPP TR 38.901 InH-Office simplifié (LOS) :
      PL_InH_LOS = 32.4 + 17.3 * log10(d_3D) + 20 * log10(fc)
    Params : fc en GHz, d_3D en mètres
    """
    dx = pos_drone['x'] - pos_gnb['x']
    dy = pos_drone['y'] - pos_gnb['y']
    dz = pos_drone['z'] - pos_gnb['z']
    distance = math.sqrt(dx*dx + dy*dy + dz*dz)

    if distance < 0.1:
        distance = 0.1

    # 3GPP TR 38.901 Table 7.4.1-1 : InH-Office LOS
    path_loss = 32.4 + 17.3 * math.log10(distance) + 20.0 * math.log10(frequency_ghz)
    rsrp = tx_power_dbm - path_loss
    return rsrp, distance


def compute_latency_5g_standalone(distance, numerology=1):
    """
    Estimation de la latence 5G NR standalone.
    Slot duration = 1ms / (2^μ)
    Latence ≈ 2 * slot_duration (UL + DL) + propagation + processing
    """
    slot_duration_ms = 1.0 / (2 ** numerology)  # μ=1 → 0.5ms
    propagation_ms = (distance / 3e8) * 1000.0
    processing_ms = 0.5  # Processing + scheduling
    # UL + DL + processing
    return 2 * slot_duration_ms + propagation_ms + processing_ms


# ============================================================
# Fonctions drone
# ============================================================

def connect_drone(instance, timeout=30):
    """Connexion à une instance ArduPilot SITL."""
    port = 5760 + instance * 10
    addr = f"tcp:127.0.0.1:{port}"
    print(f"  Drone {instance} : {addr}...", end=" ", flush=True)
    try:
        conn = mavutil.mavlink_connection(addr, source_system=255)
        conn.wait_heartbeat(timeout=timeout)
        print(f"OK (sysid={conn.target_system})")
        return conn
    except Exception as e:
        print(f"ERREUR : {e}")
        return None


def get_local_position(conn, timeout=2):
    """Récupère la position locale NED."""
    msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=timeout)
    if msg:
        return {
            'x': msg.x,
            'y': msg.y,
            'z': -msg.z,  # NED z-down → z-up
        }
    return None


def request_streams(conn, rate_hz=4):
    conn.mav.request_data_stream_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION, rate_hz, 1
    )


# ============================================================
# Écriture/Lecture CSV
# ============================================================

def write_positions_csv(positions):
    """Écrit les positions actuelles pour NS-3."""
    with open(POS_FILE, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['drone_id', 'x', 'y', 'z'])
        for i, pos in enumerate(positions):
            if pos:
                writer.writerow([i, f"{pos['x']:.4f}", f"{pos['y']:.4f}", f"{pos['z']:.4f}"])
            else:
                writer.writerow([i, 0, 0, 0])


def read_ns3_5g_output():
    """Lit les dernières métriques 5G de NS-3."""
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
# Programme principal
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description='Bridge ArduPilot SITL ↔ NS-3 5G NR (RSRP + latence)'
    )
    parser.add_argument('--drones', type=int, default=3,
                        help='Nombre de drones (défaut: 3)')
    parser.add_argument('--rate', type=float, default=2.0,
                        help='Fréquence de mise à jour en Hz (défaut: 2)')
    parser.add_argument('--duration', type=int, default=0,
                        help='Durée en secondes, 0=infini (défaut: 0)')
    parser.add_argument('--no-ns3', action='store_true',
                        help='Mode standalone : calcul RSRP/latence sans NS-3')
    parser.add_argument('--ns3-sim-time', type=int, default=60,
                        help='Durée de simulation NS-3 en secondes (défaut: 60)')
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
    interval = 1.0 / args.rate
    use_ns3 = not args.no_ns3
    frequency_ghz = args.frequency / 1e9

    # Position gNB
    gnb_pos = {'x': args.gnb_x, 'y': args.gnb_y, 'z': args.gnb_z}

    print()
    print("=" * 70)
    print(f"  BRIDGE ArduPilot ↔ NS-3 5G NR — {n_drones} drones")
    print(f"  Mode      : {'NS-3 + 5G-LENA' if use_ns3 else 'Standalone (3GPP InH)'}")
    print(f"  Fréquence : {frequency_ghz} GHz ({'FR1' if frequency_ghz < 7 else 'FR2'})")
    print(f"  Bande     : {args.bandwidth / 1e6} MHz")
    print(f"  Numérologie: μ={args.numerology} (SCS={15 * (2**args.numerology)} kHz)")
    print(f"  Scénario  : {args.scenario}")
    print(f"  gNB       : ({args.gnb_x}, {args.gnb_y}, {args.gnb_z})")
    print("=" * 70)
    print()

    # ============================================================
    # Connexion aux drones
    # ============================================================
    print(f"[1/3] Connexion aux {n_drones} drones...")
    connections = []
    for i in range(n_drones):
        conn = connect_drone(i)
        if conn is None:
            print(f"\nERREUR: Drone {i} inaccessible.")
            print("Vérifie que 06_launch_multi_drones.sh tourne.")
            sys.exit(1)
        connections.append(conn)
    print()

    for conn in connections:
        request_streams(conn)

    # ============================================================
    # Lancer NS-3 5G si activé
    # ============================================================
    ns3_process = None
    if use_ns3:
        print("[2/3] Lancement de NS-3 5G NR...")
        ns3_exe = os.path.join(NS3_DIR, "ns3")
        if not os.path.exists(ns3_exe):
            print(f"  ATTENTION: NS-3 non trouvé dans {NS3_DIR}")
            print(f"  Passage en mode standalone.")
            use_ns3 = False
        else:
            # Copier le scénario 5G dans scratch/
            scenario_src = os.path.join(WORKSPACE, "ns3_scenarios", "drone-5g-scenario.cc")
            scenario_dst = os.path.join(NS3_DIR, "scratch", "drone-5g-scenario.cc")
            if os.path.exists(scenario_src):
                os.makedirs(os.path.dirname(scenario_dst), exist_ok=True)
                import shutil
                shutil.copy2(scenario_src, scenario_dst)
                print(f"  Scénario 5G copié dans scratch/")
                # Build
                print(f"  Build NS-3 + 5G-LENA...", end=" ", flush=True)
                build_result = subprocess.run(
                    [ns3_exe, "build"], cwd=NS3_DIR,
                    capture_output=True, text=True, timeout=300
                )
                if build_result.returncode != 0:
                    print("ERREUR")
                    print(build_result.stderr[-500:] if build_result.stderr else "")
                    print("  Passage en mode standalone.")
                    use_ns3 = False
                else:
                    print("OK")

        if use_ns3:
            # Nettoyer l'ancien output
            if os.path.exists(NS3_OUTPUT):
                os.remove(NS3_OUTPUT)

            # Écrire les positions initiales
            write_positions_csv([{'x': 0, 'y': i * 3.0, 'z': 0.19} for i in range(n_drones)])

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
                time.sleep(3)  # Plus de temps pour l'init 5G NR
                if ns3_process.poll() is not None:
                    stderr = ns3_process.stderr.read().decode('utf-8', errors='replace')
                    print(f"  ERREUR: NS-3 5G s'est arrêté immédiatement")
                    print(f"  stderr: {stderr[-500:]}")
                    print(f"  Passage en mode standalone.")
                    use_ns3 = False
                else:
                    print(f"  NS-3 5G NR actif (PID={ns3_process.pid}) ✓")
            except Exception as e:
                print(f"  ERREUR lancement NS-3 5G: {e}")
                print(f"  Passage en mode standalone.")
                use_ns3 = False
    else:
        print("[2/3] Mode standalone — pas de NS-3")

    print()

    # ============================================================
    # Préparer le log unifié
    # ============================================================
    print(f"[3/3] Démarrage du bridge 5G...")
    print(f"  Log unifié  : {UNIFIED_LOG}")
    print(f"  Positions   : {POS_FILE}")
    if use_ns3:
        print(f"  NS-3 output : {NS3_OUTPUT}")
    print()

    csv_file = open(UNIFIED_LOG, 'w', newline='')
    writer = csv.writer(csv_file)

    # En-tête CSV avec métriques 5G
    header = ['timestamp_s']
    for i in range(n_drones):
        header.extend([f'd{i}_x', f'd{i}_y', f'd{i}_z'])
    for i in range(n_drones):
        header.append(f'd{i}_dist_gnb_m')
        header.append(f'd{i}_rsrp_dbm')
    for i in range(n_drones):
        for j in range(i+1, n_drones):
            header.extend([
                f'd{i}_d{j}_dist_m',
                f'd{i}_d{j}_rsrp_pair_dbm',
                f'd{i}_d{j}_latency_ms'
            ])
    writer.writerow(header)

    # ============================================================
    # En-tête d'affichage
    # ============================================================
    print("┌──────┬", end="")
    for i in range(n_drones):
        print(f"── Drone {i} ──┬", end="")
    for i in range(n_drones):
        for j in range(i+1, n_drones):
            print(f"── {i}↔{j} RSRP/Lat ──┬", end="")
    print()

    # ============================================================
    # Boucle principale
    # ============================================================
    running = True

    def signal_handler(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, signal_handler)

    start_time = time.time()
    sample_count = 0

    try:
        while running:
            elapsed = time.time() - start_time
            if args.duration > 0 and elapsed > args.duration:
                break

            # --- Lire les positions ---
            positions = []
            for conn in connections:
                pos = get_local_position(conn, timeout=1)
                positions.append(pos)

            # --- Écrire le CSV partagé pour NS-3 ---
            write_positions_csv(positions)

            # --- Construire la ligne CSV ---
            row = [f"{elapsed:.3f}"]

            # Positions
            for pos in positions:
                if pos:
                    row.extend([f"{pos['x']:.3f}", f"{pos['y']:.3f}", f"{pos['z']:.3f}"])
                else:
                    row.extend(['NaN', 'NaN', 'NaN'])

            # Métriques par drone (distance et RSRP vers la gNB)
            drone_rsrps = []
            for i in range(n_drones):
                if positions[i]:
                    rsrp, dist_gnb = compute_rsrp_standalone(
                        positions[i], gnb_pos,
                        tx_power_dbm=30.0, frequency_ghz=frequency_ghz
                    )
                    drone_rsrps.append(rsrp)
                    row.extend([f"{dist_gnb:.3f}", f"{rsrp:.1f}"])
                else:
                    drone_rsrps.append(None)
                    row.extend(['NaN', 'NaN'])

            # Métriques par paire
            line = f" {elapsed:6.1f}s |"
            for i, pos in enumerate(positions):
                if pos:
                    line += f" ({pos['x']:5.1f},{pos['y']:5.1f},{pos['z']:4.1f}) |"
                else:
                    line += "    N/A       |"

            for i in range(n_drones):
                for j in range(i+1, n_drones):
                    if positions[i] and positions[j]:
                        if use_ns3:
                            # Lire depuis NS-3 5G output
                            ns3_data = read_ns3_5g_output()
                            rsrp = None
                            latency = None
                            dist = None
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
                        else:
                            # Mode standalone
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
                        line += f" {rsrp:6.1f}dBm {latency:5.2f}ms |"
                    else:
                        row.extend(['NaN', 'NaN', 'NaN'])
                        line += "    N/A          |"

            writer.writerow(row)
            sample_count += 1

            print(line)

            if sample_count % 10 == 0:
                csv_file.flush()

            time.sleep(interval)

    except KeyboardInterrupt:
        pass
    finally:
        csv_file.close()

        if ns3_process:
            ns3_process.terminate()
            try:
                ns3_process.wait(timeout=5)
            except Exception:
                ns3_process.kill()

        print()
        print("=" * 70)
        print(f"  Bridge 5G arrêté — {sample_count} échantillons")
        print(f"  Log unifié : {UNIFIED_LOG}")
        print(f"  Durée      : {time.time() - start_time:.1f}s")
        print("=" * 70)

        for conn in connections:
            try:
                conn.close()
            except Exception:
                pass


if __name__ == '__main__':
    main()
