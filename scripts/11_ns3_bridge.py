#!/usr/bin/env python3
"""
Script 11 : Bridge ArduPilot SITL ↔ NS-3

Ce script est le CŒUR de la Phase 3. Il fait le lien entre :
  - ArduPilot SITL (simulation physique des drones)
  - NS-3 (simulation réseau / communication)

Fonctionnement :
  1. Se connecte aux N drones via pymavlink
  2. Lit les positions en temps réel
  3. Écrit les positions dans un CSV partagé (/tmp/drone_positions.csv)
  4. Lance NS-3 en arrière-plan (lit le CSV, écrit RSSI + latence)
  5. Lit les résultats NS-3 et les affiche
  6. Injecte les délais de communication dans un log unifié

Usage :
    python3 11_ns3_bridge.py --drones 3
    python3 11_ns3_bridge.py --drones 3 --duration 60 --ns3-sim-time 60
    python3 11_ns3_bridge.py --drones 3 --no-ns3   (mode standalone, sans NS-3)

Prérequis :
  - 06_launch_multi_drones.sh tourne (drones actifs)
  - NS-3 installé (09 + 10) — OU mode --no-ns3 pour test

Sortie :
  - /tmp/drone_positions.csv   : positions mises à jour en temps réel
  - /tmp/ns3_output.csv        : RSSI + latence par paire (sortie NS-3)
  - ~/simulation_mc02/comm_metrics.csv : log unifié (positions + RSSI + latence)
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

POS_FILE = "/tmp/drone_positions.csv"
NS3_OUTPUT = "/tmp/ns3_output.csv"
WORKSPACE = os.path.expanduser("~/simulation_mc02")
UNIFIED_LOG = os.path.join(WORKSPACE, "comm_metrics.csv")

NS3_DIR = os.path.expanduser("~/ns-allinone-3.40/ns-3.40")
NS3_SCENARIO = "scratch/drone-wifi-scenario"


# ============================================================
# Modèle de propagation standalone (quand NS-3 n'est pas dispo)
# Log-Distance Path Loss Model — indoor warehouse
# ============================================================

def compute_rssi_standalone(pos_i, pos_j, tx_power_dbm=20.0, path_loss_exp=3.0, ref_loss_db=40.0):
    """
    Calcul du RSSI entre deux drones sans NS-3.
    Modèle Log-Distance : PL(d) = PL(d0) + 10*n*log10(d/d0)
    Params indoor warehouse : n=3.0, PL(1m)=40dB, TxPower=20dBm
    """
    dx = pos_i['x'] - pos_j['x']
    dy = pos_i['y'] - pos_j['y']
    dz = pos_i['z'] - pos_j['z']
    distance = math.sqrt(dx*dx + dy*dy + dz*dz)

    if distance < 0.01:
        distance = 0.01

    path_loss = ref_loss_db + 10.0 * path_loss_exp * math.log10(distance)
    rssi = tx_power_dbm - path_loss
    return rssi, distance


def compute_latency_standalone(distance, mac_delay_ms=2.0):
    """
    Estimation de la latence : propagation + MAC contention.
    Propagation : d / c (speed of light)
    MAC delay : ~2ms (WiFi Ad-Hoc typique)
    """
    propagation_ms = (distance / 3e8) * 1000.0
    return propagation_ms + mac_delay_ms


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
# Écriture des positions pour NS-3
# ============================================================

def write_positions_csv(positions):
    """Écrit les positions actuelles dans le fichier partagé pour NS-3."""
    with open(POS_FILE, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['drone_id', 'x', 'y', 'z'])
        for i, pos in enumerate(positions):
            if pos:
                writer.writerow([i, f"{pos['x']:.4f}", f"{pos['y']:.4f}", f"{pos['z']:.4f}"])
            else:
                writer.writerow([i, 0, 0, 0])


# ============================================================
# Lecture des résultats NS-3
# ============================================================

def read_ns3_output():
    """Lit les dernières métriques de NS-3."""
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
        description='Bridge ArduPilot SITL ↔ NS-3 (RSSI + latence)'
    )
    parser.add_argument('--drones', type=int, default=3,
                        help='Nombre de drones (défaut: 3)')
    parser.add_argument('--rate', type=float, default=2.0,
                        help='Fréquence de mise à jour en Hz (défaut: 2)')
    parser.add_argument('--duration', type=int, default=0,
                        help='Durée en secondes, 0=infini (défaut: 0)')
    parser.add_argument('--no-ns3', action='store_true',
                        help='Mode standalone : calcul RSSI/latence sans NS-3')
    parser.add_argument('--ns3-sim-time', type=int, default=60,
                        help='Durée de simulation NS-3 en secondes (défaut: 60)')
    args = parser.parse_args()

    n_drones = args.drones
    interval = 1.0 / args.rate
    use_ns3 = not args.no_ns3

    print()
    print("=" * 70)
    print(f"  BRIDGE ArduPilot ↔ NS-3 — {n_drones} drones")
    print(f"  Mode : {'NS-3' if use_ns3 else 'Standalone (Log-Distance Model)'}")
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
    # Lancer NS-3 si activé
    # ============================================================
    ns3_process = None
    if use_ns3:
        print("[2/3] Lancement de NS-3...")
        ns3_exe = os.path.join(NS3_DIR, "ns3")
        if not os.path.exists(ns3_exe):
            print(f"  ATTENTION: NS-3 non trouvé dans {NS3_DIR}")
            print(f"  Passage en mode standalone.")
            use_ns3 = False
        else:
            # Copier le scénario dans scratch/ si nécessaire
            scenario_src = os.path.join(WORKSPACE, "ns3_scenarios", "drone-wifi-scenario.cc")
            scenario_dst = os.path.join(NS3_DIR, "scratch", "drone-wifi-scenario.cc")
            if os.path.exists(scenario_src) and not os.path.exists(scenario_dst):
                os.makedirs(os.path.dirname(scenario_dst), exist_ok=True)
                import shutil
                shutil.copy2(scenario_src, scenario_dst)
                print(f"  Scénario copié dans scratch/")
                # Build
                subprocess.run([ns3_exe, "build"], cwd=NS3_DIR,
                             capture_output=True, timeout=120)

            cmd = [
                ns3_exe, "run",
                f"scratch/drone-wifi-scenario"
                f" --nDrones={n_drones}"
                f" --posFile={POS_FILE}"
                f" --outFile={NS3_OUTPUT}"
                f" --simTime={args.ns3_sim_time}"
            ]
            try:
                ns3_process = subprocess.Popen(
                    cmd, cwd=NS3_DIR,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE
                )
                print(f"  NS-3 lancé (PID={ns3_process.pid})")
            except Exception as e:
                print(f"  ERREUR lancement NS-3: {e}")
                print(f"  Passage en mode standalone.")
                use_ns3 = False
    else:
        print("[2/3] Mode standalone — pas de NS-3")

    print()

    # ============================================================
    # Préparer le log unifié
    # ============================================================
    print(f"[3/3] Démarrage du bridge...")
    print(f"  Log unifié : {UNIFIED_LOG}")
    print(f"  Positions  : {POS_FILE}")
    if use_ns3:
        print(f"  NS-3 output: {NS3_OUTPUT}")
    print()

    csv_file = open(UNIFIED_LOG, 'w', newline='')
    writer = csv.writer(csv_file)
    header = ['timestamp_s']
    for i in range(n_drones):
        header.extend([f'd{i}_x', f'd{i}_y', f'd{i}_z'])
    # Paires de drones
    for i in range(n_drones):
        for j in range(i+1, n_drones):
            header.extend([
                f'd{i}_d{j}_dist_m',
                f'd{i}_d{j}_rssi_dbm',
                f'd{i}_d{j}_latency_ms'
            ])
    writer.writerow(header)

    # ============================================================
    # En-tête d'affichage
    # ============================================================
    print("┌──────┬", end="")
    for i in range(n_drones):
        print(f"─── Drone {i} ────┬", end="")
    for i in range(n_drones):
        for j in range(i+1, n_drones):
            print(f"── {i}↔{j} RSSI/Lat ──┬", end="")
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

            # --- Calculer les métriques ---
            row = [f"{elapsed:.3f}"]

            # Positions
            for pos in positions:
                if pos:
                    row.extend([f"{pos['x']:.3f}", f"{pos['y']:.3f}", f"{pos['z']:.3f}"])
                else:
                    row.extend(['NaN', 'NaN', 'NaN'])

            # Métriques par paire
            line = f" {elapsed:6.1f}s |"
            for i, pos in enumerate(positions):
                if pos:
                    line += f" ({pos['x']:6.1f},{pos['y']:6.1f},{pos['z']:4.1f}) |"
                else:
                    line += "    N/A       |"

            pair_metrics = []
            for i in range(n_drones):
                for j in range(i+1, n_drones):
                    if positions[i] and positions[j]:
                        if use_ns3:
                            # Lire depuis NS-3 output
                            ns3_data = read_ns3_output()
                            rssi = None
                            latency = None
                            for d in reversed(ns3_data):
                                if int(d.get('drone_i', -1)) == i and int(d.get('drone_j', -1)) == j:
                                    rssi = float(d['rssi_dbm'])
                                    latency = float(d['latency_ms'])
                                    break
                            if rssi is None:
                                rssi, dist = compute_rssi_standalone(positions[i], positions[j])
                                latency = compute_latency_standalone(dist)
                        else:
                            rssi, dist = compute_rssi_standalone(positions[i], positions[j])
                            latency = compute_latency_standalone(dist)

                        _, dist = compute_rssi_standalone(positions[i], positions[j])
                        row.extend([f"{dist:.3f}", f"{rssi:.1f}", f"{latency:.3f}"])
                        line += f" {rssi:6.1f}dBm {latency:5.2f}ms |"
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

        # Arrêter NS-3
        if ns3_process:
            ns3_process.terminate()
            ns3_process.wait(timeout=5)

        print()
        print("=" * 70)
        print(f"  Bridge arrêté — {sample_count} échantillons")
        print(f"  Log unifié : {UNIFIED_LOG}")
        print(f"  Durée      : {time.time() - start_time:.1f}s")
        print("=" * 70)

        # Fermer connexions
        for conn in connections:
            try:
                conn.close()
            except Exception:
                pass


if __name__ == '__main__':
    main()
