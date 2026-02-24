#!/usr/bin/env python3
"""
Script 7 : Tracker les positions de N drones en temps réel.

Usage :
    python3 07_track_positions.py --drones 3
    python3 07_track_positions.py --drones 5 --rate 4 --output flight_log.csv
    python3 07_track_positions.py --drones 3 --duration 60

Connexion via pymavlink aux instances ArduPilot SITL.
Chaque drone est sur tcp:127.0.0.1:5760+I*10

Sortie :
    - Terminal : tableau des positions mis à jour en temps réel
    - CSV      : log complet (timestamp, x, y, z, vx, vy, vz par drone)
"""

import argparse
import csv
import os
import signal
import sys
import time
from datetime import datetime

try:
    from pymavlink import mavutil
except ImportError:
    print("ERREUR: pymavlink non installé.")
    print("Installe avec : pip3 install pymavlink")
    sys.exit(1)


# ============================================================
# Fonctions utilitaires
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


def request_data_streams(conn, rate_hz=4):
    """Demander les flux de données de position au drone."""
    conn.mav.request_data_stream_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        rate_hz,
        1  # start
    )
    conn.mav.request_data_stream_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
        rate_hz,
        1
    )


def get_local_position(conn, timeout=2):
    """Récupère la position locale NED d'un drone."""
    msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=timeout)
    if msg:
        return {
            'x': msg.x,         # Nord (m)
            'y': msg.y,         # Est (m)
            'z': -msg.z,        # Altitude (converti de NED z-down → z-up)
            'vx': msg.vx,       # Vitesse Nord (m/s)
            'vy': msg.vy,       # Vitesse Est (m/s)
            'vz': -msg.vz,      # Vitesse verticale (m/s, positif = montée)
        }
    return None


def get_global_position(conn, timeout=2):
    """Récupère la position GPS globale d'un drone."""
    msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout)
    if msg:
        return {
            'lat': msg.lat / 1e7,
            'lon': msg.lon / 1e7,
            'alt': msg.relative_alt / 1000.0,
        }
    return None


def clear_line():
    """Efface la ligne courante du terminal."""
    sys.stdout.write('\033[2K\r')


# ============================================================
# Programme principal
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description='Tracker les positions de N drones ArduPilot SITL en temps réel'
    )
    parser.add_argument('--drones', type=int, default=3,
                        help='Nombre de drones (défaut: 3)')
    parser.add_argument('--rate', type=float, default=2.0,
                        help='Fréquence d\'échantillonnage en Hz (défaut: 2)')
    parser.add_argument('--output', type=str, default='positions.csv',
                        help='Fichier CSV de sortie (défaut: positions.csv)')
    parser.add_argument('--duration', type=int, default=0,
                        help='Durée en secondes, 0=infini (défaut: 0)')
    args = parser.parse_args()

    n_drones = args.drones
    rate_hz = args.rate
    interval = 1.0 / rate_hz

    print("=" * 65)
    print(f"  TRACKER DE POSITIONS — {n_drones} drones @ {rate_hz} Hz")
    print("=" * 65)
    print()

    # --- Connexion à tous les drones ---
    print("Connexion aux drones :")
    connections = []
    for i in range(n_drones):
        conn = connect_drone(i)
        if conn is None:
            print(f"\nERREUR: Impossible de connecter le drone {i}.")
            print("Vérifie que 06_launch_multi_drones.sh tourne dans un autre terminal.")
            sys.exit(1)
        connections.append(conn)

    print(f"\n=> {n_drones} drones connectés !")
    print()

    # --- Demander les flux de données ---
    for conn in connections:
        request_data_streams(conn, rate_hz=int(rate_hz * 2))

    # --- Préparer le fichier CSV ---
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_path = os.path.join(script_dir, '..', args.output)
    csv_path = os.path.abspath(csv_path)

    csv_file = open(csv_path, 'w', newline='')
    writer = csv.writer(csv_file)

    # En-tête CSV
    header = ['timestamp_s']
    for i in range(n_drones):
        header.extend([
            f'drone{i}_x', f'drone{i}_y', f'drone{i}_z',
            f'drone{i}_vx', f'drone{i}_vy', f'drone{i}_vz'
        ])
    writer.writerow(header)

    print(f"Sauvegarde dans : {csv_path}")
    print("Ctrl+C pour arrêter")
    print()

    # --- Afficher l'en-tête du tableau ---
    col_header = "  t(s)   |"
    for i in range(n_drones):
        col_header += f"   Drone {i} (x, y, z)       |"
    print(col_header)
    print("-" * len(col_header))

    # --- Boucle principale ---
    running = True

    def signal_handler(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, signal_handler)

    start_time = time.time()
    sample_count = 0

    try:
        while running:
            # Vérifier la durée
            elapsed = time.time() - start_time
            if args.duration > 0 and elapsed > args.duration:
                print(f"\n\nDurée de {args.duration}s atteinte.")
                break

            timestamp = elapsed
            row = [f"{timestamp:.3f}"]
            line = f" {timestamp:7.1f} |"

            for i, conn in enumerate(connections):
                pos = get_local_position(conn, timeout=1)
                if pos:
                    row.extend([
                        f"{pos['x']:.3f}", f"{pos['y']:.3f}", f"{pos['z']:.3f}",
                        f"{pos['vx']:.3f}", f"{pos['vy']:.3f}", f"{pos['vz']:.3f}"
                    ])
                    line += f" ({pos['x']:7.2f}, {pos['y']:7.2f}, {pos['z']:5.2f}m) |"
                else:
                    row.extend(['NaN'] * 6)
                    line += "  (    N/A    )         |"

            writer.writerow(row)
            sample_count += 1

            # Afficher dans le terminal
            print(line)

            # Flush CSV régulièrement
            if sample_count % 10 == 0:
                csv_file.flush()

            time.sleep(interval)

    except KeyboardInterrupt:
        pass
    finally:
        csv_file.close()
        print()
        print("=" * 50)
        print(f"  {sample_count} échantillons sauvegardés")
        print(f"  Fichier : {csv_path}")
        print(f"  Durée   : {time.time() - start_time:.1f}s")
        print("=" * 50)

        # Fermer les connexions
        for conn in connections:
            try:
                conn.close()
            except Exception:
                pass


if __name__ == '__main__':
    main()
