#!/usr/bin/env python3
"""
Script 15 : Bridge 5G-LENA — RSSI & Latence par paire de drones

Usage:
    python3 15_5g_lena_bridge.py              # mode live
    python3 15_5g_lena_bridge.py --test       # mode test (sans SITL)
    python3 15_5g_lena_bridge.py --interval 5
"""

import argparse, csv, itertools, math, os, signal, subprocess, sys, time
from datetime import datetime

MAX_STALE_SEC = 10   # positions plus vieilles que ca = stale

NS3_DIR     = os.path.expanduser("~/ns-allinone-3.40/ns-3.40")
NS3_BIN     = os.path.join(NS3_DIR, "ns3")
POS_CSV     = "/tmp/drone_positions.csv"
METRICS_CSV = "/tmp/drone_5g_metrics.csv"
LOG_CSV     = "/tmp/drone_5g_log.csv"

running = True


def signal_handler(sig, frame):
    global running
    running = False
    print("\n  Arret...")

signal.signal(signal.SIGINT, signal_handler)


def read_positions(path):
    """Lit CSV positions -> {drone_id: (x,y,z)}."""
    pos = {}
    try:
        with open(path) as f:
            for row in csv.reader(f):
                if not row or row[0].startswith("#"):
                    continue
                if row[0].strip() in ("drone_id", "timestamp"):
                    continue
                try:
                    if len(row) >= 5:
                        did, x, y, z = int(row[1]), float(row[2]), float(row[3]), float(row[4])
                    else:
                        did, x, y, z = int(row[0]), float(row[1]), float(row[2]), float(row[3])
                    pos[did] = (x, y, z)
                except (ValueError, IndexError):
                    continue
    except FileNotFoundError:
        pass
    return pos


def read_pair_metrics(path):
    """Lit CSV metriques par paire -> liste de dicts."""
    pairs = []
    try:
        with open(path) as f:
            for row in csv.DictReader(f):
                pairs.append({
                    "a": int(row["drone_a"]),
                    "b": int(row["drone_b"]),
                    "rssi_a": float(row["rssi_a_dBm"]),
                    "rssi_b": float(row["rssi_b_dBm"]),
                    "latency": float(row["latency_ms"]),
                    "jitter": float(row["jitter_ms"]),
                    "dist": float(row["dist_ab_m"]),
                    "rx": int(row["rx_packets"]),
                })
    except Exception:
        pass
    return pairs


def run_ns3():
    """Lance la simulation NS-3 5G-LENA. Retourne (ok, stdout, stderr)."""
    try:
        r = subprocess.run(
            [NS3_BIN, "run", "drone-5g-nr-scenario", "--no-build"],
            cwd=NS3_DIR, capture_output=True, text=True, timeout=60)
        return r.returncode == 0, r.stdout, r.stderr
    except subprocess.TimeoutExpired:
        return False, "", "timeout"
    except Exception as e:
        return False, "", str(e)


def is_file_fresh(path, max_age=MAX_STALE_SEC):
    """Verifie que le fichier a ete modifie recemment."""
    try:
        return (time.time() - os.path.getmtime(path)) < max_age
    except OSError:
        return False


def wait_for_positions(test_mode):
    """Attend que le fichier de positions existe, soit FRAIS, et contienne >= 2 drones."""
    if test_mode:
        # Mode test : ecrire directement des positions
        with open(POS_CSV, "w") as f:
            f.write("0,-3.00,0.00,4.00\n1,0.00,0.00,5.00\n2,5.00,2.00,3.50\n")
        pos = read_positions(POS_CSV)
        print(f"  Mode test: {len(pos)} drones positionnes.\n")
        return pos

    # Mode live : supprimer l'ancien fichier pour eviter les donnees stale
    if os.path.exists(POS_CSV):
        age = time.time() - os.path.getmtime(POS_CSV)
        if age > MAX_STALE_SEC:
            print(f"  Ancien fichier positions detecte ({age:.0f}s) — ignore.")
            os.remove(POS_CSV)

    print(f"  En attente de positions fraiches ({POS_CSV})...")
    print(f"  Lance le script de vol (08b) dans un autre terminal.")
    waited = 0
    while running:
        if os.path.exists(POS_CSV) and is_file_fresh(POS_CSV):
            pos = read_positions(POS_CSV)
            if len(pos) >= 2:
                print(f"  {len(pos)} drones detectes !\n")
                return pos
        waited += 1
        if waited % 5 == 0:
            now = datetime.now().strftime("%H:%M:%S")
            print(f"  [{now}] Toujours en attente... ({waited}s)")
        time.sleep(1)
    return {}


def write_test_positions(tick):
    """Positions fictives qui bougent."""
    t = tick * 0.5
    pos = {
        0: (-3.0 + 2.0 * math.sin(t),       2.0 * math.cos(t),       4.0),
        1: (      1.5 * math.sin(t + 2.1),   3.0 * math.cos(t + 2.1), 5.0),
        2: ( 5.0 + 2.0 * math.sin(t + 4.2),  2.0 * math.cos(t + 4.2), 3.5),
    }
    with open(POS_CSV, "w") as f:
        for did, (x, y, z) in sorted(pos.items()):
            f.write(f"{did},{x:.4f},{y:.4f},{z:.4f}\n")
    return pos


def print_table(pairs, positions, tick, now):
    """Affiche les resultats par paire."""
    print(f"\n  ─── Mesure #{tick} @ {now} ─────────────────────────────────────────")
    print(f"  ┌───────────┬───────────┬───────────┬────────────┬────────────┬─────────┐")
    print(f"  │  Paire    │  RSSI A   │  RSSI B   │ Latence ms │ Jitter ms  │ Dist m  │")
    print(f"  ├───────────┼───────────┼───────────┼────────────┼────────────┼─────────┤")
    for p in pairs:
        print(f"  │ {p['a']:>2} <-> {p['b']:<2} "
              f"│ {p['rssi_a']:9.1f} │ {p['rssi_b']:9.1f} "
              f"│ {p['latency']:10.2f} │ {p['jitter']:10.2f} "
              f"│ {p['dist']:7.1f} │")
    print(f"  └───────────┴───────────┴───────────┴────────────┴────────────┴─────────┘\n")


def main():
    parser = argparse.ArgumentParser(description="5G-LENA Bridge — RSSI & Latence par paire")
    parser.add_argument("--interval", type=int, default=3, help="Intervalle (s)")
    parser.add_argument("--test", action="store_true", help="Mode test sans SITL")
    parser.add_argument("--duration", type=int, default=0, help="Duree max (0=infini)")
    args = parser.parse_args()

    if not os.path.exists(NS3_BIN):
        print(f"  ERREUR: NS-3 non trouve: {NS3_BIN}")
        sys.exit(1)

    print()
    print("  5G-LENA BRIDGE — RSSI & Latence par paire de drones")
    print("  Architecture: Drone_A (UE) --NR--> gNB --S1U--> EPC --S1U--> gNB --NR--> Drone_B (UE)")
    print("  Canal: 3GPP TR 38.901 InH-OfficeOpen | Freq: 3.5 GHz | BW: 20 MHz")
    print()

    # Init log
    with open(LOG_CSV, "w") as f:
        f.write("timestamp,tick,drone_a,drone_b,rssi_a_dBm,rssi_b_dBm,"
                "latency_ms,jitter_ms,dist_ab_m,rx_packets\n")

    # ATTENDRE les positions avant de commencer la boucle
    positions = wait_for_positions(args.test)
    if not positions:
        print("  Aucune position recue. Fin.")
        return

    tick = 0
    t_start = time.time()

    while running:
        tick += 1
        now = datetime.now().strftime("%H:%M:%S")

        if args.duration > 0 and (time.time() - t_start) > args.duration:
            print(f"  Duree max ({args.duration}s) atteinte.")
            break

        # Mettre a jour les positions (test ou live)
        if args.test:
            positions = write_test_positions(tick)
        else:
            if not is_file_fresh(POS_CSV, max_age=30):
                print(f"  [{now}] Positions stale (>30s) — en attente...")
                time.sleep(args.interval)
                continue
            new_pos = read_positions(POS_CSV)
            if len(new_pos) >= 2:
                positions = new_pos
            else:
                print(f"  [{now}] Pas assez de drones — en attente...")
                time.sleep(args.interval)
                continue

        # Simulation NS-3 5G NR
        print(f"  [{now}] #{tick}: {len(positions)} drones ...", end=" ", flush=True)
        t0 = time.time()
        ok, stdout, stderr = run_ns3()
        dt = time.time() - t0

        if not ok:
            print(f"ERREUR ({dt:.1f}s)")
            if stderr:
                for line in stderr.strip().split("\n")[-2:]:
                    print(f"    {line}")
            time.sleep(args.interval)
            continue

        print(f"OK ({dt:.1f}s)")

        # Lire les metriques par paire
        pairs = read_pair_metrics(METRICS_CSV)
        if not pairs:
            print(f"  Pas de metriques.")
            time.sleep(args.interval)
            continue

        print_table(pairs, positions, tick, now)

        # Log CSV
        with open(LOG_CSV, "a") as f:
            for p in pairs:
                pos_a = positions.get(p["a"], (0, 0, 0))
                pos_b = positions.get(p["b"], (0, 0, 0))
                f.write(f"{now},{tick},{p['a']},{p['b']},"
                        f"{p['rssi_a']:.2f},{p['rssi_b']:.2f},"
                        f"{p['latency']:.2f},{p['jitter']:.2f},"
                        f"{p['dist']:.2f},{p['rx']}\n")

        time.sleep(args.interval)

    print(f"\n  Bridge arrete apres {tick} mesures.")
    print(f"  Log: {LOG_CSV}")
    print(f"  Metriques: {METRICS_CSV}\n")


if __name__ == "__main__":
    main()
