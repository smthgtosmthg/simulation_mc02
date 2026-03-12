#!/usr/bin/env python3

import argparse, csv, itertools, math, os, signal, subprocess, sys, time
from datetime import datetime
import numpy as np

MAX_STALE_SEC = 10  
LANDED_ALT    = 0.5  
UNCHANGED_MAX = 3    

NS3_DIR     = os.path.expanduser("~/ns-allinone-3.40/ns-3.40")
NS3_BIN     = os.path.join(NS3_DIR, "ns3")
POS_CSV     = "/tmp/drone_positions.csv"
METRICS_CSV = "/tmp/drone_5g_metrics.csv"
LOG_CSV     = "/tmp/drone_5g_log.csv"
RSSI_CSV    = "/tmp/drone_rssi_sionna.csv"
LATENCY_CSV = "/tmp/drone_latency_ns3.csv"

# Radio parameters
SCENE_XML = os.path.expanduser(
    "~/ns-allinone-3.40/ns-3.40/contrib/sionna/model/ns3sionna/"
    "models/warehouse/warehouse.xml"
)
TX_POWER_DBM    = 23.0         
FREQUENCY_GHZ   = 3.5        
BANDWIDTH_MHZ   = 20
GNB_POSITION    = (0.0, 0.0, 6.0)  

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
    """Lit CSV metriques NS-3 par paire -> liste de dicts.
    On ne garde que latence/jitter/rx de NS-3 (le RSSI vient de Sionna)."""
    pairs = []
    try:
        with open(path) as f:
            for row in csv.DictReader(f):
                pairs.append({
                    "a": int(row["drone_a"]),
                    "b": int(row["drone_b"]),
                    "latency": float(row["latency_ms"]),
                    "jitter": float(row["jitter_ms"]),
                    "dist": float(row["dist_ab_m"]),
                    "rx": int(row["rx_packets"]),
                })
    except Exception:
        pass
    return pairs




def compute_rssi_sionna(scene, pos_drone, pos_gnb=GNB_POSITION):
    """Ray-trace gNB → Drone (lien descendant 5G NR).
    Retourne (rssi_dBm, delay_ns) ou (None, None) si bloque.
    TX = gNB (position fixe), RX = drone."""
    from sionna.rt import Transmitter, Receiver, PlanarArray, PathSolver

    for name in list(scene.transmitters.keys()):
        scene.remove(name)
    for name in list(scene.receivers.keys()):
        scene.remove(name)

    scene.add(Transmitter(name="gnb", position=list(pos_gnb), orientation=[0, 0, 0]))
    scene.add(Receiver(name="ue", position=list(pos_drone), orientation=[0, 0, 0]))

    scene.tx_array = PlanarArray(num_rows=1, num_cols=1,
                                  vertical_spacing=0.5, horizontal_spacing=0.5,
                                  pattern="dipole", polarization="V")
    scene.rx_array = PlanarArray(num_rows=1, num_cols=1,
                                  vertical_spacing=0.5, horizontal_spacing=0.5,
                                  pattern="dipole", polarization="V")

    solver = PathSolver()
    paths = solver(scene=scene, max_depth=3, samples_per_src=10**6,
                   los=True, specular_reflection=True, diffuse_reflection=False,
                   refraction=True, synthetic_array=False,
                   diffraction=False, edge_diffraction=False)

    a, tau = paths.cir(sampling_frequency=1e9, normalize_delays=False, out_type="numpy")
    tau_flat = np.squeeze(tau)
    valid = tau_flat[tau_flat >= 0]

    if len(valid) == 0:
        return None, None  
    delay_ns = int(round(np.min(valid) * 1e9))

    num_sc = 64
    sc_spacing = BANDWIDTH_MHZ * 1e6 / num_sc
    freqs = np.arange(num_sc) * sc_spacing
    freqs = freqs - np.mean(freqs) + FREQUENCY_GHZ * 1e9

    h = np.squeeze(paths.cfr(frequencies=freqs, sampling_frequency=1.0,
                              num_time_steps=1, normalize_delays=True,
                              normalize=False, out_type="numpy"))
    mean_power = np.mean(np.abs(h) ** 2)
    if mean_power <= 0:
        return None, None

    path_loss_dB = float(-10 * np.log10(mean_power))
    rssi_dBm = TX_POWER_DBM - path_loss_dB
    return rssi_dBm, delay_ns


def compute_all_rssi(scene, positions):
    """Calcule le RSSI gNB→Drone pour chaque drone (cache par tick).
    Retourne {drone_id: (rssi_dBm, delay_ns)}."""
    rssi_cache = {}
    for did, pos in positions.items():
        try:
            rssi, delay = compute_rssi_sionna(scene, pos)
        except Exception:
            rssi, delay = None, None
        rssi_cache[did] = (rssi, delay)
    return rssi_cache


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
        with open(POS_CSV, "w") as f:
            f.write("0,-3.00,0.00,4.00\n1,0.00,0.00,5.00\n2,5.00,2.00,3.50\n")
        pos = read_positions(POS_CSV)
        print(f"  Mode test: {len(pos)} drones positionnes.\n")
        return pos

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


def rssi_bar(rssi_val):
    """Barre de qualite signal."""
    if rssi_val is None:
        return "BLOCKED"
    elif rssi_val > -50:
        return "█████ Excellent"
    elif rssi_val > -60:
        return "████░ Good"
    elif rssi_val > -70:
        return "███░░ Fair"
    else:
        return "██░░░ Weak"


def print_table(pairs, positions, rssi_cache, tick, now):
    """Affiche les resultats hybrides (Sionna RSSI drone→gNB + NS-3 latence)."""
    print(f"\n  ─── Mesure #{tick} @ {now} ─────────────────────────────────────────────────────")
    print(f"  ┌─────────┬────────────────┬──────────┬─────────────────┐")
    print(f"  │  Drone  │ RSSI→gNB  dBm  │ Delay ns │ Qualite         │")
    print(f"  │   (UE)  │ (Sionna RT)    │ (Sionna) │                 │")
    print(f"  ├─────────┼────────────────┼──────────┼─────────────────┤")
    for did in sorted(rssi_cache.keys()):
        rssi, delay = rssi_cache[did]
        bar = rssi_bar(rssi)
        if rssi is None:
            rssi_str = "  BLOCKED  "
            delay_str = "   ---  "
        else:
            rssi_str = f"{rssi:12.1f}  "
            delay_str = f"{delay:>6}  "
        pos = positions.get(did, (0,0,0))
        dist_gnb = math.sqrt(sum((u - v) ** 2 for u, v in zip(pos, GNB_POSITION)))
        print(f"  │  D{did:<4} │ {rssi_str}│ {delay_str}│ {bar:<15} │  ({pos[0]:+.1f},{pos[1]:+.1f},{pos[2]:.1f}m) d_gNB={dist_gnb:.1f}m")
    print(f"  └─────────┴────────────────┴──────────┴─────────────────┘")

    print(f"  ┌───────────┬────────────┬────────────┬─────────┐")
    print(f"  │  Paire    │ Latence ms │ Jitter ms  │ Dist m  │")
    print(f"  │           │  (NS-3 NR) │  (NS-3 NR) │  A↔B    │")
    print(f"  ├───────────┼────────────┼────────────┼─────────┤")
    for p in pairs:
        print(f"  │ {p['a']:>2} <-> {p['b']:<2} "
              f"│ {p['latency']:10.2f} │ {p['jitter']:10.2f} "
              f"│ {p['dist']:7.1f} │")
    print(f"  └───────────┴────────────┴────────────┴─────────┘\n")


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
    print("  5G-LENA + SIONNA HYBRID BRIDGE")
    print("  RSSI    : Sionna ray-tracing (geometrie reelle du warehouse)")
    print("  Latence : NS-3 5G NR (scheduling + EPC bout-en-bout)")
    print("  Architecture: Drone_A (UE) --NR--> gNB --S1U--> EPC --S1U--> gNB --NR--> Drone_B (UE)")
    print(f"  gNB: ({GNB_POSITION[0]}, {GNB_POSITION[1]}, {GNB_POSITION[2]})m")
    print(f"  Freq: {FREQUENCY_GHZ} GHz | BW: {BANDWIDTH_MHZ} MHz | TX: {TX_POWER_DBM} dBm")
    print()

    # Init CSV logs 
    with open(RSSI_CSV, "w") as f:
        f.write("timestamp,tick,drone_id,x,y,z,dist_gnb_m,"
                "rssi_gnb_dBm,delay_gnb_ns,status\n")
    with open(LATENCY_CSV, "w") as f:
        f.write("timestamp,tick,drone_a,drone_b,"
                "latency_ms,jitter_ms,dist_ab_m,rx_packets\n")
    with open(LOG_CSV, "w") as f:
        f.write("timestamp,tick,drone_a,drone_b,"
                "rssi_a_gnb_dBm,delay_a_gnb_ns,rssi_b_gnb_dBm,delay_b_gnb_ns,"
                "latency_ns3_ms,jitter_ns3_ms,dist_ab_m,rx_packets,status_a,status_b\n")

    # Charger la scene Sionna 
    print("  Chargement de la scene Sionna (warehouse)...")
    from sionna.rt import load_scene
    scene = load_scene(SCENE_XML, merge_shapes=False)
    scene.frequency = FREQUENCY_GHZ * 1e9
    scene.bandwidth = BANDWIDTH_MHZ * 1e6
    print(f"  Scene chargee — {len(scene.objects)} objets")
    print()

    # ATTENDRE les positions avant de commencer la boucle
    positions = wait_for_positions(args.test)
    if not positions:
        print("  Aucune position recue. Fin.")
        return

    tick = 0
    t_start = time.time()
    prev_positions = None
    unchanged_count = 0

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

        # Detection vol termine 
        if prev_positions is not None and positions == prev_positions:
            unchanged_count += 1
        else:
            unchanged_count = 0
        prev_positions = dict(positions)

        all_landed = all(z < LANDED_ALT for _, _, z in positions.values())

        if unchanged_count >= UNCHANGED_MAX and all_landed:
            print(f"\n  [{now}] Vol termine detecte : positions inchangees "
                  f"depuis {unchanged_count} ticks et tous les drones poses (z < {LANDED_ALT}m).")
            break

        if unchanged_count >= UNCHANGED_MAX:
            print(f"  [{now}] Positions inchangees depuis {unchanged_count} ticks "
                  f"(le script de vol est peut-etre termine).")
            break

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

        # Lire les metriques NS-3
        ns3_pairs = read_pair_metrics(METRICS_CSV)
        if not ns3_pairs:
            print(f"  Pas de metriques NS-3.")
            time.sleep(args.interval)
            continue

        # Calculer RSSI Sionna pour chaque drone → gNB
        print(f"  [{now}] Sionna ray-tracing (drone→gNB)...", end=" ", flush=True)
        t_sionna = time.time()
        rssi_cache = compute_all_rssi(scene, positions)
        dt_sionna = time.time() - t_sionna
        n_blocked = sum(1 for r, _ in rssi_cache.values() if r is None)
        print(f"OK ({dt_sionna:.1f}s) — {len(rssi_cache)} drones, {n_blocked} blocked")

        # Construire les paires avec RSSI par drone
        pairs = []
        for ns3p in ns3_pairs:
            a_id, b_id = ns3p["a"], ns3p["b"]
            pos_a = positions.get(a_id, (0, 0, 0))
            pos_b = positions.get(b_id, (0, 0, 0))
            dist = math.sqrt(sum((u - v) ** 2 for u, v in zip(pos_a, pos_b)))

            rssi_a, delay_a = rssi_cache.get(a_id, (None, None))
            rssi_b, delay_b = rssi_cache.get(b_id, (None, None))

            pairs.append({
                "a": a_id,
                "b": b_id,
                "rssi_a": rssi_a,          
                "delay_a": delay_a,       
                "rssi_b": rssi_b,           
                "delay_b": delay_b,         
                "latency": ns3p["latency"], 
                "jitter": ns3p["jitter"],   
                "dist": dist,
                "rx": ns3p["rx"],         
            })

        print_table(pairs, positions, rssi_cache, tick, now)

        with open(RSSI_CSV, "a") as f:
            for did in sorted(rssi_cache.keys()):
                rssi, delay = rssi_cache[did]
                pos = positions.get(did, (0, 0, 0))
                dist_gnb = math.sqrt(sum((u - v) ** 2 for u, v in zip(pos, GNB_POSITION)))
                rssi_str = f"{rssi:.2f}" if rssi is not None else "BLOCKED"
                delay_str = f"{delay}" if delay is not None else "BLOCKED"
                status = "LOS" if rssi is not None else "BLOCKED"
                f.write(f"{now},{tick},{did},{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f},"
                        f"{dist_gnb:.2f},{rssi_str},{delay_str},{status}\n")

        with open(LATENCY_CSV, "a") as f:
            for p in pairs:
                f.write(f"{now},{tick},{p['a']},{p['b']},"
                        f"{p['latency']:.2f},{p['jitter']:.2f},"
                        f"{p['dist']:.2f},{p['rx']}\n")

        with open(LOG_CSV, "a") as f:
            for p in pairs:
                rssi_a_str = f"{p['rssi_a']:.2f}" if p['rssi_a'] is not None else "BLOCKED"
                delay_a_str = f"{p['delay_a']}" if p['delay_a'] is not None else "BLOCKED"
                rssi_b_str = f"{p['rssi_b']:.2f}" if p['rssi_b'] is not None else "BLOCKED"
                delay_b_str = f"{p['delay_b']}" if p['delay_b'] is not None else "BLOCKED"
                status_a = "LOS" if p['rssi_a'] is not None else "BLOCKED"
                status_b = "LOS" if p['rssi_b'] is not None else "BLOCKED"
                f.write(f"{now},{tick},{p['a']},{p['b']},"
                        f"{rssi_a_str},{delay_a_str},{rssi_b_str},{delay_b_str},"
                        f"{p['latency']:.2f},{p['jitter']:.2f},"
                        f"{p['dist']:.2f},{p['rx']},{status_a},{status_b}\n")

        time.sleep(args.interval)

    print(f"\n  Bridge arrete apres {tick} mesures.")
    print(f"  RSSI Sionna  : {RSSI_CSV}")
    print(f"  Latence NS-3 : {LATENCY_CSV}")
    print(f"  Log combine  : {LOG_CSV}\n")


if __name__ == "__main__":
    main()
