#!/usr/bin/env python3
"""
Script 14 : Live Bridge — Watches position CSV → RSSI/Latency → 3D Render.

Decoupled from MAVLink: script 08 writes positions to CSV, this script
watches that file and computes RSSI & latency using Sionna ray-tracing.

Architecture:
    Script 06 (Gazebo+SITL)  →  Script 08 (flight + writes /tmp/drone_positions.csv)
                                      ↓
    Script 14 (this) watches CSV → computes RSSI → renders 3D → prints table

Usage:
    python3 14_live_bridge.py              # watch /tmp/drone_positions.csv
    python3 14_live_bridge.py --test       # fake moving drones (no SITL needed)

Output files:
    /tmp/drone_rssi_latency.csv    — RSSI & latency per pair
    /tmp/sionna_renders/latest.png — 3D render
    /tmp/drone_bridge_log.csv      — full log with timestamps
"""

import argparse, csv, itertools, math, os, signal, sys, time
from datetime import datetime
import numpy as np

# ── Paths ──────────────────────────────────────────────────
SCENE_XML = os.path.expanduser(
    "~/ns-allinone-3.40/ns-3.40/contrib/sionna/model/ns3sionna/"
    "models/warehouse/warehouse.xml"
)
POS_CSV    = "/tmp/drone_positions.csv"
RSSI_CSV   = "/tmp/drone_rssi_latency.csv"
LOG_CSV    = "/tmp/drone_bridge_log.csv"
RENDER_DIR = "/tmp/sionna_renders"
RENDER_PNG = os.path.join(RENDER_DIR, "latest.png")

# ── WiFi 802.11ax parameters ──────────────────────────────
TX_POWER_DBM  = 20.0
FREQUENCY_GHZ = 5.18
BANDWIDTH_MHZ = 20

# ── Drone colors for render ───────────────────────────────
COLORS = [(1, 0, 0), (0, 0.8, 0), (0.2, 0.4, 1)]

running = True


def signal_handler(sig, frame):
    global running
    running = False
    print("\n  Stopping...")


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  Position reading: watch CSV file written by script 08
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━


def read_positions_csv(path):
    """Read CSV → {drone_id: (x,y,z)}, keeps latest per drone.
    Handles 4-col (id,x,y,z) and 5-col (ts,id,x,y,z) formats."""
    latest = {}
    try:
        with open(path) as f:
            for row in csv.reader(f):
                if not row or row[0].startswith("#") or row[0].strip() in ("drone_id","timestamp"):
                    continue
                try:
                    if len(row) >= 5:
                        did, x, y, z = int(row[1]), float(row[2]), float(row[3]), float(row[4])
                    else:
                        did, x, y, z = int(row[0]), float(row[1]), float(row[2]), float(row[3])
                    latest[did] = (x, y, z)
                except (ValueError, IndexError):
                    continue
    except FileNotFoundError:
        pass
    return latest


def fake_positions(tick):
    """Generate 3 drones orbiting inside the warehouse (--test mode)."""
    t = tick * 0.3
    return {
        0: (-3.0 + 1.5 * math.sin(t),       1.0 * math.cos(t),       4.0),
        1: ( 0.0 + 1.5 * math.sin(t + 2.1), 1.0 * math.cos(t + 2.1), 5.0),
        2: ( 3.0 + 1.5 * math.sin(t + 4.2), 1.0 * math.cos(t + 4.2), 6.0),
    }


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  RSSI computation (reuse logic from script 13)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━


def compute_rssi(scene, pos_a, pos_b):
    """Ray-trace between two positions → (path_loss_dB, delay_ns) or (None, None)."""
    from sionna.rt import Transmitter, Receiver, PlanarArray, PathSolver

    # Clean old devices
    for name in list(scene.transmitters.keys()):
        scene.remove(name)
    for name in list(scene.receivers.keys()):
        scene.remove(name)

    scene.add(Transmitter(name="tx", position=list(pos_a), orientation=[0, 0, 0]))
    scene.add(Receiver(name="rx", position=list(pos_b), orientation=[0, 0, 0]))

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

    return float(-10 * np.log10(mean_power)), delay_ns


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  3D Render (reuse logic from script 12)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━


def render_scene(scene, positions):
    """Render warehouse with drones from top-down view."""
    from sionna.rt import Transmitter, Camera

    # Remove old transmitters (drones from render)
    for name in list(scene.transmitters.keys()):
        scene.remove(name)
    for name in list(scene.receivers.keys()):
        scene.remove(name)

    # Add drones as colored spheres
    for did, (x, y, z) in sorted(positions.items()):
        color = COLORS[did % len(COLORS)]
        scene.add(Transmitter(
            name=f"drone_{did}", position=[x, y, z],
            color=color, power_dbm=20, display_radius=0.5
        ))

    cam = Camera(position=[0, 0, 25], look_at=[0, 0, 0])
    os.makedirs(RENDER_DIR, exist_ok=True)
    scene.render_to_file(
        camera=cam, filename=RENDER_PNG,
        resolution=(1280, 960), num_samples=64,
        show_devices=True, show_orientations=False,
        clip_at=5.9
    )


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  File I/O
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━


def write_fake_positions_csv(positions):
    """Write test positions to CSV (only for --test mode)."""
    with open(POS_CSV, "w") as f:
        for did, (x, y, z) in sorted(positions.items()):
            f.write(f"{did},{x:.4f},{y:.4f},{z:.4f}\n")


def write_rssi_csv(results):
    """Write RSSI results to CSV (overwrite)."""
    with open(RSSI_CSV, "w") as f:
        f.write("drone_a,drone_b,rssi_dBm,delay_ns,distance_m\n")
        for row in results:
            f.write(",".join(str(v) for v in row) + "\n")


def append_log(tick, positions, results):
    """Append one line per pair to the running log."""
    ts = time.strftime("%H:%M:%S")
    write_header = not os.path.exists(LOG_CSV) or os.path.getsize(LOG_CSV) == 0
    with open(LOG_CSV, "a") as f:
        if write_header:
            f.write("time,tick,drone_a,drone_b,rssi_dBm,delay_ns,dist_m,"
                    "ax,ay,az,bx,by,bz\n")
        for id_a, id_b, rssi, delay, dist in results:
            pa = positions[id_a]
            pb = positions[id_b]
            f.write(f"{ts},{tick},{id_a},{id_b},{rssi},{delay},{dist},"
                    f"{pa[0]:.2f},{pa[1]:.2f},{pa[2]:.2f},"
                    f"{pb[0]:.2f},{pb[1]:.2f},{pb[2]:.2f}\n")


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  Live display (continuous scrolling)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

W = 72  # display width


def rssi_bar(rssi_val):
    """Return a signal-quality bar + label from RSSI."""
    if rssi_val > -50:
        return "█████ Excellent"
    elif rssi_val > -60:
        return "████░ Good"
    elif rssi_val > -70:
        return "███░░ Fair"
    else:
        return "██░░░ Weak"


def print_table(tick, positions, results, elapsed):
    """Print a compact, continuously scrolling block for one tick."""
    now = datetime.now().strftime("%H:%M:%S")

    # ── separator / tick header ──
    label = f"  Tick {tick}  ·  {now}  ·  {elapsed:.1f}s  "
    print(f"\n{'─' * 4}{label}{'─' * max(4, W - 4 - len(label))}")

    # ── positions on one line ──
    pos_parts = []
    for did in sorted(positions.keys()):
        x, y, z = positions[did]
        pos_parts.append(f"D{did}({x:.1f},{y:.1f},{z:.1f})")
    print(f"  Pos: {'  '.join(pos_parts)}")

    # ── RSSI pairs ──
    for i, (id_a, id_b, rssi, delay, dist) in enumerate(results):
        if rssi == "blocked" or rssi == "error":
            line = f"  {id_a}↔{id_b}  {dist:>6}m   {'BLOCKED':>10}   {'---':>6}   ⛔ No path"
        else:
            rssi_f = float(rssi)
            bar = rssi_bar(rssi_f)
            line = f"  {id_a}↔{id_b}  {dist:>6}m  {rssi:>8} dBm  {delay:>4} ns   {bar}"
        print(line)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  Main
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━


def main():
    parser = argparse.ArgumentParser(description="Live Bridge: watches position CSV → Sionna RSSI")
    parser.add_argument("--test", action="store_true",
                        help="Fake moving drones (no SITL needed)")
    parser.add_argument("--csv", default=POS_CSV,
                        help=f"Positions CSV to watch (default: {POS_CSV})")
    parser.add_argument("--cycles", type=int, default=0,
                        help="Number of cycles (0 = infinite)")
    parser.add_argument("--no-render", action="store_true",
                        help="Skip 3D render (faster)")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, signal_handler)

    # ── Startup banner ──
    print()
    print("┌" + "─" * (W - 2) + "┐")
    print("│" + "  LIVE BRIDGE — Sionna Ray-Tracing ↔ Drone Positions".ljust(W - 2) + "│")
    print("│" + f"  802.11ax · {FREQUENCY_GHZ} GHz · BW {BANDWIDTH_MHZ} MHz · TX {TX_POWER_DBM:.0f} dBm".ljust(W - 2) + "│")
    mode_str = "TEST (fake drones)" if args.test else f"WATCHING {args.csv}"
    print("│" + f"  Mode: {mode_str}".ljust(W - 2) + "│")
    print("└" + "─" * (W - 2) + "┘")

    # Load Sionna scene ONCE
    print("\n  ⏳ Loading Sionna scene...")
    from sionna.rt import load_scene
    scene = load_scene(SCENE_XML, merge_shapes=False)
    scene.frequency = FREQUENCY_GHZ * 1e9
    scene.bandwidth = BANDWIDTH_MHZ * 1e6
    print(f"  ✓ Scene loaded — {len(scene.objects)} objects")

    # Clear old log
    if os.path.exists(LOG_CSV):
        os.remove(LOG_CSV)

    print(f"  ✓ Ready — Ctrl+C to stop")
    if not args.test:
        print(f"  → Start script 08 in another terminal to fly drones")

    tick = 0
    last_positions = {}
    while running:
        t0 = time.time()

        # ── 1. Get positions ─────────────────────────────
        if args.test:
            positions = fake_positions(tick)
            write_fake_positions_csv(positions)
        else:
            positions = read_positions_csv(args.csv)
            if len(positions) < 2:
                if tick % 10 == 0:  # don't spam
                    print(f"  Waiting for positions in {args.csv}... "
                          f"(found {len(positions)} drones)")
                time.sleep(1)
                tick += 1
                continue

        # Skip RSSI if positions haven't changed
        if positions == last_positions:
            time.sleep(0.5)
            tick += 1
            continue
        last_positions = dict(positions)

        # ── 3. Compute RSSI for all pairs ────────────────
        drone_ids = sorted(positions.keys())
        pairs = list(itertools.combinations(drone_ids, 2))
        results = []

        for id_a, id_b in pairs:
            pa, pb = positions[id_a], positions[id_b]
            dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(pa, pb)))

            try:
                pl, delay = compute_rssi(scene, pa, pb)
                if pl is None:
                    results.append((id_a, id_b, "blocked", "blocked", f"{dist:.2f}"))
                else:
                    rssi = TX_POWER_DBM - pl
                    results.append((id_a, id_b, f"{rssi:.2f}", f"{delay}", f"{dist:.2f}"))
            except Exception as e:
                results.append((id_a, id_b, "error", "error", f"{dist:.2f}"))

        # ── 4. Write RSSI CSV ────────────────────────────
        write_rssi_csv(results)

        # ── 5. Append log ────────────────────────────────
        append_log(tick, positions, results)

        # ── 6. Render (optional) ─────────────────────────
        if not args.no_render:
            try:
                render_scene(scene, positions)
            except Exception as e:
                pass  # don't crash the loop for render errors

        elapsed = time.time() - t0

        # ── 7. Display table ─────────────────────────────
        print_table(tick, positions, results, elapsed)

        tick += 1
        if args.cycles > 0 and tick >= args.cycles:
            break

    # Cleanup
    print(f"\n{'─' * W}")
    print(f"  Bridge stopped.  Log → {LOG_CSV}")
    print(f"{'─' * W}")


if __name__ == "__main__":
    main()
