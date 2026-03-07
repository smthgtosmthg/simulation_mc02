#!/usr/bin/env python3
"""
Script 13 : RSSI & latency between drone pairs using Sionna ray-tracing.

Simulates 802.11ax WiFi channel through the warehouse using Sionna's
ray-tracing engine. For each drone pair, computes:
  - Path loss (dB): how much signal is lost through the environment
  - RSSI (dBm): received signal strength = TX power - path loss
  - Latency (ns): propagation delay from ray-tracing

Usage:
    python3 13_ns3_rssi.py --test     # fake positions
    python3 13_ns3_rssi.py            # read /tmp/drone_positions.csv

Output: /tmp/drone_rssi_latency.csv

How it works:
  1. Load warehouse scene into Sionna ray-tracer
  2. Set frequency to 5.18 GHz (WiFi 6 / 802.11ax, channel 36)
  3. For each pair of drones (0↔1, 0↔2, 1↔2):
     - Place one drone as Transmitter, other as Receiver
     - Run ray-tracing: shoots millions of rays, traces reflections
       off walls/shelves/crates to find all signal paths
     - Extract path loss from channel frequency response (CFR)
     - Extract propagation delay from channel impulse response (CIR)
  4. RSSI = TX_POWER - path_loss
  5. Write results to CSV
"""

import argparse, csv, os, sys, itertools
import numpy as np

SCENE_XML = os.path.expanduser(
    "~/ns-allinone-3.40/ns-3.40/contrib/sionna/model/ns3sionna/"
    "models/warehouse/warehouse.xml"
)
POS_CSV  = "/tmp/drone_positions.csv"
OUT_CSV  = "/tmp/drone_rssi_latency.csv"

# 802.11ax parameters
TX_POWER_DBM  = 20.0     # typical drone WiFi TX power (100 mW)
FREQUENCY_GHZ = 5.18     # WiFi 6 channel 36
BANDWIDTH_MHZ = 20       # channel bandwidth


def read_positions(path):
    """Read CSV → {drone_id: (x,y,z)}, keeps latest per drone."""
    latest = {}
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
    return latest


def compute_rssi(scene, pos_a, pos_b):
    """
    Compute path loss and delay between two positions using Sionna ray-tracing.

    Steps:
      1. Remove old TX/RX from scene
      2. Place TX at pos_a, RX at pos_b
      3. Run PathSolver (ray-tracing)
      4. Get CIR → extract delay (min propagation time)
      5. Get CFR → extract path loss (mean power across subcarriers)
      6. Return (path_loss_dB, delay_ns)
    """
    from sionna.rt import Transmitter, Receiver, PlanarArray, PathSolver

    # Clean old devices
    for name in list(scene.transmitters.keys()):
        scene.remove(name)
    for name in list(scene.receivers.keys()):
        scene.remove(name)

    # Place TX and RX
    tx = Transmitter(name="tx", position=list(pos_a), orientation=[0, 0, 0])
    rx = Receiver(name="rx", position=list(pos_b), orientation=[0, 0, 0])
    scene.add(tx)
    scene.add(rx)

    # Simple antenna: single omnidirectional element per device
    scene.tx_array = PlanarArray(num_rows=1, num_cols=1,
                                  vertical_spacing=0.5, horizontal_spacing=0.5,
                                  pattern="dipole", polarization="V")
    scene.rx_array = PlanarArray(num_rows=1, num_cols=1,
                                  vertical_spacing=0.5, horizontal_spacing=0.5,
                                  pattern="dipole", polarization="V")

    # Ray-tracing (fast mode: fewer bounces, no diffraction)
    solver = PathSolver()
    paths = solver(scene=scene,
                   max_depth=3,
                   samples_per_src=10**6,
                   los=True,
                   specular_reflection=True,
                   diffuse_reflection=False,
                   refraction=True,
                   synthetic_array=False,
                   diffraction=False,
                   edge_diffraction=False)

    # CIR: channel impulse response → delays
    # a shape: [num_rx, num_rx_ant, num_tx, num_tx_ant, num_paths, num_time_steps]
    # tau shape: [num_rx, num_tx, num_paths]
    a, tau = paths.cir(sampling_frequency=1e9, normalize_delays=False, out_type="numpy")

    tau_flat = np.squeeze(tau)
    valid_delays = tau_flat[tau_flat >= 0]

    if len(valid_delays) == 0:
        # No paths found (complete blockage)
        return None, None

    delay_ns = int(round(np.min(valid_delays) * 1e9))

    # CFR: channel frequency response → path loss
    # Subcarrier frequencies for 20 MHz bandwidth
    num_subcarriers = 64  # standard for 20 MHz 802.11ax
    subcarrier_spacing = BANDWIDTH_MHZ * 1e6 / num_subcarriers
    frequencies = np.arange(num_subcarriers) * subcarrier_spacing
    frequencies = frequencies - np.mean(frequencies) + FREQUENCY_GHZ * 1e9

    h_raw = paths.cfr(frequencies=frequencies,
                      sampling_frequency=1.0,
                      num_time_steps=1,
                      normalize_delays=True,
                      normalize=False,
                      out_type="numpy")

    h = np.squeeze(h_raw)
    mean_power = np.mean(np.abs(h) ** 2)

    if mean_power <= 0:
        return None, None

    path_loss_dB = float(-10 * np.log10(mean_power))
    return path_loss_dB, delay_ns


def main():
    parser = argparse.ArgumentParser(description="NS-3/Sionna RSSI & latency")
    parser.add_argument("--test", action="store_true", help="Use fake positions")
    parser.add_argument("--csv", default=POS_CSV, help="Positions CSV path")
    args = parser.parse_args()

    print("\n  NS-3 RSSI & LATENCY (Sionna Ray-Tracing)\n")

    if args.test:
        with open(args.csv, "w") as f:
            f.write("0,-3.0,0.0,4.0\n1,0.0,0.0,5.0\n2,3.0,0.0,6.0\n")
        print("  Test CSV written")

    if not os.path.exists(args.csv):
        print(f"  ERROR: {args.csv} not found. Use --test")
        sys.exit(1)

    positions = read_positions(args.csv)
    if len(positions) < 2:
        print("  ERROR: need at least 2 drones")
        sys.exit(1)

    # Import Sionna (heavy)
    from sionna.rt import load_scene

    print("  Loading warehouse scene...")
    scene = load_scene(SCENE_XML, merge_shapes=False)
    scene.frequency = FREQUENCY_GHZ * 1e9
    scene.bandwidth = BANDWIDTH_MHZ * 1e6
    print(f"  Frequency: {FREQUENCY_GHZ} GHz, Bandwidth: {BANDWIDTH_MHZ} MHz")
    print(f"  TX power: {TX_POWER_DBM} dBm\n")

    # Compute RSSI for every drone pair
    drone_ids = sorted(positions.keys())
    pairs = list(itertools.combinations(drone_ids, 2))
    results = []

    for id_a, id_b in pairs:
        pos_a = positions[id_a]
        pos_b = positions[id_b]
        dist = np.sqrt(sum((a - b)**2 for a, b in zip(pos_a, pos_b)))

        print(f"  {id_a}↔{id_b}  dist={dist:.1f}m  ", end="", flush=True)

        path_loss, delay_ns = compute_rssi(scene, pos_a, pos_b)

        if path_loss is None:
            print("NO PATH (blocked)")
            results.append((id_a, id_b, "blocked", "blocked", f"{dist:.2f}"))
        else:
            rssi = TX_POWER_DBM - path_loss
            print(f"loss={path_loss:.1f}dB  RSSI={rssi:.1f}dBm  delay={delay_ns}ns")
            results.append((id_a, id_b, f"{rssi:.2f}", f"{delay_ns}", f"{dist:.2f}"))

    # Write output CSV
    with open(OUT_CSV, "w") as f:
        f.write("drone_a,drone_b,rssi_dBm,delay_ns,distance_m\n")
        for row in results:
            f.write(",".join(str(v) for v in row) + "\n")

    print(f"\n  Saved: {OUT_CSV}")
    print()


if __name__ == "__main__":
    main()
