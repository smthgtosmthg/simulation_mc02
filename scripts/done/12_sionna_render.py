#!/usr/bin/env python3
"""
Script 12 : Sionna 3D render — warehouse with drone positions.

Usage:
    python3 12_sionna_render.py --test     # fake positions → render
    python3 12_sionna_render.py            # read /tmp/drone_positions.csv → render

Output: /tmp/sionna_renders/latest.png
"""

import argparse, csv, os, sys

SCENE_XML = os.path.expanduser(
    "~/ns-allinone-3.40/ns-3.40/contrib/sionna/model/ns3sionna/"
    "models/warehouse/warehouse.xml"
)
CSV_PATH   = "/tmp/drone_positions.csv"
OUTPUT_DIR = "/tmp/sionna_renders"
OUTPUT     = os.path.join(OUTPUT_DIR, "latest.png")

# Drone colors: red, green, blue
COLORS = [(1,0,0), (0,0.8,0), (0.2,0.4,1), (1,0.8,0), (1,0,1), (0,1,1)]


def read_positions(path):
    """Read CSV, return {drone_id: (x,y,z)} — keeps latest per drone.
    Handles 4-col (id,x,y,z) and 5-col (timestamp,id,x,y,z) formats."""
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


def render(positions):
    """Load warehouse, place drones, render PNG."""
    from sionna.rt import load_scene, Transmitter, Camera

    print("  Loading scene...")
    scene = load_scene(SCENE_XML, merge_shapes=False)

    # Add drones as colored spheres
    for did, (x, y, z) in sorted(positions.items()):
        color = COLORS[did % len(COLORS)]
        scene.add(Transmitter(
            name=f"drone_{did}", position=[x, y, z],
            color=color, power_dbm=20, display_radius=0.5
        ))
        print(f"  Drone {did}: ({x:.1f}, {y:.1f}, {z:.1f})")

    # Camera: top-down bird's-eye view, ceiling clipped so we see inside
    cam = Camera(position=[0, 0, 25], look_at=[0, 0, 0])

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    scene.render_to_file(
        camera=cam, filename=OUTPUT,
        resolution=(1280, 960), num_samples=64,
        show_devices=True, show_orientations=False,
        clip_at=5.9  # clip ceiling (at 6.05m) so we see inside warehouse
    )
    print(f"\n  Saved: {OUTPUT}")


def main():
    parser = argparse.ArgumentParser(description="Sionna 3D warehouse render")
    parser.add_argument("--test", action="store_true", help="Use fake positions")
    parser.add_argument("--csv", default=CSV_PATH, help="Positions CSV path")
    args = parser.parse_args()

    print("\n  SIONNA 3D WAREHOUSE RENDER\n")

    if args.test:
        # Write 3 test drones inside the warehouse
        os.makedirs(os.path.dirname(args.csv) or ".", exist_ok=True)
        with open(args.csv, "w") as f:
            f.write("0,-3.0,0.0,4.0\n1,0.0,0.0,5.0\n2,3.0,0.0,6.0\n")
        print("  Test CSV written")

    if not os.path.exists(args.csv):
        print(f"  ERROR: {args.csv} not found. Use --test or provide --csv")
        sys.exit(1)

    positions = read_positions(args.csv)
    if not positions:
        print("  ERROR: no positions in CSV")
        sys.exit(1)

    render(positions)


if __name__ == "__main__":
    main()
