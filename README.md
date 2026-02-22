# Simulation Multi-Drones - Digital Twin

## Gazebo + PX4 SITL + NS-3/Sionna RT

---

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    SIMULATION STACK                      │
│                                                         │
│  ┌─────────────┐   ┌─────────────┐   ┌──────────────┐  │
│  │   Gazebo     │   │  PX4 SITL   │   │   NS-3 +     │  │
│  │  (Physics)   │◄─►│  (Flight    │   │  Sionna RT   │  │
│  │  Warehouse   │   │  Controller)│   │  (Channel)   │  │
│  │  3D env      │   │  x N drones │   │  RSSI+Latency│  │
│  └─────────────┘   └─────────────┘   └──────────────┘  │
│         │                 │                  │           │
│         └────────┬────────┘                  │           │
│                  │                           │           │
│           ┌──────▼──────┐            ┌───────▼────────┐  │
│           │   ROS 2     │            │  Sionna RT     │  │
│           │   Humble    │            │  (GPU Ray      │  │
│           │   MAVROS    │            │   Tracing)     │  │
│           └─────────────┘            └────────────────┘  │
└─────────────────────────────────────────────────────────┘
```

---

## Prérequis

- **OS** : Ubuntu 22.04 LTS
- **GPU** : NVIDIA (pour Sionna RT ray tracing + Gazebo rendering)
- **RAM** : 16 Go minimum recommandé
- **Disque** : ~50 Go d'espace libre
- **Internet** : Nécessaire pour télécharger les paquets

---

## Scripts (ordre d'exécution)

| #   | Script                           | Description                        | Durée estimée |
| --- | -------------------------------- | ---------------------------------- | ------------- |
| 0   | `00_run_all.sh`                  | **Script maître** - lance tout     | ~2-3h total   |
| 1   | `01_system_setup.sh`             | Dépendances, NVIDIA drivers, CUDA  | ~20 min       |
| 2   | `02_install_gazebo_px4.sh`       | ROS 2 Humble, Gazebo, PX4 SITL     | ~45 min       |
| 3   | `03_install_ns3_sionna.sh`       | NS-3.40 + ns3sionna (TU Berlin)    | ~30 min       |
| 4   | `04_install_ns3rt.sh`            | _(Alternative)_ ns3-rt (Pegurri)   | ~20 min       |
| 5   | `05_launch_multi_drone.sh`       | Lance N drones dans le Warehouse   | Runtime       |
| 6   | `06_verify_installation.sh`      | Vérifie toute l'installation       | ~1 min        |
| 7   | `07_ns3_rssi_latency_example.sh` | Mesure RSSI + latence inter-drones | ~5 min        |

---

## Installation rapide (tout d'un coup)

```bash
# Copier les scripts sur le PC Ubuntu
# Puis:
cd ~/scripts_pc_salle_mc02_ubuntu_machine
chmod +x *.sh
./00_run_all.sh
```

## Installation étape par étape

```bash
chmod +x *.sh

# 1. Système + NVIDIA
./01_system_setup.sh
# ⚠️ REBOOT si drivers NVIDIA installés pour la première fois
sudo reboot

# 2. Gazebo + PX4
./02_install_gazebo_px4.sh

# 3. NS-3 + Sionna (choisir A OU B)
./03_install_ns3_sionna.sh    # Option A: ns3sionna (recommandé)
# ./04_install_ns3rt.sh       # Option B: ns3-rt (alternative)

# 4. Vérifier
./06_verify_installation.sh

# 5. Tester RSSI + Latence
./07_ns3_rssi_latency_example.sh

# 6. Lancer multi-drones
./05_launch_multi_drone.sh 3
```

---

## Commandes utiles après installation

### Tester un drone seul

```bash
cd ~/drone_simulation/PX4-Autopilot
make px4_sitl gz_x500
```

### Lancer 3 drones dans le Warehouse

```bash
./05_launch_multi_drone.sh 3
```

### Mode headless (sans rendering, pas besoin de GPU)

```bash
HEADLESS=1 ./05_launch_multi_drone.sh 3
```

### Voir les topics ROS 2

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /fmu/out/vehicle_local_position
```

### Lancer NS-3 Sionna (2 terminaux)

```bash
# Terminal 1: Serveur Sionna RT
cd ~/drone_simulation/ns-allinone-3.40/ns-3.40/contrib/sionna/model/ns3sionna
source sionna-venv/bin/activate
./run.sh

# Terminal 2: NS-3
cd ~/drone_simulation/ns-allinone-3.40/ns-3.40
./ns3 run "scratch/drone-rssi-latency --numDrones=3 --simTime=60"
```

### Voir les résultats RSSI/Latence

```bash
cd ~/drone_simulation/ns-allinone-3.40/ns-3.40
cat drone_rssi.csv       # Signal strength (RSSI)
cat drone_latency.csv    # Latence entre drones
cat drone_positions.csv  # Positions 3D
```

---

## Choix: ns3sionna vs ns3-rt

| Critère             | ns3sionna (TU Berlin)          | ns3-rt (Pegurri)             |
| ------------------- | ------------------------------ | ---------------------------- |
| **Repo**            | github.com/tkn-tub/ns3sionna   | github.com/robpegurri/ns3-rt |
| **Communication**   | ZMQ (Protocol Buffers)         | UDP Socket (port 8103)       |
| **NS-3 version**    | 3.40 (module contrib)          | 3.40 (fork complet)          |
| **Sionna**          | Sionna 1.2.1                   | Sionna RT v1.0.1+            |
| **Machine séparée** | Non documenté                  | Oui (2 machines possible)    |
| **Docker**          | Oui                            | Oui                          |
| **Recommandé**      | ✅ Plus actif, docs meilleures | Pour scénarios distribués    |

---

## Structure des fichiers générés

```
~/drone_simulation/
├── PX4-Autopilot/           # PX4 flight controller
│   └── build/px4_sitl_default/
├── ns-allinone-3.40/        # NS-3 network simulator
│   └── ns-3.40/
│       ├── contrib/sionna/   # ns3sionna module
│       │   └── model/ns3sionna/
│       │       └── sionna-venv/  # Python venv
│       └── scratch/
│           └── drone-rssi-latency.cc
├── ns3-rt/                   # (si Option B installée)
├── Micro-XRCE-DDS-Agent/    # Communication PX4↔ROS2
└── gazebo_worlds/
    └── warehouse.sdf         # Monde entrepôt
```

---

## Outputs attendus (pour la soutenance)

1. **RSSI (dBm)** entre chaque paire de drones qui évolue dynamiquement
2. **Latence (ms)** réaliste entre drones selon distance/environnement
3. **Positions 3D** des drones dans le warehouse au cours du temps
4. **Simulation visuelle** Gazebo avec drones dans l'entrepôt
5. **Communication modélisée** avec channel model indoor (path loss + fading)
