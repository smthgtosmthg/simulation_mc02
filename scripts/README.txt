###############################################################################
#                    Phase 1 — Installation & Test Drone
#                    Simulation MC02 — Guide d'exécution
###############################################################################

SYSTÈME VÉRIFIÉ :
  - Ubuntu 22.04.5 LTS
  - GPU : NVIDIA RTX 2060 SUPER (8GB)
  - RAM : 32 GB
  - CPU : 20 cœurs
  - Driver NVIDIA : 535.288.01, CUDA 12.2

==========================================================================
                        ORDRE D'EXÉCUTION
==========================================================================

  Ouvre un terminal et exécute les scripts dans cet ordre :

  cd ~/simulation_mc02/scripts

  ┌─────────────────────────────────────────────────────────────────────┐
  │  Étape 1 : Installer les dépendances                              │
  │  $ chmod +x *.sh                                                   │
  │  $ ./01_install_dependencies.sh                                    │
  │  Durée estimée : ~5 min                                            │
  └─────────────────────────────────────────────────────────────────────┘

  ┌─────────────────────────────────────────────────────────────────────┐
  │  Étape 2 : Installer Gazebo Garden                                 │
  │  $ ./02_install_gazebo.sh                                          │
  │  Durée estimée : ~5-10 min                                         │
  │                                                                     │
  │  Test rapide : $ gz sim -v 4 shapes.sdf                            │
  │  (une fenêtre 3D devrait apparaître avec des formes)               │
  └─────────────────────────────────────────────────────────────────────┘

  ┌─────────────────────────────────────────────────────────────────────┐
  │  Étape 3 : Installer et compiler PX4 SITL                         │
  │  $ ./03_install_px4_sitl.sh                                        │
  │  Durée estimée : ~15-20 min (première compilation)                 │
  │                                                                     │
  │  NOTE : Ce script clone PX4 dans ~/PX4-Autopilot                  │
  │         et lance une première simulation pour vérifier.            │
  │         Ferme avec Ctrl+C une fois que tu vois le shell pxh>      │
  └─────────────────────────────────────────────────────────────────────┘

  ┌─────────────────────────────────────────────────────────────────────┐
  │  Étape 4 : Tester un drone                                        │
  │  $ ./04_test_single_drone.sh                                       │
  │  Lance PX4 SITL + Gazebo avec un quadcopter x500.                 │
  │                                                                     │
  │  Dans le shell pxh>, essaie :                                      │
  │    commander takeoff    → le drone décolle                         │
  │    commander land       → le drone atterrit                        │
  └─────────────────────────────────────────────────────────────────────┘

==========================================================================
                        ARCHITECTURE
==========================================================================

  ┌─────────────────────────────────────────────────────┐
  │               Gazebo Harmonic                       │
  │       (Physique : aérodynamique, gravité)           │
  │       (Rendering 3D via GPU — optionnel)            │
  │            Environnement : Warehouse                │
  │            30m x 20m x 6m + étagères                │
  └──────────────────────┬──────────────────────────────┘
                         │  Plugin ardupilot_gazebo
                         │  (JSON, ports 9002+I*10)
  ┌──────────────────────┴──────────────────────────────┐
  │            ArduPilot SITL (ArduCopter)              │
  │         Flight Controller simulé — Iris             │
  │         Protocole : MAVLink                         │
  │         TCP : 5760 + I*10 (par instance)            │
  └──────────────────────┬──────────────────────────────┘
                         │
  ┌──────────────────────┴──────────────────────────────┐
  │        pymavlink / MAVProxy / QGroundControl        │
  │          (Contrôle et monitoring externe)            │
  └─────────────────────────────────────────────────────┘

==========================================================================
                    PHASE 2 : MULTI-DRONES + WAREHOUSE
==========================================================================

  ┌─────────────────────────────────────────────────────────────────────┐
  │  Étape 5 : Setup Warehouse                                        │
  │  $ chmod +x 05_setup_warehouse.sh && ./05_setup_warehouse.sh       │
  │  Crée un entrepôt 30m x 20m avec étagères, caisses, éclairage.   │
  │                                                                     │
  │  Test : gz sim -v 4 -r ~/simulation_mc02/worlds/warehouse.sdf     │
  └─────────────────────────────────────────────────────────────────────┘

  ┌─────────────────────────────────────────────────────────────────────┐
  │  Étape 6 : Lancer N drones dans le warehouse (Terminal 1)         │
  │  $ chmod +x 06_launch_multi_drones.sh                              │
  │  $ ./06_launch_multi_drones.sh 3      → 3 drones                  │
  │  $ ./06_launch_multi_drones.sh 5      → 5 drones                  │
  │                                                                     │
  │  Crée N copies du modèle iris avec ports uniques.                  │
  │  Génère le monde SDF et lance Gazebo + N SITL.                     │
  │  Drones espacés de 3m. Ctrl+C pour tout arrêter.                   │
  │                                                                     │
  │  Ports MAVLink :                                                    │
  │    Drone 0 → tcp:127.0.0.1:5760                                   │
  │    Drone 1 → tcp:127.0.0.1:5770                                   │
  │    Drone 2 → tcp:127.0.0.1:5780                                   │
  └─────────────────────────────────────────────────────────────────────┘

  ┌─────────────────────────────────────────────────────────────────────┐
  │  Étape 7 : Tracker les positions (Terminal 2)                      │
  │  $ python3 07_track_positions.py --drones 3                        │
  │  Affiche les positions en temps réel + sauvegarde en CSV.          │
  │                                                                     │
  │  Options :                                                          │
  │    --rate 4          → 4 Hz                                        │
  │    --output log.csv  → nom du fichier de sortie                    │
  │    --duration 60     → arrêter après 60 secondes                   │
  └─────────────────────────────────────────────────────────────────────┘

  ┌─────────────────────────────────────────────────────────────────────┐
  │  Étape 8 : Vol automatique multi-drones (Terminal 2)               │
  │  $ python3 08_multi_drone_flight.py --drones 3                     │
  │  GUIDED → Arm → Takeoff → Hover 15s → Land                        │
  │                                                                     │
  │  Options :                                                          │
  │    --altitude 5      → altitude de base 5m                         │
  │    --alt-step 2      → 2m d'écart entre drones                    │
  │    --hover 30        → hover 30 secondes                           │
  └─────────────────────────────────────────────────────────────────────┘

==========================================================================
                    PHASE 3 : NS-3 + COMMUNICATION
==========================================================================

  ┌─────────────────────────────────────────────────────────────────────┐
  │  Étape 9 : Installer NS-3.40                                      │
  │  $ chmod +x 09_install_ns3.sh && ./09_install_ns3.sh               │
  │  Télécharge, compile NS-3.40 (requis par NS3-Sionna).             │
  │  Durée : ~10-15 min (première compilation)                         │
  │                                                                     │
  │  Test : cd ~/ns-allinone-3.40/ns-3.40 && ./ns3 run hello-simulator│
  └─────────────────────────────────────────────────────────────────────┘

  ┌─────────────────────────────────────────────────────────────────────┐
  │  Étape 10 : Installer NS3-Sionna                                  │
  │  $ chmod +x 10_install_ns3sionna.sh && ./10_install_ns3sionna.sh   │
  │  Clone NS3-Sionna dans contrib/, recompile NS-3,                   │
  │  installe le serveur Python Sionna (venv + Sionna 1.2.1).         │
  │                                                                     │
  │  Lancer le serveur Sionna :                                        │
  │    cd ~/ns-allinone-3.40/ns-3.40/contrib/sionna/model/ns3sionna   │
  │    source sionna-venv/bin/activate && ./run.sh                     │
  └─────────────────────────────────────────────────────────────────────┘

  ┌─────────────────────────────────────────────────────────────────────┐
  │  Étape 11 : Bridge ArduPilot ↔ NS-3 (Terminal 2)                  │
  │  $ python3 11_ns3_bridge.py --drones 3                             │
  │  Lit les positions des drones, calcule RSSI + latence,             │
  │  écrit les métriques dans ~/simulation_mc02/comm_metrics.csv       │
  │                                                                     │
  │  Options :                                                          │
  │    --no-ns3         → mode standalone (sans NS-3, Log-Distance)    │
  │    --duration 60    → arrêter après 60 secondes                    │
  │    --rate 4         → 4 Hz                                         │
  │                                                                     │
  │  Prérequis : 06_launch_multi_drones.sh tourne dans un terminal     │
  └─────────────────────────────────────────────────────────────────────┘

  ┌─────────────────────────────────────────────────────────────────────┐
  │  Étape 12 : Démo complète (tout-en-un)                            │
  │  $ chmod +x 12_demo_full.sh && ./12_demo_full.sh 3                │
  │  Lance tout : Warehouse + N drones + Bridge + Vol auto             │
  │                                                                     │
  │  Modes :                                                            │
  │    ./12_demo_full.sh 3             → 3 drones, standalone          │
  │    ./12_demo_full.sh 3 ns3         → 3 drones, avec NS-3          │
  │    ./12_demo_full.sh 5             → 5 drones, standalone          │
  │                                                                     │
  │  Sortie : ~/simulation_mc02/comm_metrics.csv                       │
  │  (timestamp, positions, distance, RSSI dBm, latence ms)            │
  └─────────────────────────────────────────────────────────────────────┘

  Le scénario NS-3 (C++) est dans :
    ~/simulation_mc02/ns3_scenarios/drone-wifi-scenario.cc

  Il utilise :
    - WiFi 802.11n Ad-Hoc (2.4 GHz)
    - Log-Distance Path Loss (n=3.0, indoor warehouse)
    - TxPower = 20 dBm
    - UDP Echo entre drones (mesure latence réelle)
    - FlowMonitor (statistiques de flux)

==========================================================================
                    PHASE 4+ : À VENIR
==========================================================================

  - Intégration complète NS3-Sionna (ray tracing 3D dans warehouse)
  - Intelligence partagée entre drones (RL / Active Inference)
  - Navigation autonome collaborative (prise en compte RSSI/latence)
  - Digital Twin complet

==========================================================================
