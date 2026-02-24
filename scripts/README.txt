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

  ┌─────────────────────────────────────────────────────────────────────┐
  │  Étape 5 (bonus) : Vol automatique                                 │
  │  Pendant que le script 04 tourne dans un terminal,                 │
  │  ouvre un AUTRE terminal et lance :                                │
  │  $ ./05_auto_flight_test.sh                                        │
  │  → arm, takeoff 10m, hover 10s, land automatiquement              │
  └─────────────────────────────────────────────────────────────────────┘

==========================================================================
                        ARCHITECTURE
==========================================================================

  ┌─────────────────────────────────────────────────────┐
  │                   Gazebo Garden                     │
  │         (Physique : aérodynamique, gravité)         │
  │         (Rendering 3D via GPU RTX 2060)             │
  │              Environnement : Warehouse              │
  └──────────────────────┬──────────────────────────────┘
                         │  Bridge (automatique)
  ┌──────────────────────┴──────────────────────────────┐
  │                  PX4 SITL (x500)                    │
  │         (Flight Controller simulé)                  │
  │         Protocole : MAVLink                         │
  │         Port UDP : 14540 / 14550                    │
  └──────────────────────┬──────────────────────────────┘
                         │
  ┌──────────────────────┴──────────────────────────────┐
  │        MAVProxy / QGroundControl / pymavlink        │
  │         (Contrôle et monitoring externe)            │
  └─────────────────────────────────────────────────────┘

==========================================================================
                    PROCHAINES ÉTAPES (Phase 2+)
==========================================================================

  --- PHASE 2 : MULTI-DRONES + WAREHOUSE ---

  ┌─────────────────────────────────────────────────────────────────────┐
  │  Étape 6 : Créer le monde Warehouse                               │
  │  $ chmod +x 07_setup_warehouse.sh && ./07_setup_warehouse.sh       │
  │  Crée un entrepôt 30m x 15m avec étagères et obstacles.           │
  │                                                                     │
  │  Test : $ gz sim -v 4 -r worlds/warehouse.sdf                     │
  └─────────────────────────────────────────────────────────────────────┘

  ┌─────────────────────────────────────────────────────────────────────┐
  │  Étape 7 : Lancer plusieurs drones (Terminal 1)                    │
  │  $ chmod +x 06_launch_multi_drones.sh                              │
  │  $ ./06_launch_multi_drones.sh 3      → 3 drones                  │
  │  $ ./06_launch_multi_drones.sh 5      → 5 drones                  │
  │                                                                     │
  │  Les drones sont espacés de 3m dans le warehouse.                  │
  │  Chaque drone a son propre port MAVLink (14550, 14551, 14552...)   │
  └─────────────────────────────────────────────────────────────────────┘

  ┌─────────────────────────────────────────────────────────────────────┐
  │  Étape 8 : Tracker les positions (Terminal 2)                      │
  │  $ python3 08_track_positions.py --drones 3                        │
  │  Affiche les positions en temps réel + sauvegarde en CSV.          │
  │                                                                     │
  │  Options :                                                          │
  │    --rate 2       → 2 Hz (mise à jour 2x par seconde)              │
  │    --output log.csv  → nom du fichier de sortie                    │
  └─────────────────────────────────────────────────────────────────────┘

  ┌─────────────────────────────────────────────────────────────────────┐
  │  Étape 9 : Vol automatique multi-drones (Terminal 2)               │
  │  $ python3 09_multi_drone_flight.py --drones 3                     │
  │  Arm → Takeoff (altitudes différentes) → Hover 15s → Land         │
  │                                                                     │
  │  Options :                                                          │
  │    --hover 30         → hover 30 secondes                          │
  │    --base-alt 5       → altitude de base 5m                        │
  │    --alt-step 2       → 2m d'écart entre drones                    │
  └─────────────────────────────────────────────────────────────────────┘

  --- PHASE 3+ : À VENIR ---

  - NS-3 / NS3-Sionna : simuler la communication (RSSI, latence)
  - Intelligence partagée entre drones (RL / Active Inference)

==========================================================================
