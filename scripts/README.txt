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

  - Multi-drones : instancier plusieurs UAV dans Gazebo
  - NS-3 / NS3-Sionna : simuler la communication (RSSI, latence)
  - Environnement Warehouse dans Gazebo
  - Intelligence partagée entre drones (RL / Active Inference)

==========================================================================
