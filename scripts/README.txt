###############################################################################
#                    Phase 1 — Installation & Test Drone
#                    Simulation MC02 — Guide d'exécution
#                    Stack : ArduPilot SITL + Gazebo Harmonic
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
  │  Étape 2 : Installer Gazebo Harmonic                               │
  │  $ ./02_install_gazebo.sh                                          │
  │  Durée estimée : ~5-10 min                                         │
  │                                                                     │
  │  Test rapide : $ gz sim -v 4 shapes.sdf                            │
  │  (une fenêtre 3D devrait apparaître avec des formes)               │
  └─────────────────────────────────────────────────────────────────────┘

  ┌─────────────────────────────────────────────────────────────────────┐
  │  Étape 3 : Installer ArduPilot SITL + Plugin Gazebo               │
  │  $ ./03_install_px4_sitl.sh                                        │
  │  Durée estimée : ~10-15 min (première compilation)                 │
  │                                                                     │
  │  NOTE : Ce script :                                                │
  │    - Clone ArduPilot dans ~/ardupilot                              │
  │    - Installe les prérequis ArduPilot                              │
  │    - Compile ArduCopter pour SITL                                  │
  │    - Clone et compile ardupilot_gazebo dans ~/ardupilot_gazebo     │
  │    - Configure les variables d'environnement dans ~/.bashrc        │
  └─────────────────────────────────────────────────────────────────────┘

  ┌─────────────────────────────────────────────────────────────────────┐
  │  Étape 4 : Tester un drone                                        │
  │  $ ./04_test_single_drone.sh                                       │
  │  Lance ArduPilot SITL + Gazebo avec un quadcopter Iris.            │
  │                                                                     │
  │  Dans le prompt MAVProxy (STABILIZE>), essaie :                    │
  │    mode guided     → passer en mode guidé                          │
  │    arm throttle    → armer les moteurs                             │
  │    takeoff 10      → le drone décolle à 10m                       │
  │    mode land       → le drone atterrit                             │
  └─────────────────────────────────────────────────────────────────────┘

==========================================================================
                        ARCHITECTURE
==========================================================================

  ┌─────────────────────────────────────────────────────┐
  │                 Gazebo Harmonic                      │
  │         (Physique : aérodynamique, gravité)          │
  │         (Rendering 3D via GPU RTX 2060)              │
  │              Environnement : Warehouse               │
  │         Plugin : ardupilot_gazebo (JSON)             │
  └──────────────────────┬──────────────────────────────┘
                         │  JSON bridge (port 9002)
  ┌──────────────────────┴──────────────────────────────┐
  │             ArduPilot SITL (ArduCopter)              │
  │         (Flight Controller simulé)                   │
  │         sim_vehicle.py                               │
  │         Protocole : MAVLink                          │
  │         Port UDP : 14550                             │
  └──────────────────────┬──────────────────────────────┘
                         │
  ┌──────────────────────┴──────────────────────────────┐
  │        MAVProxy / QGroundControl / pymavlink         │
  │         (Contrôle et monitoring externe)             │
  └─────────────────────────────────────────────────────┘

==========================================================================
                    PROCHAINES ÉTAPES (Phase 2+)
==========================================================================

  - Multi-drones : instancier plusieurs ArduCopter dans Gazebo
      → sim_vehicle.py supporte -I pour l'instance ID
      → ex: sim_vehicle.py -v ArduCopter -f gazebo-iris -I 0
      →      sim_vehicle.py -v ArduCopter -f gazebo-iris -I 1
  - NS-3 / NS3-Sionna : simuler la communication (RSSI, latence)
  - Environnement Warehouse dans Gazebo
  - Intelligence partagée entre drones (RL / Active Inference)

==========================================================================
