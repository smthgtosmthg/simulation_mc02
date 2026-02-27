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
                    PHASE 4 : 5G NR (5G-LENA)
==========================================================================

  ┌─────────────────────────────────────────────────────────────────────┐
  │  Étape 13 : Installer 5G-LENA (module NR)                         │
  │  $ chmod +x 13_install_5g_lena.sh && ./13_install_5g_lena.sh       │
  │  Clone 5G-LENA v2.6.y dans contrib/nr/, recompile NS-3.           │
  │  Durée : ~15-20 min (compilation avec module NR)                   │
  │                                                                     │
  │  Test : cd ~/ns-allinone-3.40/ns-3.40 && ./ns3 run cttc-nr-demo   │
  └─────────────────────────────────────────────────────────────────────┘

  ┌─────────────────────────────────────────────────────────────────────┐
  │  Étape 14 : Bridge ArduPilot ↔ NS-3 5G NR (Terminal 2)            │
  │  $ python3 14_ns3_5g_bridge.py --drones 3                          │
  │  Comme le script 11, mais avec un réseau 5G NR :                   │
  │    - 1 gNB au centre de l'entrepôt                                 │
  │    - N drones comme UEs                                            │
  │    - Métriques : RSRP, latence, distance à la gNB                 │
  │                                                                     │
  │  Options :                                                          │
  │    --no-ns3              → mode standalone (sans NS-3)             │
  │    --frequency 28e9      → FR2 mmWave                              │
  │    --numerology 3        → numérologie FR2                         │
  │    --scenario InH-OfficeOpen → scénario indoor ouvert              │
  │                                                                     │
  │  Sortie : ~/simulation_mc02/comm_metrics_5g.csv                    │
  └─────────────────────────────────────────────────────────────────────┘

  ┌─────────────────────────────────────────────────────────────────────┐
  │  Étape 15 : Vol + Bridge 5G NR (tout-en-un)                       │
  │  $ python3 15_flight_and_5g_bridge.py --drones 3                   │
  │  Combine vol automatique + bridge 5G NR en un seul script.         │
  │                                                                     │
  │  Exemples :                                                         │
  │    python3 15_flight_and_5g_bridge.py --drones 3                   │
  │    python3 15_flight_and_5g_bridge.py --drones 3 --frequency 28e9  │
  │    python3 15_flight_and_5g_bridge.py --drones 5 --hover 60        │
  │                                                                     │
  │  Sortie : ~/simulation_mc02/comm_metrics_5g.csv                    │
  └─────────────────────────────────────────────────────────────────────┘

  Le scénario NS-3 5G (C++) est dans :
    ~/simulation_mc02/ns3_scenarios/drone-5g-scenario.cc

  Il utilise :
    - 5G NR via 5G-LENA (gNB + UEs)
    - FR1 : 3.5 GHz, 20 MHz, μ=1 (30 kHz SCS)  ← par défaut
    - FR2 : 28 GHz, 100 MHz, μ=3 (120 kHz SCS)  ← option mmWave
    - Modèle de canal 3GPP TR 38.901 InH-Office
    - Beamforming idéal (DirectPath)
    - EPC (réseau cœur) pour routage inter-drones
    - FlowMonitor (latence réelle)

  Documentation 5G : docs/documentation_drone_5g_scenario.md

  Comparaison WiFi vs 5G :
    ┌──────────────┬──────────────────┬──────────────────────┐
    │              │ WiFi 802.11n     │ 5G NR (FR1)          │
    ├──────────────┼──────────────────┼──────────────────────┤
    │ Topologie    │ Ad-Hoc (mesh)    │ Étoile (gNB)         │
    │ Fréquence    │ 2.4 GHz          │ 3.5 GHz              │
    │ Métrique     │ RSSI             │ RSRP                 │
    │ Latence      │ ~2-5 ms          │ ~1-3 ms              │
    │ Scénario     │ drone-wifi-*.cc  │ drone-5g-*.cc        │
    │ Bridge       │ 11 / 12          │ 14 / 15              │
    │ Sortie CSV   │ comm_metrics.csv │ comm_metrics_5g.csv  │
    └──────────────┴──────────────────┴──────────────────────┘

==========================================================================
                    PHASE 5+ : À VENIR
==========================================================================

  - Intégration Sionna RT (ray-tracing GPU) avec 5G-LENA
  - Intelligence partagée entre drones (RL / Active Inference)
  - Navigation autonome collaborative (prise en compte RSRP/latence)
  - Digital Twin complet

==========================================================================
