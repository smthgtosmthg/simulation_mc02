# Guide d'exécution — simulation_mc02

## Prérequis

- Ubuntu 22.04+
- Connexion internet
- ~15 Go d'espace disque

---

## Étape 1 : Installation (dans l'ordre)

Exécuter chaque script depuis `scripts/installation/` :

```bash
cd simulation_mc02/scripts/installation

# 1. Dépendances système (build-essential, python3, pymavlink, dronekit, MAVProxy, GStreamer)
chmod +x *.sh
./01_install_dependencies.sh

# 2. Gazebo Harmonic
./02_install_gazebo.sh

# 3. ArduPilot SITL + plugin Gazebo
#    - Clone ArduPilot, compile ArduCopter SITL
#    - Clone ardupilot_gazebo, compile le plugin
#    - Configure GZ_SIM_SYSTEM_PLUGIN_PATH et GZ_SIM_RESOURCE_PATH dans .bashrc
./03_install_ardupilot_sitl.sh
source ~/.bashrc

# 4. NS-3.40 (avec ZMQ + ProtoBuf pour NS3-Sionna)
./04_install_ns3.sh

# 5. NS3-Sionna (module contrib + venv Python Sionna)
./05_install_ns3sionna.sh
source ~/.bashrc
```

**Vérification** : `gz sim --version` doit retourner Harmonic. `sim_vehicle.py --help` doit fonctionner.

---

## Étape 2 : Copier les scénarios NS-3

Copier les fichiers C++ dans le dossier scratch de NS-3 :

```bash
cp simulation_mc02/scenarios/drone-wifi-scenario.cc ~/ns-allinone-3.40/ns-3.40/scratch/
cp simulation_mc02/scenarios/drone-5g-nr-scenario.cc ~/ns-allinone-3.40/ns-3.40/scratch/
```

Recompiler NS-3 :

```bash
cd ~/ns-allinone-3.40/ns-3.40
./ns3 build
```

> **Note** : Le scénario 5G NR nécessite le module `nr` (5G-LENA) installé dans `contrib/nr`.

---

## Étape 3 : Lancer Gazebo + drones SITL

Terminal 1 :

```bash
cd simulation_mc02/scripts
chmod +x 06_launch_multi_drones.sh

# Lancer 3 drones (défaut). Changer le nombre si besoin.
./06_launch_multi_drones.sh 3
```

Ce script :

- Génère les modèles `iris_instance_X` avec ports FDM uniques (9002, 9012, 9022…)
- Génère `worlds/warehouse_drones.sdf` avec les drones positionnés
- Lance Gazebo (GUI par défaut, ajouter `HEADLESS=1` avant la commande pour sans GUI)
- Lance N instances `sim_vehicle.py` sur ports MAVLink 5760, 5770, 5780…

**Attendre** que tous les drones apparaissent dans Gazebo et que les terminaux SITL affichent `APM: EKF3 IMU0 is using GPS`.

---

## Étape 4 : Vol autonome

### Option A — Vol basique (décollage + hover + atterrissage)

Terminal 2 :

```bash
cd simulation_mc02/scripts
python3 07_multi_drone_flight.py
```

- Connecte les 3 drones via pymavlink (ports 5762, 5772, 5782)
- Décollage à altitudes décalées (4m, 5m, 6m)
- Hover pendant durée configurable
- Écrit les positions dans `/tmp/drone_positions.csv`
- Atterrissage automatique

### Option B — Vol dynamique (patrouille avec waypoints)

Terminal 2 :

```bash
python3 07b_dynamic_flight.py
```

- 3 drones naviguent dans l'entrepôt via 6 waypoints chacun
- Trajectoires variées : derrière les étagères (NLOS), centre (LOS), altitudes variables
- Génère des patterns réalistes RSSI/latence pour les bridges
- Boucles configurables

---

## Étape 5 : Bridge réseau (pendant le vol)

Lancer **pendant** que les drones volent (étape 4 en cours).

### Option A — Bridge WiFi (Sionna + NS-3 WiFi 802.11n)

Terminal 3 :

```bash
cd simulation_mc02/scripts
python3 08_wifi_bridge.py
```

- Lit les positions depuis `/tmp/drone_positions.csv`
- RSSI calculé par Sionna (ray-tracing)
- Latence calculée par NS-3 (`drone-wifi-scenario`, mode temps réel)
- Rendu 3D de la scène
- Résultats dans `/tmp/drone_rssi_latency.csv` et `/tmp/drone_bridge_log.csv`

Mode test (sans drones réels) :

```bash
python3 08_wifi_bridge.py --test
```

### Option B — Bridge 5G NR (Sionna + NS-3 5G-LENA)

Terminal 3 :

```bash
python3 09_5g_lena_bridge.py
```

- gNB positionné à (0, 0, 6m)
- RSSI via Sionna ray-tracing (gNB → chaque drone)
- Latence via NS-3 5G NR (scheduling + EPC end-to-end)
- Résultats dans CSVs séparés (RSSI Sionna + latence NS-3)
- Détecte automatiquement la fin du vol

---

## Résumé de l'ordre d'exécution

| Étape | Terminal | Commande                                                      | Rôle          |
| ----- | -------- | ------------------------------------------------------------- | ------------- |
| 1     | T1       | `./06_launch_multi_drones.sh 3`                               | Gazebo + SITL |
| 2     | T2       | `python3 07_multi_drone_flight.py` ou `07b_dynamic_flight.py` | Vol autonome  |
| 3     | T3       | `python3 08_wifi_bridge.py` ou `09_5g_lena_bridge.py`         | Bridge réseau |

---

## Ports utilisés

| Drone | Port FDM (Gazebo↔SITL) | Port MAVLink (contrôle) | Port pymavlink |
| ----- | ---------------------- | ----------------------- | -------------- |
| 0     | 9002                   | 5760                    | 5762           |
| 1     | 9012                   | 5770                    | 5772           |
| 2     | 9022                   | 5780                    | 5782           |

---

## Fichiers de sortie

| Fichier                       | Contenu                                             |
| ----------------------------- | --------------------------------------------------- |
| `/tmp/drone_positions.csv`    | Positions XYZ des drones (mis à jour en temps réel) |
| `/tmp/drone_rssi_latency.csv` | RSSI + latence combinés (bridge WiFi)               |
| `/tmp/drone_bridge_log.csv`   | Log détaillé du bridge                              |

---

## Dépannage

- **Drones ne décollent pas** : Vérifier que EKF est initialisé (`APM: EKF3 IMU0 is using GPS`)
- **`sim_vehicle.py` introuvable** : `source ~/.bashrc`, vérifier PATH ArduPilot
- **NS-3 build échoue** : Vérifier que les scénarios sont dans `scratch/`
- **Sionna import error** : Activer le venv → `source ~/ns-allinone-3.40/ns-3.40/sionna_venv/bin/activate`
- **Port déjà utilisé** : Tuer les anciens processus → `pkill -f sim_vehicle.py ; pkill -f gz`
- **Mode headless** : `HEADLESS=1 ./06_launch_multi_drones.sh 3`
