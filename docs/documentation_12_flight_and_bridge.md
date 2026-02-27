# Documentation Complète — `12_flight_and_bridge.py`

## Vol Multi-Drones + Bridge NS-3 (Script Combiné)

---

## Table des Matières

1. [Vue d'ensemble](#1-vue-densemble)
2. [Architecture du script](#2-architecture-du-script)
3. [Prérequis et dépendances](#3-prérequis-et-dépendances)
4. [Les imports](#4-les-imports)
5. [Constantes ArduCopter (modes de vol)](#5-constantes-arducopter-modes-de-vol)
6. [Configuration des fichiers](#6-configuration-des-fichiers)
7. [État partagé — classe `SharedState`](#7-état-partagé--classe-sharedstate)
8. [Modèle de propagation standalone](#8-modèle-de-propagation-standalone)
9. [Fonctions drone (contrôle MAVLink)](#9-fonctions-drone-contrôle-mavlink)
10. [Fonctions CSV (lecture/écriture)](#10-fonctions-csv-lectureécriture)
11. [Thread FLIGHT — séquence de vol](#11-thread-flight--séquence-de-vol)
12. [Thread BRIDGE — métriques réseau](#12-thread-bridge--métriques-réseau)
13. [Programme principal `main()`](#13-programme-principal-main)
14. [Arguments en ligne de commande](#14-arguments-en-ligne-de-commande)
15. [Flux d'exécution complet](#15-flux-dexécution-complet)
16. [Schéma de communication entre threads](#16-schéma-de-communication-entre-threads)
17. [Format des fichiers de sortie](#17-format-des-fichiers-de-sortie)
18. [Exemples d'utilisation](#18-exemples-dutilisation)
19. [Dépannage et erreurs courantes](#19-dépannage-et-erreurs-courantes)
20. [Glossaire](#20-glossaire)

---

## 1. Vue d'ensemble

### Qu'est-ce que ce script fait ?

Ce script contrôle **plusieurs drones simulés** (via PX4/ArduCopter dans Gazebo) et mesure **les communications réseau** entre eux en temps réel grâce au simulateur réseau NS-3.

Concrètement, il fait **deux choses en parallèle** :

| Thread | Rôle |
|--------|------|
| **FLIGHT** | Fait décoller les drones, les maintient en vol (hover), puis les fait atterrir |
| **BRIDGE** | Lit les positions GPS des drones, calcule les métriques réseau (RSSI, latence, distance) et enregistre tout dans un fichier CSV |

### Pourquoi un seul script au lieu de deux ?

Dans les versions précédentes, il y avait deux scripts séparés :
- `08_multi_drone_flight.py` (contrôle du vol)
- `11_ns3_bridge.py` (métriques réseau)

Le **problème** : chaque script ouvrait sa propre connexion MAVLink au drone. Or, MAVLink ne supporte pas bien les connexions multiples simultanées au même drone. Le résultat : conflits, timeouts, données corrompues.

La **solution** : ce script **connecte les drones UNE SEULE FOIS**, puis partage ces connexions entre deux threads qui tournent en parallèle.

---

## 2. Architecture du script

```
┌─────────────────────────────────────────────────────────────┐
│                    main() — Programme principal              │
│                                                              │
│  1. Parse les arguments CLI                                  │
│  2. Connecte les N drones (UNE SEULE FOIS)                  │
│  3. Lance NS-3 en sous-processus                            │
│  4. Crée l'état partagé (SharedState)                       │
│  5. Lance 2 threads en parallèle                            │
│  6. Attend la fin du vol                                     │
│  7. Nettoie et ferme tout                                   │
└──────────────┬──────────────────────┬───────────────────────┘
               │                      │
    ┌──────────▼──────────┐ ┌────────▼─────────────┐
    │   Thread FLIGHT     │ │   Thread BRIDGE       │
    │                     │ │                       │
    │ - Attente EKF       │ │ - Lit positions       │
    │ - Mode GUIDED       │ │ - Écrit CSV partagé   │
    │ - Armement          │ │ - Lit sortie NS-3     │
    │ - Décollage         │ │ - Calcule RSSI/lat.   │
    │ - Hover (N sec)     │ │ - Écrit log unifié    │
    │ - Atterrissage      │ │ - Boucle à X Hz      │
    └─────────────────────┘ └───────────────────────┘
               │                      │
               │     SharedState      │
               │  (positions, flags)  │
               └──────────┬───────────┘
                          │
              ┌───────────▼───────────┐
              │   NS-3 (sous-proc.)   │
              │                       │
              │ Lit: drone_positions  │
              │ Écrit: ns3_output     │
              └───────────────────────┘
```

---

## 3. Prérequis et dépendances

### Logiciels nécessaires

| Logiciel | Rôle | Script d'installation |
|----------|------|-----------------------|
| **Gazebo** | Simulateur physique 3D pour les drones | `02_install_gazebo.sh` |
| **PX4 SITL** | Autopilote simulé (ArduCopter) | `03_install_px4_sitl.sh` |
| **NS-3** | Simulateur réseau | `09_install_ns3.sh` |
| **pymavlink** | Bibliothèque Python pour communiquer avec les drones via MAVLink | `pip3 install pymavlink` |

### Ce qui doit tourner AVANT de lancer ce script

```bash
# Terminal 1 : Lancer Gazebo + les N drones simulés
./06_launch_multi_drones.sh
```

Ce script (`06`) lance Gazebo avec le monde `warehouse.sdf` et démarre N instances de PX4/ArduCopter, chacune écoutant sur un port TCP différent.

---

## 4. Les imports

```python
import argparse       # Gestion des arguments en ligne de commande (--drones 3, etc.)
import csv            # Lecture/écriture de fichiers CSV
import math           # Fonctions mathématiques (sqrt, log10)
import os             # Opérations sur le système de fichiers (chemins, existence)
import signal         # Gestion du signal CTRL+C (SIGINT)
import subprocess     # Lancer des programmes externes (NS-3)
import sys            # Accès aux fonctions système (exit, stdout)
import threading      # Création de threads pour le parallélisme
import time           # Fonctions de temps (sleep, time)
from datetime import datetime  # Horodatage (non utilisé dans le code actuel)
```

### Import conditionnel de pymavlink

```python
try:
    from pymavlink import mavutil
except ImportError:
    print("ERREUR: pymavlink non installé.")
    print("pip3 install pymavlink")
    sys.exit(1)
```

**Explication** :
- `pymavlink` est la bibliothèque qui permet de communiquer avec les drones via le protocole **MAVLink**
- Le `try/except` vérifie que la bibliothèque est installée
- Si elle manque, le script affiche un message d'erreur utile et s'arrête proprement (`sys.exit(1)`)
- `mavutil` est le module principal de pymavlink, il fournit les fonctions pour se connecter aux drones et envoyer/recevoir des commandes

---

## 5. Constantes ArduCopter (modes de vol)

```python
MODE_STABILIZE = 0    # Le pilote contrôle manuellement, le drone stabilise seulement l'attitude
MODE_ALT_HOLD  = 2    # Maintien automatique de l'altitude
MODE_AUTO      = 3    # Suit une mission préprogrammée (waypoints)
MODE_GUIDED    = 4    # Accepte des commandes de position/vitesse envoyées par programme
MODE_LOITER    = 5    # Maintien de position GPS automatique
MODE_RTL       = 6    # Return To Launch — retour automatique au point de départ
MODE_LAND      = 9    # Atterrissage automatique sur place
```

### Pourquoi ces modes sont importants ?

Le script utilise principalement **deux modes** :

1. **`MODE_GUIDED` (4)** : C'est le mode utilisé pour le décollage et le hover. En mode GUIDED, le drone accepte des commandes envoyées par notre script Python (takeoff, goto, etc.). C'est le mode **indispensable** pour le contrôle automatisé.

2. **`MODE_LAND` (9)** : Utilisé à la fin du vol pour faire atterrir le drone automatiquement.

```python
MODE_NAMES = {
    0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
    5: "LOITER", 6: "RTL", 9: "LAND"
}
```

Ce dictionnaire associe les numéros de mode à leurs noms pour l'affichage (utile pour le debug).

---

## 6. Configuration des fichiers

```python
POS_FILE    = "/tmp/drone_positions.csv"
NS3_OUTPUT  = "/tmp/ns3_output.csv"
WORKSPACE   = os.path.expanduser("~/simulation_mc02")
UNIFIED_LOG = os.path.join(WORKSPACE, "comm_metrics.csv")
NS3_DIR     = os.path.expanduser("~/ns-allinone-3.40/ns-3.40")
NS3_SCENARIO = "scratch/drone-wifi-scenario"
```

### Détail de chaque fichier

| Variable | Chemin | Rôle | Qui écrit | Qui lit |
|----------|--------|------|-----------|---------|
| `POS_FILE` | `/tmp/drone_positions.csv` | Positions actuelles des drones | Thread BRIDGE | NS-3 |
| `NS3_OUTPUT` | `/tmp/ns3_output.csv` | Résultats réseau calculés par NS-3 | NS-3 | Thread BRIDGE |
| `UNIFIED_LOG` | `~/simulation_mc02/comm_metrics.csv` | **Log final** avec positions + métriques | Thread BRIDGE | L'utilisateur |
| `NS3_DIR` | `~/ns-allinone-3.40/ns-3.40` | Répertoire d'installation de NS-3 | — | main() |
| `NS3_SCENARIO` | `scratch/drone-wifi-scenario` | Le scénario NS-3 compilé | — | main() |

### Flux des données entre fichiers

```
Drones (MAVLink)  ──→  Thread BRIDGE  ──→  /tmp/drone_positions.csv
                             │                       │
                             │                       ▼
                             │                  NS-3 (lit)
                             │                       │
                             │                       ▼
                             │              /tmp/ns3_output.csv
                             │                       │
                             ◄───────────────────────┘
                             │
                             ▼
                    comm_metrics.csv (LOG FINAL)
```

---

## 7. État partagé — classe `SharedState`

```python
class SharedState:
    def __init__(self, n_drones):
        self.n_drones = n_drones
        self.lock = threading.Lock()
        self.positions = [None] * n_drones
        self.running = True
        self.drones_airborne = threading.Event()
        self.flight_done = threading.Event()
```

### Pourquoi cette classe existe ?

Les deux threads (FLIGHT et BRIDGE) tournent **en parallèle** et doivent communiquer. En Python, quand deux threads accèdent aux mêmes données, il faut un mécanisme de **synchronisation** pour éviter les conflits.

### Attributs expliqués un par un

#### `self.n_drones` — Nombre de drones
Simple entier stockant le nombre de drones dans la simulation.

#### `self.lock` — Verrou mutex
```python
self.lock = threading.Lock()
```
Un **Lock** (verrou) est comme une clé unique. Quand un thread "prend" le lock (`with self.lock:`), l'autre thread doit attendre qu'il le relâche avant de pouvoir accéder aux données protégées. Cela empêche les deux threads de modifier `self.positions` en même temps.

**Analogie** : Imagine une salle de bain avec un seul verrou. Une seule personne peut y entrer à la fois. L'autre attend dehors.

#### `self.positions` — Liste des positions
```python
self.positions = [None] * n_drones
```
Crée une liste de N éléments `None`. Chaque élément sera remplacé par un dictionnaire `{'x': ..., 'y': ..., 'z': ...}` quand le bridge lira la position du drone correspondant.

Exemple pour 3 drones : `[None, None, None]` → `[{'x': 0.1, 'y': 0.0, 'z': 4.0}, {'x': 0.2, 'y': 3.0, 'z': 5.0}, {'x': 0.0, 'y': 6.0, 'z': 6.0}]`

#### `self.running` — Flag d'arrêt global
```python
self.running = True
```
Quand ce booléen passe à `False`, **les deux threads s'arrêtent**. C'est le mécanisme d'arrêt propre : chaque boucle vérifie `if not state.running: return`.

#### `self.drones_airborne` — Signal "drones en l'air"
```python
self.drones_airborne = threading.Event()
```
Un **Event** est un signal qu'un thread peut envoyer aux autres. Ici, le thread FLIGHT appelle `state.drones_airborne.set()` quand tous les drones ont décollé. Le thread BRIDGE peut utiliser `state.drones_airborne.wait()` pour attendre ce signal.

#### `self.flight_done` — Signal "vol terminé"
```python
self.flight_done = threading.Event()
```
Même principe : le thread FLIGHT appelle `state.flight_done.set()` quand le vol est complètement terminé. Le thread BRIDGE vérifie `state.flight_done.is_set()` pour savoir s'il doit s'arrêter.

---

## 8. Modèle de propagation standalone

Ces deux fonctions calculent les métriques réseau **sans NS-3**, en utilisant un modèle mathématique simplifié. Elles servent de **fallback** (solution de secours) quand NS-3 n'a pas encore produit de résultats.

### `compute_rssi_standalone()`

```python
def compute_rssi_standalone(pos_i, pos_j, tx_power_dbm=20.0,
                            path_loss_exp=3.0, ref_loss_db=40.0):
```

**Objectif** : Calculer la puissance du signal Wi-Fi reçu (RSSI) entre deux drones.

#### Paramètres

| Paramètre | Type | Défaut | Description |
|-----------|------|--------|-------------|
| `pos_i` | dict | — | Position du drone émetteur `{'x': ..., 'y': ..., 'z': ...}` |
| `pos_j` | dict | — | Position du drone récepteur |
| `tx_power_dbm` | float | 20.0 | Puissance d'émission en dBm (décibels-milliwatts) |
| `path_loss_exp` | float | 3.0 | Exposant de perte de propagation (3.0 = environnement intérieur) |
| `ref_loss_db` | float | 40.0 | Perte de référence à 1 mètre en dB |

#### Que fait cette fonction étape par étape ?

```python
# 1. Calcul de la distance 3D entre les deux drones
dx = pos_i['x'] - pos_j['x']       # Différence en X
dy = pos_i['y'] - pos_j['y']       # Différence en Y
dz = pos_i['z'] - pos_j['z']       # Différence en Z
distance = math.sqrt(dx*dx + dy*dy + dz*dz)  # Théorème de Pythagore 3D
```

**Formule** : $d = \sqrt{(x_1-x_2)^2 + (y_1-y_2)^2 + (z_1-z_2)^2}$

```python
# 2. Protection contre la division par zéro
if distance < 0.01:
    distance = 0.01    # Minimum 1 cm
```

Si deux drones sont exactement au même endroit, la distance serait 0 et `log10(0)` causerait une erreur mathématique.

```python
# 3. Modèle Log-Distance de perte de propagation
path_loss = ref_loss_db + 10.0 * path_loss_exp * math.log10(distance)
```

**Formule** : $PL = PL_0 + 10 \cdot n \cdot \log_{10}(d)$

Où :
- $PL$ = perte de propagation totale (en dB)
- $PL_0$ = perte à 1 mètre (40 dB par défaut)
- $n$ = exposant de propagation (3.0 = intérieur avec obstacles)
- $d$ = distance en mètres

```python
# 4. RSSI = puissance émise — pertes
rssi = tx_power_dbm - path_loss
return rssi, distance
```

**Formule** : $RSSI = P_{tx} - PL$

**Exemple concret** :
- Deux drones à 10 mètres de distance
- $PL = 40 + 10 \times 3.0 \times \log_{10}(10) = 40 + 30 = 70$ dB
- $RSSI = 20 - 70 = -50$ dBm

**Interprétation du RSSI** :
| RSSI (dBm) | Qualité du signal |
|------------|-------------------|
| > -30 | Excellent |
| -30 à -50 | Bon |
| -50 à -70 | Moyen |
| -70 à -80 | Faible |
| < -80 | Très mauvais / perte de connexion |

### `compute_latency_standalone()`

```python
def compute_latency_standalone(distance, mac_delay_ms=2.0):
    propagation_ms = (distance / 3e8) * 1000.0
    return propagation_ms + mac_delay_ms
```

**Objectif** : Estimer la latence (délai) de communication entre deux drones.

#### Deux composantes de la latence

1. **Délai de propagation** : Le temps que met l'onde radio à parcourir la distance
   - Formule : $t_{prop} = \frac{d}{c}$ où $c = 3 \times 10^8$ m/s (vitesse de la lumière)
   - Pour 10 m : $\frac{10}{3 \times 10^8} = 3.33 \times 10^{-8}$ s = 0.0000333 ms
   - **C'est négligeable** à ces distances !

2. **Délai MAC** : Le temps de traitement par la couche MAC (Medium Access Control) du Wi-Fi
   - Valeur fixe par défaut : 2.0 ms
   - Inclut : le temps d'accès au canal, les ACK (accusés de réception), etc.

**Résultat** : En pratique, la latence est presque toujours ≈ 2 ms pour des drones dans un entrepôt.

---

## 9. Fonctions drone (contrôle MAVLink)

Toutes ces fonctions communiquent avec les drones via le protocole **MAVLink**. Chaque fonction prend un paramètre `lock` pour être **thread-safe** (utilisable en parallèle sans conflit).

### `connect_drone(instance, timeout=30)`

```python
def connect_drone(instance, timeout=30):
    port = 5760 + instance * 10
    addr = f"tcp:127.0.0.1:{port}"
```

**Logique des ports** :

| Instance | Port | Adresse complète |
|----------|------|------------------|
| Drone 0 | 5760 | `tcp:127.0.0.1:5760` |
| Drone 1 | 5770 | `tcp:127.0.0.1:5770` |
| Drone 2 | 5780 | `tcp:127.0.0.1:5780` |

Chaque drone PX4/ArduCopter simulé écoute sur son propre port TCP. La formule est : `port = 5760 + instance × 10`.

```python
    conn = mavutil.mavlink_connection(addr, source_system=255)
    conn.wait_heartbeat(timeout=timeout)
```

- `mavlink_connection()` : Crée la connexion TCP vers le drone
- `source_system=255` : Notre script s'identifie avec l'ID 255 (convention pour une station au sol / GCS)
- `wait_heartbeat()` : Attend de recevoir un **heartbeat** (battement de cœur) du drone. C'est un message envoyé régulièrement par le drone pour dire "je suis vivant". Si aucun heartbeat n'arrive en 30s, la connexion a échoué.

**Retour** : L'objet `conn` (connexion MAVLink) ou `None` en cas d'erreur.

### `set_mode(conn, lock, mode_id, timeout=10)`

```python
def set_mode(conn, lock, mode_id, timeout=10):
    with lock:
        conn.mav.set_mode_send(
            conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
```

**Objectif** : Changer le mode de vol du drone (ex: passer en GUIDED ou LAND).

- `with lock:` : Prend le verrou pendant l'envoi pour éviter les conflits
- `conn.target_system` : L'ID du drone (obtenu lors de la connexion)
- `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED` : Flag qui dit "utilise le mode personnalisé" (les modes ArduCopter sont des modes custom dans MAVLink)
- `mode_id` : Le numéro du mode souhaité (ex: 4 pour GUIDED)

Ensuite, la fonction **vérifie** que le mode a bien changé en lisant les heartbeats :

```python
    while time.time() - t0 < timeout:
        with lock:
            hb = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb and hb.custom_mode == mode_id:
            return True    # Le mode a changé avec succès
    return False           # Timeout : le mode n'a pas changé
```

### `arm_drone(conn, lock, timeout=30)`

```python
conn.mav.command_long_send(
    conn.target_system,
    conn.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,        # confirmation
    1,        # 1 = armer, 0 = désarmer
    0, 0, 0, 0, 0, 0  # paramètres non utilisés
)
```

**Objectif** : Armer les moteurs du drone.

**C'est quoi "armer" ?** : Par sécurité, les moteurs d'un drone sont désactivés par défaut. "Armer" signifie les préparer à tourner. Sans armement, le drone ne peut pas décoller.

La vérification se fait en lisant le flag `MAV_MODE_FLAG_SAFETY_ARMED` dans le heartbeat :
```python
if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
    return True  # Les moteurs sont armés
```

L'opérateur `&` est un **ET binaire** : il vérifie si le bit "armé" est activé dans le champ `base_mode`.

### `takeoff(conn, lock, altitude)`

```python
conn.mav.command_long_send(
    conn.target_system,
    conn.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,                    # confirmation
    0, 0, 0, 0, 0, 0,   # paramètres non utilisés
    altitude              # altitude cible en mètres
)
```

**Objectif** : Ordonner au drone de décoller jusqu'à l'altitude spécifiée.

Le drone monte automatiquement jusqu'à l'altitude demandée, puis maintient cette altitude (hover). C'est l'autopilote ArduCopter qui gère toute la physique du décollage.

### `get_altitude(conn, lock, timeout=2)`

```python
msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=timeout)
if msg:
    return -msg.z    # L'axe Z est inversé dans NED
```

**C'est quoi NED ?** : NED = North-East-Down. C'est un système de coordonnées où :
- X = Nord (positif vers le nord)
- Y = Est (positif vers l'est)
- **Z = Bas (positif vers le bas !)**

Donc si le drone est à 5 mètres d'altitude, `msg.z` vaut **-5**. On retourne `-msg.z` pour obtenir une altitude **positive**.

### `get_local_position(conn, lock, timeout=2)`

Même principe que `get_altitude()`, mais retourne les 3 coordonnées dans un dictionnaire :

```python
return {'x': msg.x, 'y': msg.y, 'z': -msg.z}
```

### `request_data_streams(conn, lock, rate_hz=4)`

```python
conn.mav.request_data_stream_send(
    conn.target_system, conn.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,  # Type de données : position
    rate_hz,   # Fréquence en Hz (4 = 4 fois par seconde)
    1          # 1 = démarrer le flux, 0 = l'arrêter
)
```

**Objectif** : Demander au drone d'envoyer ses données de position à intervalle régulier.

Par défaut, le drone n'envoie pas forcément sa position. Il faut explicitement lui demander d'activer ce flux de données. Ici on demande 4 messages par seconde.

### `wait_altitude(conn, lock, target_alt, tolerance=1.0, timeout=30)`

```python
while time.time() - t0 < timeout:
    alt = get_altitude(conn, lock, timeout=2)
    if alt is not None and abs(alt - target_alt) < tolerance:
        return True
    time.sleep(0.5)
return False
```

**Objectif** : Attendre que le drone atteigne une altitude cible.

- Lit l'altitude en boucle toutes les 0.5 secondes
- Vérifie si `|altitude_actuelle - altitude_cible| < tolérance`
- Avec `tolerance=1.0`, si la cible est 5m, le drone est considéré "arrivé" entre 4m et 6m
- Si après 30s le drone n'a pas atteint l'altitude, retourne `False`

---

## 10. Fonctions CSV (lecture/écriture)

### `write_positions_csv(positions)`

```python
def write_positions_csv(positions):
    with open(POS_FILE, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['drone_id', 'x', 'y', 'z'])
        for i, pos in enumerate(positions):
            if pos:
                writer.writerow([i, f"{pos['x']:.4f}", f"{pos['y']:.4f}", f"{pos['z']:.4f}"])
            else:
                writer.writerow([i, 0, 0, 0])
```

**Objectif** : Écrire les positions des drones dans un fichier CSV que NS-3 peut lire.

**Format de sortie** (`/tmp/drone_positions.csv`) :
```csv
drone_id,x,y,z
0,0.1234,0.0000,4.0000
1,0.2345,3.0000,5.0000
2,0.0123,6.0000,6.0000
```

NS-3 lit ce fichier périodiquement pour mettre à jour la position de ses nœuds Wi-Fi simulés.

### `read_ns3_output()`

```python
def read_ns3_output():
    results = []
    if not os.path.exists(NS3_OUTPUT):
        return results
    try:
        with open(NS3_OUTPUT, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                results.append(row)
    except Exception:
        pass
    return results
```

**Objectif** : Lire les résultats calculés par NS-3.

- `csv.DictReader` lit chaque ligne comme un dictionnaire (clés = en-têtes)
- Le `try/except` silencieux protège contre les lectures concurrentes (NS-3 peut être en train d'écrire pendant qu'on lit)
- Retourne une liste de dictionnaires, chacun contenant `drone_i`, `drone_j`, `rssi_dbm`, `latency_ms`, `distance_m`

---

## 11. Thread FLIGHT — séquence de vol

```python
def flight_thread(connections, locks, state, args):
```

Ce thread exécute la séquence de vol complète en **5 étapes**.

### Étape 1/5 : Attente calibration EKF

```python
print(f"\n[FLIGHT 1/5] Attente calibration EKF ({args.ekf_wait}s)...")
for s in range(args.ekf_wait, 0, -1):
    if not state.running:
        return
    time.sleep(1)
```

**C'est quoi l'EKF ?** : EKF = Extended Kalman Filter (Filtre de Kalman Étendu). C'est un algorithme mathématique que l'autopilote utilise pour estimer sa position et son orientation en fusionnant les données de plusieurs capteurs (GPS, accéléromètre, gyroscope, baromètre, etc.).

**Pourquoi attendre ?** : Au démarrage, l'EKF n'a pas encore assez de données pour donner une estimation fiable. Si on essaie de décoller trop tôt, le drone ne sait pas où il est et peut avoir un comportement erratique. L'attente (20 secondes par défaut) laisse le temps à l'EKF de converger.

Le `if not state.running: return` vérifie à chaque seconde si l'utilisateur a appuyé sur CTRL+C.

### Étape 2/5 : Passage en mode GUIDED

```python
for i, conn in enumerate(connections):
    success = set_mode(conn, locks[i], MODE_GUIDED)
```

Chaque drone est mis en mode GUIDED pour accepter les commandes automatisées.

### Étape 3/5 : Armement des moteurs

```python
for i, conn in enumerate(connections):
    success = arm_drone(conn, locks[i], timeout=15)
    if not success:
        time.sleep(5)
        success = arm_drone(conn, locks[i], timeout=15)  # Réessai
```

- Tente d'armer chaque drone
- En cas d'échec, attend 5 secondes puis réessaie (parfois l'EKF n'est pas encore prêt)

### Étape 4/5 : Décollage

```python
target_alts = []
for i, conn in enumerate(connections):
    alt = args.altitude + i * args.alt_step
    target_alts.append(alt)
    takeoff(conn, locks[i], alt)
    time.sleep(2)   # 2s entre chaque décollage
```

**Altitudes échelonnées** : Chaque drone décolle à une altitude différente pour éviter les collisions.

| Drone | Altitude (défaut) | Calcul |
|-------|-------------------|--------|
| 0 | 4.0 m | 4.0 + 0 × 1.0 |
| 1 | 5.0 m | 4.0 + 1 × 1.0 |
| 2 | 6.0 m | 4.0 + 2 × 1.0 |

Le `time.sleep(2)` entre chaque décollage évite de surcharger le simulateur.

### Étape 5/5 : Hover (vol stationnaire)

```python
state.drones_airborne.set()  # Signal au bridge : les drones sont en l'air !

for s in range(args.hover):
    time.sleep(1)
    # Affichage des altitudes de chaque drone
    line = f"  [FLIGHT] t={s+1:3d}s |"
    for i, conn in enumerate(connections):
        alt = get_altitude(conn, locks[i], timeout=1)
        line += f" D{i}={alt:5.1f}m"
    print(line)
```

Pendant le hover, le script affiche chaque seconde l'altitude de chaque drone. C'est pendant cette phase que le bridge collecte les métriques réseau les plus intéressantes (drones stables en l'air).

### Atterrissage

```python
for i, conn in enumerate(connections):
    success = set_mode(conn, locks[i], MODE_LAND)
```

Chaque drone passe en mode LAND. L'autopilote gère la descente automatiquement. Le script attend ensuite 30 secondes en affichant les altitudes toutes les 5 secondes pour confirmer que les drones sont bien au sol.

### Signal de fin

```python
state.flight_done.set()  # Signal au bridge : le vol est terminé
```

---

## 12. Thread BRIDGE — métriques réseau

```python
def bridge_thread(connections, locks, state, args, ns3_process):
```

Ce thread tourne **en continu** pendant tout le vol et collecte les métriques réseau.

### Initialisation du log CSV

```python
csv_file = open(UNIFIED_LOG, 'w', newline='')
writer = csv.writer(csv_file)
header = ['timestamp_s']
for i in range(n):
    header.extend([f'd{i}_x', f'd{i}_y', f'd{i}_z'])
for i in range(n):
    for j in range(i+1, n):
        header.extend([
            f'd{i}_d{j}_dist_m',
            f'd{i}_d{j}_rssi_dbm',
            f'd{i}_d{j}_latency_ms'
        ])
writer.writerow(header)
```

**Construction de l'en-tête** pour 3 drones :
```
timestamp_s, d0_x, d0_y, d0_z, d1_x, d1_y, d1_z, d2_x, d2_y, d2_z,
d0_d1_dist_m, d0_d1_rssi_dbm, d0_d1_latency_ms,
d0_d2_dist_m, d0_d2_rssi_dbm, d0_d2_latency_ms,
d1_d2_dist_m, d1_d2_rssi_dbm, d1_d2_latency_ms
```

Les paires sont générées par la double boucle `for i / for j` avec `j > i`, ce qui donne :
- Paire (0,1), Paire (0,2), Paire (1,2) — soit $\frac{n(n-1)}{2}$ paires.

### Boucle principale

```python
while state.running and not state.flight_done.is_set():
    elapsed = time.time() - start_time
```

La boucle tourne tant que :
1. `state.running` est `True` (pas de CTRL+C)
2. `state.flight_done` n'est pas signalé (le vol n'est pas terminé)

#### Lecture des positions

```python
positions = []
for i, conn in enumerate(connections):
    pos = get_local_position(conn, locks[i], timeout=1)
    positions.append(pos)
    with state.lock:
        state.positions[i] = pos
```

Pour chaque drone, on lit sa position via MAVLink et on la stocke dans l'état partagé.

#### Écriture du CSV partagé

```python
write_positions_csv(positions)
```

Écrit les positions dans `/tmp/drone_positions.csv` pour que NS-3 puisse les lire.

#### Récupération des données NS-3

```python
ns3_data = read_ns3_output()
```

Lit `/tmp/ns3_output.csv` qui contient les dernières métriques calculées par NS-3.

#### Calcul des métriques par paire

Pour chaque paire de drones (i, j) :

```python
for d in reversed(ns3_data):
    # Cherche la dernière entrée NS-3 pour cette paire
    di = int(d.get('drone_i', -1))
    dj = int(d.get('drone_j', -1))
    if di == i and dj == j:
        rssi = float(d['rssi_dbm'])
        latency = float(d['latency_ms'])
        dist_ns3 = float(d['distance_m'])
        break
```

- `reversed(ns3_data)` : On parcourt le fichier **à l'envers** pour trouver la **dernière** (la plus récente) entrée pour chaque paire
- Si NS-3 n'a pas encore de données pour cette paire → **fallback** vers le modèle standalone

```python
if rssi is None or latency is None:
    rssi_fb, dist = compute_rssi_standalone(positions[i], positions[j])
    rssi = rssi if rssi is not None else rssi_fb
    latency = latency if latency is not None else compute_latency_standalone(dist)
    src = "fb"    # Fallback
else:
    src = "ns3"   # Données NS-3 réelles
```

Le `src` permet de savoir d'où viennent les métriques dans l'affichage (`ns3` ou `fb`).

#### Affichage et écriture

```python
# Afficher toutes les 5 secondes
if sample_count % max(1, int(5 * args.rate)) == 0:
    print(bridge_line)

# Flush le CSV toutes les 10 lignes
if sample_count % 10 == 0:
    csv_file.flush()

time.sleep(interval)  # interval = 1/rate, ex: 0.5s pour 2 Hz
```

- L'affichage est limité pour ne pas polluer la console
- `csv_file.flush()` force l'écriture sur disque (sinon Python bufferise)
- `time.sleep(interval)` contrôle la fréquence d'échantillonnage

### Nettoyage à la fin

```python
finally:
    csv_file.close()
    if ns3_process:
        ns3_process.terminate()      # Envoie SIGTERM à NS-3
        try:
            ns3_process.wait(timeout=5)  # Attend que NS-3 s'arrête
        except Exception:
            ns3_process.kill()         # Force SIGKILL si NS-3 ne répond pas
```

---

## 13. Programme principal `main()`

### Étape 1 : Parsing des arguments

```python
parser = argparse.ArgumentParser(description='Vol multi-drones + Bridge NS-3 (combiné)')
parser.add_argument('--drones', type=int, default=3, ...)
# ... (voir section 14 pour tous les arguments)
args = parser.parse_args()
```

### Étape 2 : Auto-calcul du temps de simulation NS-3

```python
if args.ns3_sim_time <= 0:
    args.ns3_sim_time = args.ekf_wait + 20 + 15 + args.hover + 35 + 30
```

| Composante | Durée estimée |
|------------|---------------|
| EKF wait | 20s (défaut) |
| Armement | ~20s |
| Décollage | ~15s |
| Hover | Variable (défaut 15s) |
| Atterrissage | ~35s |
| Marge de sécurité | 30s |
| **Total (défaut)** | **135s** |

NS-3 doit simuler assez longtemps pour couvrir tout le vol.

### Étape 3 : Connexion aux drones

```python
connections = []
locks = []
for i in range(n_drones):
    conn = connect_drone(i)
    if conn is None:
        sys.exit(1)
    connections.append(conn)
    locks.append(threading.Lock())
```

- Crée **un lock par connexion** (pas un lock global) pour maximiser le parallélisme
- Si un drone est inaccessible, le script s'arrête immédiatement

### Étape 4 : Lancement de NS-3

```python
# 1. Copier le scénario dans scratch/
shutil.copy2(scenario_src, scenario_dst)

# 2. Build NS-3
subprocess.run([ns3_exe, "build"], cwd=NS3_DIR, ...)

# 3. Lancer NS-3 en arrière-plan
cmd_args = (
    f"scratch/drone-wifi-scenario"
    f" --nDrones={n_drones}"
    f" --posFile={POS_FILE}"
    f" --outFile={NS3_OUTPUT}"
    f" --simTime={args.ns3_sim_time}"
    f" --updateInterval=0.5"
    f" --channelModel={channel_model}"
)
ns3_process = subprocess.Popen(cmd, cwd=NS3_DIR, ...)
```

**Ce que fait cette étape** :
1. **Copie** le fichier C++ du scénario NS-3 dans le dossier `scratch/` de NS-3
2. **Compile** NS-3 (avec le scénario inclus)
3. **Lance** NS-3 comme un processus en arrière-plan avec les bons paramètres

**Les arguments passés à NS-3** :
| Argument | Signification |
|----------|---------------|
| `--nDrones` | Nombre de drones dans la simulation réseau |
| `--posFile` | Fichier CSV d'entrée avec les positions des drones |
| `--outFile` | Fichier CSV de sortie avec les métriques réseau |
| `--simTime` | Durée totale de la simulation réseau |
| `--updateInterval` | Fréquence de relecture des positions (0.5s) |
| `--channelModel` | Modèle de canal : `log-distance` ou `sionna` |

### Étape 5 : Gestion du CTRL+C

```python
def signal_handler(sig, frame):
    print("\n\n*** CTRL+C détecté — arrêt en cours... ***\n")
    state.running = False
    state.flight_done.set()

signal.signal(signal.SIGINT, signal_handler)
```

Quand l'utilisateur appuie sur CTRL+C :
1. `state.running = False` → Les deux threads vérifient cette variable et s'arrêtent
2. `state.flight_done.set()` → Débloque le bridge s'il attend

### Étape 6 : Lancement des threads

```python
t_flight = threading.Thread(target=flight_thread, args=(...), daemon=True)
t_bridge = threading.Thread(target=bridge_thread, args=(...), daemon=True)

t_bridge.start()
t_flight.start()

t_flight.join()           # Attend la fin du vol
state.flight_done.set()   # Au cas où
t_bridge.join(timeout=5)  # Attend le bridge max 5s
```

**`daemon=True`** : Les threads démons sont automatiquement tués quand le programme principal se termine. C'est une sécurité pour éviter des threads "zombie".

**Ordre de démarrage** :
1. Le bridge démarre en premier pour être prêt à enregistrer dès le début
2. Le flight démarre ensuite et exécute la séquence de vol
3. `t_flight.join()` bloque le main thread jusqu'à la fin du vol
4. Ensuite on signale au bridge de s'arrêter et on attend max 5 secondes

### Étape 7 : Nettoyage

```python
state.running = False

for conn in connections:
    try:
        conn.close()
    except Exception:
        pass
```

Ferme proprement toutes les connexions MAVLink.

---

## 14. Arguments en ligne de commande

| Argument | Type | Défaut | Description |
|----------|------|--------|-------------|
| `--drones` | int | 3 | Nombre de drones à contrôler |
| `--altitude` | float | 4.0 | Altitude de base en mètres (drone 0) |
| `--alt-step` | float | 1.0 | Écart d'altitude entre chaque drone (en mètres) |
| `--hover` | int | 15 | Durée du vol stationnaire en secondes |
| `--ekf-wait` | int | 20 | Temps d'attente de calibration EKF en secondes |
| `--rate` | float | 2.0 | Fréquence de collecte des métriques en Hz |
| `--ns3-sim-time` | int | 0 (auto) | Durée simulation NS-3 (0 = calculé automatiquement) |
| `--use-sionna` | flag | false | Active le ray-tracing Sionna au lieu du log-distance |
| `--sionna-env` | str | `simple_room/simple_room.xml` | Fichier de scène 3D Sionna |
| `--sionna-url` | str | `tcp://localhost:5555` | Adresse du serveur Sionna |

---

## 15. Flux d'exécution complet

Voici l'ordre chronologique complet de ce qui se passe quand on lance le script :

```
t=0s    main() démarre
        │
        ├── Parse des arguments
        ├── Connexion aux 3 drones (ports 5760, 5770, 5780)
        ├── Configuration des flux de données MAVLink
        ├── Copie du scénario NS-3
        ├── Build NS-3 (compilation C++)
        ├── Lancement de NS-3 en arrière-plan
        ├── Création de SharedState
        ├── Installation du handler CTRL+C
        │
        ├── Démarrage thread BRIDGE ──────────────────►  [BRIDGE boucle à 2 Hz]
        │                                                 │ Lit positions
        ├── Démarrage thread FLIGHT                       │ Écrit CSV
        │   │                                             │ Lit NS-3
        │   ├── [1/5] Attente EKF (20s)                  │ Écrit log
        │   │                                             │ ...
        │   ├── [2/5] Mode GUIDED (3 drones)              │
        │   │                                             │
        │   ├── [3/5] Armement moteurs                    │
        │   │                                             │
        │   ├── [4/5] Décollage                           │
        │   │   ├── Drone 0 → 4m                         │
        │   │   ├── Drone 1 → 5m                         │
        │   │   └── Drone 2 → 6m                         │
        │   │                                             │
        │   ├── state.drones_airborne.set() ──────────►   │ (signal reçu)
        │   │                                             │
        │   ├── [5/5] Hover (15s)                         │
        │   │   ├── t=1s  D0=4.0m D1=5.0m D2=6.0m       │
        │   │   ├── t=2s  ...                             │
        │   │   └── t=15s ...                             │
        │   │                                             │
        │   ├── Atterrissage (30s)                        │
        │   │                                             │
        │   └── state.flight_done.set() ──────────────►   │ (arrêt boucle)
        │                                                 │
        ├── t_flight.join() ◄──── flight terminé          │
        ├── t_bridge.join(5s) ◄──────────────── bridge terminé
        │
        ├── Fermeture connexions MAVLink
        └── FIN
```

---

## 16. Schéma de communication entre threads

```
┌─────────────┐     connexions      ┌─────────────┐
│             │     partagées       │             │
│   FLIGHT    │◄───────────────────►│   BRIDGE    │
│   Thread    │                     │   Thread    │
│             │                     │             │
│ Contrôle    │  SharedState        │ Collecte    │
│ le vol      │ ┌──────────────┐   │ métriques   │
│             │ │ .running     │   │             │
│ Écrit:      │ │ .positions[] │   │ Écrit:      │
│ - modes     │ │ .flight_done │   │ - positions │
│ - arm       │ │ .drones_airb.│   │ - comm_metr │
│ - takeoff   │ └──────────────┘   │             │
│ - land      │                     │ Lit:        │
│             │                     │ - NS-3 out  │
└─────────────┘                     └──────────────┘
       │                                   │
       │        ┌──────────┐               │
       │        │  NS-3    │               │
       │        │ Process  │◄──────────────┘
       │        │          │  (drone_positions.csv)
       │        │          │───────────────►│
       │        └──────────┘  (ns3_output.csv)
       │
       ▼
  ┌──────────┐
  │  Drones  │
  │ (MAVLink)│
  └──────────┘
```

### Locks : qui protège quoi ?

| Lock | Protège | Utilisé par |
|------|---------|-------------|
| `locks[0]` | Connexion MAVLink du drone 0 | FLIGHT + BRIDGE |
| `locks[1]` | Connexion MAVLink du drone 1 | FLIGHT + BRIDGE |
| `locks[2]` | Connexion MAVLink du drone 2 | FLIGHT + BRIDGE |
| `state.lock` | `state.positions[]` | BRIDGE (écriture) |

Chaque drone a **son propre lock**. Cela signifie que le thread FLIGHT peut envoyer une commande au drone 0 pendant que le thread BRIDGE lit la position du drone 1, **sans conflit**. C'est plus performant qu'un lock global unique.

---

## 17. Format des fichiers de sortie

### `comm_metrics.csv` — Log unifié (SORTIE PRINCIPALE)

C'est le fichier le plus important, celui que vous analyserez après la simulation.

**Exemple pour 3 drones** :

```csv
timestamp_s,d0_x,d0_y,d0_z,d1_x,d1_y,d1_z,d2_x,d2_y,d2_z,d0_d1_dist_m,d0_d1_rssi_dbm,d0_d1_latency_ms,d0_d2_dist_m,d0_d2_rssi_dbm,d0_d2_latency_ms,d1_d2_dist_m,d1_d2_rssi_dbm,d1_d2_latency_ms
0.500,0.100,0.000,4.000,0.200,3.000,5.000,0.000,6.000,6.000,3.162,-54.5,2.000,6.325,-64.0,2.000,3.162,-54.5,2.000
1.000,0.105,0.001,4.010,0.198,3.002,5.020,0.003,5.998,6.010,...
```

**Colonnes** :

| Colonne | Description | Unité |
|---------|-------------|-------|
| `timestamp_s` | Temps écoulé depuis le début | secondes |
| `d{i}_x` | Position X du drone i | mètres |
| `d{i}_y` | Position Y du drone i | mètres |
| `d{i}_z` | Position Z (altitude) du drone i | mètres |
| `d{i}_d{j}_dist_m` | Distance entre drone i et drone j | mètres |
| `d{i}_d{j}_rssi_dbm` | Signal reçu entre i et j | dBm |
| `d{i}_d{j}_latency_ms` | Latence de communication entre i et j | millisecondes |

### `/tmp/drone_positions.csv` — Positions partagées (intermédiaire)

```csv
drone_id,x,y,z
0,0.1000,0.0000,4.0000
1,0.2000,3.0000,5.0000
2,0.0000,6.0000,6.0000
```

Ce fichier est **réécrit** à chaque cycle du bridge (2 fois par seconde par défaut). NS-3 le lit pour mettre à jour ses nœuds.

### `/tmp/ns3_output.csv` — Sortie NS-3 (intermédiaire)

```csv
time_s,drone_i,drone_j,distance_m,rssi_dbm,latency_ms
0.5,0,1,3.162,-54.5,2.011
0.5,0,2,6.325,-64.0,2.021
0.5,1,2,3.162,-54.5,2.011
```

Ce fichier est **écrit par NS-3** et lu par le bridge.

---

## 18. Exemples d'utilisation

### Usage de base — 3 drones, paramètres par défaut

```bash
python3 12_flight_and_bridge.py --drones 3
```

Résultat : 3 drones, altitudes 4/5/6m, hover 15s, modèle log-distance.

### Vol long avec plus de drones

```bash
python3 12_flight_and_bridge.py --drones 5 --altitude 3 --alt-step 2 --hover 60
```

Résultat :
- 5 drones
- Altitudes : 3m, 5m, 7m, 9m, 11m
- Hover : 60 secondes
- Modèle : log-distance

### Avec ray-tracing Sionna

```bash
python3 12_flight_and_bridge.py --drones 3 --use-sionna --hover 30
```

Nécessite un serveur Sionna en cours d'exécution. Utilise le ray-tracing pour un modèle de canal plus réaliste.

### Collecte haute fréquence

```bash
python3 12_flight_and_bridge.py --drones 3 --rate 10 --hover 30
```

Collecte les métriques 10 fois par seconde (au lieu de 2). Fichier CSV plus gros mais données plus fines.

### EKF rapide (pour tests)

```bash
python3 12_flight_and_bridge.py --drones 3 --ekf-wait 5 --hover 10
```

Réduit l'attente EKF à 5s et le hover à 10s. Utile pour tester rapidement.

---

## 19. Dépannage et erreurs courantes

### Erreur : "pymavlink non installé"

```
ERREUR: pymavlink non installé.
```

**Solution** :
```bash
pip3 install pymavlink
```

### Erreur : "Drone X non accessible"

```
ERREUR: Drone 0 non accessible.
Vérifie que 06_launch_multi_drones.sh tourne.
```

**Causes possibles** :
1. Le script `06_launch_multi_drones.sh` n'est pas lancé
2. Le drone n'a pas encore fini de démarrer (attendre 30s après le lancement)
3. Le port est déjà utilisé par un autre programme

**Solution** :
```bash
# Vérifier que les ports sont ouverts
ss -tlnp | grep 576

# Relancer les drones
./06_launch_multi_drones.sh
```

### Erreur : "NS-3 non trouvé"

```
ERREUR: NS-3 non trouvé dans ~/ns-allinone-3.40/ns-3.40
```

**Solution** : Installer NS-3 avec les scripts 09 et 10 :
```bash
./09_install_ns3.sh
./10_install_ns3sionna.sh  # optionnel, pour Sionna
```

### Erreur : "NS-3 s'est arrêté immédiatement"

NS-3 a crashé au démarrage. Vérifier :
```bash
# Tester NS-3 manuellement
cd ~/ns-allinone-3.40/ns-3.40
./ns3 run "scratch/drone-wifi-scenario --nDrones=3 --simTime=10"
```

### Les métriques affichent "(fb)" au lieu de "(ns3)"

Le bridge n'arrive pas à lire les données NS-3. Causes :
- NS-3 n'a pas encore produit de résultat (attendre quelques secondes)
- Le fichier `/tmp/ns3_output.csv` n'existe pas ou est vide
- NS-3 a crashé silencieusement

### CTRL+C ne fonctionne pas proprement

Attendre quelques secondes. Le script doit d'abord finir les opérations en cours, fermer les fichiers, et tuer NS-3. Si ça ne marche pas :
```bash
# Forcer l'arrêt
kill -9 $(pgrep -f 12_flight_and_bridge)
kill -9 $(pgrep -f drone-wifi-scenario)
```

---

## 20. Glossaire

| Terme | Définition |
|-------|------------|
| **MAVLink** | Protocole de communication léger pour les drones (Micro Air Vehicle Link) |
| **ArduCopter** | Autopilote open-source pour les multi-rotors (partie du projet ArduPilot) |
| **PX4 SITL** | Software In The Loop — simulation logicielle de l'autopilote PX4 |
| **Gazebo** | Simulateur physique 3D open-source pour la robotique |
| **NS-3** | Network Simulator 3 — simulateur réseau discret open-source |
| **Sionna** | Framework de simulation de canal radio basé sur le ray-tracing (par NVIDIA) |
| **EKF** | Extended Kalman Filter — algorithme de fusion de capteurs pour l'estimation d'état |
| **RSSI** | Received Signal Strength Indicator — puissance du signal reçu (en dBm) |
| **dBm** | Décibels-milliwatts — unité logarithmique de puissance |
| **Latence** | Délai entre l'envoi et la réception d'un message |
| **GUIDED** | Mode de vol où le drone accepte des commandes envoyées par programme |
| **Hover** | Vol stationnaire — le drone reste immobile en l'air |
| **Thread** | Fil d'exécution — permet d'exécuter du code en parallèle |
| **Lock / Mutex** | Mécanisme de synchronisation pour protéger les données partagées |
| **Event** | Signal qu'un thread peut envoyer à d'autres threads |
| **Daemon** | Thread qui est automatiquement tué quand le programme principal se termine |
| **NED** | North-East-Down — système de coordonnées (Z positif vers le bas) |
| **MAC** | Medium Access Control — sous-couche du protocole réseau Wi-Fi |
| **Heartbeat** | Message périodique envoyé par le drone pour signaler qu'il est actif |
| **Path Loss** | Perte de puissance du signal radio avec la distance |
| **Fallback** | Solution de secours utilisée quand la méthode principale échoue |
| **CSV** | Comma-Separated Values — format de fichier texte tabulaire |
| **Ray-tracing** | Technique de simulation qui trace le trajet physique des ondes radio |
| **GCS** | Ground Control Station — station de contrôle au sol |

---

## Résumé visuel final

```
┌──────────────────────────────────────────────────────────────────┐
│                                                                  │
│  python3 12_flight_and_bridge.py --drones 3 --hover 30           │
│                                                                  │
│  ┌──────────┐   MAVLink    ┌──────────────────────────────────┐ │
│  │ Drone 0  │◄────────────►│                                  │ │
│  │ port 5760│              │                                  │ │
│  └──────────┘              │     12_flight_and_bridge.py      │ │
│  ┌──────────┐              │                                  │ │
│  │ Drone 1  │◄────────────►│  ┌─────────┐  ┌──────────────┐  │ │
│  │ port 5770│              │  │ FLIGHT  │  │   BRIDGE     │  │ │
│  └──────────┘              │  │ thread  │  │   thread     │  │ │
│  ┌──────────┐              │  │         │  │              │  │ │
│  │ Drone 2  │◄────────────►│  │ arm     │  │ positions    │  │ │
│  │ port 5780│              │  │ takeoff │  │ RSSI/latency │  │ │
│  └──────────┘              │  │ hover   │  │ CSV logging  │  │ │
│                            │  │ land    │  │              │  │ │
│                            │  └────┬────┘  └──────┬───────┘  │ │
│                            │       │  SharedState  │          │ │
│                            │       └───────┬───────┘          │ │
│                            └───────────────┼──────────────────┘ │
│                                            │                    │
│                            ┌───────────────▼──────────────────┐ │
│                            │            NS-3                  │ │
│                            │    Simulation réseau Wi-Fi       │ │
│                            └──────────────────────────────────┘ │
│                                            │                    │
│                            ┌───────────────▼──────────────────┐ │
│                            │       comm_metrics.csv           │ │
│                            │    (RÉSULTAT FINAL)              │ │
│                            └──────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────┘
```

---

*Documentation générée pour `12_flight_and_bridge.py` — Simulation multi-drones avec bridge NS-3*
