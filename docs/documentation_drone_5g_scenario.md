# Documentation Complète — `drone-5g-scenario.cc`

## Scénario NS-3 + 5G-LENA : Réseau 5G NR entre Drones

---

## Table des Matières

1. [Vue d'ensemble](#1-vue-densemble)
2. [Architecture 5G NR vs Wi-Fi Ad-Hoc](#2-architecture-5g-nr-vs-wi-fi-ad-hoc)
3. [Prérequis et installation](#3-prérequis-et-installation)
4. [Les includes et modules](#4-les-includes-et-modules)
5. [Variables globales](#5-variables-globales)
6. [Configuration du spectre 5G NR](#6-configuration-du-spectre-5g-nr)
7. [Modèle de canal 3GPP TR 38.901](#7-modèle-de-canal-3gpp-tr-38901)
8. [Configuration de la gNB et des UEs](#8-configuration-de-la-gnb-et-des-ues)
9. [EPC (Evolved Packet Core)](#9-epc-evolved-packet-core)
10. [Beamforming et antennes](#10-beamforming-et-antennes)
11. [Calcul du RSRP](#11-calcul-du-rsrp)
12. [Mesure de latence via FlowMonitor](#12-mesure-de-latence-via-flowmonitor)
13. [Mode temps réel](#13-mode-temps-réel)
14. [Format des fichiers d'entrée/sortie](#14-format-des-fichiers-dentréesortie)
15. [Arguments en ligne de commande](#15-arguments-en-ligne-de-commande)
16. [Scripts Python associés](#16-scripts-python-associés)
17. [Comparaison WiFi vs 5G NR](#17-comparaison-wifi-vs-5g-nr)
18. [Exemples d'utilisation](#18-exemples-dutilisation)
19. [Intégration Sionna RT (futur)](#19-intégration-sionna-rt-futur)
20. [Dépannage et erreurs courantes](#20-dépannage-et-erreurs-courantes)
21. [Glossaire 5G](#21-glossaire-5g)

---

## 1. Vue d'ensemble

### Qu'est-ce que ce programme fait ?

Ce programme C++ est un **scénario NS-3** utilisant le module **5G-LENA** pour simuler un réseau 5G NR (New Radio) entre des drones et une station de base (gNB). Il fait le pont entre le monde physique (positions des drones dans Gazebo) et le monde réseau (communications 5G simulées).

### En une phrase

> Il lit les positions des drones, simule un réseau 5G NR avec une gNB au centre de l'entrepôt, mesure le RSRP et la latence, et écrit les résultats dans un fichier CSV.

### Qu'est-ce que 5G-LENA ?

**5G-LENA** est le module NR (New Radio) pour NS-3, développé par le CTTC (Centre Tecnològic de Telecomunicacions de Catalunya). Il implémente le standard 3GPP pour les réseaux 5G, incluant :

- **PHY** : OFDMA, numerologie configurable, beamforming
- **MAC** : Scheduler TDMA/OFDMA (Round Robin, Proportional Fair)
- **RLC/PDCP** : Couches de liaison de données
- **RRC** : Gestion de la connexion radio
- **EPC** : Réseau cœur (SGW, PGW)

Site officiel : https://5g-lena.cttc.es/
Blog référence : https://5g-lena.cttc.es/blog/33/

---

## 2. Architecture 5G NR vs Wi-Fi Ad-Hoc

### Architecture Wi-Fi (script 12)

```
  Drone 0 ←─── Wi-Fi Ad-Hoc ───→ Drone 1
    ↕                                 ↕
    └─── Wi-Fi Ad-Hoc ──── Drone 2 ──┘

  Chaque drone communique directement avec tous les autres.
  RSSI = force du signal entre deux drones.
```

### Architecture 5G NR (script 15)

```
                    ┌─────────────┐
                    │   EPC       │
                    │ (SGW + PGW) │
                    └──────┬──────┘
                           │
                    ┌──────┴──────┐
                    │    gNB      │
                    │ (Base Station)│
                    │  Centre du   │
                    │  warehouse   │
                    └──┬───┬───┬──┘
                      ╱    │    ╲
                    ╱      │      ╲
              ┌────┐  ┌────┐  ┌────┐
              │ UE0│  │ UE1│  │ UE2│
              │Drone0│ │Drone1│ │Drone2│
              └────┘  └────┘  └────┘

  Tous les drones se connectent à la gNB.
  Communication inter-drones : UE → gNB → EPC → gNB → UE
  RSRP = puissance du signal reçu de la gNB.
```

### Tableau comparatif

| Aspect | Wi-Fi Ad-Hoc | 5G NR |
|--------|-------------|-------|
| **Topologie** | Maillée (mesh) | Étoile (via gNB) |
| **Standard** | IEEE 802.11n | 3GPP NR (Rel-15+) |
| **Fréquence** | 2.4 GHz | 3.5 GHz (FR1) ou 28 GHz (FR2) |
| **Bande passante** | 20/40 MHz | 10-400 MHz |
| **Modulation** | OFDM | OFDMA avec numerologie |
| **Métrique signal** | RSSI (dBm) | RSRP (dBm) |
| **Latence typique** | ~2-5 ms | ~1-2 ms (FR1) |
| **Portée indoor** | ~30-50 m | ~100-200 m (FR1) |
| **Canal** | Log-Distance | 3GPP TR 38.901 InH |
| **Infrastructure** | Aucune | gNB + EPC |

---

## 3. Prérequis et installation

### Étape 1 : NS-3.40

```bash
# Déjà installé via script 09
cd ~/ns-allinone-3.40/ns-3.40
./ns3 run hello-simulator
```

### Étape 2 : 5G-LENA (module NR)

```bash
# Script d'installation automatique
cd ~/simulation_mc02/scripts
chmod +x 13_install_5g_lena.sh
./13_install_5g_lena.sh
```

Ce script :
1. Installe les prérequis (libeigen3-dev, sqlite3)
2. Clone `5g-lena-v2.6.y` dans `contrib/nr/`
3. Recompile NS-3 avec le module NR
4. Teste avec `cttc-nr-demo`

### Étape 3 : Compiler le scénario drone 5G

```bash
cp ~/simulation_mc02/ns3_scenarios/drone-5g-scenario.cc \
   ~/ns-allinone-3.40/ns-3.40/scratch/
cd ~/ns-allinone-3.40/ns-3.40
./ns3 build
```

### Version compatible

| Module | Version | Compatibilité |
|--------|---------|---------------|
| NS-3 | 3.40 | Base requise |
| 5G-LENA (NR) | v2.6.y | Compatible NS-3.40 |
| NS3-Sionna | - | Optionnel (WiFi) |

---

## 4. Les includes et modules

```cpp
// Modules NS-3 standard (comme le scénario WiFi)
#include "ns3/core-module.h"           // Simulation engine
#include "ns3/network-module.h"        // Nodes, NetDevices
#include "ns3/mobility-module.h"       // Positions
#include "ns3/internet-module.h"       // TCP/IP stack
#include "ns3/applications-module.h"   // UDP Echo
#include "ns3/flow-monitor-module.h"   // Latence réelle
#include "ns3/propagation-module.h"    // 3GPP propagation

// Module 5G-LENA (NR)
#include "ns3/nr-module.h"             // ← NOUVEAU : tout le stack 5G NR
```

Le `nr-module.h` inclut automatiquement :
- `NrHelper` : configuration du réseau NR
- `NrPointToPointEpcHelper` : réseau cœur
- `IdealBeamformingHelper` : beamforming
- `CcBwpCreator` : création des bandes de fréquence
- `NrMacSchedulerTdmaRR` : scheduler TDMA Round Robin
- `DirectPathBeamforming` : algorithme de beamforming

---

## 5. Variables globales

```cpp
// === Paramètres 5G NR ===
static double      g_frequency     = 3.5e9;    // FR1 : 3.5 GHz
static double      g_bandwidth     = 20e6;     // 20 MHz
static uint32_t    g_numerology    = 1;         // μ=1 → 30 kHz SCS
static std::string g_scenario      = "InH-OfficeMixed";

// === Position gNB (centre de l'entrepôt) ===
static double g_gnbX = 15.0;  // 30m / 2
static double g_gnbY = 10.0;  // 20m / 2
static double g_gnbZ = 3.0;   // Surélevée (plafond 6m)

// === Puissances ===
static double g_gnbTxPowerDbm = 30.0;  // gNB indoor small cell
static double g_ueTxPowerDbm  = 23.0;  // UE (drone)
```

### Différences avec le scénario WiFi

| Variable WiFi | Variable 5G | Explication |
|--------------|-------------|-------------|
| `g_txPowerDbm = 20` | `g_gnbTxPowerDbm = 30` | gNB plus puissante |
| - | `g_ueTxPowerDbm = 23` | Puissance UE distincte |
| `g_channelModel` | `g_scenario` | Scénario 3GPP |
| - | `g_frequency` | Fréquence configurable |
| - | `g_numerology` | Numérologie NR |
| `g_droneNodes` | `g_gnbNodes + g_ueNodes` | Séparation gNB/UE |

---

## 6. Configuration du spectre 5G NR

### Numerologie NR

La 5G NR utilise une **numerologie** μ qui détermine l'espacement des sous-porteuses (SCS) :

| μ | SCS (kHz) | Slot (ms) | Usage typique |
|---|-----------|-----------|---------------|
| 0 | 15 | 1.000 | LTE compatibilité |
| 1 | 30 | 0.500 | **FR1 (sub-6 GHz)** ← défaut |
| 2 | 60 | 0.250 | FR1/FR2 transition |
| 3 | 120 | 0.125 | **FR2 (mmWave)** |
| 4 | 240 | 0.0625 | FR2 signalisation |

### Bandes de fréquence

| Bande | Plage | Config recommandée |
|-------|-------|--------------------|
| **FR1** | 410 MHz – 7.125 GHz | `--frequency=3.5e9 --numerology=1 --bandwidth=20e6` |
| **FR2** | 24.25 GHz – 52.6 GHz | `--frequency=28e9 --numerology=3 --bandwidth=100e6` |

### Configuration dans le code

```cpp
// Créer la bande d'opération
CcBwpCreator ccBwpCreator;
CcBwpCreator::SimpleOperationBandConf bandConf(
    g_frequency,     // Fréquence centrale (Hz)
    g_bandwidth,     // Bande passante (Hz)
    1,               // 1 Component Carrier
    bwpScenario      // Scénario 3GPP (détermine le channel model)
);
OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
nrHelper->InitializeOperationBand(&band);
```

---

## 7. Modèle de canal 3GPP TR 38.901

### Scénarios disponibles

| Scénario | Environnement | Usage |
|----------|--------------|-------|
| **InH-OfficeMixed** | Bureau indoor mixte (LOS+NLOS) | **Entrepôt** ← défaut |
| **InH-OfficeOpen** | Bureau indoor ouvert (principalement LOS) | Entrepôt ouvert |
| **UMi** | Urban Micro (outdoor) | Zone urbaine |
| **UMa** | Urban Macro (outdoor) | Grande couverture |
| **RMa** | Rural Macro | Zone rurale |

### Formule InH-Office LOS (simplifiée)

```
PL_InH_LOS(d) = 32.4 + 17.3 × log₁₀(d₃D[m]) + 20 × log₁₀(fc[GHz])
```

Pour `fc = 3.5 GHz` et `d = 10m` :
```
PL = 32.4 + 17.3 × log₁₀(10) + 20 × log₁₀(3.5)
   = 32.4 + 17.3 + 10.88
   ≈ 60.6 dB
RSRP = 30 - 60.6 = -30.6 dBm
```

### Différence avec Log-Distance (WiFi)

| Aspect | Log-Distance (WiFi) | 3GPP InH (5G) |
|--------|---------------------|----------------|
| Exposant | n=3.0 (fixe) | 17.3×log₁₀(d) (variable) |
| Perte ref. | 40 dB à 1m | 32.4 + 20×log₁₀(fc) |
| Fréquence | 2.4 GHz fixe | Configurable |
| LOS/NLOS | Non modélisé | Modèle probabiliste |
| Shadowing | Non | Optionnel (σ = 3-8 dB) |

---

## 8. Configuration de la gNB et des UEs

### gNB (station de base)

```cpp
// Position au centre de l'entrepôt (30×20×6m)
g_gnbNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(
    Vector(15.0, 10.0, 3.0));  // Centre, surélevée à 3m

// Puissance : 30 dBm (indoor small cell)
nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(30.0));

// Antenne : 4×2 UPA = 8 éléments
nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(2));
```

### UEs (drones)

```cpp
// Positions lues depuis le CSV (mise à jour dynamique)
// Puissance : 23 dBm (typique pour un UE mobile)
nrHelper->SetUePhyAttribute("TxPower", DoubleValue(23.0));

// Antenne : 1×1 (omnidirectionnelle sur le drone)
nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));
```

---

## 9. EPC (Evolved Packet Core)

L'EPC fournit le réseau cœur pour la communication inter-drones :

```
UE (Drone) → gNB → S1-U → SGW → S5 → PGW → Internet/Autres UEs
```

### Configuration dans le code

```cpp
// Créer l'EPC
Ptr<NrPointToPointEpcHelper> epcHelper =
    CreateObject<NrPointToPointEpcHelper>();
nrHelper->SetEpcHelper(epcHelper);

// Assigner les IPs aux UEs via l'EPC
Ipv4InterfaceContainer ueIpIface =
    epcHelper->AssignUeIpv4Address(g_ueDevices);

// Route par défaut → passerelle EPC
for (uint32_t i = 0; i < nDrones; ++i) {
    Ptr<Ipv4StaticRouting> ueRouting =
        ipv4RoutingHelper.GetStaticRouting(
            g_ueNodes.Get(i)->GetObject<Ipv4>());
    ueRouting->SetDefaultRoute(
        epcHelper->GetUeDefaultGatewayAddress(), 1);
}

// Attacher les UEs à la gNB
nrHelper->AttachToClosestGnb(g_ueDevices, g_gnbDevices);
```

---

## 10. Beamforming et antennes

### Beamforming idéal

```cpp
Ptr<IdealBeamformingHelper> idealBf =
    CreateObject<IdealBeamformingHelper>();
idealBf->SetAttribute("BeamformingMethod",
    TypeIdValue(DirectPathBeamforming::GetTypeId()));
nrHelper->SetBeamformingHelper(idealBf);
```

- **DirectPathBeamforming** : pointe le faisceau vers la direction LOS (optimal pour indoor avec LOS)
- **CellScanBeamforming** : scanne toutes les directions et choisit la meilleure (plus réaliste mais plus lent)

### Antennes UPA (Uniform Planar Array)

```
gNB : 4×2 = 8 éléments      Drone : 1×1 = omnidirectionnel
┌──┬──┐                      ┌──┐
│░░│░░│                      │░░│
├──┼──┤                      └──┘
│░░│░░│
├──┼──┤
│░░│░░│
├──┼──┤
│░░│░░│
└──┴──┘
```

---

## 11. Calcul du RSRP

### Méthode

Le RSRP (Reference Signal Received Power) est calculé via le modèle de propagation 3GPP :

```cpp
// Modèle 3GPP séparé pour CalcRxPower()
Ptr<ThreeGppIndoorOfficePropagationLossModel> propLoss = ...;
propLoss->SetAttribute("Frequency", DoubleValue(3.5e9));
propLoss->SetChannelConditionModel(condModel);

// RSRP = TxPower_gNB - PathLoss(gNB → UE)
double rsrp = propLoss->CalcRxPower(30.0, gnbMob, ueMob);
```

### RSRP vs RSSI

| Métrique | Définition | Usage |
|----------|-----------|-------|
| **RSSI** (WiFi) | Puissance totale reçue | Qualité du signal WiFi |
| **RSRP** (5G) | Puissance du signal de référence | Qualité du lien 5G |
| **SINR** (5G) | Signal / (Interférence + Bruit) | Performance réelle |

### RSRP par paire de drones

En 5G, la communication inter-drones passe par la gNB. Le RSRP effectif pour une paire (i, j) est le **minimum** des deux liens :

```
RSRP_pair(i, j) = min(RSRP(i → gNB), RSRP(j → gNB))
```

Le lien le plus faible est le "bottleneck" de la communication.

---

## 12. Mesure de latence via FlowMonitor

Identique au scénario WiFi, mais la latence 5G inclut :

1. **Scheduling** : attente du slot OFDMA (~0.5ms pour μ=1)
2. **Transmission** : durée du slot (0.5ms pour μ=1)
3. **Propagation** : d/c (~0.03μs pour 10m)
4. **Processing** : décodage + HARQ (~0.5ms)
5. **EPC** : routage dans le réseau cœur (~0.1ms)

**Latence totale typique (FR1, μ=1) : ~1-3 ms**

Comparaison avec WiFi :
- WiFi Ad-Hoc : ~2-5 ms (contention MAC CSMA/CA)
- 5G NR FR1 : ~1-3 ms (scheduling OFDMA déterministe)
- 5G NR FR2 : ~0.5-1 ms (slots plus courts)

---

## 13. Mode temps réel

Identique au scénario WiFi :

```cpp
GlobalValue::Bind("SimulatorImplementationType",
                  StringValue("ns3::RealtimeSimulatorImpl"));
```

Le simulateur NS-3 avance au même rythme que le temps réel, permettant la synchronisation avec Gazebo et ArduPilot SITL.

---

## 14. Format des fichiers d'entrée/sortie

### Entrée : `/tmp/drone_positions.csv`

```csv
drone_id,x,y,z
0,1.2345,0.5678,4.0000
1,4.5678,3.0000,5.0000
2,7.8901,6.0000,6.0000
```

(Même format que le scénario WiFi)

### Sortie : `/tmp/ns3_5g_output.csv`

```csv
time_s,drone_i,drone_j,distance_m,rsrp_dbm,latency_ms,dist_gnb_i,dist_gnb_j,rsrp_i,rsrp_j,xi,yi,zi,xj,yj,zj
2.0,0,1,3.142,-25.3,1.52,14.2,12.8,-22.1,-25.3,1.23,0.57,4.00,4.57,3.00,5.00
```

### Colonnes de sortie

| Colonne | Type | Description |
|---------|------|-------------|
| `time_s` | float | Temps de simulation (s) |
| `drone_i` | int | ID du premier drone |
| `drone_j` | int | ID du second drone |
| `distance_m` | float | Distance entre les deux drones (m) |
| `rsrp_dbm` | float | RSRP du paire = min(RSRP_i, RSRP_j) |
| `latency_ms` | float | Latence bout-en-bout (ms) |
| `dist_gnb_i` | float | Distance drone_i → gNB (m) |
| `dist_gnb_j` | float | Distance drone_j → gNB (m) |
| `rsrp_i` | float | RSRP du drone_i vers gNB (dBm) |
| `rsrp_j` | float | RSRP du drone_j vers gNB (dBm) |
| `xi,yi,zi` | float | Position du drone_i (m) |
| `xj,yj,zj` | float | Position du drone_j (m) |

### Log unifié : `~/simulation_mc02/comm_metrics_5g.csv`

Colonnes supplémentaires par rapport au WiFi :
- `d{i}_dist_gnb_m` : distance de chaque drone à la gNB
- `d{i}_rsrp_dbm` : RSRP de chaque drone
- `d{i}_d{j}_rsrp_pair_dbm` : RSRP du lien inter-drones (via gNB)

---

## 15. Arguments en ligne de commande

```bash
./ns3 run "scratch/drone-5g-scenario
  --nDrones=3              # Nombre de drones (UEs)
  --posFile=/tmp/pos.csv   # Fichier positions
  --outFile=/tmp/out.csv   # Fichier sortie
  --simTime=60             # Durée simulation (s)
  --updateInterval=0.5     # Mise à jour positions (s)
  --frequency=3.5e9        # Fréquence (Hz)
  --bandwidth=20e6         # Bande passante (Hz)
  --numerology=1           # Numérologie μ (0-4)
  --scenario=InH-OfficeMixed  # Scénario 3GPP
  --gnbX=15.0              # Position X gNB (m)
  --gnbY=10.0              # Position Y gNB (m)
  --gnbZ=3.0               # Position Z gNB (m)
  --gnbTxPower=30          # Puissance gNB (dBm)
  --ueTxPower=23           # Puissance UE (dBm)"
```

---

## 16. Scripts Python associés

### Vue d'ensemble

| Script | Rôle | Équivalent WiFi |
|--------|------|-----------------|
| `13_install_5g_lena.sh` | Installation 5G-LENA | `10_install_ns3sionna.sh` |
| `14_ns3_5g_bridge.py` | Bridge standalone 5G | `11_ns3_bridge.py` |
| `15_flight_and_5g_bridge.py` | Vol + Bridge 5G | `12_flight_and_bridge.py` |

### Usage du bridge 5G (script 14)

```bash
# Mode NS-3 (avec 5G-LENA)
python3 14_ns3_5g_bridge.py --drones 3

# Mode standalone (sans NS-3)
python3 14_ns3_5g_bridge.py --drones 3 --no-ns3

# FR2 mmWave
python3 14_ns3_5g_bridge.py --drones 3 --frequency 28e9 --numerology 3 --bandwidth 100e6
```

### Usage du vol + bridge 5G (script 15)

```bash
# Configuration par défaut (FR1, 3.5 GHz)
python3 15_flight_and_5g_bridge.py --drones 3

# Vol long avec mmWave
python3 15_flight_and_5g_bridge.py --drones 3 --hover 60 --frequency 28e9 --numerology 3

# Altitude variable
python3 15_flight_and_5g_bridge.py --drones 5 --altitude 3 --alt-step 1.5 --hover 30
```

---

## 17. Comparaison WiFi vs 5G NR

### Résultats typiques pour 3 drones dans un entrepôt 30×20×6m

| Métrique | WiFi 802.11n | 5G NR FR1 | 5G NR FR2 |
|----------|-------------|-----------|-----------|
| RSSI/RSRP à 5m | -34 dBm | -25 dBm | -40 dBm |
| RSSI/RSRP à 10m | -49 dBm | -30 dBm | -50 dBm |
| RSSI/RSRP à 20m | -58 dBm | -35 dBm | -58 dBm |
| Latence typique | 2-5 ms | 1-3 ms | 0.5-1 ms |
| Débit max | 72.2 Mbps | 100+ Mbps | 1+ Gbps |
| Fiabilité | Moyenne | Haute | Haute (LOS) |

### Quand utiliser chaque technologie ?

| Cas d'usage | Recommandation |
|-------------|---------------|
| Entrepôt petit (<20m) | WiFi suffisant |
| Entrepôt grand (>50m) | 5G NR FR1 |
| Latence critique (<2ms) | 5G NR FR2 |
| Pas d'infrastructure | WiFi Ad-Hoc |
| Infrastructure disponible | 5G NR |
| Vidéo HD temps réel | 5G NR FR1/FR2 |
| Commandes simples | WiFi ou 5G FR1 |

---

## 18. Exemples d'utilisation

### Exemple 1 : Configuration par défaut (FR1, intérieur)

```bash
# Terminal 1 : Lancer les drones
./06_launch_multi_drones.sh 3

# Terminal 2 : Vol + Bridge 5G
python3 15_flight_and_5g_bridge.py --drones 3
```

### Exemple 2 : Configuration mmWave extérieur

```bash
python3 15_flight_and_5g_bridge.py \
    --drones 3 \
    --frequency 28e9 \
    --bandwidth 100e6 \
    --numerology 3 \
    --scenario UMi \
    --hover 60
```

### Exemple 3 : Comparaison WiFi vs 5G

```bash
# Test WiFi
python3 12_flight_and_bridge.py --drones 3 --hover 30
# → Résultat dans comm_metrics.csv

# Test 5G
python3 15_flight_and_5g_bridge.py --drones 3 --hover 30
# → Résultat dans comm_metrics_5g.csv

# Comparer les deux fichiers CSV
```

---

## 19. Intégration Sionna RT (futur)

Le blog de 5G-LENA (https://5g-lena.cttc.es/blog/33/) présente l'intégration de **Sionna RT** (ray-tracing NVIDIA) dans NS-3 pour des modèles de canal réalistes. Cette intégration :

- Remplace le modèle 3GPP stochastique par du ray-tracing déterministe
- Calcule les multi-trajets réels dans la géométrie 3D de l'entrepôt
- Utilise le GPU pour le calcul en temps réel
- S'intègre via pybind11 (pas de serveur séparé)

### Merge Request NS-3

La MR2608 (https://gitlab.com/nsnam/ns-3-dev/-/merge_requests/2608) ajoute :
- `SionnaRtChannelModel` : modèle de canal basé sur Sionna RT
- Compatible avec le pipeline MIMO de 5G-LENA
- Exemple : `sionna-rt-channel-example.cc`

### Pour activer (quand disponible)

```bash
# 1. Installer Sionna RT
pip install sionna-rt

# 2. Compiler NS-3 avec pybind11
./ns3 configure --enable-examples

# 3. Lancer avec le channel model Sionna RT
./ns3 run "scratch/drone-5g-scenario --scenario=SionnaRT"
```

---

## 20. Dépannage et erreurs courantes

### "nr: No such module"

```bash
# Le module NR n'est pas installé
cd ~/ns-allinone-3.40/ns-3.40/contrib
ls nr/  # Doit exister

# Si manquant :
./13_install_5g_lena.sh
```

### "NrHelper not found"

```bash
# Recompiler avec le module NR
cd ~/ns-allinone-3.40/ns-3.40
./ns3 configure --enable-examples
./ns3 build
```

### Erreur de version incompatible

```
Vérifier que la branche est bien 5g-lena-v2.6.y pour NS-3.40 :
  cd contrib/nr
  git branch
  # Doit afficher : * 5g-lena-v2.6.y
```

### NS-3 5G s'arrête immédiatement

```bash
# Vérifier que le scénario compile
cd ~/ns-allinone-3.40/ns-3.40
cp ~/simulation_mc02/ns3_scenarios/drone-5g-scenario.cc scratch/
./ns3 build 2>&1 | tail -20

# Tester manuellement
./ns3 run "scratch/drone-5g-scenario --nDrones=2 --simTime=5"
```

### Latence toujours à 1.0 ms (fallback)

Le FlowMonitor n'a pas encore reçu de paquets. Solutions :
- Attendre 2-3 secondes après le démarrage
- Vérifier que les UEs sont bien attachés à la gNB
- Augmenter la fréquence d'envoi des paquets UDP

---

## 21. Glossaire 5G

| Terme | Définition |
|-------|-----------|
| **gNB** | gNodeB : station de base 5G NR |
| **UE** | User Equipment : terminal mobile (ici, un drone) |
| **NR** | New Radio : interface radio 5G |
| **FR1** | Frequency Range 1 : bandes sub-6 GHz |
| **FR2** | Frequency Range 2 : bandes mmWave (24-52 GHz) |
| **RSRP** | Reference Signal Received Power : puissance du signal de référence |
| **SINR** | Signal-to-Interference-plus-Noise Ratio |
| **SCS** | Subcarrier Spacing : espacement des sous-porteuses |
| **BWP** | Bandwidth Part : partie de la bande passante |
| **CC** | Component Carrier : porteuse composante |
| **EPC** | Evolved Packet Core : réseau cœur |
| **SGW** | Serving Gateway : passerelle de service |
| **PGW** | PDN Gateway : passerelle vers le réseau de données |
| **OFDMA** | Orthogonal Frequency-Division Multiple Access |
| **TDMA** | Time Division Multiple Access |
| **UPA** | Uniform Planar Array : antenne planaire |
| **HARQ** | Hybrid Automatic Repeat Request |
| **MIMO** | Multiple-Input Multiple-Output |
| **3GPP** | 3rd Generation Partnership Project |
| **TR 38.901** | Technical Report 38.901 (modèles de canal NR) |
| **InH** | Indoor Hotspot (scénario 3GPP) |
| **UMi** | Urban Micro (scénario 3GPP) |
| **UMa** | Urban Macro (scénario 3GPP) |
| **RMa** | Rural Macro (scénario 3GPP) |
| **TTI** | Transmission Time Interval |
| **Numerologie μ** | Paramètre qui définit le SCS : SCS = 15 × 2^μ kHz |
