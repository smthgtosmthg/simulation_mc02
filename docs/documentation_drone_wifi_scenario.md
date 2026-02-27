# Documentation Complète — `drone-wifi-scenario.cc`

## Scénario NS-3 : Réseau Wi-Fi Ad-Hoc entre Drones

---

## Table des Matières

1. [Vue d'ensemble](#1-vue-densemble)
2. [Architecture du programme](#2-architecture-du-programme)
3. [Prérequis et compilation](#3-prérequis-et-compilation)
4. [Les includes (bibliothèques)](#4-les-includes-bibliothèques)
5. [Variables globales](#5-variables-globales)
6. [Structure `DronePos` et lecture CSV](#6-structure-dronepos-et-lecture-csv)
7. [Extraction de la latence — `GetPairLatencies()`](#7-extraction-de-la-latence--getpairlatencies)
8. [Mise à jour des positions — `UpdatePositions()`](#8-mise-à-jour-des-positions--updatepositions)
9. [Fonction `main()` — le cœur du programme](#9-fonction-main--le-cœur-du-programme)
10. [Configuration Wi-Fi détaillée](#10-configuration-wi-fi-détaillée)
11. [Modèles de propagation (Log-Distance vs Sionna)](#11-modèles-de-propagation-log-distance-vs-sionna)
12. [Configuration de la mobilité](#12-configuration-de-la-mobilité)
13. [Trafic UDP Echo — mesure de latence](#13-trafic-udp-echo--mesure-de-latence)
14. [FlowMonitor — métriques réseau réelles](#14-flowmonitor--métriques-réseau-réelles)
15. [Mode temps réel](#15-mode-temps-réel)
16. [Format des fichiers d'entrée/sortie](#16-format-des-fichiers-dentréesortie)
17. [Arguments en ligne de commande](#17-arguments-en-ligne-de-commande)
18. [Flux d'exécution complet](#18-flux-dexécution-complet)
19. [Interaction avec le script Python](#19-interaction-avec-le-script-python)
20. [NS3-Sionna — ray-tracing en détail](#20-ns3-sionna--ray-tracing-en-détail)
21. [Exemples d'utilisation](#21-exemples-dutilisation)
22. [Dépannage et erreurs courantes](#22-dépannage-et-erreurs-courantes)
23. [Glossaire](#23-glossaire)

---

## 1. Vue d'ensemble

### Qu'est-ce que ce programme fait ?

Ce programme C++ est un **scénario NS-3** qui simule un réseau Wi-Fi entre des drones volants. Il fait le pont entre le monde physique (positions des drones dans Gazebo) et le monde réseau (communications Wi-Fi simulées).

### En une phrase

> Il lit les positions des drones, simule un réseau Wi-Fi Ad-Hoc entre eux, mesure le RSSI (force du signal) et la latence (délai de communication), et écrit les résultats dans un fichier CSV.

### Qu'est-ce que NS-3 ?

**NS-3** (Network Simulator 3) est un simulateur réseau open-source utilisé en recherche académique et industrielle. Il simule les protocoles réseau (Wi-Fi, LTE, 5G, etc.) avec une grande fidélité. Contrairement à un simple calcul mathématique, NS-3 simule **chaque paquet** qui traverse le réseau, avec toutes les couches protocolaires (PHY, MAC, IP, transport).

### Différence avec l'ancienne version

| Aspect | Ancienne version | Cette version |
|--------|-------------------|---------------|
| **RSSI** | Calculé manuellement avec `log10()` | Calculé par NS-3 via `CalcRxPower()` |
| **Latence** | Valeur fixe de 2ms | Mesurée sur de vrais paquets via FlowMonitor |
| **Canal** | Uniquement log-distance | Log-distance **ou** NS3-Sionna (ray-tracing) |
| **Mode** | Hors ligne | **Temps réel** (synchronisé avec Gazebo) |

---

## 2. Architecture du programme

```
┌──────────────────────────────────────────────────────────────┐
│                    drone-wifi-scenario.cc                      │
│                                                                │
│  main()                                                        │
│  ├── Parse les arguments CLI                                   │
│  ├── Active le mode TEMPS RÉEL                                 │
│  ├── Crée N nœuds (drones)                                    │
│  ├── Configure Wi-Fi 802.11n Ad-Hoc                           │
│  ├── Configure le modèle de propagation                       │
│  │   ├── Log-Distance (par défaut)                            │
│  │   └── NS3-Sionna (optionnel, ray-tracing)                 │
│  ├── Installe la pile Internet (IP + routage)                 │
│  ├── Configure la mobilité (positions depuis CSV)             │
│  ├── Installe le trafic UDP Echo (pour mesurer la latence)    │
│  ├── Installe FlowMonitor (collecte les stats réseau)         │
│  ├── Planifie UpdatePositions() toutes les 0.5s               │
│  └── Simulator::Run() → boucle temps réel                    │
│                                                                │
│  UpdatePositions() [appelée périodiquement]                    │
│  ├── Lit drone_positions.csv                                  │
│  ├── Met à jour les positions des nœuds NS-3                  │
│  ├── Calcule RSSI via CalcRxPower()                           │
│  ├── Récupère latence via FlowMonitor                         │
│  └── Écrit ns3_output.csv                                     │
└──────────────────────────────────────────────────────────────┘
```

---

## 3. Prérequis et compilation

### Installation de NS-3

NS-3 doit être installé dans `~/ns-allinone-3.40/ns-3.40/`. Le script `09_install_ns3.sh` s'en charge.

### Compilation

```bash
# 1. Copier le scénario dans le dossier scratch/ de NS-3
cp drone-wifi-scenario.cc ~/ns-allinone-3.40/ns-3.40/scratch/

# 2. Compiler NS-3 (inclut automatiquement les fichiers dans scratch/)
cd ~/ns-allinone-3.40/ns-3.40
./ns3 build
```

**Pourquoi `scratch/` ?** : NS-3 compile automatiquement tous les fichiers `.cc` placés dans le dossier `scratch/`. C'est le moyen le plus simple d'ajouter un scénario personnalisé sans modifier le système de build de NS-3.

### Exécution manuelle (pour test)

```bash
./ns3 run "scratch/drone-wifi-scenario --nDrones=3 --simTime=60"
```

> **Note** : En pratique, c'est le script Python `12_flight_and_bridge.py` qui lance automatiquement ce programme.

---

## 4. Les includes (bibliothèques)

### Modules NS-3

```cpp
#include "ns3/core-module.h"          // Cœur de NS-3 : Simulator, Time, Logging, CommandLine
#include "ns3/network-module.h"       // Nœuds (Node), paquets (Packet), NetDevice
#include "ns3/wifi-module.h"          // Wi-Fi : WifiHelper, PHY, MAC, canaux
#include "ns3/mobility-module.h"      // Mobilité : modèles de position et mouvement
#include "ns3/internet-module.h"      // Pile Internet : IPv4, routage, sockets
#include "ns3/applications-module.h"  // Applications : UdpEchoClient, UdpEchoServer
#include "ns3/flow-monitor-module.h"  // FlowMonitor : statistiques sur les flux réseau
#include "ns3/propagation-module.h"   // Modèles de propagation radio (LogDistance, etc.)
```

#### Explication de chaque module en détail

**`core-module`** : Le fondement de NS-3. Contient :
- `Simulator` : Le moteur de simulation (planification d'événements)
- `Time`, `Seconds()` : Gestion du temps
- `NS_LOG_*` : Système de logs (INFO, WARN, ERROR)
- `CommandLine` : Parsing des arguments CLI
- `GlobalValue` : Configuration globale (mode temps réel, etc.)

**`network-module`** : Les briques de base du réseau :
- `Node` : Un nœud réseau (ici, chaque nœud = un drone)
- `NodeContainer` : Un groupe de nœuds
- `NetDevice` : Une interface réseau (carte Wi-Fi virtuelle)
- `Packet` : Un paquet réseau

**`wifi-module`** : Tout ce qui concerne Wi-Fi :
- `WifiHelper` : Configuration simplifiée du Wi-Fi
- `YansWifiPhyHelper` : Configuration de la couche physique (PHY)
- `YansWifiChannel` : Le canal radio (ondes entre les nœuds)
- `WifiMacHelper` : Configuration de la couche MAC (Medium Access Control)

**`mobility-module`** : Gestion des positions des nœuds :
- `MobilityHelper` : Installation de modèles de mobilité
- `ConstantPositionMobilityModel` : Position fixe (mise à jour manuellement)
- `Vector` : Coordonnées 3D (x, y, z)

**`internet-module`** : Pile protocolaire Internet :
- `InternetStackHelper` : Installe IPv4 + routage sur les nœuds
- `Ipv4AddressHelper` : Attribution d'adresses IP
- `Ipv4Address` : Une adresse IPv4

**`applications-module`** : Applications réseau :
- `UdpEchoServerHelper` / `UdpEchoClientHelper` : Serveur/client UDP Echo
- Un client envoie un paquet, le serveur le renvoie → permet de mesurer le round-trip time (RTT)

**`flow-monitor-module`** : Collecte de statistiques :
- `FlowMonitor` : Surveille tous les flux IP et collecte les stats
- `Ipv4FlowClassifier` : Identifie les flux par adresses IP source/destination

**`propagation-module`** : Modèles radio :
- `LogDistancePropagationLossModel` : Modèle de perte de signal log-distance
- `ConstantSpeedPropagationDelayModel` : Délai proportionnel à la distance

### Modules NS3-Sionna (optionnels)

```cpp
#define NS3_SIONNA_AVAILABLE
#ifdef NS3_SIONNA_AVAILABLE
#include "ns3/sionna-helper.h"                    // Aide à la configuration de Sionna
#include "ns3/sionna-propagation-cache.h"          // Cache des résultats ray-tracing
#include "ns3/sionna-propagation-delay-model.h"    // Délai calculé par ray-tracing
#include "ns3/sionna-propagation-loss-model.h"     // Pertes calculées par ray-tracing
#include "ns3/sionna-mobility-model.h"             // Modèle de mobilité spécial Sionna
#include "ns3/sionna-utils.h"                      // Utilitaires (fréquence, FFT, etc.)
#endif
```

Le `#define NS3_SIONNA_AVAILABLE` active la compilation conditionnelle. Si NS3-Sionna n'est pas installé, il suffit de commenter cette ligne pour que le code compile sans erreur (toutes les sections Sionna seront ignorées par le préprocesseur).

### Bibliothèques C++ standard

```cpp
#include <fstream>     // Lecture/écriture de fichiers (ifstream, ofstream)
#include <sstream>     // Analyse de chaînes (istringstream)
#include <vector>      // Tableaux dynamiques
#include <map>         // Dictionnaires (clé → valeur)
#include <cmath>       // Fonctions mathématiques (sqrt, etc.)
#include <algorithm>   // Algorithmes (min, max, etc.)
```

---

## 5. Variables globales

```cpp
using namespace ns3;
NS_LOG_COMPONENT_DEFINE("DroneWifiScenario");
```

- `using namespace ns3;` : Permet d'écrire `Simulator::Run()` au lieu de `ns3::Simulator::Run()`
- `NS_LOG_COMPONENT_DEFINE(...)` : Déclare un composant de log nommé "DroneWifiScenario". Cela permet d'activer/désactiver les logs de ce fichier spécifiquement avec `NS_LOG=DroneWifiScenario=info`

### Paramètres de simulation

```cpp
static uint32_t g_nDrones = 3;                           // Nombre de drones
static std::string g_posFile = "/tmp/drone_positions.csv"; // Fichier d'entrée (positions)
static std::string g_outFile = "/tmp/ns3_output.csv";      // Fichier de sortie (métriques)
static double g_simTime = 60.0;                            // Durée de la simulation (secondes)
static double g_updateInterval = 0.5;                      // Intervalle de mise à jour (secondes)
static std::string g_channelModel = "log-distance";        // Modèle de canal radio
static std::string g_sionnaEnv = "simple_room/simple_room.xml"; // Scène 3D Sionna
static std::string g_sionnaUrl = "tcp://localhost:5555";   // Adresse du serveur Sionna
```

**Pourquoi `static` ?** : Le mot-clé `static` limite la portée de ces variables à ce fichier. C'est une bonne pratique en C++ pour éviter les conflits de noms avec d'autres fichiers.

**Pourquoi le préfixe `g_` ?** : Convention de nommage pour indiquer que ce sont des variables **globales** (g = global). Cela aide à les distinguer des variables locales.

### Objets NS-3 globaux

```cpp
static NodeContainer g_droneNodes;       // Conteneur de tous les nœuds (drones)
static NetDeviceContainer g_wifiDevices; // Interfaces Wi-Fi installées sur les nœuds
static std::ofstream g_outputCsv;        // Flux de sortie pour le fichier CSV
```

| Variable | Type | Rôle |
|----------|------|------|
| `g_droneNodes` | `NodeContainer` | Groupe de N nœuds NS-3, un par drone |
| `g_wifiDevices` | `NetDeviceContainer` | Les cartes Wi-Fi virtuelles installées sur chaque nœud |
| `g_outputCsv` | `ofstream` | Fichier CSV de sortie ouvert pendant toute la simulation |

### Variables pour le RSSI et la latence

```cpp
static Ptr<PropagationLossModel> g_propLossModel;   // Modèle de perte de propagation
static double g_txPowerDbm = 20.0;                   // Puissance d'émission (20 dBm = 100 mW)
static Ptr<FlowMonitor> g_flowMonitor;               // Moniteur de flux réseau
static Ptr<Ipv4FlowClassifier> g_flowClassifier;     // Classifieur de flux (IP → index drone)
static FlowMonitor::FlowStatsContainer g_prevFlowStats; // Stats précédentes (pour le delta)
```

#### `g_propLossModel` — Le modèle de propagation

C'est un **pointeur intelligent** (`Ptr<>`, équivalent NS-3 du `shared_ptr` C++) vers le modèle qui calcule la perte de signal radio. Ce pointeur peut contenir :
- Un `LogDistancePropagationLossModel` (modèle mathématique simple)
- Ou un `SionnaPropagationLossModel` (ray-tracing réaliste)

La méthode clé est `CalcRxPower(txPowerDbm, mobA, mobB)` qui retourne la puissance reçue (RSSI) entre deux nœuds.

#### `g_txPowerDbm` — Puissance d'émission

20 dBm = 100 milliwatts. C'est la puissance typique d'une carte Wi-Fi. Formule de conversion : $P_{mW} = 10^{P_{dBm}/10}$, donc $10^{20/10} = 100$ mW.

#### `g_prevFlowStats` — Stats précédentes (pour le delta)

Le FlowMonitor accumule les statistiques depuis le début de la simulation. Pour obtenir la **latence récente** (pas la moyenne depuis le début), on calcule le **delta** : on soustrait les stats de la dernière mesure des stats actuelles. Cette variable stocke les stats de la mesure précédente.

### Pointeur Sionna (optionnel)

```cpp
#ifdef NS3_SIONNA_AVAILABLE
static SionnaHelper* g_sionnaHelper = nullptr;
#endif
```

Pointeur brut vers le helper Sionna. Initialisé à `nullptr` et créé seulement si `--channelModel=sionna`. Nettoyé à la fin avec `delete`.

---

## 6. Structure `DronePos` et lecture CSV

### Structure `DronePos`

```cpp
struct DronePos {
    double x, y, z;
};
```

Une structure simple qui stocke les coordonnées 3D d'un drone. Chaque champ est un `double` (nombre à virgule flottante 64 bits).

**Exemple** : `DronePos{0.1, 3.0, 5.0}` = un drone en position (x=0.1, y=3.0, z=5.0)

### Fonction `ReadPositions()`

```cpp
static std::vector<DronePos>
ReadPositions(const std::string& filename, uint32_t nDrones)
```

**Objectif** : Lire les positions des drones depuis le fichier CSV généré par le bridge Python.

#### Étape par étape

```cpp
// 1. Initialisation avec des positions par défaut (0,0,0)
std::vector<DronePos> positions(nDrones, {0.0, 0.0, 0.0});
```

Crée un vecteur de N positions, toutes à l'origine. Si le fichier est illisible, ces valeurs par défaut seront retournées.

```cpp
// 2. Ouverture du fichier
std::ifstream file(filename);
if (!file.is_open()) {
    NS_LOG_WARN("Cannot open position file: " << filename);
    return positions;   // Retourne les positions par défaut
}
```

`ifstream` = Input File Stream (flux de lecture). Si le fichier n'existe pas ou est verrouillé, on retourne les positions par défaut sans planter.

```cpp
// 3. Sauter l'en-tête CSV
std::string line;
if (std::getline(file, line)) {
    // header line — on la jette
}
```

La première ligne du CSV est l'en-tête (`drone_id,x,y,z`), on la lit et on l'ignore.

```cpp
// 4. Lire chaque ligne de données
while (std::getline(file, line)) {
    std::istringstream ss(line);     // Crée un flux à partir de la ligne
    std::string token;
    uint32_t id;
    double x, y, z;

    std::getline(ss, token, ','); id = std::stoi(token);  // drone_id
    std::getline(ss, token, ','); x = std::stod(token);   // x
    std::getline(ss, token, ','); y = std::stod(token);   // y
    std::getline(ss, token, ','); z = std::stod(token);   // z

    if (id < nDrones) {
        positions[id] = {x, y, z};
    }
}
```

**Détail du parsing** :
- `std::getline(ss, token, ',')` : Lit jusqu'à la prochaine virgule
- `std::stoi(token)` : Convertit la chaîne en entier (string-to-integer)
- `std::stod(token)` : Convertit la chaîne en double (string-to-double)
- `if (id < nDrones)` : Protection contre un ID invalide dans le CSV

**Exemple de lecture** :
```
CSV ligne : "1,0.2000,3.0000,5.0000"
→ id=1, x=0.2, y=3.0, z=5.0
→ positions[1] = {0.2, 3.0, 5.0}
```

---

## 7. Extraction de la latence — `GetPairLatencies()`

```cpp
static std::map<std::pair<uint32_t, uint32_t>, double>
GetPairLatencies(void)
```

**Type de retour** : `map<pair<uint32_t, uint32_t>, double>` — un dictionnaire qui associe chaque paire de drones (i, j) à sa latence en millisecondes.

**Exemple de résultat** :
```
{
  (0, 1) → 2.34 ms,
  (0, 2) → 3.12 ms,
  (1, 2) → 2.87 ms
}
```

### Fonctionnement détaillé

#### Vérification préliminaire

```cpp
if (!g_flowMonitor || !g_flowClassifier) {
    return result;   // Retourne un dictionnaire vide
}
```

Si le FlowMonitor n'est pas encore installé, on retourne un résultat vide (pas de crash).

#### Récupération des stats

```cpp
g_flowMonitor->CheckForLostPackets();
FlowMonitor::FlowStatsContainer stats = g_flowMonitor->GetFlowStats();
```

- `CheckForLostPackets()` : Force la vérification des paquets perdus (ceux qui n'ont pas reçu d'ACK dans le timeout)
- `GetFlowStats()` : Récupère toutes les statistiques pour chaque flux réseau

Un **flux** (flow) est une séquence de paquets entre une source et une destination. Pour N drones, il y a N flux UDP Echo (un par client).

#### Mapping IP → index drone

```cpp
Ipv4FlowClassifier::FiveTuple t = g_flowClassifier->FindFlow(iter.first);

uint32_t srcByte = t.sourceAddress.Get() & 0xFF;      // Dernier octet de l'IP
uint32_t dstByte = t.destinationAddress.Get() & 0xFF;  // Dernier octet de l'IP
uint32_t srcDrone = srcByte - 1;   // 10.1.1.1 → drone 0, 10.1.1.2 → drone 1, etc.
uint32_t dstDrone = dstByte - 1;
```

**Explication du mapping** :

Les drones reçoivent des adresses IP dans le sous-réseau `10.1.1.0/24` :

| Drone | Adresse IP | Dernier octet | Index |
|-------|-----------|----------------|-------|
| 0 | 10.1.1.1 | 1 | 0 (= 1-1) |
| 1 | 10.1.1.2 | 2 | 1 (= 2-1) |
| 2 | 10.1.1.3 | 3 | 2 (= 3-1) |

`t.sourceAddress.Get()` retourne l'adresse IP comme un entier 32 bits. L'opération `& 0xFF` extrait le dernier octet (masque binaire : on ne garde que les 8 bits de poids faible).

**Le FiveTuple** contient : IP source, IP destination, port source, port destination, protocole. C'est l'identifiant unique d'un flux réseau.

#### Calcul du delta de latence

```cpp
auto prevIt = g_prevFlowStats.find(iter.first);
if (prevIt != g_prevFlowStats.end()) {
    // On a des stats précédentes → calcul du delta
    uint64_t newPkts = iter.second.rxPackets - prevIt->second.rxPackets;
    if (newPkts > 0) {
        Time newDelay = iter.second.delaySum - prevIt->second.delaySum;
        delayMs = newDelay.GetSeconds() * 1000.0 / (double)newPkts;
    }
} else if (iter.second.rxPackets > 0) {
    // Première mesure → moyenne depuis le début
    delayMs = iter.second.delaySum.GetSeconds() * 1000.0
              / (double)iter.second.rxPackets;
}
```

**Pourquoi le delta ?**

Imaginez qu'après 10 secondes, le FlowMonitor rapporte :
- Total : 100 paquets reçus, délai cumulé = 250 ms → moyenne = 2.5 ms

Après 10.5 secondes :
- Total : 105 paquets reçus, délai cumulé = 268 ms → moyenne cumulative = 2.55 ms

Mais on veut la latence **récente** (les 5 derniers paquets) :
- Delta paquets : 105 - 100 = 5
- Delta délai : 268 - 250 = 18 ms
- Latence récente : 18 / 5 = **3.6 ms**

La latence récente (3.6 ms) est bien plus informative que la moyenne cumulative (2.55 ms), car elle reflète les conditions réseau **actuelles**.

#### Stockage en paire ordonnée

```cpp
auto key = std::make_pair(std::min(srcDrone, dstDrone),
                           std::max(srcDrone, dstDrone));
```

On ordonne toujours la paire (petit, grand) pour que `(0,1)` et `(1,0)` soient stockés sous la même clé. Cela évite les doublons.

```cpp
if (delayMs > 0.0) {
    auto existing = result.find(key);
    if (existing == result.end() || delayMs < existing->second) {
        result[key] = delayMs;
    }
}
```

Si plusieurs flux existent pour la même paire (un dans chaque sens), on garde la **latence la plus faible** (la plus optimiste).

#### Sauvegarde pour le prochain delta

```cpp
g_prevFlowStats = stats;
```

On sauvegarde les stats actuelles pour pouvoir calculer le delta la prochaine fois.

---

## 8. Mise à jour des positions — `UpdatePositions()`

```cpp
static void UpdatePositions(void)
```

Cette fonction est **le cœur de la boucle de simulation**. Elle est appelée périodiquement (toutes les 0.5s par défaut) et fait tout le travail de mesure.

### Étape 1 : Lire les positions depuis le CSV

```cpp
std::vector<DronePos> positions = ReadPositions(g_posFile, g_nDrones);
```

Le fichier `/tmp/drone_positions.csv` est écrit par le script Python `12_flight_and_bridge.py` et lu ici par NS-3. C'est le mécanisme de **communication inter-processus** entre Python et NS-3.

### Étape 2 : Mettre à jour les modèles de mobilité NS-3

```cpp
for (uint32_t i = 0; i < g_nDrones; ++i) {
    Ptr<MobilityModel> mob = g_droneNodes.Get(i)->GetObject<MobilityModel>();
    if (mob) {
        mob->SetPosition(Vector(positions[i].x, positions[i].y, positions[i].z));
    }
}
```

**Explication** :
- `g_droneNodes.Get(i)` : Récupère le nœud NS-3 du drone i
- `->GetObject<MobilityModel>()` : Récupère le modèle de mobilité attaché au nœud (c'est le patron de conception **aggregation** de NS-3)
- `mob->SetPosition(...)` : Met à jour la position du nœud

**Pourquoi c'est important ?** : Le modèle de propagation utilise les positions des nœuds pour calculer la distance et donc le RSSI. Si on ne met pas à jour les positions, NS-3 calculera les métriques avec les anciennes positions.

### Étape 3 : Récupérer la latence

```cpp
double simTime = Simulator::Now().GetSeconds();
auto pairLatencies = GetPairLatencies();
```

`Simulator::Now()` retourne le temps actuel de la simulation (en mode temps réel, il est synchronisé avec l'horloge système).

### Étape 4 : Calculer les métriques pour chaque paire

```cpp
for (uint32_t i = 0; i < g_nDrones; ++i) {
    for (uint32_t j = i + 1; j < g_nDrones; ++j) {
```

Double boucle avec `j > i` pour éviter les doublons : (0,1), (0,2), (1,2) — pas (1,0), (2,0), (2,1).

#### Calcul de la distance

```cpp
double dx = positions[i].x - positions[j].x;
double dy = positions[i].y - positions[j].y;
double dz = positions[i].z - positions[j].z;
double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
```

Distance euclidienne 3D : $d = \sqrt{\Delta x^2 + \Delta y^2 + \Delta z^2}$

#### Calcul du RSSI via NS-3

```cpp
Ptr<MobilityModel> mob_i = g_droneNodes.Get(i)->GetObject<MobilityModel>();
Ptr<MobilityModel> mob_j = g_droneNodes.Get(j)->GetObject<MobilityModel>();
double rssiDbm = g_propLossModel->CalcRxPower(g_txPowerDbm, mob_i, mob_j);
```

**C'est la ligne la plus importante du programme.** La méthode `CalcRxPower()` :

1. Récupère les positions des deux nœuds via leurs modèles de mobilité
2. Calcule la distance entre eux
3. Applique le modèle de propagation configuré :
   - **Log-Distance** : formule mathématique $RSSI = P_{tx} - PL_0 - 10n\log_{10}(d)$
   - **Sionna** : ray-tracing complet (réflexions, diffractions, obstacles 3D)
4. Retourne la puissance reçue en dBm

**Avantage** : Le RSSI est calculé par le **même modèle** que celui utilisé pour la simulation du trafic Wi-Fi. Il y a donc cohérence totale entre le RSSI mesuré et le comportement réel du réseau.

#### Récupération de la latence

```cpp
double latencyMs = 0.0;
auto pairKey = std::make_pair(i, j);
auto latIt = pairLatencies.find(pairKey);
if (latIt != pairLatencies.end() && latIt->second > 0.0) {
    latencyMs = latIt->second;    // Latence mesurée sur vrais paquets
} else {
    // Fallback : estimation simple
    double propagationMs = (distance / 3e8) * 1000.0;
    latencyMs = propagationMs + 2.0;
}
```

**Deux cas** :
1. **FlowMonitor a des données** : On utilise la latence réelle mesurée sur les paquets UDP Echo. Cette latence inclut le délai de propagation, le délai MAC (attente du canal, backoff), le traitement des files d'attente, etc.
2. **Pas encore de données** (premiers instants de la simulation) : On utilise un fallback simple (propagation + 2ms).

#### Écriture dans le CSV de sortie

```cpp
g_outputCsv << simTime << ","
           << i << "," << j << ","
           << distance << ","
           << rssiDbm << ","
           << latencyMs << ","
           << positions[i].x << "," << positions[i].y << "," << positions[i].z << ","
           << positions[j].x << "," << positions[j].y << "," << positions[j].z
           << std::endl;
```

Chaque ligne du CSV contient : temps, paire (i,j), distance, RSSI, latence, et les positions des deux drones.

`std::endl` force un **flush** (écriture immédiate sur disque), ce qui est important car le script Python lit ce fichier en temps réel.

### Étape 5 : Planifier la prochaine mise à jour

```cpp
Simulator::Schedule(Seconds(g_updateInterval), &UpdatePositions);
```

**C'est la récursion temporelle.** La fonction se re-planifie elle-même pour être appelée dans 0.5 secondes. C'est le mécanisme standard de NS-3 pour les événements périodiques :

```
t=0.0s → UpdatePositions() → planifie t=0.5s
t=0.5s → UpdatePositions() → planifie t=1.0s
t=1.0s → UpdatePositions() → planifie t=1.5s
...
```

`Simulator::Schedule()` ne bloque pas. Il ajoute un événement à la file d'événements de NS-3, et la fonction retourne immédiatement.

---

## 9. Fonction `main()` — le cœur du programme

La fonction `main()` initialise toute la simulation NS-3. Voici chaque section en détail.

### Parsing des arguments CLI

```cpp
CommandLine cmd;
cmd.AddValue("nDrones", "Number of drones", g_nDrones);
cmd.AddValue("posFile", "Input CSV with drone positions", g_posFile);
cmd.AddValue("outFile", "Output CSV with RSSI and latency", g_outFile);
cmd.AddValue("simTime", "Simulation time in seconds", g_simTime);
cmd.AddValue("updateInterval", "Position update interval (s)", g_updateInterval);
cmd.AddValue("channelModel", "Channel model: log-distance or sionna", g_channelModel);
cmd.AddValue("sionnaEnv", "Sionna XML scene file", g_sionnaEnv);
cmd.AddValue("sionnaUrl", "Sionna ZMQ server URL", g_sionnaUrl);
cmd.Parse(argc, argv);
```

`CommandLine` de NS-3 fonctionne comme `argparse` en Python. `AddValue()` lie un argument CLI à une variable globale. Quand `cmd.Parse()` est appelé, les valeurs des variables sont mises à jour avec les arguments fournis.

### Activation du mode temps réel

```cpp
GlobalValue::Bind("SimulatorImplementationType",
                  StringValue("ns3::RealtimeSimulatorImpl"));
```

Cette ligne est **cruciale**. Par défaut, NS-3 simule le plus vite possible (le temps simulé avance bien plus vite que le temps réel). Avec `RealtimeSimulatorImpl`, NS-3 synchronise son horloge interne avec l'horloge système : 1 seconde simulée = 1 seconde réelle.

**Pourquoi c'est nécessaire ?** : Parce que les drones volent en temps réel dans Gazebo. Si NS-3 simulait 60 secondes en 2 secondes, les positions ne correspondraient plus au vol réel.

### Création des nœuds

```cpp
g_droneNodes.Create(g_nDrones);
```

Crée N objets `Node` vides (pas encore de Wi-Fi, pas d'IP, pas de position). Chaque nœud représente un drone dans la simulation réseau.

---

## 10. Configuration Wi-Fi détaillée

### Standard Wi-Fi

```cpp
WifiHelper wifi;
wifi.SetStandard(WIFI_STANDARD_80211n);
```

**802.11n** (Wi-Fi 4) est choisi car :
- Il fonctionne à **2.4 GHz** (bonne portée, bien adapté aux drones)
- Il supporte les techniques MIMO
- Il est largement disponible sur les modules Wi-Fi pour drones
- NS-3 le supporte bien

### Rate Manager

```cpp
wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                              "DataMode", StringValue("HtMcs7"),
                              "ControlMode", StringValue("HtMcs0"));
```

Le **Rate Manager** détermine le débit Wi-Fi utilisé.

- `ConstantRateWifiManager` : Utilise un débit fixe (pas d'adaptation dynamique)
- `HtMcs7` : Modulation and Coding Scheme 7 pour les données (le plus rapide en 802.11n, ~65 Mbps en canal 20 MHz)
- `HtMcs0` : MCS 0 pour les trames de contrôle (le plus robuste, ~6.5 Mbps)

**Pourquoi un débit fixe ?** : Pour simplifier les résultats. Un rate manager adaptatif changerait le débit en fonction du RSSI, ce qui ajouterait une variable supplémentaire à l'analyse.

### Couche PHY (physique)

```cpp
YansWifiPhyHelper wifiPhy;
wifiPhy.Set("TxPowerStart", DoubleValue(g_txPowerDbm));
wifiPhy.Set("TxPowerEnd", DoubleValue(g_txPowerDbm));
```

**YANS** = Yet Another Network Simulator. C'est le modèle PHY standard de NS-3 pour Wi-Fi.

- `TxPowerStart` et `TxPowerEnd` : Puissance d'émission minimale et maximale. En les mettant à la même valeur (20 dBm), on force une puissance constante.

### Canal radio

```cpp
Ptr<YansWifiChannel> wifiChannel = CreateObject<YansWifiChannel>();
```

Le `YansWifiChannel` est le "médium" radio. Tous les nœuds Wi-Fi sont connectés au même canal, comme dans la réalité où tous les appareils partagent les mêmes ondes radio.

### Modèle de délai de propagation

```cpp
Ptr<ConstantSpeedPropagationDelayModel> delayModel =
    CreateObject<ConstantSpeedPropagationDelayModel>();
wifiChannel->SetPropagationDelayModel(delayModel);
```

Ce modèle calcule le délai de propagation des ondes radio : $t = d / c$ où $c = 3 \times 10^8$ m/s. Pour 10 mètres, le délai est d'environ 33 nanosecondes.

### Couche MAC (Ad-Hoc)

```cpp
WifiMacHelper wifiMac;
wifiMac.SetType("ns3::AdhocWifiMac");
```

**Mode Ad-Hoc** : Pas de point d'accès (AP). Tous les nœuds communiquent directement entre eux, en pair-à-pair. C'est le mode le plus adapté pour un réseau de drones, car il n'y a pas de station de base au sol.

Comparaison des modes :
| Mode | Description | AP nécessaire ? |
|------|-------------|-----------------|
| Infrastructure | Tous les appareils passent par un AP | Oui |
| **Ad-Hoc** | Communication directe pair-à-pair | **Non** |
| Mesh | Routage multi-sauts | Non |

### Installation

```cpp
g_wifiDevices = wifi.Install(wifiPhy, wifiMac, g_droneNodes);
```

Cette ligne fait **tout le travail** :
1. Crée une carte Wi-Fi virtuelle pour chaque nœud
2. Configure la couche PHY avec les paramètres définis
3. Configure la couche MAC en mode Ad-Hoc
4. Connecte tout au canal radio
5. Retourne les `NetDevice` installés

---

## 11. Modèles de propagation (Log-Distance vs Sionna)

### Modèle Log-Distance (par défaut)

```cpp
Ptr<LogDistancePropagationLossModel> lossModel =
    CreateObject<LogDistancePropagationLossModel>();
lossModel->SetAttribute("Exponent", DoubleValue(3.0));
lossModel->SetAttribute("ReferenceLoss", DoubleValue(40.0));
g_propLossModel = lossModel;
```

**Formule** : $PL(d) = PL_0 + 10 \cdot n \cdot \log_{10}(d)$

| Paramètre | Valeur | Signification |
|-----------|--------|---------------|
| `Exponent` (n) | 3.0 | Environnement intérieur avec obstacles (entrepôt) |
| `ReferenceLoss` ($PL_0$) | 40.0 dB | Perte à 1 mètre de distance à 2.4 GHz |

**Valeurs typiques de l'exposant** :
| Environnement | Exposant n |
|---------------|-----------|
| Espace libre | 2.0 |
| Bureau ouvert | 2.5-3.0 |
| **Entrepôt (ici)** | **3.0** |
| Couloir étroit | 1.5-2.0 |
| Bâtiment dense | 4.0-6.0 |

**Avantages** : Simple, rapide, prédictible.
**Inconvénients** : Ne modélise pas les réflexions, diffractions, obstacles spécifiques.

### Modèle NS3-Sionna (ray-tracing)

```cpp
g_sionnaHelper = new SionnaHelper(g_sionnaEnv, g_sionnaUrl);
g_sionnaHelper->SetMode(SionnaHelper::MODE_P2P);

Ptr<SionnaPropagationCache> propagationCache =
    CreateObject<SionnaPropagationCache>();
propagationCache->SetSionnaHelper(*g_sionnaHelper);
propagationCache->SetCaching(true);

Ptr<SionnaPropagationLossModel> sionnaLoss =
    CreateObject<SionnaPropagationLossModel>();
sionnaLoss->SetPropagationCache(propagationCache);
g_propLossModel = sionnaLoss;
```

**Architecture Sionna** :

```
NS-3                          Serveur Python Sionna
┌──────────────┐              ┌──────────────────┐
│ SionnaHelper │◄── ZMQ ────►│  run_server.py   │
│              │              │                   │
│ PropCache    │              │  Sionna RT        │
│ LossModel    │              │  (ray-tracing)    │
│ DelayModel   │              │                   │
└──────────────┘              │  Scène 3D (.xml)  │
                              └──────────────────┘
```

**Comment ça marche** :
1. NS-3 envoie les positions des nœuds au serveur Python via ZMQ
2. Le serveur Sionna lance un **ray-tracing** : il trace des milliers de rayons dans la scène 3D
3. Les rayons se réfléchissent sur les murs, le sol, le plafond, les obstacles
4. Sionna calcule le **CSI** (Channel State Information) = réponse complète du canal
5. Le résultat (atténuation, délai) est renvoyé à NS-3 via ZMQ
6. NS-3 utilise ces valeurs pour `CalcRxPower()`

**Mode P2P** : Calcul point-à-point, une paire de nœuds à la fois. Plus lent que le mode P2MP (point-à-multipoint) mais plus stable et suffisant pour notre cas.

**Cache** : `SetCaching(true)` active le cache des résultats. Si deux nœuds n'ont pas bougé, Sionna ne recalcule pas le ray-tracing.

**Avantages** : Très réaliste, prend en compte la géométrie 3D de l'environnement.
**Inconvénients** : Lent, nécessite un serveur externe, complexe à configurer.

---

## 12. Configuration de la mobilité

### Pile Internet

```cpp
InternetStackHelper internet;
internet.Install(g_droneNodes);

Ipv4AddressHelper ipv4;
ipv4.SetBase("10.1.1.0", "255.255.255.0");
ipv4.Assign(g_wifiDevices);
```

- `InternetStackHelper` installe IPv4 + le routage sur chaque nœud
- Adresses IP attribuées dans le sous-réseau `10.1.1.0/24` :
  - Drone 0 → `10.1.1.1`
  - Drone 1 → `10.1.1.2`
  - Drone 2 → `10.1.1.3`

### Modèle de mobilité

```cpp
#ifdef NS3_SIONNA_AVAILABLE
    if (g_channelModel == "sionna") {
        mobility.SetMobilityModel("ns3::SionnaMobilityModel");
    } else {
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    }
#else
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
#endif
mobility.Install(g_droneNodes);
```

**`ConstantPositionMobilityModel`** : Le nœud reste à la position qu'on lui assigne. Il ne bouge pas tout seul. Nos drones ne bougent pas dans NS-3 de manière autonome — c'est `UpdatePositions()` qui met à jour leur position manuellement depuis le CSV.

**`SionnaMobilityModel`** : Version spéciale exigée par Sionna. Elle hérite de `ConstantPositionMobilityModel` mais ajoute des métadonnées nécessaires au ray-tracing (Sionna fait un `DynamicCast` interne et plante si le type n'est pas `SionnaMobilityModel`).

### Positions initiales

```cpp
std::vector<DronePos> initPos = ReadPositions(g_posFile, g_nDrones);
for (uint32_t i = 0; i < g_nDrones; ++i) {
    Ptr<MobilityModel> mob = g_droneNodes.Get(i)->GetObject<MobilityModel>();
    mob->SetPosition(Vector(initPos[i].x, initPos[i].y, initPos[i].z));
}
```

Lit les positions initiales depuis le CSV (le script Python y écrit des valeurs initiales avant de lancer NS-3) et les applique aux nœuds.

### Démarrage de Sionna (si activé)

```cpp
if (g_channelModel == "sionna" && g_sionnaHelper != nullptr) {
    double channelWidth = get_channel_width(g_wifiDevices.Get(0));
    int centerFreq = (int)get_center_freq(g_wifiDevices.Get(0));
    int fftSize = getFFTSize(WIFI_STANDARD_80211n, channelWidth);
    int subcarrierSpacing = getSubcarrierSpacing(WIFI_STANDARD_80211n);

    g_sionnaHelper->Configure(centerFreq, (int)channelWidth,
                               fftSize, subcarrierSpacing, 500);
    g_sionnaHelper->Start();
}
```

**Paramètres envoyés au serveur Sionna** :

| Paramètre | Valeur typique | Signification |
|-----------|---------------|---------------|
| `centerFreq` | 2412 MHz | Fréquence centrale du canal Wi-Fi |
| `channelWidth` | 20 MHz | Largeur de bande |
| `fftSize` | 64 | Taille de la FFT (OFDM) |
| `subcarrierSpacing` | 312500 Hz | Espacement entre sous-porteuses |
| Coherence time | 500 ms | Temps max avant recalcul du CSI |

**Ordre d'initialisation critique** : `Configure()` et `Start()` doivent être appelés **APRÈS** `mobility.Install()` et `SetPosition()`. Si Sionna démarre avant que les nœuds aient des positions, le serveur Python reçoit des nœuds sans position → crash (`KeyError`).

---

## 13. Trafic UDP Echo — mesure de latence

```cpp
uint16_t port = 9;
for (uint32_t i = 0; i < g_nDrones; ++i) {
    uint32_t j = (i + 1) % g_nDrones;   // Destination = drone suivant (circulaire)
    ...
}
```

### Topologie du trafic

La boucle crée un **anneau** de communication entre les drones :

```
Pour 3 drones :
  Drone 0 → Drone 1   (i=0, j=1)
  Drone 1 → Drone 2   (i=1, j=2)
  Drone 2 → Drone 0   (i=2, j=0)  ← %3 = 0

      ┌───────┐
      │Drone 0│
      └───┬───┘
    Echo  │  ▲  Echo
    0→1   │  │  2→0
          ▼  │
  ┌───────┐  │  ┌───────┐
  │Drone 1│──┼──│Drone 2│
  └───────┘  │  └───────┘
        Echo 1→2
```

### Serveur UDP Echo

```cpp
UdpEchoServerHelper echoServer(port + i);
ApplicationContainer serverApp = echoServer.Install(dstNode);
serverApp.Start(Seconds(0.0));
serverApp.Stop(Seconds(g_simTime));
```

Le serveur écoute sur un port spécifique (9 + i) et **renvoie chaque paquet reçu** à l'expéditeur. C'est le mécanisme classique du **ping** (mais en UDP, pas ICMP).

### Client UDP Echo

```cpp
UdpEchoClientHelper echoClient(dstAddr, port + j);
echoClient.SetAttribute("MaxPackets", UintegerValue(999999));
echoClient.SetAttribute("Interval", TimeValue(Seconds(0.1)));
echoClient.SetAttribute("PacketSize", UintegerValue(256));
```

| Attribut | Valeur | Signification |
|----------|--------|---------------|
| `MaxPackets` | 999999 | Nombre quasi-illimité de paquets (continue pendant toute la simulation) |
| `Interval` | 0.1s | Envoie un paquet **toutes les 100ms** (10 paquets/seconde) |
| `PacketSize` | 256 octets | Taille de chaque paquet (petit, typique d'un message de contrôle drone) |

**Pourquoi 0.1s ?** Le commentaire dans le code l'explique :

> Interval must be < updateInterval (0.5s) to ensure FlowMonitor has fresh packets at every measurement tick. 0.2s → ~2-3 packets per update window, eliminating fallback to the 2.0ms constant.

Avec un intervalle de 0.1s et un `updateInterval` de 0.5s, il y a ~5 paquets entre chaque mesure. Le FlowMonitor a donc toujours des données fraîches et n'a jamais besoin du fallback.

### Timing

```cpp
serverApp.Start(Seconds(0.0));   // Serveurs prêts immédiatement
clientApp.Start(Seconds(1.0));   // Clients démarrent à t=1s
```

Les clients démarrent 1 seconde après les serveurs pour s'assurer que les serveurs sont prêts à recevoir.

---

## 14. FlowMonitor — métriques réseau réelles

```cpp
FlowMonitorHelper flowHelper;
g_flowMonitor = flowHelper.InstallAll();
g_flowClassifier = DynamicCast<Ipv4FlowClassifier>(flowHelper.GetClassifier());
```

### Qu'est-ce que le FlowMonitor ?

Le FlowMonitor est un outil NS-3 qui s'installe sur **tous les nœuds** du réseau et collecte des statistiques sur chaque flux IP :

| Statistique | Description |
|------------|-------------|
| `txPackets` | Nombre de paquets envoyés |
| `rxPackets` | Nombre de paquets reçus |
| `lostPackets` | Nombre de paquets perdus |
| `delaySum` | Somme totale des délais (bout-en-bout) |
| `jitterSum` | Somme totale de la gigue (variation du délai) |
| `txBytes` / `rxBytes` | Volume de données |

### Comment ça marche ?

```
Paquet envoyé par Drone 0 :

  [Émission]  t=10.000s  →  Drone 0 crée le paquet
  [PHY]       t=10.001s  →  Le paquet est mis en file d'attente MAC
  [MAC]       t=10.003s  →  Le canal est libre, le paquet est transmis
  [Canal]     t=10.003s  →  Le signal se propage (33ns pour 10m)
  [PHY]       t=10.003s  →  Drone 1 reçoit le paquet
  [MAC]       t=10.004s  →  Le paquet passe la couche MAC
  [IP]        t=10.004s  →  Le FlowMonitor enregistre l'arrivée
                             Délai = 10.004 - 10.000 = 4ms
```

Le FlowMonitor enregistre le **timestamps d'envoi** (quand le paquet entre dans la pile IP de l'émetteur) et le **timestamp de réception** (quand il sort de la pile IP du récepteur). La différence est le **délai bout-en-bout**.

### Statistiques finales

À la fin de la simulation :

```cpp
for (auto &iter : stats) {
    Ipv4FlowClassifier::FiveTuple t = g_flowClassifier->FindFlow(iter.first);
    double meanDelayMs = iter.second.delaySum.GetSeconds() * 1000.0
                          / (double)iter.second.rxPackets;
    NS_LOG_INFO("Flow " << iter.first
                << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")"
                << " Tx=" << iter.second.txPackets
                << " Rx=" << iter.second.rxPackets
                << " Lost=" << iter.second.lostPackets
                << " MeanDelay=" << meanDelayMs << "ms");
}
```

Ceci affiche un résumé de tous les flux :
```
Flow 1 (10.1.1.1 -> 10.1.1.2) Tx=590 Rx=585 Lost=5 MeanDelay=3.45ms
Flow 2 (10.1.1.2 -> 10.1.1.3) Tx=590 Rx=588 Lost=2 MeanDelay=2.87ms
Flow 3 (10.1.1.3 -> 10.1.1.1) Tx=590 Rx=580 Lost=10 MeanDelay=4.12ms
```

---

## 15. Mode temps réel

### Simulation classique vs temps réel

| Aspect | Simulation classique | Mode temps réel |
|--------|---------------------|-----------------|
| Horloge | Aussi vite que possible | Synchronisée avec le mur |
| 60s simulées | ~0.5s réelles | 60s réelles |
| Utilisation | Évaluation de performances | Couplage avec systèmes externes |
| Usage ici | — | **Oui** (couplé avec Gazebo) |

### Activation

```cpp
GlobalValue::Bind("SimulatorImplementationType",
                  StringValue("ns3::RealtimeSimulatorImpl"));
```

### Boucle de simulation

```cpp
Simulator::Schedule(Seconds(0.0), &UpdatePositions);
Simulator::Stop(Seconds(g_simTime));
Simulator::Run();
```

- `Schedule(0.0s)` : Le premier appel à `UpdatePositions()` est planifié à t=0
- `Stop(simTime)` : La simulation s'arrête après `simTime` secondes réelles
- `Run()` : Lance la boucle d'événements. **Cette fonction bloque** jusqu'à ce que `Simulator::Stop()` soit atteint

---

## 16. Format des fichiers d'entrée/sortie

### Fichier d'entrée : `/tmp/drone_positions.csv`

Écrit par le script Python, lu par NS-3.

```csv
drone_id,x,y,z
0,0.1000,0.0000,4.0000
1,0.2000,3.0000,5.0000
2,0.0000,6.0000,6.0000
```

| Colonne | Type | Description |
|---------|------|-------------|
| `drone_id` | int | Index du drone (0, 1, 2, ...) |
| `x` | float | Position X en mètres (système NED, Nord) |
| `y` | float | Position Y en mètres (système NED, Est) |
| `z` | float | Position Z en mètres (altitude, déjà positive) |

### Fichier de sortie : `/tmp/ns3_output.csv`

Écrit par NS-3, lu par le script Python.

```csv
time_s,drone_i,drone_j,distance_m,rssi_dbm,latency_ms,xi,yi,zi,xj,yj,zj
0.5,0,1,3.162,-54.5,3.21,0.100,0.000,4.000,0.200,3.000,5.000
0.5,0,2,6.325,-64.0,4.05,0.100,0.000,4.000,0.000,6.000,6.000
0.5,1,2,3.162,-54.5,3.33,0.200,3.000,5.000,0.000,6.000,6.000
1.0,0,1,3.170,-54.6,3.18,...
```

| Colonne | Type | Description |
|---------|------|-------------|
| `time_s` | float | Temps de simulation en secondes |
| `drone_i` | int | Index du premier drone de la paire |
| `drone_j` | int | Index du second drone de la paire |
| `distance_m` | float | Distance 3D entre les deux drones (mètres) |
| `rssi_dbm` | float | RSSI calculé par NS-3 (dBm) |
| `latency_ms` | float | Latence mesurée par FlowMonitor (ms) |
| `xi, yi, zi` | float | Position du drone i |
| `xj, yj, zj` | float | Position du drone j |

### Nombre de lignes par cycle

À chaque cycle (toutes les 0.5s), NS-3 écrit $\frac{N(N-1)}{2}$ lignes :

| N drones | Paires | Lignes/cycle | Lignes pour 60s |
|----------|--------|-------------|-----------------|
| 2 | 1 | 1 | 120 |
| 3 | 3 | 3 | 360 |
| 5 | 10 | 10 | 1200 |
| 10 | 45 | 45 | 5400 |

---

## 17. Arguments en ligne de commande

| Argument | Type | Défaut | Description |
|----------|------|--------|-------------|
| `--nDrones` | uint32 | 3 | Nombre de drones dans la simulation |
| `--posFile` | string | `/tmp/drone_positions.csv` | Fichier CSV d'entrée (positions) |
| `--outFile` | string | `/tmp/ns3_output.csv` | Fichier CSV de sortie (métriques) |
| `--simTime` | double | 60.0 | Durée de la simulation en secondes |
| `--updateInterval` | double | 0.5 | Intervalle de mise à jour (secondes) |
| `--channelModel` | string | `log-distance` | Modèle de canal : `log-distance` ou `sionna` |
| `--sionnaEnv` | string | `simple_room/simple_room.xml` | Scène 3D pour Sionna |
| `--sionnaUrl` | string | `tcp://localhost:5555` | URL du serveur ZMQ Sionna |

---

## 18. Flux d'exécution complet

```
t=0.0s  main() démarre
        ├── Parse arguments CLI
        ├── Active mode TEMPS RÉEL
        ├── Crée 3 nœuds (drones)
        │
        ├── Configure Wi-Fi 802.11n Ad-Hoc
        │   ├── PHY : TxPower = 20 dBm
        │   ├── Canal : YansWifiChannel
        │   ├── Propagation : LogDistance(n=3.0, PL0=40dB)
        │   │   (ou Sionna ray-tracing si --channelModel=sionna)
        │   └── MAC : AdhocWifiMac
        │
        ├── Installe pile Internet
        │   ├── Drone 0 → 10.1.1.1
        │   ├── Drone 1 → 10.1.1.2
        │   └── Drone 2 → 10.1.1.3
        │
        ├── Configure mobilité
        │   ├── ConstantPositionMobilityModel (ou SionnaMobilityModel)
        │   └── Positions initiales depuis CSV
        │
        ├── Configure Sionna (si activé)
        │   ├── SionnaHelper.Configure(freq, bw, fft, scs, 500)
        │   └── SionnaHelper.Start()
        │
        ├── Installe trafic UDP Echo
        │   ├── Drone 0 → Drone 1 (port 10, 0.1s, 256B)
        │   ├── Drone 1 → Drone 2 (port 11, 0.1s, 256B)
        │   └── Drone 2 → Drone 0 (port 9,  0.1s, 256B)
        │
        ├── Ouvre g_outputCsv (ns3_output.csv)
        ├── Installe FlowMonitor
        ├── Schedule UpdatePositions() à t=0
        │
        └── Simulator::Run() ← BOUCLE TEMPS RÉEL
            │
            ├── t=0.0s  UpdatePositions()
            │   ├── Lit drone_positions.csv
            │   ├── Met à jour positions des nœuds
            │   ├── CalcRxPower() pour chaque paire
            │   ├── GetPairLatencies() depuis FlowMonitor
            │   ├── Écrit ns3_output.csv
            │   └── Schedule(0.5s) → prochain appel
            │
            ├── t=0.5s  UpdatePositions()
            │   └── ... (même chose)
            │
            ├── t=1.0s  UpdatePositions() + premiers paquets UDP arrivent
            │   └── FlowMonitor a des données → latence réelle
            │
            ├── ... (chaque 0.5s pendant simTime secondes)
            │
            └── t=simTime → Simulator::Stop()

        Après Run() :
        ├── Affiche stats FlowMonitor finales
        ├── Ferme g_outputCsv
        ├── Détruit Sionna (si utilisé)
        ├── Simulator::Destroy()
        └── return 0
```

---

## 19. Interaction avec le script Python

Ce programme C++ ne tourne **jamais seul** en production. Il est toujours lancé par le script Python `12_flight_and_bridge.py`.

### Chronologie complète

```
Script Python (12_flight_and_bridge.py)          NS-3 (drone-wifi-scenario)
────────────────────────────────────────          ─────────────────────────
                                                  
1. Copie .cc dans scratch/
2. ./ns3 build (compile le C++)
3. Écrit positions initiales dans
   /tmp/drone_positions.csv
4. Lance NS-3 via subprocess.Popen(...)  ──────► 5. main() démarre
                                                  6. Lit positions initiales
                                                  7. Configure Wi-Fi, FlowMonitor
                                                  8. Simulator::Run()
                                                     │
9. Thread BRIDGE démarre                              │
   │                                                  │
   ├── Lit position drone via MAVLink                 │
   ├── Écrit /tmp/drone_positions.csv    ──────►      ├── UpdatePositions()
   │                                                  │   ├── Lit CSV
   │                                                  │   ├── CalcRxPower()
   │                                                  │   ├── FlowMonitor
   ├── Lit /tmp/ns3_output.csv           ◄──────      │   └── Écrit CSV
   ├── Écrit comm_metrics.csv                         │
   │                                                  │
   ├── (boucle toutes les 0.5s)                       ├── (boucle toutes les 0.5s)
   │                                                  │
   │                                                  │
10. Vol terminé                                       │
11. state.flight_done.set()                           │
   │                                                  │
12. bridge_thread nettoyage                           │
    └── ns3_process.terminate() ───────────────►    13. Simulation s'arrête
                                                     14. Stats finales
                                                     15. Nettoyage + return 0
```

### Mécanisme de communication

La communication entre Python et NS-3 se fait via **fichiers partagés** :

```
    Python                                NS-3
      │                                    │
      │  ──── drone_positions.csv ────►    │
      │       (positions des drones)       │
      │                                    │
      │  ◄──── ns3_output.csv ─────        │
      │       (RSSI, latence, dist)        │
      │                                    │
```

Ce mécanisme est simple mais efficace. Les deux processus lisent/écrivent dans des fichiers dans `/tmp/`. Les écritures atomiques sont assurées par `std::endl` (flush après chaque ligne) côté NS-3 et par le fallback côté Python (si le fichier est en cours d'écriture, Python ignore l'erreur).

---

## 20. NS3-Sionna — ray-tracing en détail

### Qu'est-ce que le ray-tracing ?

Le ray-tracing (tracé de rayons) est une technique qui simule le trajet physique des ondes radio dans un environnement 3D. Au lieu d'utiliser une formule simplifiée ($PL = PL_0 + 10n\log_{10}(d)$), le ray-tracing :

1. Lance des milliers de rayons depuis l'émetteur
2. Chaque rayon se propage en ligne droite
3. Quand un rayon frappe un mur, il est **réfléchi** et/ou **transmis**
4. Les rayons peuvent aussi être **diffractés** par les bords d'obstacles
5. Tous les rayons qui atteignent le récepteur sont combinés
6. Le résultat donne le RSSI et le délai **exacts** pour cette configuration

```
         Émetteur (Drone 0)
              │
    ┌─────────┼─────────────────┐
    │         │      Entrepôt   │
    │    ╱────┘                  │
    │   ╱  Rayon direct          │
    │  ╱                         │
    │ ╱          ┌──────┐       │
    │╱           │Étagère│       │
    ┤            │      │       │
    │╲           └──────┘       │
    │ ╲                         │
    │  ╲  Rayon réfléchi        │
    │   ╲────────────────╲      │
    │                     ╲     │
    │              Récepteur    │
    │              (Drone 1)    │
    └───────────────────────────┘
```

### Séquence d'initialisation Sionna dans le code

Le code suit exactement la séquence recommandée par NS3-Sionna :

```
Étape 1 : SionnaHelper(scène, url)          → Connexion au serveur
Étape 2 : SetMode(MODE_P2P)                 → Mode point-à-point
Étape 3 : SionnaPropagationCache()           → Cache des résultats
Étape 4 : SetSionnaHelper() + SetCaching()   → Lien cache ↔ helper
Étape 5 : SionnaPropagationLossModel()       → Modèle de perte
          SionnaPropagationDelayModel()       → Modèle de délai
Étape 6 : Attacher au YansWifiChannel        → Intégration avec Wi-Fi NS-3
Étape 7 : mobility.Install() + SetPosition() → Positions des nœuds
Étape 8 : Configure(freq, bw, fft, scs, coh) → Paramètres radio
Étape 9 : Start()                            → Envoi des infos au serveur
```

**ATTENTION à l'ordre** : `Start()` doit être appelé **après** `mobility.Install()` et `SetPosition()`. Le serveur Sionna a besoin de connaître les positions des nœuds au moment du `Start()`.

### Modes de fonctionnement Sionna

| Mode | Description | Utilisation ici |
|------|-------------|-----------------|
| `MODE_P2P` | Point-to-point, calcul indépendant par paire | **Oui** (choisi) |
| `MODE_P2MP_LAH` | Point-to-multipoint avec look-ahead | Non (instable) |

Le commentaire dans le code explique pourquoi `MODE_P2P` est préféré :

> MODE_P2MP_LAH (mode 3, défaut) plante quand sub_mode < nDrones-1, car look_ahead = floor(sub_mode / (nRx)) = 0 → aucun RX placé. P2P est plus stable et suffisant pour notre cas (RSSI + latence).

---

## 21. Exemples d'utilisation

### Test basique (sans le script Python)

```bash
cd ~/ns-allinone-3.40/ns-3.40

# Créer un fichier de positions test
echo "drone_id,x,y,z
0,0,0,5
1,0,10,5
2,10,0,5" > /tmp/drone_positions.csv

# Lancer la simulation (10 secondes)
./ns3 run "scratch/drone-wifi-scenario \
  --nDrones=3 \
  --simTime=10 \
  --updateInterval=1.0"

# Vérifier les résultats
cat /tmp/ns3_output.csv
```

### Avec le script Python (utilisation normale)

```bash
# Terminal 1 : Lancer les drones
./06_launch_multi_drones.sh

# Terminal 2 : Vol + Bridge (lance NS-3 automatiquement)
python3 12_flight_and_bridge.py --drones 3 --hover 30
```

### Avec Sionna (ray-tracing)

```bash
# Terminal 1 : Serveur Sionna
cd ~/ns-allinone-3.40/ns-3.40/contrib/sionna/model/ns3sionna
source sionna-venv/bin/activate
python3 run_server.py

# Terminal 2 : Lancer les drones
./06_launch_multi_drones.sh

# Terminal 3 : Vol + Bridge avec Sionna
python3 12_flight_and_bridge.py --drones 3 --use-sionna --hover 30
```

### Activer les logs NS-3

```bash
NS_LOG="DroneWifiScenario=info" ./ns3 run "scratch/drone-wifi-scenario --nDrones=3 --simTime=10"
```

Ceci active les messages `NS_LOG_INFO()` du code, affichant les détails de chaque mesure :
```
t=0.5 D0<->D1 dist=10.0m RSSI=-60.0dBm lat=3.21ms
t=0.5 D0<->D2 dist=10.0m RSSI=-60.0dBm lat=3.45ms
t=0.5 D1<->D2 dist=14.14m RSSI=-64.5dBm lat=4.01ms
```

---

## 22. Dépannage et erreurs courantes

### Erreur de compilation

```
error: 'SionnaHelper' was not declared in this scope
```

**Cause** : NS3-Sionna n'est pas installé mais `#define NS3_SIONNA_AVAILABLE` est actif.

**Solution** : Commenter la ligne dans le fichier :
```cpp
// #define NS3_SIONNA_AVAILABLE
```

Puis recompiler : `./ns3 build`

### NS-3 s'arrête immédiatement

```
ERREUR: NS-3 s'est arrêté immédiatement
```

**Causes possibles** :
1. Le fichier `/tmp/drone_positions.csv` n'existe pas
2. Erreur de compilation non détectée
3. Le mode temps réel échoue (système trop lent)

**Solution** :
```bash
# Tester manuellement
echo "drone_id,x,y,z
0,0,0,0" > /tmp/drone_positions.csv

cd ~/ns-allinone-3.40/ns-3.40
./ns3 run "scratch/drone-wifi-scenario --nDrones=1 --simTime=5" 2>&1
```

### Latence toujours à ~2ms (fallback)

Si le FlowMonitor ne rapporte jamais de données, toutes les latences seront le fallback (propagation + 2ms).

**Causes** :
- Les paquets UDP Echo n'arrivent pas (RSSI trop faible → perte totale)
- L'intervalle d'envoi est trop grand

**Solution** : Vérifier les stats finales. Si `rxPackets = 0`, le signal est trop faible. Rapprocher les drones ou augmenter `TxPowerDbm`.

### Erreur Sionna : "KeyError" ou "no nodes"

**Cause** : `Start()` a été appelé avant que les nœuds aient des positions.

**Solution** : C'est déjà géré dans le code (l'ordre est correct). Si le problème survient, vérifier que le serveur Sionna tourne AVANT de lancer NS-3.

### Fichier de sortie vide

**Cause** : NS-3 a crashé ou le `simTime` est trop court.

**Solution** :
```bash
# Vérifier que NS-3 tourne encore
ps aux | grep drone-wifi-scenario

# Vérifier le fichier
wc -l /tmp/ns3_output.csv
```

---

## 23. Glossaire

| Terme | Définition |
|-------|------------|
| **NS-3** | Network Simulator 3 — simulateur réseau à événements discrets |
| **YANS** | Yet Another Network Simulator — modèle Wi-Fi PHY/channel de NS-3 |
| **802.11n** | Wi-Fi 4 — standard à 2.4/5 GHz, jusqu'à 600 Mbps théorique |
| **Ad-Hoc** | Réseau Wi-Fi sans point d'accès, communication directe pair-à-pair |
| **MAC** | Medium Access Control — sous-couche qui gère l'accès au canal radio |
| **PHY** | Physical layer — couche physique (modulation, codage, puissance) |
| **RSSI** | Received Signal Strength Indicator — puissance du signal reçu (dBm) |
| **dBm** | Décibels-milliwatts — échelle logarithmique de puissance |
| **CalcRxPower()** | Méthode NS-3 qui calcule la puissance reçue entre deux nœuds |
| **PropagationLossModel** | Classe abstraite NS-3 pour les modèles de perte radio |
| **LogDistance** | Modèle de propagation basé sur $PL = PL_0 + 10n\log_{10}(d)$ |
| **FlowMonitor** | Outil NS-3 de collecte de statistiques sur les flux réseau |
| **FiveTuple** | Identifiant de flux : (IP src, IP dst, port src, port dst, proto) |
| **UDP Echo** | Application client/serveur : le serveur renvoie chaque paquet reçu |
| **NodeContainer** | Groupe de nœuds NS-3 |
| **NetDevice** | Interface réseau virtuelle (carte Wi-Fi) |
| **MobilityModel** | Modèle qui gère la position/mouvement d'un nœud |
| **Simulator::Schedule()** | Planifie un événement futur dans la file d'événements |
| **RealtimeSimulatorImpl** | Mode NS-3 synchronisé avec l'horloge système |
| **Sionna** | Framework NVIDIA de simulation radio basé sur le ray-tracing |
| **Ray-tracing** | Technique de simulation traçant le parcours physique des ondes |
| **ZMQ** | ZeroMQ — bibliothèque de messagerie inter-processus |
| **CSI** | Channel State Information — réponse complète du canal radio |
| **OFDM** | Multiplexage par division en fréquences orthogonales (Wi-Fi) |
| **FFT** | Fast Fourier Transform — algorithme utilisé par OFDM |
| **MCS** | Modulation and Coding Scheme — combinaison modulation + codage |
| **HtMcs7** | MCS le plus rapide en 802.11n (64-QAM, taux de codage 5/6) |
| **Ptr<>** | Pointeur intelligent NS-3 (reference counting, comme shared_ptr) |
| **DynamicCast** | Conversion de type sûre (vérifie le type à l'exécution) |
| **Delta** | Différence entre deux mesures consécutives |
| **Backoff** | Attente aléatoire avant retransmission (mécanisme Wi-Fi CSMA/CA) |
| **Flush** | Force l'écriture immédiate du buffer sur le disque |

---

## Résumé visuel final

```
┌───────────────────────────────────────────────────────────────────────┐
│                      drone-wifi-scenario.cc                           │
│                                                                       │
│   ENTRÉE                        TRAITEMENT                  SORTIE    │
│                                                                       │
│  ┌──────────────┐    ┌────────────────────────────┐   ┌────────────┐ │
│  │ drone_       │    │  NS-3 (temps réel)         │   │ ns3_       │ │
│  │ positions.csv│───►│                            │──►│ output.csv │ │
│  │              │    │  ┌─────────────────────┐   │   │            │ │
│  │ (positions   │    │  │ Wi-Fi 802.11n AdHoc │   │   │ (RSSI,     │ │
│  │  des drones) │    │  │                     │   │   │  latence,  │ │
│  └──────────────┘    │  │  Nœud 0  10.1.1.1  │   │   │  distance) │ │
│                      │  │  Nœud 1  10.1.1.2  │   │   └────────────┘ │
│  Écrit par Python    │  │  Nœud 2  10.1.1.3  │   │   Lu par Python  │
│  (bridge thread)     │  └─────────────────────┘   │   (bridge thread)│
│                      │                            │                   │
│                      │  ┌─────────────────────┐   │                   │
│                      │  │ Modèle propagation  │   │                   │
│                      │  │  • LogDistance       │   │                   │
│                      │  │  • ou Sionna (RT)   │   │                   │
│                      │  │  → CalcRxPower()    │   │                   │
│                      │  └─────────────────────┘   │                   │
│                      │                            │                   │
│                      │  ┌─────────────────────┐   │                   │
│                      │  │ FlowMonitor         │   │                   │
│                      │  │  → Latence réelle   │   │                   │
│                      │  │  → Paquets perdus   │   │                   │
│                      │  └─────────────────────┘   │                   │
│                      │                            │                   │
│                      │  ┌─────────────────────┐   │                   │
│                      │  │ UDP Echo (0.1s)     │   │                   │
│                      │  │  D0→D1→D2→D0       │   │                   │
│                      │  │  (trafic de mesure) │   │                   │
│                      │  └─────────────────────┘   │                   │
│                      └────────────────────────────┘                   │
└───────────────────────────────────────────────────────────────────────┘
```

---

*Documentation générée pour `drone-wifi-scenario.cc` — Scénario NS-3 pour réseau Wi-Fi ad-hoc entre drones*
