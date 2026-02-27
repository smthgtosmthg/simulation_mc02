/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * drone-5g-scenario.cc
 *
 * Scénario NS-3 + 5G-LENA : N drones communiquant via 5G NR
 *
 * Architecture :
 *   - 1 gNB (station de base au centre de l'entrepôt)
 *   - N drones comme UEs (User Equipment)
 *   - Lien 5G NR en bande FR1 (3.5 GHz) ou FR2 (28 GHz)
 *   - Modèle de canal 3GPP Indoor Office (InH)
 *
 * Ce programme :
 *   1. Lit les positions des drones depuis un fichier CSV (temps réel)
 *   2. Simule un réseau 5G NR entre la gNB et les drones
 *   3. Calcule le RSRP via le modèle de propagation 3GPP
 *   4. Mesure la latence réelle via FlowMonitor
 *   5. Écrit les résultats dans un fichier CSV de sortie
 *
 * Différences avec le scénario Wi-Fi (drone-wifi-scenario.cc) :
 *   - Infrastructure 5G (gNB + UEs) au lieu de Wi-Fi Ad-Hoc
 *   - Core Network (EPC) pour le routage inter-drones
 *   - Modèle de canal 3GPP TR 38.901 (au lieu de Log-Distance)
 *   - Scheduling OFDMA avec numerologie configurable
 *   - Beamforming avec antennes planaires (UPA)
 *
 * Compilation (depuis le répertoire ns-3) :
 *   cp drone-5g-scenario.cc scratch/
 *   ./ns3 build
 *
 * Exécution :
 *   ./ns3 run "scratch/drone-5g-scenario
 *     --nDrones=3
 *     --posFile=/tmp/drone_positions.csv
 *     --outFile=/tmp/ns3_5g_output.csv
 *     --simTime=60
 *     --frequency=3.5e9
 *     --bandwidth=20e6
 *     --numerology=1"
 *
 * Avec FR2 (mmWave 28 GHz) :
 *   ./ns3 run "scratch/drone-5g-scenario
 *     --nDrones=3 --frequency=28e9 --bandwidth=100e6 --numerology=3"
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/propagation-module.h"
#include "ns3/point-to-point-helper.h"

/* === Module 5G-LENA (NR) === */
#include "ns3/nr-module.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nr-mac-scheduler-tdma-rr.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-ue-net-device.h"
#include <ns3/antenna-module.h>

#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("Drone5gScenario");

// ============================================================
// Variables globales
// ============================================================

static uint32_t    g_nDrones       = 3;
static std::string g_posFile       = "/tmp/drone_positions.csv";
static std::string g_outFile       = "/tmp/ns3_5g_output.csv";
static double      g_simTime       = 60.0;
static double      g_updateInterval = 0.5;
static double      g_frequency     = 3.5e9;    // FR1 : 3.5 GHz (par défaut)
static double      g_bandwidth     = 20e6;     // 20 MHz
static uint32_t    g_numerology    = 1;         // μ=1 → 30 kHz SCS
static std::string g_scenario      = "InH-OfficeMixed";  // 3GPP scenario

// Position de la gNB (centre de l'entrepôt, surélevée)
static double g_gnbX = 15.0;  // Centre X du warehouse (30m / 2)
static double g_gnbY = 10.0;  // Centre Y du warehouse (20m / 2)
static double g_gnbZ = 3.0;   // Hauteur de la gNB (plafond à 6m)

// Puissance d'émission
static double g_gnbTxPowerDbm = 30.0;  // gNB indoor small cell
static double g_ueTxPowerDbm  = 23.0;  // UE (drone)

// Nœuds
static NodeContainer g_gnbNodes;
static NodeContainer g_ueNodes;       // drones
static NetDeviceContainer g_gnbDevices;
static NetDeviceContainer g_ueDevices;

// Fichier de sortie
static std::ofstream g_outputCsv;

// Modèle de propagation 3GPP pour calcul direct du RSRP
static Ptr<PropagationLossModel> g_propLossModel;

// FlowMonitor pour la latence réelle
static Ptr<FlowMonitor>          g_flowMonitor;
static Ptr<Ipv4FlowClassifier>   g_flowClassifier;
static FlowMonitor::FlowStatsContainer g_prevFlowStats;

// ============================================================
// Structure pour les positions des drones
// ============================================================
struct DronePos {
    double x, y, z;
};

// ============================================================
// Lecture des positions depuis le CSV
// Format : drone_id,x,y,z (une ligne par drone)
// ============================================================
static std::vector<DronePos>
ReadPositions(const std::string& filename, uint32_t nDrones)
{
    std::vector<DronePos> positions(nDrones, {0.0, 0.0, 0.0});
    std::ifstream file(filename);
    if (!file.is_open()) {
        NS_LOG_WARN("Cannot open position file: " << filename);
        return positions;
    }

    std::string line;
    // Sauter l'en-tête
    if (std::getline(file, line)) {
        // header line
    }

    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string token;
        uint32_t id;
        double x, y, z;

        std::getline(ss, token, ','); id = std::stoi(token);
        std::getline(ss, token, ','); x = std::stod(token);
        std::getline(ss, token, ','); y = std::stod(token);
        std::getline(ss, token, ','); z = std::stod(token);

        if (id < nDrones) {
            positions[id] = {x, y, z};
        }
    }
    return positions;
}

// ============================================================
// Extraction de la latence par paire depuis le FlowMonitor
// ============================================================
static std::map<std::pair<uint32_t, uint32_t>, double>
GetFlowLatencies(void)
{
    std::map<std::pair<uint32_t, uint32_t>, double> result;

    if (!g_flowMonitor || !g_flowClassifier) {
        return result;
    }

    g_flowMonitor->CheckForLostPackets();
    FlowMonitor::FlowStatsContainer stats = g_flowMonitor->GetFlowStats();

    for (auto& iter : stats) {
        Ipv4FlowClassifier::FiveTuple t = g_flowClassifier->FindFlow(iter.first);

        // Les IPs des UEs sont attribuées par l'EPC (typiquement 7.0.0.x)
        // On extrait l'octet de poids faible pour mapper au drone
        uint32_t srcByte = t.sourceAddress.Get() & 0xFF;
        uint32_t dstByte = t.destinationAddress.Get() & 0xFF;

        if (srcByte == 0 || dstByte == 0) continue;
        uint32_t srcDrone = srcByte - 1;
        uint32_t dstDrone = dstByte - 1;
        if (srcDrone >= g_nDrones || dstDrone >= g_nDrones) continue;

        // Calcul du délai récent (delta)
        double delayMs = 0.0;
        auto prevIt = g_prevFlowStats.find(iter.first);
        if (prevIt != g_prevFlowStats.end()) {
            uint64_t newPkts = iter.second.rxPackets - prevIt->second.rxPackets;
            if (newPkts > 0) {
                Time newDelay = iter.second.delaySum - prevIt->second.delaySum;
                delayMs = newDelay.GetSeconds() * 1000.0 / (double)newPkts;
            }
        } else if (iter.second.rxPackets > 0) {
            delayMs = iter.second.delaySum.GetSeconds() * 1000.0
                      / (double)iter.second.rxPackets;
        }

        // Paire ordonnée (min, max)
        auto key = std::make_pair(std::min(srcDrone, dstDrone),
                                   std::max(srcDrone, dstDrone));
        if (delayMs > 0.0) {
            auto existing = result.find(key);
            if (existing == result.end() || delayMs < existing->second) {
                result[key] = delayMs;
            }
        }
    }

    g_prevFlowStats = stats;
    return result;
}

// ============================================================
// Mise à jour périodique des positions + calcul des métriques
// ============================================================
static void
UpdatePositions(void)
{
    // 1. Lire les positions depuis le CSV
    std::vector<DronePos> positions = ReadPositions(g_posFile, g_nDrones);

    // 2. Mettre à jour les modèles de mobilité des UEs
    for (uint32_t i = 0; i < g_nDrones; ++i) {
        Ptr<MobilityModel> mob = g_ueNodes.Get(i)->GetObject<MobilityModel>();
        if (mob) {
            mob->SetPosition(Vector(positions[i].x, positions[i].y, positions[i].z));
        }
    }

    double simTime = Simulator::Now().GetSeconds();

    // 3. Récupérer la position de la gNB
    Ptr<MobilityModel> gnbMob = g_gnbNodes.Get(0)->GetObject<MobilityModel>();

    // 4. Latence réelle depuis le FlowMonitor
    auto pairLatencies = GetFlowLatencies();

    // 5. Calculer les métriques pour chaque paire de drones
    for (uint32_t i = 0; i < g_nDrones; ++i) {
        for (uint32_t j = i + 1; j < g_nDrones; ++j) {
            // --- Distance entre drones ---
            double dx = positions[i].x - positions[j].x;
            double dy = positions[i].y - positions[j].y;
            double dz = positions[i].z - positions[j].z;
            double distanceDrones = std::sqrt(dx * dx + dy * dy + dz * dz);

            // --- RSRP (dBm) : puissance reçue de la gNB par chaque drone ---
            // En 5G, la communication inter-drones passe par la gNB.
            // Le lien le plus faible (bottleneck) détermine la qualité.
            Ptr<MobilityModel> mob_i = g_ueNodes.Get(i)->GetObject<MobilityModel>();
            Ptr<MobilityModel> mob_j = g_ueNodes.Get(j)->GetObject<MobilityModel>();

            // RSRP = TxPower_gNB - PathLoss(gNB → UE)
            double rsrp_i = g_propLossModel->CalcRxPower(g_gnbTxPowerDbm, gnbMob, mob_i);
            double rsrp_j = g_propLossModel->CalcRxPower(g_gnbTxPowerDbm, gnbMob, mob_j);

            // Le RSRP du lien inter-drones = min(RSRP_i, RSRP_j)
            // car le signal doit traverser les deux liens (UE→gNB→UE)
            double rsrpPair = std::min(rsrp_i, rsrp_j);

            // --- Distance drone → gNB ---
            double distGnb_i = mob_i->GetDistanceFrom(gnbMob);
            double distGnb_j = mob_j->GetDistanceFrom(gnbMob);

            // --- Latence ---
            double latencyMs = 0.0;
            auto pairKey = std::make_pair(i, j);
            auto latIt = pairLatencies.find(pairKey);
            if (latIt != pairLatencies.end() && latIt->second > 0.0) {
                latencyMs = latIt->second;
            } else {
                // Fallback 5G NR : ~1ms (slot duration) + propagation + scheduling
                double propagationMs = (distanceDrones / 3e8) * 1000.0;
                latencyMs = propagationMs + 1.0;  // 1ms TTI typique en 5G NR
            }

            // --- Écriture CSV ---
            // Format : time_s,drone_i,drone_j,distance_m,rsrp_dbm,
            //          latency_ms,dist_gnb_i,dist_gnb_j,rsrp_i,rsrp_j,
            //          xi,yi,zi,xj,yj,zj
            g_outputCsv << simTime << ","
                       << i << "," << j << ","
                       << distanceDrones << ","
                       << rsrpPair << ","
                       << latencyMs << ","
                       << distGnb_i << "," << distGnb_j << ","
                       << rsrp_i << "," << rsrp_j << ","
                       << positions[i].x << "," << positions[i].y << "," << positions[i].z << ","
                       << positions[j].x << "," << positions[j].y << "," << positions[j].z
                       << std::endl;

            NS_LOG_INFO("t=" << simTime
                       << " D" << i << "<->D" << j
                       << " dist=" << distanceDrones << "m"
                       << " RSRP_pair=" << rsrpPair << "dBm"
                       << " RSRP_i=" << rsrp_i << "dBm"
                       << " RSRP_j=" << rsrp_j << "dBm"
                       << " lat=" << latencyMs << "ms"
                       << " d_gNB_i=" << distGnb_i << "m"
                       << " d_gNB_j=" << distGnb_j << "m");
        }
    }

    // Prochain update
    Simulator::Schedule(Seconds(g_updateInterval), &UpdatePositions);
}

// ============================================================
// Fonction main
// ============================================================
int
main(int argc, char *argv[])
{
    // --- Arguments en ligne de commande ---
    CommandLine cmd;
    cmd.AddValue("nDrones",        "Nombre de drones (UEs)",          g_nDrones);
    cmd.AddValue("posFile",        "Fichier CSV des positions",       g_posFile);
    cmd.AddValue("outFile",        "Fichier CSV de sortie",           g_outFile);
    cmd.AddValue("simTime",        "Durée de simulation (s)",         g_simTime);
    cmd.AddValue("updateInterval", "Intervalle de MAJ positions (s)", g_updateInterval);
    cmd.AddValue("frequency",      "Fréquence centrale (Hz)",         g_frequency);
    cmd.AddValue("bandwidth",      "Bande passante (Hz)",             g_bandwidth);
    cmd.AddValue("numerology",     "Numerologie NR (0-4)",            g_numerology);
    cmd.AddValue("scenario",       "Scénario 3GPP (InH-OfficeMixed, InH-OfficeOpen, UMi)", g_scenario);
    cmd.AddValue("gnbX",           "Position X de la gNB (m)",        g_gnbX);
    cmd.AddValue("gnbY",           "Position Y de la gNB (m)",        g_gnbY);
    cmd.AddValue("gnbZ",           "Position Z de la gNB (m)",        g_gnbZ);
    cmd.AddValue("gnbTxPower",     "Puissance gNB (dBm)",             g_gnbTxPowerDbm);
    cmd.AddValue("ueTxPower",      "Puissance UE/drone (dBm)",        g_ueTxPowerDbm);
    cmd.Parse(argc, argv);

    // *** MODE TEMPS RÉEL ***
    GlobalValue::Bind("SimulatorImplementationType",
                      StringValue("ns3::RealtimeSimulatorImpl"));

    NS_LOG_INFO("=== Drone 5G NR Scenario (5G-LENA) ===");
    NS_LOG_INFO("Drones (UEs) : " << g_nDrones);
    NS_LOG_INFO("Fréquence    : " << g_frequency / 1e9 << " GHz");
    NS_LOG_INFO("Bande passante: " << g_bandwidth / 1e6 << " MHz");
    NS_LOG_INFO("Numérologie  : μ=" << g_numerology
               << " (SCS=" << (15 * (1 << g_numerology)) << " kHz)");
    NS_LOG_INFO("Scénario 3GPP: " << g_scenario);
    NS_LOG_INFO("gNB position : (" << g_gnbX << ", " << g_gnbY << ", " << g_gnbZ << ")");
    NS_LOG_INFO("Fichier pos  : " << g_posFile);
    NS_LOG_INFO("Fichier out  : " << g_outFile);
    NS_LOG_INFO("Sim time     : " << g_simTime << "s (real-time)");

    // ============================================================
    // 1. Créer les nœuds
    // ============================================================
    g_gnbNodes.Create(1);     // 1 gNB (station de base)
    g_ueNodes.Create(g_nDrones);  // N drones comme UEs

    // ============================================================
    // 2. Configurer 5G-LENA (NR Helper + EPC)
    // ============================================================

    // --- EPC (Evolved Packet Core) ---
    // Fournit le réseau cœur (SGW, PGW) pour le routage inter-UEs
    Ptr<NrPointToPointEpcHelper> epcHelper =
        CreateObject<NrPointToPointEpcHelper>();

    // --- Beamforming idéal ---
    Ptr<IdealBeamformingHelper> idealBeamformingHelper =
        CreateObject<IdealBeamformingHelper>();
    // DirectPathBeamforming = beamforming optimal vers la direction LOS
    idealBeamformingHelper->SetAttribute(
        "BeamformingMethod",
        TypeIdValue(DirectPathBeamforming::GetTypeId()));

    // --- NR Helper principal ---
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(epcHelper);

    // ============================================================
    // 3. Configurer le spectre (bande + BWP)
    // ============================================================

    // Déterminer le scénario 3GPP pour le modèle de canal
    BandwidthPartInfo::Scenario bwpScenario;
    if (g_scenario == "InH-OfficeOpen") {
        bwpScenario = BandwidthPartInfo::InH_OfficeOpen;
    } else if (g_scenario == "InH-OfficeMixed") {
        bwpScenario = BandwidthPartInfo::InH_OfficeMixed;
    } else if (g_scenario == "UMi") {
        bwpScenario = BandwidthPartInfo::UMi_StreetCanyon;
    } else if (g_scenario == "UMa") {
        bwpScenario = BandwidthPartInfo::UMa;
    } else if (g_scenario == "RMa") {
        bwpScenario = BandwidthPartInfo::RMa;
    } else {
        NS_LOG_WARN("Scénario inconnu: " << g_scenario << ", fallback InH-OfficeMixed");
        bwpScenario = BandwidthPartInfo::InH_OfficeMixed;
    }

    // Créer la bande d'opération
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;  // 1 Component Carrier

    CcBwpCreator::SimpleOperationBandConf bandConf(
        g_frequency,     // Fréquence centrale (Hz)
        g_bandwidth,     // Bande passante (Hz)
        numCcPerBand,    // Nombre de CC
        bwpScenario      // Scénario de canal 3GPP
    );

    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    // Configurer les paramètres du scheduler
    nrHelper->SetSchedulerTypeId(NrMacSchedulerTdmaRR::GetTypeId());

    // Configurer les puissances de transmission
    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(g_gnbTxPowerDbm));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(g_ueTxPowerDbm));

    // Configurer la numérologie
    nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(g_numerology));

    // Configurer les antennes
    // gNB : 4x2 UPA (Uniform Planar Array) = 8 éléments
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(2));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
        PointerValue(CreateObject<IsotropicAntennaModel>()));

    // UE (drone) : 1x1 = antenne omnidirectionnelle
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
        PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Désactiver le shadowing pour des résultats déterministes
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));

    // Initialiser la bande
    nrHelper->InitializeOperationBand(&band);

    // Récupérer toutes les BWPs
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    // ============================================================
    // 4. Mobilité (AVANT InstallDevice — requis par NR Helper)
    // ============================================================

    // gNB : position fixe au centre de l'entrepôt
    MobilityHelper gnbMobility;
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gnbMobility.Install(g_gnbNodes);
    g_gnbNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(
        Vector(g_gnbX, g_gnbY, g_gnbZ));

    // UEs (drones) : positions lues depuis le CSV
    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    ueMobility.Install(g_ueNodes);

    // Positions initiales
    std::vector<DronePos> initPos = ReadPositions(g_posFile, g_nDrones);
    for (uint32_t i = 0; i < g_nDrones; ++i) {
        Ptr<MobilityModel> mob = g_ueNodes.Get(i)->GetObject<MobilityModel>();
        mob->SetPosition(Vector(initPos[i].x, initPos[i].y, initPos[i].z));
    }

    // ============================================================
    // 5. Installer les dispositifs NR
    // ============================================================
    g_gnbDevices = nrHelper->InstallGnbDevice(g_gnbNodes, allBwps);
    g_ueDevices  = nrHelper->InstallUeDevice(g_ueNodes, allBwps);

    // Mettre à jour les configurations (per-device UpdateConfig)
    for (auto it = g_gnbDevices.Begin(); it != g_gnbDevices.End(); ++it) {
        DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
    }
    for (auto it = g_ueDevices.Begin(); it != g_ueDevices.End(); ++it) {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }

    // ============================================================
    // 6. Pile Internet + adresses IP
    // ============================================================

    // Créer un Remote Host connecté au PGW (nécessaire pour le routage)
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);

    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // Lien P2P entre PGW et Remote Host
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.000)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

    Ipv4AddressHelper ipv4h;
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

    // Route vers le réseau UE (7.0.0.0/8)
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(
        Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // Installer la pile Internet sur les UEs
    internet.Install(g_ueNodes);

    // Assigner les IPs aux UEs via l'EPC
    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(g_ueDevices);

    // Route par défaut pour chaque UE → passerelle EPC
    for (uint32_t i = 0; i < g_nDrones; ++i) {
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(
                g_ueNodes.Get(i)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(
            epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // Attacher les UEs à la gNB la plus proche
    nrHelper->AttachToClosestEnb(g_ueDevices, g_gnbDevices);

    // ============================================================
    // 7. Modèle de propagation pour calcul direct du RSRP
    // ============================================================
    // On crée un modèle 3GPP séparé pour CalcRxPower(),
    // indépendant du pipeline interne de 5G-LENA.
    // Cela nous donne le RSRP de référence pour chaque lien.

    Ptr<ChannelConditionModel> condModel;
    if (g_scenario == "InH-OfficeOpen") {
        condModel = CreateObject<ThreeGppIndoorOpenOfficeChannelConditionModel>();
    } else if (g_scenario == "InH-OfficeMixed") {
        condModel = CreateObject<ThreeGppIndoorMixedOfficeChannelConditionModel>();
    } else {
        // Fallback pour outdoor : UMi
        condModel = CreateObject<ThreeGppUmiStreetCanyonChannelConditionModel>();
    }

    Ptr<ThreeGppPropagationLossModel> propLoss;
    if (g_scenario.find("InH") != std::string::npos) {
        propLoss = CreateObject<ThreeGppIndoorOfficePropagationLossModel>();
    } else if (g_scenario == "UMi") {
        propLoss = CreateObject<ThreeGppUmiStreetCanyonPropagationLossModel>();
    } else if (g_scenario == "UMa") {
        propLoss = CreateObject<ThreeGppUmaPropagationLossModel>();
    } else if (g_scenario == "RMa") {
        propLoss = CreateObject<ThreeGppRmaPropagationLossModel>();
    } else {
        propLoss = CreateObject<ThreeGppIndoorOfficePropagationLossModel>();
    }

    propLoss->SetAttribute("Frequency", DoubleValue(g_frequency));
    propLoss->SetAttribute("ShadowingEnabled", BooleanValue(false));
    propLoss->SetChannelConditionModel(condModel);
    g_propLossModel = propLoss;

    // ============================================================
    // 8. Trafic UDP entre drones (via EPC)
    // ============================================================
    // On installe un serveur UDP Echo sur chaque drone et un client
    // vers le drone suivant. Le trafic passe par : UE→gNB→EPC→gNB→UE
    // FlowMonitor mesure la latence bout-en-bout réelle.

    uint16_t port = 5000;

    for (uint32_t i = 0; i < g_nDrones; ++i) {
        uint32_t j = (i + 1) % g_nDrones;

        Ipv4Address dstAddr = ueIpIface.GetAddress(j);

        // Serveur sur le drone destination
        UdpEchoServerHelper echoServer(port + i);
        ApplicationContainer serverApp = echoServer.Install(g_ueNodes.Get(j));
        serverApp.Start(Seconds(0.5));
        serverApp.Stop(Seconds(g_simTime));

        // Client sur le drone source
        UdpEchoClientHelper echoClient(dstAddr, port + i);
        echoClient.SetAttribute("MaxPackets", UintegerValue(999999));
        echoClient.SetAttribute("Interval", TimeValue(Seconds(0.1)));
        echoClient.SetAttribute("PacketSize", UintegerValue(256));

        ApplicationContainer clientApp = echoClient.Install(g_ueNodes.Get(i));
        clientApp.Start(Seconds(1.0));
        clientApp.Stop(Seconds(g_simTime));
    }

    // ============================================================
    // 9. Ouvrir le CSV de sortie
    // ============================================================
    g_outputCsv.open(g_outFile);
    g_outputCsv << "time_s,drone_i,drone_j,distance_m,rsrp_dbm,"
                << "latency_ms,dist_gnb_i,dist_gnb_j,rsrp_i,rsrp_j,"
                << "xi,yi,zi,xj,yj,zj"
                << std::endl;

    // ============================================================
    // 10. FlowMonitor pour la latence réelle
    // ============================================================
    FlowMonitorHelper flowHelper;
    g_flowMonitor    = flowHelper.InstallAll();
    g_flowClassifier = DynamicCast<Ipv4FlowClassifier>(flowHelper.GetClassifier());

    // ============================================================
    // 11. Planifier les mises à jour périodiques
    // ============================================================
    Simulator::Schedule(Seconds(2.0), &UpdatePositions);

    // ============================================================
    // 12. Lancer la simulation
    // ============================================================
    Simulator::Stop(Seconds(g_simTime));
    Simulator::Run();

    // ============================================================
    // 13. Statistiques finales
    // ============================================================
    NS_LOG_INFO("\n=== Statistiques FlowMonitor (5G NR) ===");
    g_flowMonitor->CheckForLostPackets();
    FlowMonitor::FlowStatsContainer stats = g_flowMonitor->GetFlowStats();

    for (auto &iter : stats) {
        Ipv4FlowClassifier::FiveTuple t = g_flowClassifier->FindFlow(iter.first);
        double meanDelayMs = 0;
        if (iter.second.rxPackets > 0) {
            meanDelayMs = iter.second.delaySum.GetSeconds() * 1000.0
                          / (double)iter.second.rxPackets;
        }
        NS_LOG_INFO("Flow " << iter.first
                    << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")"
                    << " Tx=" << iter.second.txPackets
                    << " Rx=" << iter.second.rxPackets
                    << " Lost=" << iter.second.lostPackets
                    << " MeanDelay=" << meanDelayMs << "ms");
    }

    g_outputCsv.close();
    Simulator::Destroy();

    NS_LOG_INFO("5G output saved to: " << g_outFile);
    return 0;
}
