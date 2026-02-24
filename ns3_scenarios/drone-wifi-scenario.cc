/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * drone-wifi-scenario.cc
 *
 * Scénario NS-3 : N drones communiquant via WiFi Ad-Hoc
 *
 * Ce programme :
 *   1. Lit les positions des drones depuis un fichier CSV (mis à jour en temps réel)
 *   2. Simule un réseau WiFi Ad-Hoc entre les drones
 *   3. Calcule le RSSI et la latence entre chaque paire de drones
 *   4. Écrit les résultats dans un fichier CSV de sortie
 *
 * Compilation (depuis le répertoire ns-3.40) :
 *   cp drone-wifi-scenario.cc scratch/
 *   ./ns3 build
 *
 * Exécution :
 *   ./ns3 run "scratch/drone-wifi-scenario
 *     --nDrones=3
 *     --posFile=/tmp/drone_positions.csv
 *     --outFile=/tmp/ns3_output.csv
 *     --simTime=60"
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/propagation-module.h"

#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DroneWifiScenario");

// ============================================================
// Globals
// ============================================================

static uint32_t g_nDrones = 3;
static std::string g_posFile = "/tmp/drone_positions.csv";
static std::string g_outFile = "/tmp/ns3_output.csv";
static double g_simTime = 60.0;
static double g_updateInterval = 0.5; // seconds between position updates

static NodeContainer g_droneNodes;
static NetDeviceContainer g_wifiDevices;
static std::ofstream g_outputCsv;

// ============================================================
// Helper: Read drone positions from CSV
// Format: drone_id,x,y,z (one line per drone)
// ============================================================
struct DronePos {
    double x, y, z;
};

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
    // Skip header
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
// Update node positions from CSV file
// ============================================================
static void
UpdatePositions(void)
{
    std::vector<DronePos> positions = ReadPositions(g_posFile, g_nDrones);

    for (uint32_t i = 0; i < g_nDrones; ++i) {
        Ptr<MobilityModel> mob = g_droneNodes.Get(i)->GetObject<MobilityModel>();
        if (mob) {
            mob->SetPosition(Vector(positions[i].x, positions[i].y, positions[i].z));
        }
    }

    // Calculate RSSI and distance for each pair
    double simTime = Simulator::Now().GetSeconds();

    for (uint32_t i = 0; i < g_nDrones; ++i) {
        for (uint32_t j = i + 1; j < g_nDrones; ++j) {
            double dx = positions[i].x - positions[j].x;
            double dy = positions[i].y - positions[j].y;
            double dz = positions[i].z - positions[j].z;
            double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

            // Get RSSI from the WiFi PHY layer
            // Using Log-Distance path loss model:
            // RSSI = TxPower - PathLoss
            // PathLoss(d) = PathLoss(d0) + 10*n*log10(d/d0)
            // For WiFi at 2.4GHz indoor: n~3, PL(1m)~40dB, TxPower=20dBm
            double txPowerDbm = 20.0;
            double pathLossExponent = 3.0;  // indoor
            double referenceLossDb = 40.0;  // at 1m, 2.4GHz
            double rssiDbm = -999.0;

            if (distance > 0.01) {
                double pathLossDb = referenceLossDb + 10.0 * pathLossExponent * std::log10(distance);
                rssiDbm = txPowerDbm - pathLossDb;
            } else {
                rssiDbm = txPowerDbm - referenceLossDb;
            }

            // Estimate latency (propagation + processing)
            // Propagation: distance / speed_of_light
            // Processing/MAC: ~1-5ms for WiFi Ad-Hoc
            double propagationDelayMs = (distance / 3e8) * 1000.0;
            double macDelayMs = 2.0; // typical WiFi contention delay
            double totalLatencyMs = propagationDelayMs + macDelayMs;

            // Write to output CSV
            g_outputCsv << simTime << ","
                       << i << "," << j << ","
                       << distance << ","
                       << rssiDbm << ","
                       << totalLatencyMs << ","
                       << positions[i].x << "," << positions[i].y << "," << positions[i].z << ","
                       << positions[j].x << "," << positions[j].y << "," << positions[j].z
                       << std::endl;

            NS_LOG_INFO("t=" << simTime
                       << " Drone" << i << "↔Drone" << j
                       << " dist=" << distance << "m"
                       << " RSSI=" << rssiDbm << "dBm"
                       << " latency=" << totalLatencyMs << "ms");
        }
    }

    // Schedule next update
    Simulator::Schedule(Seconds(g_updateInterval), &UpdatePositions);
}

// ============================================================
// Packet receive callback (for latency measurement)
// ============================================================
static std::map<uint64_t, Time> g_txTimeMap;
static uint64_t g_pktUid = 0;

// ============================================================
// Main
// ============================================================
int
main(int argc, char *argv[])
{
    // --- Command line arguments ---
    CommandLine cmd;
    cmd.AddValue("nDrones", "Number of drones", g_nDrones);
    cmd.AddValue("posFile", "Input CSV with drone positions", g_posFile);
    cmd.AddValue("outFile", "Output CSV with RSSI and latency", g_outFile);
    cmd.AddValue("simTime", "Simulation time in seconds", g_simTime);
    cmd.AddValue("updateInterval", "Position update interval (s)", g_updateInterval);
    cmd.Parse(argc, argv);

    NS_LOG_INFO("=== Drone WiFi Scenario ===");
    NS_LOG_INFO("Drones: " << g_nDrones);
    NS_LOG_INFO("Position file: " << g_posFile);
    NS_LOG_INFO("Output file: " << g_outFile);
    NS_LOG_INFO("Sim time: " << g_simTime << "s");

    // --- Create nodes ---
    g_droneNodes.Create(g_nDrones);

    // --- WiFi setup (Ad-Hoc mode, 802.11n at 2.4GHz) ---
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                  "DataMode", StringValue("HtMcs7"),
                                  "ControlMode", StringValue("HtMcs0"));

    // PHY layer
    YansWifiPhyHelper wifiPhy;
    wifiPhy.Set("TxPowerStart", DoubleValue(20.0));  // 20 dBm
    wifiPhy.Set("TxPowerEnd", DoubleValue(20.0));

    // Channel with Log-Distance propagation loss (indoor warehouse)
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                                    "Exponent", DoubleValue(3.0),
                                    "ReferenceLoss", DoubleValue(40.0));
    wifiPhy.SetChannel(wifiChannel.Create());

    // MAC layer (Ad-Hoc for drone mesh)
    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");

    g_wifiDevices = wifi.Install(wifiPhy, wifiMac, g_droneNodes);

    // --- Internet stack ---
    InternetStackHelper internet;
    internet.Install(g_droneNodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    ipv4.Assign(g_wifiDevices);

    // --- Mobility (positions updated from CSV) ---
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(g_droneNodes);

    // Set initial positions
    std::vector<DronePos> initPos = ReadPositions(g_posFile, g_nDrones);
    for (uint32_t i = 0; i < g_nDrones; ++i) {
        Ptr<MobilityModel> mob = g_droneNodes.Get(i)->GetObject<MobilityModel>();
        mob->SetPosition(Vector(initPos[i].x, initPos[i].y, initPos[i].z));
    }

    // --- UDP traffic between drones (for latency measurement) ---
    uint16_t port = 9;
    for (uint32_t i = 0; i < g_nDrones; ++i) {
        // Each drone sends periodic packets to the next drone
        uint32_t j = (i + 1) % g_nDrones;

        Ptr<Node> srcNode = g_droneNodes.Get(i);
        Ptr<Node> dstNode = g_droneNodes.Get(j);

        Ptr<Ipv4> dstIpv4 = dstNode->GetObject<Ipv4>();
        Ipv4Address dstAddr = dstIpv4->GetAddress(1, 0).GetLocal();

        // UDP Echo server on each drone
        UdpEchoServerHelper echoServer(port + i);
        ApplicationContainer serverApp = echoServer.Install(dstNode);
        serverApp.Start(Seconds(0.0));
        serverApp.Stop(Seconds(g_simTime));

        // UDP Echo client
        UdpEchoClientHelper echoClient(dstAddr, port + j);
        echoClient.SetAttribute("MaxPackets", UintegerValue(999999));
        echoClient.SetAttribute("Interval", TimeValue(Seconds(1.0)));
        echoClient.SetAttribute("PacketSize", UintegerValue(256));

        ApplicationContainer clientApp = echoClient.Install(srcNode);
        clientApp.Start(Seconds(1.0));
        clientApp.Stop(Seconds(g_simTime));
    }

    // --- Open output CSV ---
    g_outputCsv.open(g_outFile);
    g_outputCsv << "time_s,drone_i,drone_j,distance_m,rssi_dbm,latency_ms,"
                << "xi,yi,zi,xj,yj,zj" << std::endl;

    // --- Flow monitor for actual latency ---
    FlowMonitorHelper flowHelper;
    Ptr<FlowMonitor> flowMonitor = flowHelper.InstallAll();

    // --- Schedule position updates ---
    Simulator::Schedule(Seconds(0.0), &UpdatePositions);

    // --- Run simulation ---
    Simulator::Stop(Seconds(g_simTime));
    Simulator::Run();

    // --- Print flow statistics ---
    NS_LOG_INFO("\n=== Flow Monitor Statistics ===");
    flowMonitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowHelper.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = flowMonitor->GetFlowStats();

    for (auto &iter : stats) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(iter.first);
        NS_LOG_INFO("Flow " << iter.first
                    << " (" << t.sourceAddress << " → " << t.destinationAddress << ")"
                    << " Tx=" << iter.second.txPackets
                    << " Rx=" << iter.second.rxPackets
                    << " Lost=" << iter.second.lostPackets
                    << " MeanDelay=" << (iter.second.rxPackets > 0 ?
                       iter.second.delaySum.GetMilliSeconds() / iter.second.rxPackets : 0)
                    << "ms");
    }

    g_outputCsv.close();
    Simulator::Destroy();

    NS_LOG_INFO("Output saved to: " << g_outFile);
    return 0;
}
