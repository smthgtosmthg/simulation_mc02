#!/bin/bash
###############################################################################
# 07_ns3_rssi_latency_example.sh
# Crée et lance un exemple NS-3 qui mesure RSSI + latence entre 2 drones
# C'est le livrable principal demandé dans l'email
###############################################################################
set -e

export WORKDIR="$HOME/drone_simulation"
NS3_DIR="$WORKDIR/ns-allinone-3.40/ns-3.40"

echo "=============================================="
echo " Création de l'exemple RSSI + Latence"
echo " entre drones avec NS-3"
echo "=============================================="

# Créer le script NS-3 C++
mkdir -p "$NS3_DIR/scratch"

cat > "$NS3_DIR/scratch/drone-rssi-latency.cc" << 'NS3_EOF'
/*
 * drone-rssi-latency.cc
 * 
 * Simulation NS-3: mesure RSSI et latence entre drones (UAVs)
 * dans un environnement 3D (warehouse).
 *
 * Output: RSSI (dBm) et latence (ms) entre chaque paire de drones.
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/propagation-module.h"

#include <fstream>
#include <iomanip>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DroneRssiLatency");

// Variables globales pour logging
std::ofstream g_rssiLog;
std::ofstream g_latencyLog;
std::ofstream g_positionLog;
uint32_t g_numDrones = 3;

// Callback pour capturer le RSSI (via MonitorSniffer)
void MonitorSniffRx(Ptr<const Packet> packet, uint16_t channelFreqMhz,
                    WifiTxVector txVector, MpduInfo aMpdu,
                    SignalNoiseDbm signalNoise, uint16_t staId)
{
    g_rssiLog << Simulator::Now().GetSeconds() << ","
              << signalNoise.signal << ","
              << signalNoise.noise << ","
              << (signalNoise.signal - signalNoise.noise) << std::endl;
}

// Callback pour mesurer la latence (envoi → réception)
class LatencyTag : public Tag
{
public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::LatencyTag")
            .SetParent<Tag>()
            .AddConstructor<LatencyTag>();
        return tid;
    }

    TypeId GetInstanceTypeId() const override { return GetTypeId(); }
    uint32_t GetSerializedSize() const override { return 8; }
    void Serialize(TagBuffer buf) const override { buf.WriteU64(m_sendTime); }
    void Deserialize(TagBuffer buf) override { m_sendTime = buf.ReadU64(); }
    void Print(std::ostream& os) const override { os << "SendTime=" << m_sendTime; }

    void SetSendTime(uint64_t t) { m_sendTime = t; }
    uint64_t GetSendTime() const { return m_sendTime; }

private:
    uint64_t m_sendTime = 0;
};

// Log les positions des drones
void LogPositions(NodeContainer& nodes)
{
    for (uint32_t i = 0; i < nodes.GetN(); i++)
    {
        Ptr<MobilityModel> mob = nodes.Get(i)->GetObject<MobilityModel>();
        Vector pos = mob->GetPosition();
        g_positionLog << Simulator::Now().GetSeconds() << ","
                      << i << ","
                      << pos.x << "," << pos.y << "," << pos.z << std::endl;
    }

    // Calculer distances entre chaque paire
    for (uint32_t i = 0; i < nodes.GetN(); i++)
    {
        for (uint32_t j = i + 1; j < nodes.GetN(); j++)
        {
            Ptr<MobilityModel> mobA = nodes.Get(i)->GetObject<MobilityModel>();
            Ptr<MobilityModel> mobB = nodes.Get(j)->GetObject<MobilityModel>();
            double dist = mobA->GetDistanceFrom(mobB);
            NS_LOG_INFO("t=" << Simulator::Now().GetSeconds()
                        << " Distance drone" << i << "-drone" << j
                        << ": " << dist << " m");
        }
    }

    // Re-planifier chaque seconde
    Simulator::Schedule(Seconds(1.0), &LogPositions, std::ref(nodes));
}

int main(int argc, char* argv[])
{
    // Paramètres
    double simTime = 60.0;         // Durée simulation (secondes)
    g_numDrones = 3;               // Nombre de drones
    double warehouseWidth = 30.0;  // Largeur entrepôt (m)
    double warehouseHeight = 30.0; // Longueur entrepôt (m)
    double flightAltitude = 2.0;   // Altitude de vol (m)
    double droneSpeed = 2.0;       // Vitesse max drone (m/s)

    CommandLine cmd;
    cmd.AddValue("numDrones", "Number of drones", g_numDrones);
    cmd.AddValue("simTime", "Simulation time (s)", simTime);
    cmd.AddValue("speed", "Drone speed (m/s)", droneSpeed);
    cmd.Parse(argc, argv);

    NS_LOG_INFO("=== Drone RSSI & Latency Simulation ===");
    NS_LOG_INFO("Drones: " << g_numDrones << ", Duration: " << simTime << "s");

    // Ouvrir fichiers de log
    g_rssiLog.open("drone_rssi.csv");
    g_rssiLog << "time_s,signal_dbm,noise_dbm,snr_db" << std::endl;

    g_latencyLog.open("drone_latency.csv");
    g_latencyLog << "time_s,src,dst,latency_ms,packet_size" << std::endl;

    g_positionLog.open("drone_positions.csv");
    g_positionLog << "time_s,drone_id,x,y,z" << std::endl;

    // -- Créer les noeuds (drones) --
    NodeContainer drones;
    drones.Create(g_numDrones);

    // -- Modèle de propagation (indoor warehouse) --
    // Log-distance avec exposant adapté à un intérieur
    Ptr<LogDistancePropagationLossModel> lossModel =
        CreateObject<LogDistancePropagationLossModel>();
    lossModel->SetAttribute("Exponent", DoubleValue(3.0));       // Indoor
    lossModel->SetAttribute("ReferenceLoss", DoubleValue(46.67)); // @ 1m, 2.4GHz

    Ptr<NakagamiPropagationLossModel> fadingModel =
        CreateObject<NakagamiPropagationLossModel>();
    lossModel->SetNext(fadingModel);

    Ptr<ConstantSpeedPropagationDelayModel> delayModel =
        CreateObject<ConstantSpeedPropagationDelayModel>();

    // -- WiFi Ad-Hoc (communication drone ↔ drone) --
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                  "DataMode", StringValue("HtMcs7"),
                                  "ControlMode", StringValue("HtMcs0"));

    YansWifiChannelHelper channel;
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                                "Exponent", DoubleValue(3.0),
                                "ReferenceLoss", DoubleValue(46.67));
    channel.AddPropagationLoss("ns3::NakagamiPropagationLossModel");

    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    phy.Set("TxPowerStart", DoubleValue(20.0));  // 20 dBm
    phy.Set("TxPowerEnd", DoubleValue(20.0));
    phy.Set("RxSensitivity", DoubleValue(-90.0));

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    NetDeviceContainer devices = wifi.Install(phy, mac, drones);

    // -- Connecter le callback RSSI --
    for (uint32_t i = 0; i < drones.GetN(); i++)
    {
        Config::ConnectWithoutContext(
            "/NodeList/" + std::to_string(i) +
            "/DeviceList/0/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",
            MakeCallback(&MonitorSniffRx));
    }

    // -- Internet stack --
    InternetStackHelper internet;
    internet.Install(drones);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

    // -- Mobilité (Random Walk dans le warehouse, altitude fixe) --
    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::RandomBoxPositionAllocator",
        "X", StringValue("ns3::UniformRandomVariable[Min=-10.0|Max=10.0]"),
        "Y", StringValue("ns3::UniformRandomVariable[Min=-10.0|Max=10.0]"),
        "Z", StringValue("ns3::ConstantRandomVariable[Constant=" +
                          std::to_string(flightAltitude) + "]"));

    // Random Walk 2D (les drones bougent dans le warehouse)
    mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
        "Bounds", RectangleValue(Rectangle(-warehouseWidth/2, warehouseWidth/2,
                                            -warehouseHeight/2, warehouseHeight/2)),
        "Speed", StringValue("ns3::UniformRandomVariable[Min=0.5|Max=" +
                              std::to_string(droneSpeed) + "]"),
        "Distance", DoubleValue(5.0));

    mobility.Install(drones);

    // -- Applications: chaque drone envoie des paquets aux autres --
    uint16_t port = 9;

    for (uint32_t src = 0; src < g_numDrones; src++)
    {
        for (uint32_t dst = 0; dst < g_numDrones; dst++)
        {
            if (src == dst) continue;

            // Serveur UDP (réception)
            UdpEchoServerHelper echoServer(port + src * 10 + dst);
            ApplicationContainer serverApp = echoServer.Install(drones.Get(dst));
            serverApp.Start(Seconds(1.0));
            serverApp.Stop(Seconds(simTime));

            // Client UDP (envoi)
            UdpEchoClientHelper echoClient(interfaces.GetAddress(dst),
                                            port + src * 10 + dst);
            echoClient.SetAttribute("MaxPackets", UintegerValue(1000));
            echoClient.SetAttribute("Interval", TimeValue(Seconds(1.0)));
            echoClient.SetAttribute("PacketSize", UintegerValue(512));

            ApplicationContainer clientApp = echoClient.Install(drones.Get(src));
            clientApp.Start(Seconds(2.0 + src * 0.1));
            clientApp.Stop(Seconds(simTime));
        }
    }

    // -- Flow Monitor (mesure latence exacte) --
    FlowMonitorHelper flowHelper;
    Ptr<FlowMonitor> flowMonitor = flowHelper.InstallAll();

    // -- Log positions --
    Simulator::Schedule(Seconds(1.0), &LogPositions, std::ref(drones));

    // -- Lancer simulation --
    NS_LOG_INFO("Démarrage simulation...");
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // -- Extraire les stats de latence du FlowMonitor --
    flowMonitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowHelper.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = flowMonitor->GetFlowStats();

    std::cout << "\n=== RÉSULTATS ===" << std::endl;
    std::cout << std::fixed << std::setprecision(3);

    for (auto& iter : stats)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(iter.first);
        double avgLatency = 0;
        if (iter.second.rxPackets > 0)
        {
            avgLatency = iter.second.delaySum.GetMilliSeconds() /
                         (double)iter.second.rxPackets;
        }

        std::cout << "Flow " << t.sourceAddress << " → " << t.destinationAddress
                  << " | Sent: " << iter.second.txPackets
                  << " | Recv: " << iter.second.rxPackets
                  << " | Lost: " << iter.second.lostPackets
                  << " | Avg Latency: " << avgLatency << " ms"
                  << std::endl;

        g_latencyLog << simTime << ","
                     << t.sourceAddress << ","
                     << t.destinationAddress << ","
                     << avgLatency << ","
                     << iter.second.rxBytes << std::endl;
    }

    // -- Fermer fichiers --
    g_rssiLog.close();
    g_latencyLog.close();
    g_positionLog.close();

    Simulator::Destroy();

    std::cout << "\n=== Fichiers générés ===" << std::endl;
    std::cout << "  drone_rssi.csv      - Puissance signal (RSSI) + SNR" << std::endl;
    std::cout << "  drone_latency.csv   - Latence entre drones" << std::endl;
    std::cout << "  drone_positions.csv - Positions 3D des drones" << std::endl;

    return 0;
}
NS3_EOF

echo "[OK] Script NS-3 créé: $NS3_DIR/scratch/drone-rssi-latency.cc"

# Compiler
echo "[INFO] Compilation..."
cd "$NS3_DIR"
./ns3 build -j$(nproc)

echo "[OK] Compilation terminée"

# Lancer
echo ""
echo "[INFO] Lancement de la simulation RSSI + Latence..."
echo ""
./ns3 run "scratch/drone-rssi-latency --numDrones=3 --simTime=60"

echo ""
echo "=============================================="
echo " [DONE] Résultats dans:"
echo "   $(pwd)/drone_rssi.csv"
echo "   $(pwd)/drone_latency.csv"
echo "   $(pwd)/drone_positions.csv"
echo "=============================================="
