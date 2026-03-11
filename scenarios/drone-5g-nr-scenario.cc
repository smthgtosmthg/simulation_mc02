/* -*- Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */


#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"

#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <vector>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("Drone5gNr");

struct DronePos { int id; double x, y, z; };

std::vector<DronePos> ReadPositions(const std::string& path)
{
    std::map<int, DronePos> latest;
    std::ifstream f(path);
    if (!f.is_open()) return {};
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        if (line.find("drone_id") != std::string::npos) continue;
        std::stringstream ss(line); std::string tok;
        std::vector<std::string> t;
        while (std::getline(ss, tok, ',')) t.push_back(tok);
        DronePos d;
        try {
            if (t.size() >= 5) { d.id=std::stoi(t[1]); d.x=std::stod(t[2]); d.y=std::stod(t[3]); d.z=std::stod(t[4]); }
            else if (t.size() >= 4) { d.id=std::stoi(t[0]); d.x=std::stod(t[1]); d.y=std::stod(t[2]); d.z=std::stod(t[3]); }
            else continue;
            latest[d.id] = d;
        } catch (...) { continue; }
    }
    std::vector<DronePos> res;
    for (auto& kv : latest) res.push_back(kv.second);
    return res;
}

int main(int argc, char* argv[])
{
    std::string posFile = "/tmp/drone_positions.csv";
    std::string outFile = "/tmp/drone_5g_metrics.csv";
    double simTimeSec = 2.0;
    double gnbTxPower = 23.0;
    double frequency  = 3.5e9;
    double bw         = 20e6;
    double gnbX=0, gnbY=0, gnbZ=6;
    uint32_t seed = 0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("posFile","CSV positions",posFile);
    cmd.AddValue("outFile","CSV sortie",outFile);
    cmd.AddValue("simTime","Duree (s)",simTimeSec);
    cmd.AddValue("txPower","gNB TX (dBm)",gnbTxPower);
    cmd.AddValue("frequency","Hz",frequency);
    cmd.AddValue("bandwidth","Hz",bw);
    cmd.AddValue("gnbX","m",gnbX); cmd.AddValue("gnbY","m",gnbY); cmd.AddValue("gnbZ","m",gnbZ);
    cmd.AddValue("seed","RNG seed (0=auto)",seed);
    cmd.Parse(argc, argv);

    if (seed == 0) seed = static_cast<uint32_t>(std::time(nullptr)) % 100000;
    RngSeedManager::SetSeed(seed);
    RngSeedManager::SetRun(seed);

    Time simTime  = Seconds(simTimeSec);
    Time appStart = MilliSeconds(400);

    // 1. Positions
    auto drones = ReadPositions(posFile);
    uint16_t nUe = drones.size();
    if (nUe < 2) { std::cerr << "ERREUR: >= 2 drones requis\n"; return 1; }

    std::cout << "\n  5G NR | " << nUe << " drones | "
              << frequency/1e9 << " GHz | " << bw/1e6 << " MHz\n";
    for (auto& d : drones)
        std::cout << "  UE " << d.id << " @ (" << d.x << "," << d.y << "," << d.z << ")\n";
    std::cout << "  gNB @ (" << gnbX << "," << gnbY << "," << gnbZ << ")\n\n";

    // 2. Noeuds
    NodeContainer gnbNodes; gnbNodes.Create(1);
    NodeContainer ueNodes;  ueNodes.Create(nUe);

    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob.Install(gnbNodes); mob.Install(ueNodes);
    gnbNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(gnbX,gnbY,gnbZ));
    for (uint16_t i=0;i<nUe;i++)
        ueNodes.Get(i)->GetObject<MobilityModel>()->SetPosition(Vector(drones[i].x,drones[i].y,drones[i].z));

    // 3. Stack 5G NR + EPC (coeur 5G)
    auto epcHelper = CreateObject<NrPointToPointEpcHelper>();
    auto bfHelper  = CreateObject<IdealBeamformingHelper>();
    auto nrHelper  = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(bfHelper);
    nrHelper->SetEpcHelper(epcHelper);

    CcBwpCreator ccBwp;
    CcBwpCreator::SimpleOperationBandConf bandConf(frequency,bw,1,BandwidthPartInfo::InH_OfficeOpen);
    OperationBandInfo band = ccBwp.CreateOperationBandContiguousCc(bandConf);

    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",TimeValue(MilliSeconds(0)));
    nrHelper->SetChannelConditionModelAttribute("UpdatePeriod",TimeValue(MilliSeconds(0)));
    nrHelper->SetPathlossAttribute("ShadowingEnabled",BooleanValue(true));
    nrHelper->InitializeOperationBand(&band);
    auto allBwps = CcBwpCreator::GetAllBwps({band});

    bfHelper->SetAttribute("BeamformingMethod",TypeIdValue(DirectPathBeamforming::GetTypeId()));
    epcHelper->SetAttribute("S1uLinkDelay",TimeValue(MilliSeconds(2)));

    nrHelper->SetGnbAntennaAttribute("NumRows",UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns",UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetUeAntennaAttribute("NumRows",UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns",UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement",PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB",UintegerValue(0));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB",UintegerValue(0));
    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize",UintegerValue(999999999));

    // 4. Devices NR
    NetDeviceContainer gnbDev = nrHelper->InstallGnbDevice(gnbNodes,allBwps);
    NetDeviceContainer ueDev  = nrHelper->InstallUeDevice(ueNodes,allBwps);
    int64_t s=1; s+=nrHelper->AssignStreams(gnbDev,s); s+=nrHelper->AssignStreams(ueDev,s);
    nrHelper->GetGnbPhy(gnbDev.Get(0),0)->SetAttribute("Numerology",UintegerValue(1));
    nrHelper->GetGnbPhy(gnbDev.Get(0),0)->SetAttribute("TxPower",DoubleValue(gnbTxPower));
    for (auto it=gnbDev.Begin();it!=gnbDev.End();++it) DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
    for (auto it=ueDev.Begin();it!=ueDev.End();++it)   DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();

    // 5. IP + EPC
    InternetStackHelper internet;
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIp = epcHelper->AssignUeIpv4Address(ueDev);
    Ipv4StaticRoutingHelper routeH;
    for (uint32_t j=0;j<ueNodes.GetN();++j) {
        auto r = routeH.GetStaticRouting(ueNodes.Get(j)->GetObject<Ipv4>());
        r->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(),1);
    }
    nrHelper->AttachToClosestEnb(ueDev,gnbDev);

    // 6. Trafic UDP entre chaque PAIRE de drones (A -> gNB -> EPC -> gNB -> B)
    uint16_t basePort = 5000;
    ApplicationContainer servers, clients;
    // On stocke le mapping port -> (a,b) pour retrouver les paires apres
    std::map<uint16_t, std::pair<uint32_t,uint32_t>> portToPair;

    for (uint32_t a=0; a<nUe; a++) {
        for (uint32_t b=a+1; b<nUe; b++) {
            uint16_t port = basePort++;

            // Serveur sur drone B (ecoute)
            UdpServerHelper sink(port);
            servers.Add(sink.Install(ueNodes.Get(b)));

            // Client sur drone A (envoie vers B a travers le reseau 5G)
            UdpClientHelper cli;
            cli.SetAttribute("RemotePort",   UintegerValue(port));
            cli.SetAttribute("RemoteAddress", AddressValue(ueIp.GetAddress(b)));
            cli.SetAttribute("MaxPackets",    UintegerValue(0xFFFFFFFF));
            cli.SetAttribute("PacketSize",    UintegerValue(500));
            cli.SetAttribute("Interval",      TimeValue(MilliSeconds(10)));
            clients.Add(cli.Install(ueNodes.Get(a)));

            // Bearer dedie
            EpsBearer bearer(EpsBearer::NGBR_LOW_LAT_EMBB);
            Ptr<EpcTft> tft = Create<EpcTft>();
            EpcTft::PacketFilter pf;
            pf.localPortStart = port; pf.localPortEnd = port;
            tft->Add(pf);
            nrHelper->ActivateDedicatedEpsBearer(ueDev.Get(b),bearer,tft);

            portToPair[port] = {a, b};
        }
    }
    servers.Start(appStart); clients.Start(appStart);
    servers.Stop(simTime);   clients.Stop(simTime);

    // 7. FlowMonitor
    FlowMonitorHelper fmH;
    Ptr<FlowMonitor> monitor = fmH.Install(ueNodes);

    // 8. Run
    Simulator::Stop(simTime);
    Simulator::Run();

    // 9. RSSI 
    auto plModel = CreateObject<ThreeGppIndoorOfficePropagationLossModel>();
    plModel->SetAttribute("Frequency", DoubleValue(frequency));
    plModel->SetAttribute("ShadowingEnabled", BooleanValue(true));
    plModel->SetChannelConditionModel(CreateObject<ThreeGppIndoorOpenOfficeChannelConditionModel>());
    auto gnbMob = gnbNodes.Get(0)->GetObject<MobilityModel>();
    std::vector<double> rssi(nUe);
    for (uint16_t i=0;i<nUe;i++)
        rssi[i] = plModel->CalcRxPower(gnbTxPower, gnbMob, ueNodes.Get(i)->GetObject<MobilityModel>());

    // 10. Latence par paire
    monitor->CheckForLostPackets();
    auto cls = DynamicCast<Ipv4FlowClassifier>(fmH.GetClassifier());
    auto stats = monitor->GetFlowStats();

    std::map<Ipv4Address,uint32_t> ipIdx;
    for (uint32_t i=0;i<nUe;i++) ipIdx[ueIp.GetAddress(i)]=i;

    struct PR { double lat=-1,jit=0; uint32_t rx=0; };
    std::map<std::pair<uint32_t,uint32_t>,PR> pairRes;

    for (auto& st : stats) {
        auto ft = cls->FindFlow(st.first);
        auto iA=ipIdx.find(ft.sourceAddress), iB=ipIdx.find(ft.destinationAddress);
        if (iA!=ipIdx.end() && iB!=ipIdx.end() && st.second.rxPackets>0) {
            uint32_t a=std::min(iA->second,iB->second), b=std::max(iA->second,iB->second);
            PR pr;
            pr.lat = 1000.0*st.second.delaySum.GetSeconds()/st.second.rxPackets;
            pr.jit = 1000.0*st.second.jitterSum.GetSeconds()/st.second.rxPackets;
            pr.rx  = st.second.rxPackets;
            pairRes[{a,b}] = pr;
        }
    }

    std::ofstream out(outFile);
    out << "drone_a,drone_b,rssi_a_dBm,rssi_b_dBm,latency_ms,jitter_ms,dist_ab_m,rx_packets\n";

    std::cout << std::fixed;
    std::cout << "  ┌───────────┬───────────┬───────────┬────────────┬────────────┬─────────┐\n";
    std::cout << "  │  Paire    │  RSSI A   │  RSSI B   │ Latence ms │ Jitter ms  │ Dist m  │\n";
    std::cout << "  ├───────────┼───────────┼───────────┼────────────┼────────────┼─────────┤\n";

    for (uint32_t a=0;a<nUe;a++) {
        for (uint32_t b=a+1;b<nUe;b++) {
            double dx=drones[a].x-drones[b].x, dy=drones[a].y-drones[b].y, dz=drones[a].z-drones[b].z;
            double dist=std::sqrt(dx*dx+dy*dy+dz*dz);
            auto it=pairRes.find({a,b});
            double lat=(it!=pairRes.end())?it->second.lat:-1;
            double jit=(it!=pairRes.end())?it->second.jit:0;
            uint32_t rx=(it!=pairRes.end())?it->second.rx:0;

            out << std::fixed << std::setprecision(2)
                << drones[a].id<<","<<drones[b].id<<","
                << rssi[a]<<","<<rssi[b]<<","
                << lat<<","<<jit<<","<<dist<<","<<rx<<"\n";

            std::cout << "  │ " << std::setw(2) << drones[a].id << " <-> " << std::setw(2) << drones[b].id
                      << " │ " << std::setw(9) << std::setprecision(1) << rssi[a]
                      << " │ " << std::setw(9) << std::setprecision(1) << rssi[b]
                      << " │ " << std::setw(10) << std::setprecision(2) << lat
                      << " │ " << std::setw(10) << std::setprecision(2) << jit
                      << " │ " << std::setw(7) << std::setprecision(1) << dist << " │\n";
        }
    }
    std::cout << "  └───────────┴───────────┴───────────┴────────────┴────────────┴─────────┘\n";
    std::cout << "  CSV: " << outFile << "\n\n";

    out.close();
    Simulator::Destroy();
    return 0;
}
