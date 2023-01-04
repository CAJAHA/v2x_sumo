/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
  This software was developed at the National Institute of Standards and
  Technology by employees of the Federal Government in the course of
  their official duties. Pursuant to titleElement 17 Section 105 of the United
  States Code this software is not subject to copyright protection and
  is in the public domain.
  NIST assumes no responsibility whatsoever for its use by other parties,
  and makes no guarantees, expressed or implied, about its quality,
  reliability, or any other characteristic.

  We would appreciate acknowledgement if the software is used.

  NIST ALLOWS FREE USE OF THIS SOFTWARE IN ITS "AS IS" CONDITION AND
  DISCLAIM ANY LIABILITY OF ANY KIND FOR ANY DAMAGES WHATSOEVER RESULTING
  FROM THE USE OF THIS SOFTWARE.

 * Modified by: Fabian Eckermann <fabian.eckermann@udo.edu> (CNI)
 *              Moritz Kahlert <moritz.kahlert@udo.edu> (CNI)
 */

#include "ns3/lte-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/lte-v2x-helper.h"
#include "ns3/config-store.h"
#include "ns3/lte-hex-grid-enb-topology-helper.h"
#include <ns3/buildings-helper.h>
#include <ns3/cni-urbanmicrocell-propagation-loss-model.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/spectrum-analyzer-helper.h>
#include <ns3/multi-model-spectrum-channel.h>
#include "ns3/ns2-mobility-helper.h"
#include "cut-in-mobility-model.h"
#include "ns3/rng-seed-manager.h"
#include <cfloat>
#include <sstream>
#include <ctime>
#include <iostream>


using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("v2x_communication_mode_4");

// Output 
std::string rx_data = "_rxdata.csv";
Ptr<OutputStreamWrapper> log_rx_data;

std::string dist_data = "_dist.csv";
Ptr<OutputStreamWrapper> log_dist_data_100m;
Ptr<OutputStreamWrapper> log_dist_data_150m;
Ptr<OutputStreamWrapper> log_dist_data_200m;

std::string average_pos_err = "_position_error.csv";
Ptr<OutputStreamWrapper> log_poisition_error;

// Global variables
uint32_t ctr_totRx = 0; 	// Counter for total received packets
uint32_t ctr_totTx = 0; 	// Counter for total transmitted packets
uint16_t lenCam;  
double baseline= 150.0;     // Baseline distance in meter (150m for urban, 320m for freeway)

uint32_t numVeh;
// uint32_t infVeh;

// Responders users 
NodeContainer ueVeh;
NodeContainer vVeh;
// NodeContainer infVeh;
// NodeContainer allVeh;


struct SendMsg
{
    uint64_t m_genTime;
    uint32_t m_ID;
    double m_xPos;
    double m_yPos;
};


void 
PrintStatus (uint32_t s_period)
{
    if (ctr_totRx > ctr_totTx)
    {
        ctr_totRx = ctr_totTx; 
    }
	// *log_simtime->GetStream() << Simulator::Now ().GetSeconds () << ";" << ctr_totRx << ";" << ctr_totTx << ";" << (double) ctr_totRx / ctr_totTx << std::endl; 
    std::cout << "t=" <<  Simulator::Now().GetSeconds() << "\t Rx/Tx="<< ctr_totRx << "/" << ctr_totTx << "\t PRR=" << (double) ctr_totRx / ctr_totTx << std::endl;
    Simulator::Schedule(Seconds(s_period), &PrintStatus, s_period);
}

void 
SaveDistance ()
{
    uint64_t simTime = Simulator::Now().GetMilliSeconds();
    *log_dist_data_100m->GetStream() << simTime;
    *log_dist_data_150m->GetStream() << simTime;
    *log_dist_data_200m->GetStream() << simTime;

    NodeContainer::Iterator NodeItr1, NodeItr2;
    Ptr<MobilityModel> MobPtr1, MobPtr2;
    Vector v1, v2;
    double distance;
    double threshold_100 = pow(100.0, 2);
    double threshold_150 = pow(150.0, 2);
    double threshold_200 = pow(200.0, 2);
    for (NodeItr1 = vVeh.Begin(); NodeItr1 != vVeh.End(); NodeItr1++)
    {
        MobPtr1 = (*NodeItr1)->GetObject<MobilityModel>(); 
        v1 = MobPtr1->GetPosition();
        for (NodeItr2 = (NodeItr1+1); NodeItr2 != vVeh.End(); NodeItr2++)
        {
            MobPtr2 = (*NodeItr2)->GetObject<MobilityModel>(); 
            v2 = MobPtr2->GetPosition();
            distance = pow(v1.x-v2.x, 2) + pow(v1.y-v2.y, 2);
            if (distance > threshold_200)
            {
                *log_dist_data_100m->GetStream() << "<" <<(*NodeItr1)->GetId() << "," << (*NodeItr2)->GetId() << ">"; 
                *log_dist_data_150m->GetStream() << "<" <<(*NodeItr1)->GetId() << "," << (*NodeItr2)->GetId() << ">"; 
                *log_dist_data_200m->GetStream() << "<" <<(*NodeItr1)->GetId() << "," << (*NodeItr2)->GetId() << ">"; 
            }
            else if (distance > threshold_150)
            {
                *log_dist_data_100m->GetStream() << "<" <<(*NodeItr1)->GetId() << "," << (*NodeItr2)->GetId() << ">"; 
                *log_dist_data_150m->GetStream() << "<" <<(*NodeItr1)->GetId() << "," << (*NodeItr2)->GetId() << ">"; 
            }
            else if (distance > threshold_100)
                *log_dist_data_100m->GetStream() << "<" <<(*NodeItr1)->GetId() << "," << (*NodeItr2)->GetId() << ">";             
        }
    }
    *log_dist_data_100m->GetStream() << "\n";
    *log_dist_data_150m->GetStream() << "\n";
    *log_dist_data_200m->GetStream() << "\n";

    Simulator::Schedule(MilliSeconds(1), &SaveDistance);
}

void
SidelinkV2xAnnouncementMacTrace(Ptr<Socket> socket)
{
    Ptr <Node> node = socket->GetNode(); 
    Ptr<CutInMobilityModel> posMobility = node->GetObject<CutInMobilityModel>();
    Vector posTx = posMobility->GetPosition();

    CutInMobilityModel::VehState veh_state = posMobility->DoGetVehState();

    // SendMsg msg = {
    //     simTime,
    //     id,
    //     posTx.x,
    //     posTx.y
    // };

    // check for each UE distance to transmitter
    for (uint8_t i=0; i< numVeh;i++)
    {
        Ptr<MobilityModel> mob = ueVeh.Get(i)->GetObject<MobilityModel>(); 
        Vector posRx = mob->GetPosition();
        
        double distance = sqrt(pow((posTx.x - posRx.x),2.0)+pow((posTx.y - posRx.y),2.0));
        if  (distance > 0 && distance <= baseline)
        {
            ctr_totTx++;
        }
    }
    // Generate CAM 
    // std::ostringstream msgCam;
    // msgCam << id << ";" << simTime << ";" << (int) posTx.x << ";" << (int) posTx.y << '\0'; 
    Ptr<Packet> packet = Create<Packet>((uint8_t*)(&veh_state), lenCam);
    socket->Send(packet);
}

static void
ReceivePacket(Ptr<Socket> socket)
{   
    Ptr<Node> node = socket->GetNode();
    uint32_t rxID = node->GetId();
    uint64_t rxTime = Simulator::Now().GetMilliSeconds();
    Ptr<CutInMobilityModel> posMobility = node->GetObject<CutInMobilityModel>();
    Vector posRx = posMobility->GetPosition();


    // SendMsg msg;
    CutInMobilityModel::VehState veh_state;
    Ptr<Packet> packet = socket->Recv (); 
    packet->CopyData((uint8_t*)&veh_state, sizeof(CutInMobilityModel::VehState));

    posMobility->DoPassVehState(veh_state);

    double distance = sqrt(pow((veh_state.m_position.m_x - posRx.x),2.0)+pow((veh_state.m_position.m_y - posRx.y), 2.0));
    if (distance <= baseline)
    {         
        ctr_totRx++; 
    }
    
    *log_rx_data->GetStream() << veh_state.m_genTime << " " << veh_state.m_ID << " " << rxTime << " " << rxID << " " << distance << "\n";

}

int 
main (int argc, char *argv[])
{

    std::clock_t start, end;
    start = std::clock();

    LogComponentEnable ("v2x_communication_mode_4", LOG_INFO);


    // Initialize some values
    // NOTE: commandline parser is currently (05.04.2019) not working for uint8_t (Bug 2916)
    int run = 1;
    uint16_t simTime = 30;                 // Simulation time in seconds
    numVeh = 12;                  // Number of vehicles
    int infVeh = 20;
    lenCam = 190;                           // Length of CAM message in bytes [50-300 Bytes]
    double ueTxPower = 23.0;                // Transmission power in dBm
    double probResourceKeep = 0.0;          // Probability to select the previous resource again [0.0-0.8]
    uint32_t mcs = 20;                      // Modulation and Coding Scheme
    bool harqEnabled = false;               // Retransmission enabled 
    bool adjacencyPscchPssch = true;        // Subchannelization scheme
    bool partialSensing = false;            // Partial sensing enabled (actual only partialSensing is false supported)
    uint16_t sizeSubchannel = 10;           // Number of RBs per subchannel
    uint16_t numSubchannel = 2;             // Number of subchannels per subframe
    uint16_t startRbSubchannel = 0;         // Index of first RB corresponding to subchannelization
    uint16_t pRsvp = 20;				    // Resource reservation interval 
    uint16_t t1 = 4;                        // T1 value of selection window
    uint16_t t2 = 20;                      // T2 value of selection window
    uint16_t slBandwidth;                   // Sidelink bandwidth
    CutInMobilityModel::TrafficLevel level = CutInMobilityModel::TrafficLevel::L3;

    // Command line arguments
    CommandLine cmd;
    cmd.AddValue ("time", "Simulation Time", simTime);
    cmd.AddValue ("run", "run value of simulation", run);
    cmd.AddValue ("numVeh", "Number of Vehicles", numVeh);
    cmd.AddValue ("infVeh", "Number of Inf Vehicles", infVeh);
    cmd.AddValue ("adjacencyPscchPssch", "Scheme for subchannelization", adjacencyPscchPssch); 
    cmd.AddValue ("sizeSubchannel", "Number of RBs per Subchannel", sizeSubchannel);
    cmd.AddValue ("numSubchannel", "Number of Subchannels", numSubchannel);
    cmd.AddValue ("startRbSubchannel", "Index of first subchannel index", startRbSubchannel); 
    cmd.AddValue ("T1", "T1 Value of Selection Window", t1);
    cmd.AddValue ("T2", "T2 Value of Selection Window", t2);
    cmd.AddValue ("lenCam", "Packetsize in Bytes", lenCam);
    cmd.AddValue ("mcs", "Modulation and Coding Scheme", mcs);
    cmd.AddValue ("pRsvp", "Resource Reservation Interval", pRsvp); 
    cmd.AddValue ("probResourceKeep", "Probability for selecting previous resource again", probResourceKeep); 
    cmd.AddValue ("log_rx_data", "name of the rx data logfile", rx_data);
    cmd.AddValue ("baseline", "Distance in which messages are transmitted and must be received", baseline);
    cmd.Parse (argc, argv);
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(run);

    AsciiTraceHelper ascii;
    rx_data = "./simulation_results/" + std::to_string(level) + "_" + std::to_string(infVeh) + "_" + std::to_string(run) + rx_data;
    log_rx_data = ascii.CreateFileStream(rx_data);
    
    std::string dist_data_100m = "./simulation_results/" + std::to_string(level) + "_" + std::to_string(infVeh) + "_" + std::to_string(run) + "_100_" + dist_data;
    std::string dist_data_150m = "./simulation_results/" + std::to_string(level) + "_" + std::to_string(infVeh) + "_" + std::to_string(run) + "_150_" + dist_data;
    std::string dist_data_200m = "./simulation_results/" + std::to_string(level) + "_" + std::to_string(infVeh) + "_" + std::to_string(run) + "_200_" + dist_data;
    log_dist_data_100m = ascii.CreateFileStream(dist_data_100m);
    log_dist_data_150m = ascii.CreateFileStream(dist_data_150m);
    log_dist_data_200m = ascii.CreateFileStream(dist_data_200m);
    
    average_pos_err = "./simulation_results/" + std::to_string(level) + "_" + std::to_string(infVeh) + "_" + std::to_string(run) + average_pos_err;
    log_poisition_error = ascii.CreateFileStream(average_pos_err);


    NS_LOG_INFO ("Starting network configuration..."); 

    // Set the UEs power in dBm
    Config::SetDefault ("ns3::LteUePhy::TxPower", DoubleValue (ueTxPower));
    Config::SetDefault ("ns3::LteUePhy::RsrpUeMeasThreshold", DoubleValue (-10.0));
    // Enable V2X communication on PHY layer
    Config::SetDefault ("ns3::LteUePhy::EnableV2x", BooleanValue (true));

    // Set power
    Config::SetDefault ("ns3::LteUePowerControl::Pcmax", DoubleValue (ueTxPower));
    Config::SetDefault ("ns3::LteUePowerControl::PsschTxPower", DoubleValue (ueTxPower));
    Config::SetDefault ("ns3::LteUePowerControl::PscchTxPower", DoubleValue (ueTxPower));

    if (adjacencyPscchPssch) 
    {
        slBandwidth = sizeSubchannel * numSubchannel;
    }
    else 
    {
        slBandwidth = (sizeSubchannel+2) * numSubchannel; 
    }

    // Configure for UE selected
    Config::SetDefault ("ns3::LteUeMac::UlBandwidth", UintegerValue(slBandwidth));
    Config::SetDefault ("ns3::LteUeMac::EnableV2xHarq", BooleanValue(harqEnabled));
    Config::SetDefault ("ns3::LteUeMac::EnableAdjacencyPscchPssch", BooleanValue(adjacencyPscchPssch));
    Config::SetDefault ("ns3::LteUeMac::EnablePartialSensing", BooleanValue(partialSensing));
    Config::SetDefault ("ns3::LteUeMac::SlGrantMcs", UintegerValue(mcs));
    Config::SetDefault ("ns3::LteUeMac::SlSubchannelSize", UintegerValue (sizeSubchannel));
    Config::SetDefault ("ns3::LteUeMac::SlSubchannelNum", UintegerValue (numSubchannel));
    Config::SetDefault ("ns3::LteUeMac::SlStartRbSubchannel", UintegerValue (startRbSubchannel));
    Config::SetDefault ("ns3::LteUeMac::SlPrsvp", UintegerValue(pRsvp));
    Config::SetDefault ("ns3::LteUeMac::SlProbResourceKeep", DoubleValue(probResourceKeep));
    Config::SetDefault ("ns3::LteUeMac::SelectionWindowT1", UintegerValue(t1));
    Config::SetDefault ("ns3::LteUeMac::SelectionWindowT2", UintegerValue(t2));
    Config::SetDefault ("ns3::LteEnbNetDevice::UlEarfcn", StringValue ("54990"));


    ConfigStore inputConfig; 
    inputConfig.ConfigureDefaults(); 

    // Create node container to hold all UEs 
    // NodeContainer ueAllNodes; 

    NS_LOG_INFO ("Installing Mobility Model...");


    // Create nodes
    std::list<CutInMobilityModel::VehPosition> main_init_pos = 
    CutInMobilityModel::GetInitPosition(level, 
                                        CutInMobilityModel::BelongTo::MAIN_ROAD);
    uint32_t main_veh_num = main_init_pos.size();
    std::list<CutInMobilityModel::VehPosition> merge_init_pos = 
    CutInMobilityModel::GetInitPosition(level, 
                                        CutInMobilityModel::BelongTo::MERGE_ROAD);
    uint32_t merge_veh_num = merge_init_pos.size();

    numVeh = main_veh_num + merge_veh_num;
    ueVeh.Create (numVeh + infVeh);


    // Install constant random positions 


    MobilityHelper mobVeh;
    mobVeh.SetMobilityModel("ns3::CutInMobilityModel"); 

    Ptr<UniformRandomVariable> RV = CreateObject<UniformRandomVariable> ();
    std::list<CutInMobilityModel::VehPosition>::iterator posItr;
    Ptr<ListPositionAllocator> staticVeh[numVeh+infVeh];

    posItr = main_init_pos.begin();
    for (uint16_t i=0; i< main_veh_num;i++, posItr++)
    {
        staticVeh[i] = CreateObject<ListPositionAllocator>();
        staticVeh[i]->Add(Vector(posItr->m_x, posItr->m_y, 1.5)); 
        mobVeh.SetPositionAllocator(staticVeh[i]);
        mobVeh.Install(ueVeh.Get(i));

        vVeh.Add(ueVeh.Get(i));

        Ptr<CutInMobilityModel> cimm = ueVeh.Get(i)->GetObject<CutInMobilityModel>();
        cimm->ScheduleToStart(i, 5000, level, CutInMobilityModel::BelongTo::MAIN_ROAD);
    }

    posItr = merge_init_pos.begin();
    for (uint16_t i=main_veh_num; i< numVeh; i++, posItr++)
    {
        staticVeh[i] = CreateObject<ListPositionAllocator>();
        staticVeh[i]->Add(Vector(posItr->m_x, posItr->m_y, 1.5)); 
        mobVeh.SetPositionAllocator(staticVeh[i]);
        mobVeh.Install(ueVeh.Get(i));

        vVeh.Add(ueVeh.Get(i));

        Ptr<CutInMobilityModel> cimm = ueVeh.Get(i)->GetObject<CutInMobilityModel>();
        cimm->ScheduleToStart(i, 5000, level, CutInMobilityModel::BelongTo::MERGE_ROAD);
    }

    mobVeh.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    for (uint16_t i = numVeh; i < numVeh + infVeh; i++)
    {   
        double x = RV->GetValue(-575, -175);
        double y = RV->GetValue(-3.75 - 200/sqrt(3) - 3.75, 3.75);
        staticVeh[i] = CreateObject<ListPositionAllocator>();
        staticVeh[i]->Add(Vector(x, y, 1.5));

        mobVeh.SetPositionAllocator(staticVeh[i]);
        mobVeh.Install(ueVeh.Get(i));
    }



    NS_LOG_INFO ("Creating helpers...");
    // EPC
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();

    // LTE Helper
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    lteHelper->SetEpcHelper(epcHelper);
    lteHelper->DisableNewEnbPhy(); // Disable eNBs for out-of-coverage modelling
    lteHelper->SetEnbAntennaModelType ("ns3::NistParabolic3dAntennaModel");
    lteHelper->SetAttribute ("UseSameUlDlPropagationCondition", BooleanValue(true));
    lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::CniUrbanmicrocellPropagationLossModel"));
    
    // V2X Helper
    Ptr<LteV2xHelper> lteV2xHelper = CreateObject<LteV2xHelper> ();
    lteV2xHelper->SetLteHelper (lteHelper); 

    // Create eNB Container
    NodeContainer eNodeB;
    eNodeB.Create(1); 

    // Topology eNodeB
    Ptr<ListPositionAllocator> pos_eNB = CreateObject<ListPositionAllocator>(); 
    pos_eNB->Add(Vector(5,-10,30));
    MobilityHelper mob_eNB;
    mob_eNB.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob_eNB.SetPositionAllocator(pos_eNB);
    mob_eNB.Install(eNodeB);

    // Install Service
    lteHelper->InstallEnbDevice(eNodeB);

    // Required to use NIST 3GPP model
    BuildingsHelper::Install (eNodeB);
    BuildingsHelper::Install (ueVeh);
    BuildingsHelper::MakeMobilityModelConsistent (); 

    // Install LTE devices to all UEs 
    NS_LOG_INFO ("Installing UE's network devices...");
    lteHelper->SetAttribute("UseSidelink", BooleanValue (true));
    NetDeviceContainer ueDevs = lteHelper->InstallUeDevice (ueVeh);

    // Install the IP stack on the UEs
    NS_LOG_INFO ("Installing IP stack..."); 
    InternetStackHelper internet;
    internet.Install (ueVeh); 

    // Assign IP adress to UEs
    NS_LOG_INFO ("Allocating IP addresses and setting up network route...");
    Ipv4InterfaceContainer ueIpIface; 
    ueIpIface = epcHelper->AssignUeIpv4Address (ueDevs);
    Ipv4StaticRoutingHelper Ipv4RoutingHelper;

    for(uint32_t u = 0; u < ueVeh.GetN(); ++u)
    {
        Ptr<Node> ueNode = ueVeh.Get(u);
        // Set the default gateway for the UE
        Ptr<Ipv4StaticRouting> ueStaticRouting = Ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    NS_LOG_INFO("Attaching UE's to LTE network...");
    //Attach each UE to the best available eNB
    lteHelper->Attach(ueDevs); 

    NS_LOG_INFO ("Creating sidelink groups...");
    std::vector<NetDeviceContainer> txGroups;
    txGroups = lteV2xHelper->AssociateForV2xBroadcast(ueDevs, numVeh+infVeh); 
    lteV2xHelper->PrintGroups(txGroups); 

    NS_LOG_INFO ("Installing applications...");
    // Application Setup for Responders
    std::vector<uint32_t> groupL2Addresses; 
    uint32_t groupL2Address = 0x00; 
    Ipv4AddressGenerator::Init(Ipv4Address ("225.0.0.0"), Ipv4Mask("255.0.0.0"));
    Ipv4Address clientRespondersAddress = Ipv4AddressGenerator::NextAddress (Ipv4Mask ("255.0.0.0"));
    uint16_t application_port = 8000; // Application port to TX/RX

    for(std::vector<NetDeviceContainer>::iterator gIt=txGroups.begin(); gIt != txGroups.end(); gIt++)
    {
        // Create Sidelink bearers
        // Use Tx for the group transmitter and Rx for all the receivers
        // Split Tx/Rx

        NetDeviceContainer txUe ((*gIt).Get(0));
        NetDeviceContainer rxUes = lteV2xHelper->RemoveNetDevice ((*gIt), txUe.Get (0));

        uint32_t vehID = txUe.Get(0)->GetNode()->GetId();

        Ptr<LteSlTft> tft = Create<LteSlTft> (LteSlTft::TRANSMIT, clientRespondersAddress, groupL2Address);
        lteV2xHelper->ActivateSidelinkBearer (Seconds(0.0), txUe, tft);
        tft = Create<LteSlTft> (LteSlTft::RECEIVE, clientRespondersAddress, groupL2Address);
        lteV2xHelper->ActivateSidelinkBearer (Seconds(0.0), rxUes, tft);

        //Individual Socket Traffic Broadcast everyone
        Ptr<Socket> host = Socket::CreateSocket(txUe.Get(0)->GetNode(),TypeId::LookupByName ("ns3::UdpSocketFactory"));
        host->Bind();
        host->Connect(InetSocketAddress(clientRespondersAddress,application_port));
        host->SetAllowBroadcast(true);
        host->ShutdownRecv();



        Ptr<LteUeMac> ueMac = DynamicCast<LteUeMac>( txUe.Get (0)->GetObject<LteUeNetDevice> ()->GetMac () );
        if (vehID < numVeh)
            ueMac->TraceConnectWithoutContext ("SidelinkV2xAnnouncement", MakeBoundCallback (&SidelinkV2xAnnouncementMacTrace, host));

        Ptr<Socket> sink = Socket::CreateSocket(txUe.Get(0)->GetNode(),TypeId::LookupByName ("ns3::UdpSocketFactory"));
        sink->Bind(InetSocketAddress (Ipv4Address::GetAny (), application_port));
        if (vehID < numVeh)
            sink->SetRecvCallback (MakeCallback (&ReceivePacket));

        //store and increment addresses
        groupL2Addresses.push_back (groupL2Address);
        groupL2Address++;
        clientRespondersAddress = Ipv4AddressGenerator::NextAddress (Ipv4Mask ("255.0.0.0"));
    }

        NS_LOG_INFO ("Creating Sidelink Configuration...");
        Ptr<LteUeRrcSl> ueSidelinkConfiguration = CreateObject<LteUeRrcSl>();
        ueSidelinkConfiguration->SetSlEnabled(true);
        ueSidelinkConfiguration->SetV2xEnabled(true);

        LteRrcSap::SlV2xPreconfiguration preconfiguration;
        preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommPreconfigGeneral.carrierFreq = 54890;
        preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommPreconfigGeneral.slBandwidth = slBandwidth;
        
        preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.nbPools = 1;
        preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.nbPools = 1;

        SlV2xPreconfigPoolFactory pFactory;
        pFactory.SetHaveUeSelectedResourceConfig (true);
        pFactory.SetSlSubframe (std::bitset<20> (0xFFFFF));
        pFactory.SetAdjacencyPscchPssch (adjacencyPscchPssch);
        pFactory.SetSizeSubchannel (sizeSubchannel);
        pFactory.SetNumSubchannel (numSubchannel);
        pFactory.SetStartRbSubchannel (startRbSubchannel);
        pFactory.SetStartRbPscchPool (0);
        pFactory.SetDataTxP0 (-4);
        pFactory.SetDataTxAlpha (0.9);

        preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.pools[0] = pFactory.CreatePool ();
        preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.pools[0] = pFactory.CreatePool ();
        ueSidelinkConfiguration->SetSlV2xPreconfiguration (preconfiguration); 

        // Print Configuration
        *log_rx_data->GetStream() << "RxPackets;RxTime;RxId;TxId;TxTime;xPos;yPos" << std::endl;

        NS_LOG_INFO ("Installing Sidelink Configuration...");
        lteHelper->InstallSidelinkV2xConfiguration (ueDevs, ueSidelinkConfiguration);

        NS_LOG_INFO ("Enabling LTE traces...");
        lteHelper->EnableTraces();

        // *log_simtime->GetStream() << "Simtime;TotalRx;TotalTx;PRR" << std::endl; 
        Simulator::Schedule(Seconds(1), &PrintStatus, 1);
        Simulator::Schedule(Seconds(1.0), &SaveDistance);

        NS_LOG_INFO ("Starting Simulation...");
        Simulator::Stop(MilliSeconds(simTime*1000+40));
        Simulator::Run();
        Simulator::Destroy();

        NS_LOG_INFO("Simulation done.");

        end = std::clock();
        std::cout << (double)(end - start) / CLOCKS_PER_SEC << std::endl;

        CutInMobilityModel::PrintPositionError(log_poisition_error);
        return 0;  
}   