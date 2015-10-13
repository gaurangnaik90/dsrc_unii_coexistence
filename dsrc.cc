#include "ns3/node.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/node-container.h"
#include "ns3/net-device-container.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/seq-ts-header.h"
#include "ns3/wave-net-device.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/wave-helper.h"
#include "ns3/wave-bsm-helper.h"
#include "ns3/snr-tag.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/string.h"
#include "ns3/netanim-module.h"
#include "ns3/command-line.h"
#include "ns3/double.h"

using namespace ns3;
class WaveNetDeviceExample
 {
public:
  void SendWsmpExample (uint32_t noPackets, uint32_t packetSize, uint32_t simTime, float interval, double gpsAccuracyNs, int nodeSpeed, int nodePause);
  bool mobility;
  uint32_t m_rxPacketCounter;  
 
private:
  void SendOneWsmpPacket (uint32_t channel, uint32_t seq);
  bool Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender);
  void CreateWaveNodes ();

  uint32_t m_noPackets;
  uint32_t m_packetSize;
  std::vector <double> m_txSafetyRanges;
  double m_gpsAccuracyNs;
  float m_interval;
  int64_t m_streamIndex;
  uint32_t m_simTime;
  int m_nodeSpeed; //in m/s
  int m_nodePause; //in s
  double m_txSafetyRange1;
  double m_txSafetyRange2;
  double m_txSafetyRange3;
  double m_txSafetyRange4;
  double m_txSafetyRange5;
  double m_txSafetyRange6;
  double m_txSafetyRange7;
  double m_txSafetyRange8;
  double m_txSafetyRange9;
  double m_txSafetyRange10;

  WaveBsmHelper m_waveBsmHelper;

  NodeContainer nodes;
  NetDeviceContainer devices;
};

void
WaveNetDeviceExample::CreateWaveNodes ()
{
  nodes = NodeContainer ();
  nodes.Create (2);
 
  if (!mobility)
  {
  std::cout<<"WAVE Devices following constant mobility model\n";
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (50.0, 0.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);
  }
  else
  {
  //mobility starts here............................................................................
  std::cout<<"WAVE Devices following RandomWaypoint mobility model\n";
  MobilityHelper mobilityAdhoc;
  ObjectFactory pos;
  pos.SetTypeId ("ns3::RandomBoxPositionAllocator");
  pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=100.0]"));
  pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=100.0]"));
  // we need antenna height uniform [1.0 .. 2.0] for loss model
  pos.Set ("Z", StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=2.0]"));

  Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
  m_streamIndex += taPositionAlloc->AssignStreams (m_streamIndex);

  std::stringstream ssSpeed;
  ssSpeed << "ns3::UniformRandomVariable[Min=0.0|Max=" << m_nodeSpeed << "]";
  std::stringstream ssPause;
  ssPause << "ns3::ConstantRandomVariable[Constant=" << m_nodePause << "]";
  mobilityAdhoc.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                              "Speed", StringValue (ssSpeed.str ()),
                              "Pause", StringValue (ssPause.str ()),
                              "PositionAllocator", PointerValue (taPositionAlloc));
  mobilityAdhoc.SetPositionAllocator (taPositionAlloc);
  mobilityAdhoc.Install (nodes);
  m_streamIndex += mobilityAdhoc.AssignStreams (nodes, m_streamIndex);

  // initially assume all nodes are moving
  WaveBsmHelper::GetNodesMoving ().resize (2, 1); //no_nodes=2
  //mobility ends here............................................................................
  }

  YansWifiChannelHelper waveChannel = YansWifiChannelHelper::Default ();
  YansWavePhyHelper wavePhy =  YansWavePhyHelper::Default ();
  //std::cout<<"Frequency = "<<wavePhy.GetFrequency<<std::endl;
  wavePhy.SetChannel (waveChannel.Create ());
  wavePhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
  QosWaveMacHelper waveMac = QosWaveMacHelper::Default ();
  WaveHelper waveHelper = WaveHelper::Default ();
  devices = waveHelper.Install (wavePhy, waveMac, nodes);
 
  for (uint32_t i = 0; i != devices.GetN (); ++i)
     {
      Ptr<WaveNetDevice> device = DynamicCast<WaveNetDevice> (devices.Get (i));
      device->SetReceiveCallback (MakeCallback (&WaveNetDeviceExample::Receive, this));
     }

  InternetStackHelper internet;
  internet.Install (nodes);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.1.2.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = ipv4.Assign (devices);
 
   // Tracing
  wavePhy.EnablePcap ("wave-simple-device", devices);

  //m_waveBsmHelper.Install ( interfaces, Seconds(m_simTime), m_packetSize, Seconds(m_interval), m_gpsAccuracyNs/1000000.0, m_txSafetyRanges, 1, Seconds(0.0));
}

bool
WaveNetDeviceExample::Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender)
{
  
  m_rxPacketCounter++;
  SeqTsHeader seqTs;
  pkt->PeekHeader (seqTs);
  SnrTag tag;
  
  if (pkt->PeekPacketTag(tag)){
      std::cout << "WAVE receive a packet: " << std::endl
            << "  sequence = " << seqTs.GetSeq () << "," << std::endl
            << "  sendTime = " << seqTs.GetTs ().GetSeconds () << "s," << std::endl
            << "  recvTime = " << Now ().GetSeconds () << "s" << std::endl
            << "  SNR      = " << tag.Get()<<std::endl<<std::endl;   
  }
  return true;
}
 
void
WaveNetDeviceExample::SendOneWsmpPacket  (uint32_t channel, uint32_t seq)
{
  Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices.Get (0));
  Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices.Get (1));
  const static uint16_t WSMP_PROT_NUMBER = 0x88DC;
  Mac48Address bssWildcard = Mac48Address::GetBroadcast ();
 
  const TxInfo txInfo = TxInfo (channel);
  Ptr<Packet> p  = Create<Packet> (m_packetSize);
  SeqTsHeader seqTs;
  seqTs.SetSeq (seq);
  p->AddHeader (seqTs);
  sender->SendX  (p, bssWildcard, WSMP_PROT_NUMBER, txInfo);
}

void
WaveNetDeviceExample::SendWsmpExample (uint32_t noPackets, uint32_t packetSize, uint32_t simTime, float interval, double gpsAccuracyNs, int nodeSpeed, int nodePause)
{
  //m_rxPacketCounter = 0;
  m_noPackets = noPackets;
  m_packetSize = packetSize;
  m_gpsAccuracyNs = gpsAccuracyNs;
  m_interval = interval;
  m_simTime = simTime;
  m_nodeSpeed = nodeSpeed;
  m_nodePause = nodePause;
  double txDist1 = 50.0;
  double txDist2 = 100.0;
  double txDist3 = 150.0;
  double txDist4 = 200.0;
  double txDist5 = 250.0;
  double txDist6 = 300.0;
  double txDist7 = 350.0;
  double txDist8 = 350.0;
  double txDist9 = 350.0;
  double txDist10 = 350.0;

  m_txSafetyRange1 = txDist1;
  m_txSafetyRange2 = txDist2;
  m_txSafetyRange3 = txDist3;
  m_txSafetyRange4 = txDist4;
  m_txSafetyRange5 = txDist5;
  m_txSafetyRange6 = txDist6;
  m_txSafetyRange7 = txDist7;
  m_txSafetyRange8 = txDist8;
  m_txSafetyRange9 = txDist9;
  m_txSafetyRange10 = txDist10;

  m_txSafetyRanges.resize (10, 0);
  m_txSafetyRanges[0] = m_txSafetyRange1;
  m_txSafetyRanges[1] = m_txSafetyRange2;
  m_txSafetyRanges[2] = m_txSafetyRange3;
  m_txSafetyRanges[3] = m_txSafetyRange4;
  m_txSafetyRanges[4] = m_txSafetyRange5;
  m_txSafetyRanges[5] = m_txSafetyRange6;
  m_txSafetyRanges[6] = m_txSafetyRange7;
  m_txSafetyRanges[7] = m_txSafetyRange8;
  m_txSafetyRanges[8] = m_txSafetyRange9;
  m_txSafetyRanges[9] = m_txSafetyRange10;

  CreateWaveNodes ();
  Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices.Get (0));
  Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices.Get (1));
 
  // Alternating access without immediate channel switch
  const SchInfo schInfo = SchInfo (SCH1, false, EXTENDED_ALTERNATING);
  Simulator::Schedule (Seconds (0.0), &WaveNetDevice::StartSch,sender,schInfo);
  Simulator::Schedule (Seconds (0.0), &WaveNetDevice::StartSch, receiver, schInfo); // An important point is that the receiver should also be assigned channel access for the same channel to receive packets.
 
  for (uint32_t i=1; i<= m_noPackets; i++)
  {
    Simulator::Schedule (Seconds (m_interval*i), &WaveNetDeviceExample::SendOneWsmpPacket,  this, CCH, 2*i-1);
    Simulator::Schedule (Seconds (m_interval*i), &WaveNetDeviceExample::SendOneWsmpPacket,  this, SCH1, 2*i);
  }
   
}

class Interferer
{
public: 
  void Initialize (uint32_t iPacketSize, uint32_t inoPackets, double iInterval, double iStartTime, double iDistanceToRx, std::string phyMode);
  void InterfererSetup ();

private: 
  static void PrintReceivedPacket (Ptr<Socket> socket);
  static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize, uint32_t pktCount, Time pktInterval);
  void ShowFreq (std::string path);
  NodeContainer nodes;
  NetDeviceContainer devices;
  uint32_t m_iPacketSize;
  uint32_t m_inoPackets;
  double m_iInterval; // seconds
  double m_iStartTime; // seconds
  double m_iDistanceToRx; // meters
  std::string m_phyMode;
};

void 
Interferer::Initialize(uint32_t iPacketSize, uint32_t inoPackets, double iInterval, double iStartTime, double iDistanceToRx, std::string phyMode)
{
  m_iPacketSize = iPacketSize;
  m_inoPackets = inoPackets;
  m_iInterval = iInterval;
  m_iStartTime = iStartTime;
  m_iDistanceToRx = iDistanceToRx;
  m_phyMode = phyMode;
}

void
Interferer::PrintReceivedPacket (Ptr<Socket> socket)
{
  
  Address addr;
  std::ostringstream oss;
 

  while (socket->Recv ())
    {
      socket->GetSockName (addr);
      InetSocketAddress iaddr = InetSocketAddress::ConvertFrom (addr);

      oss << "802.11a Received one packet!  Socket: " << iaddr.GetIpv4 () << " port: " << iaddr.GetPort ()<< "time: "<<Simulator::Now().GetSeconds()<<" s";
    }

  std::cout<<oss.str()<<std::endl;

}

void 
Interferer::GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
      socket->Send (Create<Packet> (pktSize));
      std::cout<<"802.11a Packet sent at time = "<<Simulator::Now().GetSeconds()<<" s"<<std::endl;
      Simulator::Schedule (pktInterval, &Interferer::GenerateTraffic,
                           socket, pktSize,pktCount-1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
}

void
Interferer::ShowFreq (std::string path)
{
	std::cout<<"In MakeCallback method\n"<<path<<std::endl;
}

void
Interferer::InterfererSetup()
{
  nodes.Create(2);
  WifiHelper wifi;
  wifi.SetStandard(WIFI_PHY_STANDARD_80211a);

  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  wifiPhy.Set ("RxGain", DoubleValue (0) );
  //std::cout<<"Carrier Frequency = "<<wifiPhy.getFrequency()<<std::endl; 
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel");
  wifiPhy.SetChannel (wifiChannel.Create ());

  // Add a non-QoS upper mac, and disable rate control
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (m_phyMode),
                                "ControlMode",StringValue (m_phyMode));
  
  wifiMac.SetType ("ns3::AdhocWifiMac");
  devices = wifi.Install (wifiPhy, wifiMac, nodes.Get (0));
  //wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (0.0) );
  wifiPhy.Set ("TxGain", DoubleValue (10) );
  devices.Add (wifi.Install (wifiPhy, wifiMac, nodes.Get (1)));
  
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (m_iDistanceToRx, 0.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);

  InternetStackHelper internet;
  internet.Install (nodes);
  
  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket (nodes.Get (0), tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address ("10.1.1.1"), 80);
  recvSink->Bind (local);
  recvSink->SetRecvCallback (MakeCallback (&PrintReceivedPacket));

  Ptr<Socket> source = Socket::CreateSocket (nodes.Get (1), tid);
  InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
  source->SetAllowBroadcast (true);
  source->Connect (remote);
  
  //Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/Frequency", MakeCallback(&Interferer::ShowFreq));

  Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
                                  Seconds (m_iStartTime), &GenerateTraffic,
                                  source, m_iPacketSize, m_inoPackets, Seconds(m_iInterval));
}


int 
main (int argc, char *argv[])
{
  //----------------------- WAVE Node Configuration ----------------------------------------------
  uint32_t packetSize = 100;
  uint32_t noPackets = 100;
  uint32_t simTime = 1000;
  float interval = 0.1;
  double gpsAccuracyNs = 40;  
  int nodeSpeed = 10;
  int nodePause = 0;
  bool nodeMobility = false;
  //----------------------- IEEE 802.11a Node Configuration ----------------------------------------------
  std::string phyMode ("DsssRate1Mbps");
  uint32_t iPacketSize = 1000;
  uint32_t inoPackets = 100;
  double iInterval = 0.1; // seconds
  double iStartTime = 0.1; // seconds
  double iDistanceToRx = 75.0; // meters
  
  //----------------------- Command Line inputs ----------------------------------------------
  CommandLine cmd;
  cmd.AddValue ("nodeMobility", "Whether nodes or mobile or not", nodeMobility);
  cmd.AddValue ("packetSize", "Packet Size", packetSize);
  cmd.AddValue ("iPacketSize", "Interferer Packet Size", iPacketSize);
  cmd.AddValue ("noPackets", "Number of Packets", noPackets);
  cmd.AddValue ("inoPackets", "Interferer Number of Packets", inoPackets);
  cmd.AddValue ("interval", "Interval between packets)", interval);
  cmd.AddValue ("iInterval", "Interval between interferer packets)", iInterval);
  cmd.AddValue ("gpsAccuracyNs", "gpsAccuracy in nanoseconds", gpsAccuracyNs);
  cmd.AddValue ("nodeSpeed", "node Speed in m/s", nodeSpeed);
  cmd.AddValue ("nodePause", "node pause in seconds", nodePause);
  cmd.AddValue ("iStartTime", "Interference Start Time", iStartTime);
  cmd.AddValue ("iDistanceToRx", "Distance between Interferer source and receiver", iDistanceToRx);
  cmd.Parse (argc, argv);

 // Wave Traffic
  WaveNetDeviceExample example;
  example.mobility=nodeMobility;
  example.SendWsmpExample (noPackets, packetSize, simTime, interval, gpsAccuracyNs, nodeSpeed, nodePause);

  // 802.11a Traffic
  Interferer interferer;
  interferer.Initialize (iPacketSize, inoPackets, iInterval, iStartTime, iDistanceToRx, phyMode);
  interferer.InterfererSetup();


  Simulator::Stop (Seconds (simTime));
  AnimationInterface anim ("animation.xml");
  Simulator::Run ();
  Simulator::Destroy ();
  //std::cout<<"rxPacketCounter = "<<example.m_rxPacketCounter<<std::endl;
  std::cout<<"PDR = "<<example.m_rxPacketCounter/(2.0*noPackets)<<std::endl;

  return 0;
}

