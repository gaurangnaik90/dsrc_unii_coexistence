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

using namespace ns3;
class WaveNetDeviceExample
 {
public:
  uint32_t SendWsmpExample (uint32_t noPackets, uint32_t packetSize, uint32_t simTime, float interval, double gpsAccuracyNs, int nodeSpeed, int nodePause);
 
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
  uint32_t m_rxPacketCounter;  

  WaveBsmHelper m_waveBsmHelper;

  NodeContainer nodes;
  NetDeviceContainer devices;
};

void
WaveNetDeviceExample::CreateWaveNodes ()
{
  nodes = NodeContainer ();
  nodes.Create (2);
 
/*  
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (50.0, 0.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);
  */


  //mobility starts here............................................................................

  MobilityHelper mobilityAdhoc;

  ObjectFactory pos;
  pos.SetTypeId ("ns3::RandomBoxPositionAllocator");
  pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1500.0]"));
  pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));
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


  YansWifiChannelHelper waveChannel = YansWifiChannelHelper::Default ();
  YansWavePhyHelper wavePhy =  YansWavePhyHelper::Default ();
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
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
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
      std::cout<<"Received Packet with SNR = " << tag.Get()<<std::endl; //Display SNR
  }
  
  
  std::cout << "receive a packet: " << std::endl
            << "  sequence = " << seqTs.GetSeq () << "," << std::endl
            << "  sendTime = " << seqTs.GetTs ().GetSeconds () << "s," << std::endl
            << "  recvTime = " << Now ().GetSeconds () << "s," << std::endl;
           // << "  protocol = 0x" << std::hex << mode << std::dec  << std::endl;
 
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

uint32_t
WaveNetDeviceExample::SendWsmpExample (uint32_t noPackets, uint32_t packetSize, uint32_t simTime, float interval, double gpsAccuracyNs, int nodeSpeed, int nodePause)
{
  m_rxPacketCounter = 0;
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


  Simulator::Stop (Seconds (simTime));
  
  AnimationInterface anim ("animation.xml");

  Simulator::Run ();
  Simulator::Destroy ();
  return m_rxPacketCounter;
 }

int 
main (int argc, char *argv[])
{
  uint32_t packetSize = 500;
  uint32_t noPackets = 100;
  uint32_t simTime = noPackets/10+1;
  float interval = 0.1;
  double gpsAccuracyNs = 40;  
  int nodeSpeed = 30;
  int nodePause = 0;
  
  CommandLine cmd;
  cmd.AddValue ("packetSize", "Packet Size", packetSize);
  cmd.AddValue ("noPackets", "Number of Packets", noPackets);
  cmd.AddValue ("interval", "Interval between packets)", interval);
  cmd.AddValue ("gpsAccuracyNs", "gpsAccuracy in nanoseconds", gpsAccuracyNs);
  cmd.AddValue ("nodeSpeed", "node Speed in m/s", nodeSpeed);
  cmd.AddValue ("nodePause", "node pause in seconds", nodePause);
  cmd.Parse (argc, argv);

  WaveNetDeviceExample example;
  uint32_t rxPacketCounter=example.SendWsmpExample (noPackets, packetSize, simTime, interval, gpsAccuracyNs, nodeSpeed, nodePause);
  //std::cout<<"rxPacketCounter = "<<rxPacketCounter<<std::endl;
  std::cout<<"PDR = "<<rxPacketCounter/(2.0*noPackets)<<std::endl;
  return 0;
}
