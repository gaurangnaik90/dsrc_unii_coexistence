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
#include "ns3/uinteger.h"
#include <ns3/spectrum-helper.h>
#include <ns3/spectrum-analyzer-helper.h>
#include <ns3/random-variable-stream.h>
#include <ns3/config-store.h>

using namespace ns3;
class WaveNetDeviceExample
 {
public:
  void SendWsmpExample (Ptr<YansWifiChannel> wirelessChannel, uint32_t noPackets, uint32_t packetSize, uint32_t simTime, float interval, double gpsAccuracyNs, int nodeSpeed, int nodePause, double distanceToRx, double m_RxGain, double m_TxGain, double freq, uint32_t sifs, uint32_t pifs, uint32_t eifsnodifs, uint32_t rifs);
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
  double m_distanceToRx;
  double m_RxGain;
  double m_TxGain;
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
  double m_freq;
  uint32_t m_sifs;
  uint32_t m_pifs; 
  uint32_t m_eifsnodifs;
  uint32_t m_rifs;

  Ptr<YansWifiChannel> m_channel;
 
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
  //std::cout<<"WAVE Devices following constant mobility model\n";
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (m_distanceToRx, 0.0, 0.0));
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

  YansWavePhyHelper wavePhy =  YansWavePhyHelper::Default ();
  wavePhy.Set ("TxGain", DoubleValue (m_TxGain) );
  wavePhy.Set ("RxGain", DoubleValue (m_RxGain) );
  wavePhy.SetChannel (m_channel);
  wavePhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
  QosWaveMacHelper waveMac = QosWaveMacHelper::Default ();
  WaveHelper waveHelper = WaveHelper::Default ();
  waveMac.SetType("ns3::OcbWifiMac","Pifs",TimeValue(MicroSeconds(m_pifs)));
  waveMac.SetType("ns3::OcbWifiMac","Sifs",TimeValue(MicroSeconds(m_sifs)));
  waveMac.SetType("ns3::OcbWifiMac","EifsNoDifs",TimeValue(MicroSeconds(m_eifsnodifs)));
  waveMac.SetType("ns3::OcbWifiMac","Rifs",TimeValue(MicroSeconds(m_rifs)));
  devices = waveHelper.Install (wavePhy, waveMac, nodes);
 
  for (uint32_t i = 0; i != devices.GetN (); ++i)
     {
      Ptr<WaveNetDevice> device = DynamicCast<WaveNetDevice> (devices.Get (i));
      device->SetReceiveCallback (MakeCallback (&WaveNetDeviceExample::Receive, this));
     }

  InternetStackHelper internet;
  internet.Install (nodes);
  
  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.0.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = ipv4.Assign (devices);
 
   // Tracing
  wavePhy.EnablePcap ("wave-simple-device", devices);

}

bool
WaveNetDeviceExample::Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender)
{
  
  SeqTsHeader seqTs;
  pkt->PeekHeader (seqTs);
  SnrTag tag;
  
  if (pkt->PeekPacketTag(tag)){
    if (seqTs.GetSeq() < 1000000)
    //if (true) 
      {  
      m_rxPacketCounter++;
      std::cout << "WAVE receive a packet:"<< "  sendTime = " << seqTs.GetTs ().GetSeconds () << "s,"
            << "  recvTime = " << Now ().GetSeconds () << "s"
            << "  SNR = " << tag.Get() 
            << "  sequence = " << seqTs.GetSeq () << std::endl;
     }
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
WaveNetDeviceExample::SendWsmpExample (Ptr<YansWifiChannel> wirelessChannel, uint32_t noPackets, uint32_t packetSize, uint32_t simTime, float interval, double gpsAccuracyNs, int nodeSpeed, int nodePause, double distanceToRx, double RxGain, double TxGain, double freq, uint32_t sifs, uint32_t pifs, uint32_t eifsnodifs, uint32_t rifs)
{
  m_rxPacketCounter = 0;
  m_distanceToRx = distanceToRx;
  m_noPackets = noPackets;
  m_packetSize = packetSize;
  m_gpsAccuracyNs = gpsAccuracyNs;
  m_interval = interval;
  m_simTime = simTime;
  m_nodeSpeed = nodeSpeed;
  m_nodePause = nodePause;
  m_RxGain = RxGain;
  m_TxGain = TxGain;
  m_freq = freq;
  m_channel = wirelessChannel;
  m_sifs = sifs;
  m_pifs = pifs;
  m_eifsnodifs = eifsnodifs;
  m_rifs = rifs;  

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
  Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable> ();
  startTimeSeconds->SetAttribute ("Min", DoubleValue (0.0));
  startTimeSeconds->SetAttribute ("Max", DoubleValue (0.01));  
  //startTimeSeconds->GetValue ();

  for (uint32_t i=1; i<= m_noPackets; i++)
  {
    Simulator::Schedule (Seconds (m_interval*i), &WaveNetDeviceExample::SendOneWsmpPacket,  this, CCH, i);
    //Simulator::Schedule (Seconds (m_interval*i), &WaveNetDeviceExample::SendOneWsmpPacket,  this, CCH, 2*i-1);
    //Simulator::Schedule (Seconds (m_interval*i), &WaveNetDeviceExample::SendOneWsmpPacket,  this, SCH1, 2*i);
  }
   
}

class Interferer
 {
public:
  void SendWsmpExample (Ptr<YansWifiChannel> wirelessChannel, uint32_t noPackets, uint32_t packetSize, uint32_t simTime, float interval, double gpsAccuracyNs, int nodeSpeed, int nodePause, double distanceToRx, double m_RxGain, double m_TxGain, double freq, uint32_t sifs, uint32_t pifs, uint32_t eifsnodifs, uint32_t rifs);
  void SeeIfsValues (std::string context);  
  bool mobility;
  uint32_t m_rxPacketCounter;  
  bool iverbose;

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
  double m_distanceToRx;
  double m_RxGain;
  double m_TxGain;
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
  double m_freq;
  uint32_t m_sifs;
  uint32_t m_pifs;  
  uint32_t m_eifsnodifs;
  uint32_t m_rifs;

  Ptr<YansWifiChannel> m_channel;
 
  WaveBsmHelper m_waveBsmHelper;

  NodeContainer nodes;
  NetDeviceContainer devices;
};

void
Interferer::SeeIfsValues (std::string context)
{
  std::cout<<"I am here\n";
}

void
Interferer::CreateWaveNodes ()
{
  nodes = NodeContainer ();
  nodes.Create (2);
 
  if (!mobility)
  {
  //std::cout<<"WAVE Devices following constant mobility model\n";
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0, 25.0, 0.0));
  positionAlloc->Add (Vector (m_distanceToRx, 25.0, 0.0));
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

  YansWavePhyHelper wavePhy =  YansWavePhyHelper::Default ();
  wavePhy.Set ("TxGain", DoubleValue (m_TxGain) );
  wavePhy.Set ("RxGain", DoubleValue (m_RxGain) );
  //std::cout<<"WAVE Carrier Frequency: (MHz) "<<m_freq<<std::endl;
  //wavePhy.Set ("Frequency", UintegerValue (m_freq) );
  wavePhy.SetChannel (m_channel);
  wavePhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
  QosWaveMacHelper waveMac = QosWaveMacHelper::Default ();
  WaveHelper waveHelper = WaveHelper::Default ();
  waveMac.SetType("ns3::OcbWifiMac","Pifs",TimeValue(MicroSeconds(m_pifs)));
  waveMac.SetType("ns3::OcbWifiMac","Sifs",TimeValue(MicroSeconds(m_sifs)));
  waveMac.SetType("ns3::OcbWifiMac","EifsNoDifs",TimeValue(MicroSeconds(m_eifsnodifs)));
  waveMac.SetType("ns3::OcbWifiMac","Rifs",TimeValue(MicroSeconds(m_rifs)));
  //Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/$ns3::OcbWifiMac",MakeCallback(&Interferer::SeeIfsValues, this)); //Does not work anyway!
  devices = waveHelper.Install (wavePhy, waveMac, nodes);
 
  for (uint32_t i = 0; i != devices.GetN (); ++i)
     {
      Ptr<WaveNetDevice> device = DynamicCast<WaveNetDevice> (devices.Get (i));
      device->SetReceiveCallback (MakeCallback (&Interferer::Receive, this));
     }

  InternetStackHelper internet;
  internet.Install (nodes);
  
  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.0.2.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = ipv4.Assign (devices);
 
   // Tracing
  wavePhy.EnablePcap ("wave-simple-device", devices);
}

bool
Interferer::Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender)
{
  
  m_rxPacketCounter++;
  SeqTsHeader seqTs;
  pkt->PeekHeader (seqTs);
  SnrTag tag;
  if (iverbose){
      if (pkt->PeekPacketTag(tag)){
      	   std::cout << "Interference receiver received a packet:"<< "  sendTime = " << seqTs.GetTs ().GetSeconds () << "s,"
                     << "  recvTime = " << Now ().GetSeconds () << "s"
                     << "  SNR = " << tag.Get() 
                     << "  sequence = " << seqTs.GetSeq () << std::endl;
      }
  }
  return true;
}
 
void
Interferer::SendOneWsmpPacket  (uint32_t channel, uint32_t seq)
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
Interferer::SendWsmpExample (Ptr<YansWifiChannel> wirelessChannel, uint32_t noPackets, uint32_t packetSize, uint32_t simTime, float interval, double gpsAccuracyNs, int nodeSpeed, int nodePause, double distanceToRx, double RxGain, double TxGain, double freq, uint32_t sifs, uint32_t pifs, uint32_t eifsnodifs, uint32_t rifs)
{
  m_rxPacketCounter = 0;
  m_distanceToRx = distanceToRx;
  m_noPackets = noPackets;
  m_packetSize = packetSize;
  m_gpsAccuracyNs = gpsAccuracyNs;
  m_interval = interval;
  m_simTime = simTime;
  m_nodeSpeed = nodeSpeed;
  m_nodePause = nodePause;
  m_RxGain = RxGain;
  m_TxGain = TxGain;
  m_freq = freq;
  m_channel = wirelessChannel;
  m_sifs = sifs;
  m_pifs = pifs; 
  m_eifsnodifs = eifsnodifs;
  m_rifs = rifs;

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
  
  //Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable> ();
  //startTimeSeconds->SetAttribute ("Min", DoubleValue (0.0));
  //startTimeSeconds->SetAttribute ("Max", DoubleValue (0.01));
  //startTimeSeconds->GetValue ();
  for (uint32_t i=1; i<= m_noPackets; i++)
  { 
    //Simulator::Schedule (Seconds (startTimeSeconds->GetValue () + m_interval*i), &Interferer::SendOneWsmpPacket,  this, CCH, 1000000+2*i-1);
    //Simulator::Schedule (Seconds (startTimeSeconds->GetValue () + m_interval*i), &Interferer::SendOneWsmpPacket,  this, CCH, 1000000+2*i);
    Simulator::Schedule (Seconds (m_interval*i), &Interferer::SendOneWsmpPacket,  this, CCH, 1000000+2*i-1);
    Simulator::Schedule (Seconds (m_interval*i), &Interferer::SendOneWsmpPacket,  this, CCH, 1000000+2*i);
    //Simulator::Schedule (Seconds (m_interval*i), &Interferer::SendOneWsmpPacket,  this, CCH, 1000+2*i-1);
    //Simulator::Schedule (Seconds (m_interval*i), &Interferer::SendOneWsmpPacket,  this, SCH1, 1000+2*i);
  }
   
}

int 
main (int argc, char *argv[])
{
  //----------------------- WAVE Node Configuration ----------------------------------------------
  uint32_t packetSize = 1000;
  uint32_t noPackets = 100;
  uint32_t simTime = 1000;
  float interval = 0.1;
  double gpsAccuracyNs = 40;  
  int nodeSpeed = 10;
  int nodePause = 0;
  bool nodeMobility = false;
  double distanceToRx = 50.0;
  double RxGain = 0;
  double TxGain = 0;
  double freq = 5900;
  uint32_t sifs = 32;
  uint32_t pifs = 32 + 13;
  uint32_t eifsnodifs = 32 + 88;
  uint32_t rifs = 2;
  //----------------------- IEEE 802.11a Node Configuration ----------------------------------------------
  bool verbose = false;
  std::string phyMode ("DsssRate1Mbps");
  uint32_t iPacketSize = 1000;

  uint32_t inoPackets = 100;
  double iInterval = 0.1; // seconds
  double iStartTime = 0.1; // seconds
  double iDistanceToRx = 55.0; // meters
  uint32_t ifreq = 5900; //not sure of the value yet
  double iRxGain = 0;
  double iTxGain = 0;  
  uint32_t isifs = 16;
  uint32_t ipifs = 25;
  uint32_t ieifsnodifs = 60;
  uint32_t irifs = 2; 
  //----------------------- Command Line inputs ----------------------------------------------
  CommandLine cmd;
  cmd.AddValue ("nodeMobility", "Whether nodes or mobile or not", nodeMobility);
  cmd.AddValue ("packetSize", "Packet Size", packetSize);
  cmd.AddValue ("iPacketSize", "Interferer Packet Size", iPacketSize);
  cmd.AddValue ("noPackets", "Number of Packets", noPackets);
  cmd.AddValue ("inoPackets", "Interferer Number of Packets", inoPackets);
  cmd.AddValue ("interval", "Interval between packets", interval);
  cmd.AddValue ("iInterval", "Interval between interferer packets)", iInterval);
  cmd.AddValue ("gpsAccuracyNs", "gpsAccuracy in nanoseconds", gpsAccuracyNs);
  cmd.AddValue ("nodeSpeed", "node Speed in m/s", nodeSpeed);
  cmd.AddValue ("nodePause", "node pause in seconds", nodePause);
  cmd.AddValue ("iStartTime", "Interference Start Time", iStartTime);
  cmd.AddValue ("distanceToRx", "Distance between WAVE source and receiver (stationary case)", distanceToRx);
  cmd.AddValue ("iDistanceToRx", "Distance between Interferer source and receiver", iDistanceToRx);
  cmd.AddValue ("freq", "Frequency of WAVE node", freq);
  cmd.AddValue ("ifreq", "Frequency of 802.11a node", ifreq);
  cmd.AddValue ("RxGain", "Gain of Receiver", RxGain);
  cmd.AddValue ("TxGain", "Gain of Transmitter", TxGain);
  cmd.AddValue ("iRxGain", "Gain of Interfering Receiver", iRxGain);
  cmd.AddValue ("iTxGain", "Gain of Interfering Transmitter", iTxGain);
  cmd.AddValue ("verbose", "Interference Reception Verbose", verbose);
  cmd.AddValue ("sifs", "SIFS of WAVE", sifs);
  cmd.AddValue ("isifs", "SIFS of Interferer", isifs);
  cmd.AddValue ("pifs", "PIFS of WAVE", pifs);
  cmd.AddValue ("ipifs", "PIFS of Interferer", ipifs);
  cmd.AddValue ("eifsnodifs", "EIFSnoDIFS of WAVE", eifsnodifs);
  cmd.AddValue ("ieifsnodifs", "EIFSnoDIFS of Interferer", ieifsnodifs);
  cmd.AddValue ("rifs", "RIFS of WAVE", rifs);
  cmd.AddValue ("irifs", "RIFS of Interferer", irifs);
  cmd.Parse (argc, argv);
  // --------------------------------------------------------------------------------------
  
  ConfigStore config;
  config.ConfigureDefaults ();
  

  //---------------------------------------------------------------------------------------
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  channel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  Ptr<YansWifiChannel> wirelessChannel = channel.Create ();
  
 
  //-----------------------------------------Spectrum Channel ----------------------------------
  SpectrumChannelHelper channelHelper = SpectrumChannelHelper::Default ();
  channelHelper.SetChannel ("ns3::MultiModelSpectrumChannel");
  Ptr<SpectrumChannel> spectrumChannel = channelHelper.Create ();  
  
  /*
  //-----------------------------------------Spectrum Analyzer ----------------------------------
  NodeContainer spectrumAnalyzerNodes;
  spectrumAnalyzerNodes.Create (1);

  std::vector<double> freqs;
  for (int i = 0; i < 400; i+=10)
    {
      freqs.push_back ((i + 5700) * 1e6);
    }
  Ptr<SpectrumModel> spectrumAnalyzerFreqModel = Create<SpectrumModel> (freqs);
  SpectrumAnalyzerHelper spectrumAnalyzerHelper;
  spectrumAnalyzerHelper.SetChannel (spectrumChannel);
  spectrumAnalyzerHelper.SetRxSpectrumModel (spectrumAnalyzerFreqModel);
  spectrumAnalyzerHelper.SetPhyAttribute ("NoisePowerSpectralDensity", DoubleValue (1e-15)); // -120 dBm/Hz
  spectrumAnalyzerHelper.EnableAsciiAll ("spectrum-analyzer");
  NetDeviceContainer spectrumAnalyzerDevices = spectrumAnalyzerHelper.Install (spectrumAnalyzerNodes);
  //----------------------------------------------------------------------------------------------------
  */

  //------------------------------------------Wave Traffic----------------------------------------------
  WaveNetDeviceExample example;
  example.mobility=nodeMobility;
  example.SendWsmpExample (wirelessChannel, noPackets, packetSize, simTime, interval, gpsAccuracyNs, nodeSpeed, nodePause, distanceToRx, RxGain, TxGain, freq, sifs, pifs, eifsnodifs, rifs);

  //------------------------------------------ Interferer Traffic--------------------------------------------
  Interferer interferer;
  interferer.mobility = false;
  interferer.iverbose = verbose;
  interferer.SendWsmpExample (wirelessChannel, inoPackets, iPacketSize, simTime, interval, gpsAccuracyNs, nodeSpeed, nodePause, iDistanceToRx, iRxGain, iTxGain, freq, isifs, ipifs, ieifsnodifs, irifs);

  config.ConfigureAttributes ();

  Simulator::Stop (Seconds (simTime));
  AnimationInterface anim ("animation.xml");
  Simulator::Run ();
  Simulator::Destroy ();
  //std::cout<<"rxPacketCounter = "<<example.m_rxPacketCounter<<std::endl;
  //std::cout<<"PDR = "<<example.m_rxPacketCounter/(2.0*noPackets)<<std::endl; //This is if packets are being sent on CCH as well as SCH
  std::cout<<"PDR = "<<example.m_rxPacketCounter/(1.0*noPackets)<<std::endl;

  return 0;
}

