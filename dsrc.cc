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
#include "ns3/snr-tag.h"

using namespace ns3;
class WaveNetDeviceExample
 {
public:
  void SendWsmpExample (uint32_t noPackets, uint32_t packetSize);
 
private:
  void SendOneWsmpPacket (uint32_t channel, uint32_t seq);
  bool Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender);
  void CreateWaveNodes (uint32_t noPackets, uint32_t packetSize);

  uint32_t m_noPackets;
  uint32_t m_packetSize;
  NodeContainer nodes;
  NetDeviceContainer devices;
};

void
WaveNetDeviceExample::CreateWaveNodes (uint32_t noPackets, uint32_t packetSize)
{
  nodes = NodeContainer ();
  nodes.Create (2);
 
  m_noPackets = noPackets;
  m_packetSize = packetSize;

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (50.0, 0.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);
 
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
      //std::cout<<"Received Packet with SNR = " << tag.Get()<<std::endl; //Display SNR
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

void
WaveNetDeviceExample::SendWsmpExample (uint32_t noPackets, uint32_t packetSize)
{
  CreateWaveNodes (noPackets, packetSize);
  Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices.Get (0));
  Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices.Get (1));
 
  // Alternating access without immediate channel switch
  const SchInfo schInfo = SchInfo (SCH1, false, EXTENDED_ALTERNATING);
  Simulator::Schedule (Seconds (0.0), &WaveNetDevice::StartSch,sender,schInfo);
  Simulator::Schedule (Seconds (0.0), &WaveNetDevice::StartSch, receiver, schInfo); // An important point is that the receiver should also be assigned channel access for the same channel to receive packets.
 
  for (uint32_t i=1; i<= m_noPackets; i++)
  {
    Simulator::Schedule (Seconds (0.1*i), &WaveNetDeviceExample::SendOneWsmpPacket,  this, CCH, i);
  }


  Simulator::Stop (Seconds (noPackets+10));
  Simulator::Run ();
  Simulator::Destroy ();
 }

int 
main (int argc, char *argv[])
{
  WaveNetDeviceExample example;
  uint32_t packetSize = 1000;
  uint32_t noPackets = 100;
  std::cout << "run WAVE WSMP routing service case:" << std::endl;
  example.SendWsmpExample (noPackets, packetSize);
  return 0;
}
