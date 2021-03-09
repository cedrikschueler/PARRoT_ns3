/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *
 * This file was created upon the basis of ns-3's DSDV implementation by Hemanth Narra <hemanth@ittc.ku.com>
 *
 * Modified by: Cedrik Schüler <cedrik.schueler@tu-dortmund.de> (08.03.2021)
 * THIS ALLOWS FREE USE OF THIS SOFTWARE IN ITS "AS IS" CONDITION AND
 * DISCLAIM ANY LIABILITY OF ANY KIND FOR ANY DAMAGES WHATSOEVER RESULTING
 * FROM THE USE OF THIS SOFTWARE.
 *
 * Communication Networks Institute (CNI)
 * TU Dortmund University, Germany
 * Otto-Hahn-Str. 6
 * 44227 Dortmund
 */

#include "parrot-routing-protocol.h"
#include "ns3/inet-socket-address.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/udp-socket-factory.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/integer.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("PARRoTRoutingProtocol");

namespace parrot {

NS_OBJECT_ENSURE_REGISTERED (RoutingProtocol);

/// UDP Port for PARRoT control traffic
const uint32_t RoutingProtocol::PARROT_PORT = 2201;

TypeId
RoutingProtocol::GetTypeId (void)
{
  static TypeId tid =
      TypeId ("ns3::parrot::RoutingProtocol")
          .SetParent<Ipv4RoutingProtocol> ()
          .SetGroupName ("PARRoT")
          .AddConstructor<RoutingProtocol> ()
          .AddAttribute ("ChirpInterval", "Periodic interval for chirp exchange",
                         TimeValue (Seconds (0.5)),
                         MakeTimeAccessor (&RoutingProtocol::mhChirpInterval), MakeTimeChecker ())
          .AddAttribute ("NeighborReliabilityTimeout", "Reliabitlity timeout for neighboring nodes",
                         TimeValue (Seconds (2.5)),
                         MakeTimeAccessor (&RoutingProtocol::m_neighborReliabilityTimeout),
                         MakeTimeChecker ())
          .AddAttribute ("MaxHops", "Maximum amount of hops for chirps", IntegerValue (32),
                         MakeIntegerAccessor (&RoutingProtocol::m_maxHops),
                         MakeIntegerChecker<int> ())
          .AddAttribute ("LearningRate", "Learning rate of the reinforcement learning",
                         DoubleValue (0.5), MakeDoubleAccessor (&RoutingProtocol::qFctAlpha),
                         MakeDoubleChecker<float> ())
          .AddAttribute ("DiscountFactor", "Discount factor of the reinforcement learning",
                         DoubleValue (0.8), MakeDoubleAccessor (&RoutingProtocol::qFctGamma),
                         MakeDoubleChecker<float> ())
          .AddAttribute ("CombinationMethod", "Combination method for metrics", StringValue ("M"),
                         MakeStringAccessor (&RoutingProtocol::combinationMethod),
                         MakeStringChecker ())
          .AddAttribute ("HistorySize", "Number of stored historical positions", IntegerValue (5),
                         MakeIntegerAccessor (&RoutingProtocol::historySize),
                         MakeIntegerChecker<int> ())
          .AddAttribute ("PredictionMethod", "Prediction method", StringValue ("waypoint"),
                         MakeStringAccessor (&RoutingProtocol::predictionMethod),
                         MakeStringChecker ())
          .AddAttribute ("RangeOffset", "Offset for communication range estimation",
                         DoubleValue (0.0), MakeDoubleAccessor (&RoutingProtocol::rangeOffset),
                         MakeDoubleChecker<double> ());

  return tid;
}

RoutingProtocol::RoutingProtocol () : m_periodicUpdateTimer (Timer::CANCEL_ON_DESTROY)
{
  m_uniformRandomVariable = CreateObject<UniformRandomVariable> ();
}

RoutingProtocol::~RoutingProtocol ()
{
}

void
RoutingProtocol::DoDispose ()
{
  m_ipv4 = 0;
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::iterator iter = m_socketAddresses.begin ();
       iter != m_socketAddresses.end (); iter++)
    {
      iter->first->Close ();
    }
  m_socketAddresses.clear ();
  Ipv4RoutingProtocol::DoDispose ();
}

void
RoutingProtocol::Start ()
{
  m_scb = MakeCallback (&RoutingProtocol::Send, this);
  m_ecb = MakeCallback (&RoutingProtocol::Drop, this);
  m_periodicUpdateTimer.SetFunction (&RoutingProtocol::SendMultiHopChirp, this);
  m_periodicUpdateTimer.Schedule (MicroSeconds (m_uniformRandomVariable->GetInteger (0, 100000)));

  r_com = CalculateCommunicationRange (2.75, 2.412e9, 20.0, -85.0, 1.0, 1.0);
  mobility = this->GetObject<Node> ()->GetObject<MobilityModel> ();

  m_selfIpv4Address = m_ipv4->GetAddress (1, 0).GetLocal ();
}

Ptr<Ipv4Route>
RoutingProtocol::LoopbackRoute (const Ipv4Header &hdr, Ptr<NetDevice> oif) const
{
  NS_ASSERT (m_lo != 0);
  Ptr<Ipv4Route> rt = Create<Ipv4Route> ();
  rt->SetDestination (hdr.GetDestination ());
  std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j = m_socketAddresses.begin ();
  if (oif)
    {
      // Iterate to find an address on the oif device
      for (j = m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
        {
          Ipv4Address addr = j->second.GetLocal ();
          int32_t interface = m_ipv4->GetInterfaceForAddress (addr);
          if (oif == m_ipv4->GetNetDevice (static_cast<uint32_t> (interface)))
            {
              rt->SetSource (addr);
              break;
            }
        }
    }
  else
    {
      rt->SetSource (j->second.GetLocal ());
    }
  NS_ASSERT_MSG (rt->GetSource () != Ipv4Address (), "Valid PARRoT source address not found");
  rt->SetGateway (Ipv4Address ("127.0.0.1"));
  rt->SetOutputDevice (m_lo);
  return rt;
}

Ptr<Ipv4Route>
RoutingProtocol::RouteOutput (Ptr<Packet> p, const Ipv4Header &header, Ptr<NetDevice> oif,
                              Socket::SocketErrno &sockerr)
{
  NS_LOG_FUNCTION (this << header << (oif ? oif->GetIfIndex () : 0));

  if (!p)
    {
      return LoopbackRoute (header, oif);
    }
  if (m_socketAddresses.empty ())
    {
      sockerr = Socket::ERROR_NOROUTETOHOST;
      NS_LOG_LOGIC ("No parrot interfaces");
      Ptr<Ipv4Route> route;
      return route;
    }
  sockerr = Socket::ERROR_NOTERROR;
  Ptr<Ipv4Route> route;
  Ipv4Address dst = header.GetDestination ();
  NS_LOG_DEBUG ("Packet Size: " << p->GetSize () << ", Packet id: " << p->GetUid ()
                                << ", Destination address in Packet: " << dst);
  RoutingTableEntry rt;

  if (m_routingTable.LookupRoute (dst, rt))
    {
      route = rt.GetRoute ();
      NS_ASSERT (route != 0);
      if (oif != 0 && route->GetOutputDevice () != oif)
        {
          NS_LOG_DEBUG ("Output device doesn't match. Dropped.");
          sockerr = Socket::ERROR_NOROUTETOHOST;
          return Ptr<Ipv4Route> ();
        }
      return route;
    }
  else
    {
      refreshRoutingTable (dst);

      if (m_routingTable.LookupRoute (dst, rt))
        {
          route = rt.GetRoute ();
          NS_ASSERT (route != 0);
          if (oif != 0 && route->GetOutputDevice () != oif)
            {
              NS_LOG_DEBUG ("Output device doesn't match. Dropped.");
              sockerr = Socket::ERROR_NOROUTETOHOST;
              return Ptr<Ipv4Route> ();
            }
          return route;
        }
      else
        {
        }
    }

  return LoopbackRoute (header, oif);
}

bool
RoutingProtocol::RouteInput (Ptr<const Packet> p, const Ipv4Header &header,
                             Ptr<const NetDevice> idev, UnicastForwardCallback ucb,
                             MulticastForwardCallback mcb, LocalDeliverCallback lcb,
                             ErrorCallback ecb)
{

  NS_LOG_FUNCTION (m_mainAddress << " received packet " << p->GetUid () << " from "
                                 << header.GetSource () << " on interface " << idev->GetAddress ()
                                 << " to destination " << header.GetDestination ());
  if (m_socketAddresses.empty ())
    {
      NS_LOG_DEBUG ("No parrot interfaces");
      return false;
    }
  NS_ASSERT (m_ipv4 != 0);
  // Check if input device supports IP
  NS_ASSERT (m_ipv4->GetInterfaceForDevice (idev) >= 0);
  int32_t iif = m_ipv4->GetInterfaceForDevice (idev);

  Ipv4Address dst = header.GetDestination ();
  Ipv4Address origin = header.GetSource ();

  // PARRoT is not a multicast routing protocol
  if (dst.IsMulticast ())
    {
      return false;
    }

  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j = m_socketAddresses.begin ();
       j != m_socketAddresses.end (); ++j)
    {
      Ipv4InterfaceAddress iface = j->second;
      if (origin == iface.GetLocal ())
        {
          return true;
        }
    }
  // LOCAL DELIVARY TO PARRoT INTERFACES
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j = m_socketAddresses.begin ();
       j != m_socketAddresses.end (); ++j)
    {
      Ipv4InterfaceAddress iface = j->second;
      if (m_ipv4->GetInterfaceForAddress (iface.GetLocal ()) == iif)
        {
          if (dst == iface.GetBroadcast () || dst.IsBroadcast ())
            {
              Ptr<Packet> packet = p->Copy ();
              if (lcb.IsNull () == false)
                {
                  NS_LOG_LOGIC ("Broadcast local delivery to " << iface.GetLocal ());
                  lcb (p, header, iif);
                  // Fall through to additional processing
                }
              else
                {
                  NS_LOG_ERROR ("Unable to deliver packet locally due to null callback "
                                << p->GetUid () << " from " << origin);
                  ecb (p, header, Socket::ERROR_NOROUTETOHOST);
                }
              if (header.GetTtl () > 1)
                {
                  NS_LOG_LOGIC ("Forward broadcast. TTL " << (uint16_t) header.GetTtl ());
                  RoutingTableEntry toBroadcast;
                  if (m_routingTable.LookupRoute (dst, toBroadcast, true))
                    {
                      Ptr<Ipv4Route> route = toBroadcast.GetRoute ();
                      ucb (route, packet, header);
                    }
                  else
                    {
                      NS_LOG_DEBUG ("No route to forward. Drop packet " << p->GetUid ());
                    }
                }
              return true;
            }
        }
    }

  if (m_ipv4->IsDestinationAddress (dst, iif))
    {
      if (lcb.IsNull () == false)
        {
          NS_LOG_LOGIC ("Unicast local delivery to " << dst);
          lcb (p, header, iif);
        }
      else
        {
          NS_LOG_ERROR ("Unable to deliver packet locally due to null callback "
                        << p->GetUid () << " from " << origin);
          ecb (p, header, Socket::ERROR_NOROUTETOHOST);
        }
      return true;
    }

  // Check if input device supports IP forwarding
  if (m_ipv4->IsForwarding (iif) == false)
    {
      NS_LOG_LOGIC ("Forwarding disabled for this interface");
      ecb (p, header, Socket::ERROR_NOROUTETOHOST);
      return true;
    }

  RoutingTableEntry toDst;
  if (m_routingTable.LookupRoute (dst, toDst))
    {
      Ptr<Ipv4Route> route = toDst.GetRoute ();
      NS_LOG_LOGIC (m_mainAddress << " is forwarding packet " << p->GetUid () << " to " << dst
                                  << " from " << header.GetSource () << " via nexthop neighbor "
                                  << toDst.GetNextHop ());
      ucb (route, p, header);
      return true;
    }
  NS_LOG_LOGIC ("Drop packet " << p->GetUid () << " as there is no route to forward it.");
  return false;
}

void
RoutingProtocol::NotifyInterfaceUp (uint32_t i)
{
  NS_LOG_FUNCTION (this << m_ipv4->GetAddress (i, 0).GetLocal () << " interface is up");
  Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol> ();
  if (l3->GetNAddresses (i) > 1)
    {
      NS_LOG_WARN ("PARRoT does not work with more then one address per each interface.");
    }
  Ipv4InterfaceAddress iface = l3->GetAddress (i, 0);
  if (iface.GetLocal () == Ipv4Address ("127.0.0.1"))
    {
      return;
    }
  // Create a socket to listen only on this interface
  Ptr<Socket> socket = Socket::CreateSocket (GetObject<Node> (), UdpSocketFactory::GetTypeId ());
  NS_ASSERT (socket != 0);
  socket->SetRecvCallback (MakeCallback (&RoutingProtocol::RecvMultiHopChirp, this));
  socket->BindToNetDevice (l3->GetNetDevice (i));
  socket->Bind (InetSocketAddress (Ipv4Address::GetAny (), PARROT_PORT));
  socket->SetAllowBroadcast (true);

  socket->SetAttribute ("IpTtl", UintegerValue (1));

  m_socketAddresses.insert (std::make_pair (socket, iface));
  // Add local broadcast record to the routing table
  Ptr<NetDevice> dev = m_ipv4->GetNetDevice (m_ipv4->GetInterfaceForAddress (iface.GetLocal ()));

  // ? Add default broadcast route
  RoutingTableEntry rt (
      /*device=*/dev, /*dst=*/iface.GetBroadcast (), /*iface=*/iface,
      /*next hop=*/iface.GetBroadcast (), /*lifetime=*/Simulator::GetMaximumSimulationTime ());
  m_routingTable.AddRoute (rt);
  if (m_mainAddress == Ipv4Address ())
    {
      m_mainAddress = iface.GetLocal ();
    }
  NS_ASSERT (m_mainAddress != Ipv4Address ());

  if (i != 1)
    {
      throw;
    }
  else
    {
      interface80211ptr = iface;
      m_device = dev;
    }
}

void
RoutingProtocol::NotifyInterfaceDown (uint32_t i)
{
  Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol> ();
  Ptr<NetDevice> dev = l3->GetNetDevice (i);
  Ptr<Socket> socket = FindSocketWithInterfaceAddress (m_ipv4->GetAddress (i, 0));
  NS_ASSERT (socket);
  socket->Close ();
  m_socketAddresses.erase (socket);
  if (m_socketAddresses.empty ())
    {
      NS_LOG_LOGIC ("No parrot interfaces");
      m_routingTable.Clear ();
      return;
    }
  m_routingTable.DeleteAllRoutesFromInterface (m_ipv4->GetAddress (i, 0));
}

void
RoutingProtocol::NotifyAddAddress (uint32_t i, Ipv4InterfaceAddress address)
{
  NS_LOG_FUNCTION (this << " interface " << i << " address " << address);
  Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol> ();
  if (!l3->IsUp (i))
    {
      return;
    }
  Ipv4InterfaceAddress iface = l3->GetAddress (i, 0);
  Ptr<Socket> socket = FindSocketWithInterfaceAddress (iface);
  if (!socket)
    {
      if (iface.GetLocal () == Ipv4Address ("127.0.0.1"))
        {
          return;
        }
      Ptr<Socket> socket =
          Socket::CreateSocket (GetObject<Node> (), UdpSocketFactory::GetTypeId ());
      NS_ASSERT (socket != 0);
      socket->SetRecvCallback (MakeCallback (&RoutingProtocol::RecvMultiHopChirp, this));
      // Bind to any IP address so that broadcasts can be received
      socket->BindToNetDevice (l3->GetNetDevice (i));
      socket->Bind (InetSocketAddress (Ipv4Address::GetAny (), PARROT_PORT));
      socket->SetAllowBroadcast (true);
      m_socketAddresses.insert (std::make_pair (socket, iface));
      Ptr<NetDevice> dev =
          m_ipv4->GetNetDevice (m_ipv4->GetInterfaceForAddress (iface.GetLocal ()));
      RoutingTableEntry rt (
          /*device=*/dev, /*dst=*/iface.GetBroadcast (), /*iface=*/iface,
          /*next hop=*/iface.GetBroadcast (), /*lifetime=*/Simulator::GetMaximumSimulationTime ());
      m_routingTable.AddRoute (rt);
    }
}

void
RoutingProtocol::NotifyRemoveAddress (uint32_t i, Ipv4InterfaceAddress address)
{
  Ptr<Socket> socket = FindSocketWithInterfaceAddress (address);
  if (socket)
    {
      m_socketAddresses.erase (socket);
      Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol> ();
      if (l3->GetNAddresses (i))
        {
          Ipv4InterfaceAddress iface = l3->GetAddress (i, 0);
          // Create a socket to listen only on this interface
          Ptr<Socket> socket =
              Socket::CreateSocket (GetObject<Node> (), UdpSocketFactory::GetTypeId ());
          NS_ASSERT (socket != 0);
          socket->SetRecvCallback (MakeCallback (&RoutingProtocol::RecvMultiHopChirp, this));
          // Bind to any IP address so that broadcasts can be received
          socket->Bind (InetSocketAddress (Ipv4Address::GetAny (), PARROT_PORT));
          socket->SetAllowBroadcast (true);
          m_socketAddresses.insert (std::make_pair (socket, iface));
        }
    }
}

void
RoutingProtocol::SetIpv4 (Ptr<Ipv4> ipv4)
{
  NS_ASSERT (ipv4 != 0);
  NS_ASSERT (m_ipv4 == 0);
  m_ipv4 = ipv4;
  // Create lo route. It is asserted that the only one interface up for now is loopback
  NS_ASSERT (m_ipv4->GetNInterfaces () == 1 &&
             m_ipv4->GetAddress (0, 0).GetLocal () == Ipv4Address ("127.0.0.1"));
  m_lo = m_ipv4->GetNetDevice (0);
  NS_ASSERT (m_lo != 0);
  // Remember lo route
  RoutingTableEntry rt (
      /*device=*/m_lo, /*dst=*/
      Ipv4Address::GetLoopback (),
      /*iface=*/Ipv4InterfaceAddress (Ipv4Address::GetLoopback (), Ipv4Mask ("255.0.0.0")),
      /*next hop=*/
      Ipv4Address::GetLoopback (),
      /*lifetime=*/Simulator::GetMaximumSimulationTime ());
  m_routingTable.AddRoute (rt);
  Simulator::ScheduleNow (&RoutingProtocol::Start, this);
}

void
RoutingProtocol::Send (Ptr<Ipv4Route> route, Ptr<const Packet> packet, const Ipv4Header &header)
{
  Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol> ();
  NS_ASSERT (l3 != 0);
  Ptr<Packet> p = packet->Copy ();
  l3->Send (p, route->GetSource (), header.GetDestination (), header.GetProtocol (), route);
}

void
RoutingProtocol::Drop (Ptr<const Packet> packet, const Ipv4Header &header, Socket::SocketErrno err)
{
  NS_LOG_DEBUG (m_mainAddress << " drop packet " << packet->GetUid () << " to "
                              << header.GetDestination () << " from queue. Error " << err);
}

Ptr<Socket>
RoutingProtocol::FindSocketWithInterfaceAddress (Ipv4InterfaceAddress addr) const
{
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j = m_socketAddresses.begin ();
       j != m_socketAddresses.end (); ++j)
    {
      Ptr<Socket> socket = j->first;
      Ipv4InterfaceAddress iface = j->second;
      if (iface == addr)
        {
          return socket;
        }
    }
  Ptr<Socket> socket;
  return socket;
}

float
RoutingProtocol::CalculateCommunicationRange (float eta, float f_Hz, float txp_dBm, float rxp_dBm,
                                              float g_tx, float g_rx)
{
  // Speed of light in m/s
  float c_mps = 299792458.0;
  return pow (dBmTomW (txp_dBm) * g_tx * g_rx / dBmTomW (rxp_dBm) *
                  pow (c_mps / (4 * M_PI * f_Hz), 2),
              1 / eta);
}

} // namespace parrot
} // namespace ns3
