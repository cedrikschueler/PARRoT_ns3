/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2021 Cedrik Schüler
 *
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
 * Author: Cedrik Schüler <cedrik.schueler@tu-dortmund.de>
 * 
 * THIS ALLOWS FREE USE OF THIS SOFTWARE IN ITS "AS IS" CONDITION AND
 * DISCLAIM ANY LIABILITY OF ANY KIND FOR ANY DAMAGES WHATSOEVER RESULTING
 * FROM THE USE OF THIS SOFTWARE.
 * 
 * Communication Networks Institute (CNI)
 * TU Dortmund University, Germany
 * Otto-Hahn-Str. 6
 * 44227 Dortmund
 *
*/

#include "parrot-routing-protocol.h"

namespace ns3 {
namespace parrot {

static void
SendDelayedHandler (Ptr<Socket> sock, Ptr<Packet> packet, int i, InetSocketAddress addr)
{
  sock->SendTo (packet, i, addr);
}

void
RoutingProtocol::RecvMultiHopChirp (Ptr<Socket> socket)
{
  Address sourceAddress;
  Ptr<Packet> packet = socket->RecvFrom (sourceAddress);
  InetSocketAddress inetSourceAddr = InetSocketAddress::ConvertFrom (sourceAddress);
  Ipv4Address sender = inetSourceAddr.GetIpv4 ();
  Ipv4Address receiver = m_socketAddresses[socket].GetLocal ();
  Ptr<NetDevice> dev = m_ipv4->GetNetDevice (m_ipv4->GetInterfaceForAddress (receiver));
  MultiHopChirp incMultiHopChirp;
  packet->RemoveHeader (incMultiHopChirp);

  RoutingTableEntry rt;
  int remainingHops = handleIncomingMultiHopChirp (incMultiHopChirp, sender);

  if (remainingHops > 0 && postliminaryChecksPassed (incMultiHopChirp.GetOrig (), sender))
    {

      Vector3D forecast = forecastPosition ();
      Vector3D p = (hist_coord.size () != 0) ? hist_coord[historySize - 1] : Vector3D (0, 0, 0);
      incMultiHopChirp.SetP (p);
      Vector3D v_int =
          VecMult ((forecast - p), 1 / ((m_neighborReliabilityTimeout.GetSeconds () != 0)
                                            ? m_neighborReliabilityTimeout.GetSeconds ()
                                            : 1.0));
      incMultiHopChirp.SetP_hat (v_int);
      incMultiHopChirp.SetPhi_Coh ((float) m_Phi_Coh);
      incMultiHopChirp.SetTtl (remainingHops);
      for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j =
               m_socketAddresses.begin ();
           j != m_socketAddresses.end (); ++j)
        {
          MultiHopChirp forwardedChirp;
          forwardedChirp.SetOrig (incMultiHopChirp.GetOrig ());
          forwardedChirp.SetP (p);
          forwardedChirp.SetP_hat (v_int);

          forwardedChirp.SetV ((float) getMaxValueFor (incMultiHopChirp.GetOrig ()));
          forwardedChirp.SetPhi_Coh (m_Phi_Coh);
          forwardedChirp.SetSeq (incMultiHopChirp.GetSeq ());
          forwardedChirp.SetTtl (remainingHops);

          Ptr<Socket> socket = j->first;
          Ipv4InterfaceAddress iface = j->second;
          Ptr<Packet> packet = Create<Packet> ();

          packet->AddHeader (forwardedChirp);
          socket->Send (packet);
          // Send to all-hosts broadcast if on /32 addr, subnet-directed otherwise
          Ipv4Address destination;
          if (iface.GetMask () == Ipv4Mask::GetOnes ())
            {
              destination = Ipv4Address ("255.255.255.255");
            }
          else
            {
              destination = iface.GetBroadcast ();
            }
          Simulator::Schedule (MicroSeconds (m_uniformRandomVariable->GetInteger (0, 10000)),
                               &SendDelayedHandler, socket, packet, 0,
                               InetSocketAddress (destination, PARROT_PORT));
        }
    }
}

void
RoutingProtocol::printQ (Ptr<OutputStreamWrapper> stream, Time::Unit unit) const
{
  *stream->GetStream () << "PARRoT Q-Table at " << Simulator::Now ().As (unit) << "\n"
                        << "Destination\t\tGateway\t\tQ\t\tV\t\tSEQ\t\tLastSeen\n";
  for (std::map<Ipv4Address, std::map<Ipv4Address, PCE *>>::const_iterator dst = m_QTable.begin ();
       dst != m_QTable.end (); ++dst)
    {
      for (std::map<Ipv4Address, PCE *>::const_iterator gw = dst->second.begin ();
           gw != dst->second.end (); ++gw)
        {
          *stream->GetStream () << std::setiosflags (std::ios::fixed) << dst->first << "\t\t"
                                << gw->first << "\t\t" << gw->second->Q () << "\t\t"
                                << gw->second->V () << "\t\t" << gw->second->squNr () << "\t\t"
                                << std::setiosflags (std::ios::left) << std::setprecision (3)
                                << gw->second->lastSeen ().As (unit) << "\n";
        }
    }
  *stream->GetStream () << "\n";
}

// Chirp utils
void
RoutingProtocol::SendMultiHopChirp ()
{
  MultiHopChirp chirp;

  // Set fallback information
  Vector3D p = mobility->GetPosition ();
  trackPosition (p);

  Vector3D forecast = forecastPosition ();
  double s = 1 / ((m_neighborReliabilityTimeout.GetSeconds () != 0)
                      ? m_neighborReliabilityTimeout.GetSeconds ()
                      : 1.0);
  Vector3D v_int = VecMult (forecast - p, s);

  chirp.SetOrig (m_selfIpv4Address);
  chirp.SetP (p);
  chirp.SetP_hat (v_int);

  chirp.SetV (1.0);

  updatePhi_Coh ();
  chirp.SetPhi_Coh (m_Phi_Coh);

  chirp.SetSeq (m_squNr);
  m_squNr++;

  chirp.SetTtl (m_maxHops);

  if (m_selfIpv4Address == Ipv4Address("10.1.1.1")){
      Ptr<OutputStreamWrapper> oscout = Create<OutputStreamWrapper> (&std::cout);
      printQ(oscout, Time::S);
  }

  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j = m_socketAddresses.begin ();
       j != m_socketAddresses.end (); ++j)
    {
      Ptr<Socket> socket = j->first;
      Ipv4InterfaceAddress iface = j->second;
      Ptr<Packet> packet = Create<Packet> ();

      packet->AddHeader (chirp);
      socket->Send (packet);
      // Send to all-hosts broadcast if on /32 addr, subnet-directed otherwise
      Ipv4Address destination;
      if (iface.GetMask () == Ipv4Mask::GetOnes ())
        {
          destination = Ipv4Address ("255.255.255.255");
        }
      else
        {
          destination = iface.GetBroadcast ();
        }
      Simulator::Schedule (MicroSeconds (m_uniformRandomVariable->GetInteger (0, 10000)),
                           &SendDelayedHandler, socket, packet, 0,
                           InetSocketAddress (destination, PARROT_PORT));
    }
  m_periodicUpdateTimer.Schedule (mhChirpInterval +
                                  MicroSeconds (m_uniformRandomVariable->GetInteger (0, 10000)));
}

void
RoutingProtocol::PrintRoutingTable (Ptr<OutputStreamWrapper> stream, Time::Unit unit) const
{
  m_routingTable.Print (stream, unit);
}

bool
RoutingProtocol::postliminaryChecksPassed (Ipv4Address origin, Ipv4Address gateway)
{
  RoutingTableEntry rt;
  if (Vi.size () == 0)
    {
      // No neighbors
      return false;
    }
  else if (Vi.size () == 1)
    {
      // One neighbor, was this neighbor the sender/origin of the chirp?
      return !(Vi.begin ()->first == origin || Vi.begin ()->first == gateway);
    }
  else if (m_routingTable.LookupRoute (origin, rt) && rt.GetNextHop () != gateway)
    {
      // Neighbor was not the best choice, no need to propagate this.
      return false;
    }
  else
    {
      return true;
    }
}

//	Multi-Hop signalling (incoming)
int
RoutingProtocol::handleIncomingMultiHopChirp (MultiHopChirp chirp, Ipv4Address gateway)
{
  Ipv4Address origin = chirp.GetOrig ();

  float val = chirp.GetV ();

  Vector3D p = chirp.GetP ();
  Vector3D v = chirp.GetP_hat ();

  float Phi_Coh = chirp.GetPhi_Coh ();
  unsigned short squNr = chirp.GetSeq ();

  int hopCount = chirp.GetTtl ();

  if (origin == m_selfIpv4Address)
    {
      return 0;
    }
  std::map<Ipv4Address, PDC *>::iterator nj = Vi.find (gateway);

  RoutingTableEntry rt;

  // Ensure next hop is registered in neighbors
  if (nj != Vi.end () && origin != m_selfIpv4Address)
    {
      nj->second->lastSeen (Simulator::Now ());
      nj->second->coord (p);
      nj->second->velo (v);
      nj->second->Phi_Coh (Phi_Coh);
      nj->second->Phi_LET (Phi_LET (gateway));
    }
  else
    {
      PDC *data = new PDC ();
      data->lastSeen (Simulator::Now ());
      data->coord (p);
      data->velo (v);
      data->Phi_Coh (Phi_Coh);
      Vi.insert (std::make_pair (gateway, data));
      data->Phi_LET (Phi_LET (gateway));
    }

  if (m_QTable.find (origin) == m_QTable.end ())
    {
      // Case 1: Origin was not captured as destination at all
      PCE *data = new PCE (origin);
      data->lastSeen (Simulator::Now ());
      data->Q (0);
      data->squNr (squNr);
      data->V (val);
      std::map<Ipv4Address, PCE *> gw = {{gateway, data}};
      m_QTable[origin] = gw;
      float qval = qFunction (origin, gateway);
      PCE *data_ = m_QTable.at (origin).at (gateway);
      data_->Q (qval);
    }
  else
    {
      // Case 2: Origin was captured as destination, but not via this hop
      std::map<Ipv4Address, std::map<Ipv4Address, PCE *>>::iterator gw = m_QTable.find (origin);
      if (gw->second.find (gateway) == gw->second.end ())
        {
          // Make new entry
          PCE *data = new PCE (origin);
          data->lastSeen (Simulator::Now ());
          data->Q (0);
          data->squNr (squNr);
          data->V (val);
          gw->second[gateway] = data;
          m_QTable[origin][gateway]->Q (qFunction (origin, gateway));
        }
      else
        {
          // Case 3: Origin was already captured as destination via this hop
          if (squNr > gw->second[gateway]->squNr () ||
              (squNr == gw->second[gateway]->squNr () && val > gw->second[gateway]->V ()))
            {
              gw->second[gateway]->squNr (squNr);
              gw->second[gateway]->lastSeen (Simulator::Now ());
              gw->second[gateway]->V (val);
              gw->second[gateway]->Q (qFunction (origin, gateway));
            }
          else
            {
              return 0;
            }
        }
    }
  refreshRoutingTable (origin);
  return --hopCount;
}

void
RoutingProtocol::refreshRoutingTable (Ipv4Address origin)
{
  // Purge
  m_routingTable.Purge ();
  purgeNeighbors ();
  // Determine current route and bestRoute
  RoutingTableEntry route;
  Ipv4Address bestHop = getNextHopFor (origin);

  bool foundRoute = m_routingTable.LookupRoute (origin, route);

  // Do nothing..
  if (!foundRoute && bestHop == Ipv4Address::GetZero ())
    {
      return;
    }
  else
    {
      if (foundRoute && route.GetNextHop () == bestHop)
        {
          // Best route is already chosen, so don't change it
          return;
        }

      // Remove old route anyway
      if (foundRoute)
        {
          bool deleteSuccessfull = m_routingTable.DeleteRoute (origin);
          NS_ASSERT (deleteSuccessfull);
        }

      if (bestHop == Ipv4Address::GetZero ())
        {
          // Nothing
        }
      else
        {
          double t = Phi_LET (bestHop);
          Time alt = Seconds (t);
          RoutingTableEntry e (
              /*device=*/m_device,
              /*dst=*/origin,
              /*iface=*/interface80211ptr,
              /*next hop=*/bestHop,
              /*expirytime=*/Simulator::Now () +
                  std::min (std::max (m_neighborReliabilityTimeout, mhChirpInterval), alt));

          m_routingTable.AddRoute (e);
          NS_ASSERT (m_routingTable.LookupRoute (origin, route));
        }
    }
}

void
RoutingProtocol::purgeNeighbors ()
{
  // First delete invalid entrys
  for (std::map<Ipv4Address, std::map<Ipv4Address, PCE *>>::iterator t = m_QTable.begin ();
       t != m_QTable.end (); t++)
    {
      Ipv4Address target = t->first;
      for (std::map<Ipv4Address, PCE *>::iterator act = m_QTable.find (target)->second.begin ();
           act != m_QTable.find (target)->second.end ();)
        {
          double deltaT = (Simulator::Now () - act->second->lastSeen ()).GetSeconds ();
          if (deltaT >
              std::min (std::max (m_neighborReliabilityTimeout, mhChirpInterval).GetSeconds (),
                        Phi_LET (act->first)))
            {
              delete act->second;
              act = m_QTable.at (target).erase (act);
            }
          else
            {
              act++;
            }
        }
    }

  // Check if neighbor is still usefull
  for (std::map<Ipv4Address, PDC *>::iterator n = Vi.begin (); n != Vi.end ();)
    {
      bool useful = false;
      for (std::map<Ipv4Address, std::map<Ipv4Address, PCE *>>::iterator t = m_QTable.begin ();
           t != m_QTable.end (); t++)
        {
          useful = useful || (t->second.find (n->first) != t->second.end ());
        }
      if (!useful)
        {
          delete n->second;
          n = Vi.erase (n);
        }
      else
        {
          n++;
        }
    }
}

} // namespace parrot
} // namespace ns3
