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

#ifndef PARROT_RTABLE_H
#define PARROT_RTABLE_H

#include <cassert>
#include <map>
#include <sys/types.h>
#include "ns3/ipv4.h"
#include "ns3/ipv4-route.h"
#include "ns3/timer.h"
#include "ns3/net-device.h"
#include "ns3/output-stream-wrapper.h"


namespace ns3{
namespace parrot{

// * Routing Table entries
/**
 * \ingroup parrot
 * \brief Routing table entry
 */
class RoutingTableEntry
{
public:
  /**
   *
   * \param dev the net device
   * \param dst the destination IP address
   * \param iface the interface
   * \param nextHop the IP address of the next hop
   * \param lifetime the lifetime 
   */
  RoutingTableEntry (Ptr<NetDevice> dev = 0, Ipv4Address dst = Ipv4Address (),
                     Ipv4InterfaceAddress iface = Ipv4InterfaceAddress (), Ipv4Address nextHop = Ipv4Address (),
                     Time expiryTime = Simulator::Now ());

  ~RoutingTableEntry ();
  /**
   * Get destination IP address
   * \returns the destination IPv4 address
   */
  Ipv4Address
  GetDestination () const
  {
    return m_ipv4Route->GetDestination ();
  }
  /**
   * Get route
   * \returns the IPv4 route
   */
  Ptr<Ipv4Route>
  GetRoute () const
  {
    return m_ipv4Route;
  }
  /**
   * Set route
   * \param route the IPv4 route
   */
  void
  SetRoute (Ptr<Ipv4Route> route)
  {
    m_ipv4Route = route;
  }
  /**
   * Set next hop
   * \param nextHop the IPv4 address of the next hop
   */
  void
  SetNextHop (Ipv4Address nextHop)
  {
    m_ipv4Route->SetGateway (nextHop);
  }
  /**
   * Get next hop
   * \returns the IPv4 address of the next hop
   */
  Ipv4Address
  GetNextHop () const
  {
    return m_ipv4Route->GetGateway ();
  }
  /**
   * Set output device
   * \param device the output device
   */
  void
  SetOutputDevice (Ptr<NetDevice> device)
  {
    m_ipv4Route->SetOutputDevice (device);
  }
  /**
   * Get output device
   * \returns the output device
   */
  Ptr<NetDevice>
  GetOutputDevice () const
  {
    return m_ipv4Route->GetOutputDevice ();
  }
  /**
   * Get interface address
   * \returns the IPv4 interface address
   */
  Ipv4InterfaceAddress
  GetInterface () const
  {
    return m_iface;
  }
  /**
   * Set interface address
   * \param iface the IPv4 interface address
   */
  void
  SetInterface (Ipv4InterfaceAddress iface)
  {
    m_iface = iface;
  }
  /**
   * Set sequence number
   * \param sequenceNumber the sequence number
   */
  /**
   * Set expirytime
   * \param expiryTime the expirytime value
   */
  void
  SetExpiryTime (Time expiryTime)
  {
    m_expiryTime = expiryTime;
  }
  /**
   * Get lifetime
   * \returns the lifetime value
   */
  Time
  GetExpiryTime () const
  {
    return m_expiryTime;
  }
  /**
   * \brief Compare destination address
   * \param destination destination node IP address
   * \return true if equal
   */
  bool
  operator== (Ipv4Address const destination) const
  {
    return (m_ipv4Route->GetDestination () == destination);
  }
  /**
   * Print routing table entry
   * \param stream the output stream
   */
  void
  Print (Ptr<OutputStreamWrapper> stream, Time::Unit unit = Time::S) const;

private:
  // Fields

  /**
   * \brief Expiration  time of the route
   */
  Time m_expiryTime;
  /** Ip route, include
   *   - destination address
   *   - source address
   *   - next hop address (gateway)
   *   - output device
   */
  Ptr<Ipv4Route> m_ipv4Route;
  /// Output interface address
  Ipv4InterfaceAddress m_iface;

};


// * Routing table
/**
 * \ingroup parrot
 * \brief The Routing table used by DSDV protocol
 */
class RoutingTable
{
public:
  RoutingTable ();
  /**
   * Add routing table entry if it doesn't yet exist in routing table
   * \param r routing table entry
   * \return true in success
   */
  bool
  AddRoute (RoutingTableEntry & r);
  /**
   * Delete routing table entry with destination address dst, if it exists.
   * \param dst destination address
   * \return true on success
   */
  bool
  DeleteRoute (Ipv4Address dst);
  /**
   * Lookup routing table entry with destination address dst
   * \param dst destination address
   * \param rt entry with destination address dst, if exists
   * \return true on success
   */
  bool
  LookupRoute (Ipv4Address dst, RoutingTableEntry & rt);
  /**
   * Lookup routing table entry with destination address dst
   * \param id destination address
   * \param rt entry with destination address dst, if exists
   * \param forRouteInput for routing input
   * \return true on success
   */
  bool
  LookupRoute (Ipv4Address id, RoutingTableEntry & rt, bool forRouteInput);
  /**
   * Updating the routing Table with routing table entry rt
   * \param rt routing table entry
   * \return true on success
   */
  bool
  Update (RoutingTableEntry & rt);
  /**
   * Lookup list of addresses for which nxtHp is the next Hop address
   * \param nxtHp nexthop's address for which we want the list of destinations
   * \param dstList is the list that will hold all these destination addresses
   */
  void
  GetListOfDestinationWithNextHop (Ipv4Address nxtHp, std::map<Ipv4Address, RoutingTableEntry> & dstList);
  /**
   * Lookup list of all addresses in the routing table
   * \param allRoutes is the list that will hold all these addresses present in the nodes routing table
   */
  void
  GetListOfAllRoutes (std::map<Ipv4Address, RoutingTableEntry> & allRoutes);
  /**
   * Delete all route from interface with address iface
   * \param iface the interface
   */
  void
  DeleteAllRoutesFromInterface (Ipv4InterfaceAddress iface);
  /// Delete all entries from routing table
  void
  Clear ()
  {
    m_ipv4AddressEntry.clear ();
  }
  /**
   * Delete all outdated entries if Lifetime is expired
   * \param removedAddresses is the list of addresses to purge
   */
  void
  Purge (std::map<Ipv4Address, RoutingTableEntry> & removedAddresses);
  /**
   * Delete all outdated entries if expirytime is expired
   */
  void
  Purge ();
  /**
   * Print routing table
   * \param stream the output stream
   */
  void
  Print (Ptr<OutputStreamWrapper> stream, Time::Unit unit = Time::S) const;
  /**
   * Provides the number of routes present in that nodes routing table.
   * \returns the number of routes
   */
  uint32_t
  RoutingTableSize ();
  /**
  * Add an event for a destination address so that the update to for that destination is sent
  * after the event is completed.
  * \param address destination address for which this event is running.
  * \param id unique eventid that was generated.
  * \return true on success
  */
  bool
  AddIpv4Event (Ipv4Address address, EventId id);
  /**
  * Clear up the entry from the map after the event is completed
  * \param address destination address for which this event is running.
  * \return true on success
  */
  bool
  DeleteIpv4Event (Ipv4Address address);
  /**
  * Force delete an update waiting for settling time to complete as a better update to
  * same destination was received.
  * \param address destination address for which this event is running.
  * \return true on success
  */
  bool
  AnyRunningEvent (Ipv4Address address);
  /**
  * Force delete an update waiting for settling time to complete as a better update to
  * same destination was received.
  * \param address destination address for which this event is running.
  * \return true on finding out that an event is already running for that destination address.
  */
  bool
  ForceDeleteIpv4Event (Ipv4Address address);
  /**
    * Get the EcentId associated with that address.
    * \param address destination address for which this event is running.
    * \return EventId on finding out an event is associated else return NULL.
    */
  EventId
  GetEventId (Ipv4Address address);

  /**
   * Get hold down time (time until an invalid route may be deleted)
   * \returns the hold down time
   */
  Time Getholddowntime () const
  {
    return m_holddownTime;
  }
  /**
   * Set hold down time (time until an invalid route may be deleted)
   * \param t the hold down time
   */
  void Setholddowntime (Time t)
  {
    m_holddownTime = t;
  }

private:
  // Fields
  /// an entry in the routing table.
  std::map<Ipv4Address, RoutingTableEntry> m_ipv4AddressEntry;
  /// an entry in the event table.
  std::map<Ipv4Address, EventId> m_ipv4Events;
  /// hold down time of an expired route
  Time m_holddownTime;

};


} // namespace parrot
} // namespace ns3
#endif // PARROT_RTABLE_H 