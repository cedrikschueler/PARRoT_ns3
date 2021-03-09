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

#ifndef PARROT_ROUTING_PROTOCOL_H
#define PARROT_ROUTING_PROTOCOL_H

#pragma once

#include "parrot-packet.h"
#include "parrot-rtable.h"
#include "pce.h"
#include "pdc.h"

#include "ns3/node.h"
#include "ns3/random-variable-stream.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/mobility-model.h"
#include "ns3/controlled-random-waypoint-mobility-model.h"

#include "ns3/timer.h"

#include <algorithm>
#include <deque>
#include <map>
#include <numeric>
#include <iomanip>

namespace ns3 {
namespace parrot {

/**
 * \ingroup parrot
 * \brief PARRoT routing protocol.
 */
class RoutingProtocol : public Ipv4RoutingProtocol
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  static const uint32_t PARROT_PORT;

  RoutingProtocol ();
  virtual ~RoutingProtocol ();
  virtual void DoDispose ();

  /**
   * \brief Query routing cache for an existing route, for an outbound packet
   *
   * This lookup is used by transport protocols.  It does not cause any
   * packet to be forwarded, and is synchronous.  Can be used for
   * multicast or unicast.  The Linux equivalent is ip_route_output()
   *
   * The header input parameter may have an uninitialized value
   * for the source address, but the destination address should always be 
   * properly set by the caller.
   *
   * \param p packet to be routed.  Note that this method may modify the packet.
   *          Callers may also pass in a null pointer. 
   * \param header input parameter (used to form key to search for the route)
   * \param oif Output interface Netdevice.  May be zero, or may be bound via
   *            socket options to a particular output interface.
   * \param sockerr Output parameter; socket errno 
   *
   * \returns a code that indicates what happened in the lookup
   */
  virtual Ptr<Ipv4Route> RouteOutput (Ptr<Packet> p, const Ipv4Header &header, Ptr<NetDevice> oif,
                                      Socket::SocketErrno &sockerr);

  /**
   * \brief Route an input packet (to be forwarded or locally delivered)
   *
   * This lookup is used in the forwarding process.  The packet is
   * handed over to the Ipv4RoutingProtocol, and will get forwarded onward
   * by one of the callbacks.  The Linux equivalent is ip_route_input().
   * There are four valid outcomes, and a matching callbacks to handle each.
   *
   * \param p received packet
   * \param header input parameter used to form a search key for a route
   * \param idev Pointer to ingress network device
   * \param ucb Callback for the case in which the packet is to be forwarded
   *            as unicast
   * \param mcb Callback for the case in which the packet is to be forwarded
   *            as multicast
   * \param lcb Callback for the case in which the packet is to be locally
   *            delivered
   * \param ecb Callback to call if there is an error in forwarding
   * \returns true if the Ipv4RoutingProtocol takes responsibility for 
   *          forwarding or delivering the packet, false otherwise
   */
  virtual bool RouteInput (Ptr<const Packet> p, const Ipv4Header &header, Ptr<const NetDevice> idev,
                           UnicastForwardCallback ucb, MulticastForwardCallback mcb,
                           LocalDeliverCallback lcb, ErrorCallback ecb);

  /**
   * \param interface the index of the interface we are being notified about
   *
   * Protocols are expected to implement this method to be notified of the state change of
   * an interface in a node.
   */
  virtual void NotifyInterfaceUp (uint32_t interface);
  /**
   * \param interface the index of the interface we are being notified about
   *
   * Protocols are expected to implement this method to be notified of the state change of
   * an interface in a node.
   */
  virtual void NotifyInterfaceDown (uint32_t interface);

  /**
   * \param interface the index of the interface we are being notified about
   * \param address a new address being added to an interface
   *
   * Protocols are expected to implement this method to be notified whenever
   * a new address is added to an interface. Typically used to add a 'network route' on an
   * interface. Can be invoked on an up or down interface.
   */
  virtual void NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address);

  /**
   * \param interface the index of the interface we are being notified about
   * \param address a new address being added to an interface
   *
   * Protocols are expected to implement this method to be notified whenever
   * a new address is removed from an interface. Typically used to remove the 'network route' of an
   * interface. Can be invoked on an up or down interface.
   */
  virtual void NotifyRemoveAddress (uint32_t interface, Ipv4InterfaceAddress address);

  /**
   * \param ipv4 the ipv4 object this routing protocol is being associated with
   * 
   * Typically, invoked directly or indirectly from ns3::Ipv4::SetRoutingProtocol
   */
  virtual void SetIpv4 (Ptr<Ipv4> ipv4);

  void Send (Ptr<Ipv4Route>, Ptr<const Packet>, const Ipv4Header &);
  /// Notify that packet is dropped for some reason
  void Drop (Ptr<const Packet>, const Ipv4Header &, Socket::SocketErrno);

  // * ==============================================* //
  // !          Handle parameters
  // * ==============================================* //

private:
  /**
   * Start operation
   */
  void Start ();

  /**
 * Multiplies Vector with scalar value
 * \param vec The vector to multiply
 * \param s The scalar value
 */
  Vector3D
  VecMult (Vector3D vec, double s)
  {
    vec.x *= s;
    vec.y *= s;
    vec.z *= s;

    return vec;
  }
  /**
   * Converts power in dBm to mW
   * \param p_dBm The power in dBm
   * \returns The power in mW
   */
  float
  dBmTomW (float p_dBm)
  {
    return pow (10, p_dBm / 10);
  }

  /**
   * Estimates the communication range according to Freespace pathloss
   * \param eta The pathloss coefficient
   * \param f_Hz The carrier frequency in Hz
   * \param txp_dBm The transmission power in dBm
   * \param rxp_dBm The reception power threshold in dBm (a.k.a minimum reception power)
   * \param g_tx The transmission antenna gain
   * \param g_rx The receiver antenna gain
   * \returns The estimated communication range in m
   */
  float CalculateCommunicationRange (float eta = 2.75, float f_Hz = 2.4e9, float txp_dBm = 20.0,
                                     float rxp_dBm = -85.0, float g_tx = 1.0, float g_rx = 1.0);

  /// Timer to trigger periodic updates from a node
  Timer m_periodicUpdateTimer;
  /// Provides uniform random variables.
  Ptr<UniformRandomVariable> m_uniformRandomVariable;
  /// Raw socket per each IP interface, map socket -> iface address (IP + mask)
  std::map<Ptr<Socket>, Ipv4InterfaceAddress> m_socketAddresses;
  Ipv4InterfaceAddress interface80211ptr;
  Ptr<NetDevice> m_device;
  /// Loopback device used to defer route requests until a route is found
  Ptr<NetDevice> m_lo;
  /// Routing Table
  RoutingTable m_routingTable;
  /// Nodes IP address
  Ipv4Address m_mainAddress;
  /// IP protocol
  Ptr<Ipv4> m_ipv4;
  /// Unicast callback for own packets
  UnicastForwardCallback m_scb;
  /// Error callback for own packets
  ErrorCallback m_ecb;

  /**
   * Find socket with local interface address iface
   * \param iface the interface
   * \returns the socket
   */
  Ptr<Socket> FindSocketWithInterfaceAddress (Ipv4InterfaceAddress iface) const;
  Ptr<Ipv4Route> LoopbackRoute (const Ipv4Header &hdr, Ptr<NetDevice> oif) const;

  // * ==============================================* //
  // !          PARRoT specific parameters
  // * ==============================================* //
  int m_maxHops;
  Time m_neighborReliabilityTimeout;
  Time mhChirpInterval;
  bool m_rescheduleRoutesOnTimeoutFlag;
  float qFctAlpha;
  float qFctGamma;
  // The communication range
  float r_com;

  // * ==============================================* //
  // !                 Chirp utils
  // * ==============================================* //

  // ? Methods
  /**
   * Prints the Q Table 
   * \param stream The output stream wrapper
   * \param unit The time unit
   */
  void printQ (Ptr<OutputStreamWrapper> stream, Time::Unit unit) const;
  /**
   * Send Multihop Chirp
   */
  void SendMultiHopChirp ();
  // Receive multi hop chirp packets
  /**
   * Receive and process multi hop chirp packet
   * \param socket the socket for receiving multi hop chirp packets
   */
  void RecvMultiHopChirp (Ptr<Socket> socket);
  /** 
   * Prints the routing table
   * \param stream The output stream wrapper
   * \param unit The time unit
   */
  void PrintRoutingTable (Ptr<OutputStreamWrapper> stream, Time::Unit unit) const;
  /**
   * Checks after updates
   * \param origin The address of the mutli hop chirp originator address
   * \param gateway The address of the last forwarder
   * \returns If the checks have been passed
   */
  bool postliminaryChecksPassed (Ipv4Address origin, Ipv4Address gateway);
  /**
   * Handles an incoming multi hop chirp
   * \param chirp The incoming chirp
   * \param gateway The last forwarder from higher layer protocol
   * \returns The remaining number of hops
   */
  int handleIncomingMultiHopChirp (MultiHopChirp chirp, Ipv4Address gateway);
  /**
   * Refreshes the routing table
   * \param origin The origin to update for
   */
  void refreshRoutingTable (Ipv4Address origin);
  /**
   * Purges the neighbors
   */
  void purgeNeighbors ();

  // ? Variables
  /// List of neighbors
  std::map<Ipv4Address, PDC *> Vi;
  /// The main Q table
  std::map<Ipv4Address, std::map<Ipv4Address, PCE *>> m_QTable;
  /// The node's sequence number
  unsigned short m_squNr;
  /// The node's IP address
  Ipv4Address m_selfIpv4Address;

  // * ==============================================* //
  // !                 Brain utils
  // * ==============================================* //
  // ? Methods
  /**
   * Combines different discount metrics
   * \param gamma The vector of discount metrics
   * \returns The combined discount factor
   */
  double combineDiscounts (std::vector<double> gamma);
  /**
   * Calculates the Q Value
   * \param target The target node (state)
   * \param hop The candidate hop (action)
   * \returns The Q Value
   */
  double qFunction (Ipv4Address target, Ipv4Address hop);
  /**
   * Get best Q Value for target node
   * \param target The target node
   * \returns The best Q Value for the target node
   */
  double getMaxValueFor (Ipv4Address target);
  /**
   * Get best next hop for target node
   * \param target The target node
   * \returns The best next hop for the target node
   */
  Ipv4Address getNextHopFor (Ipv4Address target);
  /**
   * Reward function
   * \param origin The origin (state)
   * \param hop The hop (action)
   * \returns The (short-term) reward
   */
  double R (Ipv4Address origin, Ipv4Address hop);
  /**
   * Calculates the Phi_LET metric for a specific neighbor
   * \param neighbor The requested neighbor node
   * \returns Phi_LET metric
   */
  double Phi_LET (Ipv4Address neighbor);
  /**
   * Updates the node's Phi_Coh value
   */
  void updatePhi_Coh ();

  // ? Variables
  /// The chosen combination Method
  std::string combinationMethod;
  /// The last set of concerned neighbors
  std::vector<Ipv4Address> lastSetOfNeighbors;
  /// The node's Phi_Coh value
  double m_Phi_Coh;
  /// The offset for range estimations
  double rangeOffset;

  // * ==============================================* //
  // !                 Wings utils
  // * ==============================================* //
  // ? Methods
  /**
   * Tracks the position to internal storage
   * \param pos The position to track
   */
  void trackPosition (Vector3D pos);
  /**
   * Forecasts the future position
   * \returns The predicted position
   */
  Vector3D forecastPosition ();
  /**
   * Predicts the next iteration with target information
   * \param _currentData The current position data
   * \param m_updateInterval_ms The update interval in milliseconds
   * \param wp0 The target position
   * \returns The next predicted iteration step position
   */
  Vector3D predictWithTarget (Vector3D _currentData, int m_updateInterval_ms, Vector3D wp0);
  /**
   * Predicts the next iteration with history information
   * \param _historyData The history position data
   * \param times The corresponding time steps
   * \param _nextTime_ms The requestes next time step
   * \returns The next predicted iteration step position
   */
  Vector3D predictWithHistory (std::deque<Vector3D> _historyData, std::deque<Time> times,
                               int _nextTime_ms);

  // ? Variables
  /// Pointer to mobility handler
  Ptr<MobilityModel> mobility;
  /// The history size (amount of stored positions in ring buffer)
  int historySize;
  /// The ring buffer for historical positions
  std::deque<Vector3D> hist_coord;
  /// The ring buffer for timestamps of historical positions
  std::deque<Time> hist_coord_t;
  /// The chosen prediction method
  std::string predictionMethod;
};

} // namespace parrot
} // namespace ns3

#endif // PARROT_ROUTING_PROTOCOL_H
