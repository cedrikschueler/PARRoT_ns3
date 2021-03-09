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
 * Communication Networks Institute (CNI)
 * TU Dortmund University, Germany
 * Otto-Hahn-Str. 6
 * 44227 Dortmund
 *
*/

#ifndef PARROT_HELPER_H
#define PARROT_HELPER_H

#include "ns3/object-factory.h"
#include "ns3/node.h"
#include "ns3/node-container.h"
#include "ns3/ipv4-routing-helper.h"

namespace ns3 {
/**
 * \ingroup parrot
 * \brief Helper class that adds PARRoT routing to nodes.
 */
class PARRoTHelper : public Ipv4RoutingHelper
{
public:
  PARRoTHelper ();
  ~PARRoTHelper ();
  /**
   * \returns pointer to clone of this PARRoTHelper
   *
   * This method is mainly for internal use by the other helpers;
   * clients are expected to free the dynamic memory allocated by this method
   */
  PARRoTHelper* Copy (void) const;

  /**
   * \param node the node on which the routing protocol will run
   * \returns a newly-created routing protocol
   *
   * This method will be called by ns3::InternetStackHelper::Install
   *
   */
  virtual Ptr<Ipv4RoutingProtocol> Create (Ptr<Node> node) const;
  /**
   * \param name the name of the attribute to set
   * \param value the value of the attribute to set.
   *
   * This method controls the attributes of ns3::parrot::RoutingProtocol
   */
  void Set (std::string name, const AttributeValue &value);

private:
  ObjectFactory m_agentFactory; //!< Object factory
};

} // namespace ns3

#endif // PARROT_HELPER_H 