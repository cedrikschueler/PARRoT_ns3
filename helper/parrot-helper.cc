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

#include "parrot-helper.h"
#include "ns3/parrot-routing-protocol.h"
#include "ns3/node-list.h"
#include "ns3/names.h"
#include "ns3/ipv4-list-routing.h"

namespace ns3 {
PARRoTHelper::~PARRoTHelper ()
{
}

PARRoTHelper::PARRoTHelper () : Ipv4RoutingHelper ()
{
  m_agentFactory.SetTypeId ("ns3::parrot::RoutingProtocol");
}

PARRoTHelper*
PARRoTHelper::Copy (void) const
{
  return new PARRoTHelper (*this);
}

Ptr<Ipv4RoutingProtocol>
PARRoTHelper::Create (Ptr<Node> node) const
{
  Ptr<parrot::RoutingProtocol> agent = m_agentFactory.Create<parrot::RoutingProtocol> ();
  node->AggregateObject (agent);
  return agent;
}

void
PARRoTHelper::Set (std::string name, const AttributeValue &value)
{
  m_agentFactory.Set (name, value);
}

}
