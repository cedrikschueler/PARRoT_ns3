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

#ifndef PCE_H
#define PCE_H

// TODO: includes
#include "ns3/nstime.h"

namespace ns3 {
namespace parrot {

class PCE
{
public:
  PCE (Ipv4Address destination)
  {
    m_destination = destination;
  }
  ~PCE ()
  {
  }

  void
  lastSeen (Time t)
  {
      _lastSeen = t;
  }
  void
  Q (float v)
  {
    _qValue = v;
  }
  void
  V (float v)
  {
    _vValue = v;
  }
  void
  squNr (unsigned short seq)
  {
    _squNr = seq;
  }

  Ipv4Address
  getDestination ()
  {
    return m_destination;
  }
  Time
  lastSeen ()
  {
    return _lastSeen;
  }
  float
  Q ()
  {
    return _qValue;
  }
  float
  V ()
  {
    return _vValue;
  }

  unsigned short
  squNr ()
  {
    return _squNr;
  }

protected:
  Ipv4Address m_destination;
  Time _lastSeen;
  float _qValue;
  float _vValue;
  unsigned short _squNr;
};

} // namespace parrot

} // namespace ns3

#endif // PCE_H
