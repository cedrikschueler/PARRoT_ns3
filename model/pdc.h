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

#ifndef PDC_H
#define PDC_H

#include "ns3/nstime.h"

namespace ns3 {
namespace parrot {

class PDC
{
public:
  PDC ()
  {
  }
  ~PDC ()
  {
  }
  // Setter
  void
  lastSeen (Time t)
  {
    _lastSeen = t;
  }
  void
  coord (Vector3D coord)
  {
    _coord = coord;
  }
  void
  coord (double x, double y, double z)
  {
    this->coord (Vector3D (x, y, z));
  }
  void
  velo (Vector3D v)
  {
    _velo = v;
  }
  void
  velo (double vx, double vy, double vz)
  {
    this->velo (Vector3D (vx, vy, vz));
  }
  void
  Phi_Coh (float v)
  {
    _Phi_Coh = v;
  }
  void
  Phi_LET (float v)
  {
    _Phi_LET = v;
  }

  // Getter
  Time
  lastSeen ()
  {
    return _lastSeen;
  }
  Vector3D
  coord ()
  {
    return _coord;
  }
  Vector3D
  velo ()
  {
    return _velo;
  }
  float
  Phi_Coh ()
  {
    return _Phi_Coh;
  }
  float
  Phi_LET ()
  {
    return _Phi_LET;
  }

protected:
  Time _lastSeen;
  Vector3D _coord;
  Vector3D _velo;
  float _Phi_Coh = 0.0;
  float _Phi_LET = 0.0;
};
} // namespace parrot
} // namespace ns3

#endif // PDC_H