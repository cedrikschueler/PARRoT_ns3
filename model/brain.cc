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

double
RoutingProtocol::combineDiscounts (std::vector<double> gamma)
{
  if (combinationMethod == "M")
    {
      return std::accumulate (gamma.begin (), gamma.end (), 1.0, std::multiplies<double> ());
    }
  else if (combinationMethod == "G")
    {
      // nth-root(g1*g2*..*gn)
      return pow (std::accumulate (gamma.begin (), gamma.end (), 1.0, std::multiplies<double> ()),
                  (1 / static_cast<double> (gamma.size ())));
    }
  else if (combinationMethod == "A")
    {
      // 1/n * (g1 + g2 + .. + gn)
      return std::accumulate (gamma.begin (), gamma.end (), 1.0) / gamma.size ();
    }
  else if (combinationMethod == "H")
    {
      // n / (1/g1 + 1/g2 + .. + 1/gn)
      return static_cast<double> (gamma.size ()) /
             std::accumulate (gamma.begin (), gamma.end (), 1.0,
                              [&] (double res, double c) mutable {
                                res += (c != 0) ? 1 / c : INFINITY;
                                return res;
                              });
    }
  else
    {
      return std::accumulate (gamma.begin (), gamma.end (), 1.0, std::multiplies<double> ());
    }
}

double
RoutingProtocol::qFunction (Ipv4Address target, Ipv4Address hop)
{
  std::vector<double> discounts;
  discounts.push_back (qFctGamma);
  discounts.push_back (std::min (
      1.0, sqrt (std::max (Phi_LET (hop), 0.0) /
                 (std::max (m_neighborReliabilityTimeout, mhChirpInterval).GetSeconds ()))));
  discounts.push_back (Vi.at (hop)->Phi_Coh ());

  return (1 - qFctAlpha) * m_QTable.at (target).at (hop)->Q () +
         qFctAlpha * (combineDiscounts (discounts) * m_QTable.at (target).at (hop)->V ());
}

double
RoutingProtocol::getMaxValueFor (Ipv4Address target)
{

  double res = -1000;
  if (m_QTable.find (target) != m_QTable.end ())
    {
      for (std::map<Ipv4Address, PCE *>::iterator act = m_QTable.find (target)->second.begin ();
           act != m_QTable.find (target)->second.end (); act++)
        {
          double deltaT = (Simulator::Now () - act->second->lastSeen ()).GetSeconds ();
          if (deltaT <=
              std::min (std::max (m_neighborReliabilityTimeout, mhChirpInterval).GetSeconds (),
                        Phi_LET (act->first)))
            {
              if (act == m_QTable.find (target)->second.begin ())
                {
                  // First possible action, make sure the result gets this value anyway
                  res = qFunction (target, act->first);
                }
              res = std::max (res, qFunction (target, act->first));
            }
        }
    }
  return res;
}

Ipv4Address
RoutingProtocol::getNextHopFor (Ipv4Address target)
{
  Ipv4Address a = Ipv4Address::GetZero ();
  double res = -1000;
  if (m_QTable.find (target) != m_QTable.end ())
    {
      for (std::map<Ipv4Address, PCE *>::iterator act = m_QTable.find (target)->second.begin ();
           act != m_QTable.find (target)->second.end ();)
        {
          double deltaT = (Simulator::Now () - act->second->lastSeen ()).GetSeconds ();
          if (deltaT <=
              std::min (std::max (m_neighborReliabilityTimeout, mhChirpInterval).GetSeconds (),
                        Phi_LET (act->first)))
            {

              if (act == m_QTable.find (target)->second.begin ())
                {
                  // First possible action, make sure the result gets this value anyway
                  res = qFunction (target, act->first);
                  a = act->first;
                }
              else if (qFunction (target, act->first) > res)
                { 
                  res = qFunction (target, act->first); 
                  a = act->first;
                }
              act++;
            }
          else
            {
              delete act->second;
        			act = m_QTable.at(target).erase(act);
            }
        }
    }
  return a;
}

//	Rewards
double
RoutingProtocol::R (Ipv4Address origin, Ipv4Address hop)
{
  return 0.0;
}

double
RoutingProtocol::Phi_LET (Ipv4Address neighbor)
{
  double t_elapsed_since_last_hello =
      (Simulator::Now () - Vi.at (neighbor)->lastSeen ()).GetSeconds ();
  Vector3D vj;
  Vector3D pj;

  vj = Vi.at (neighbor)->velo ();
  pj = Vi.at (neighbor)->coord () + VecMult (vj, t_elapsed_since_last_hello);

  Vector3D pi = (hist_coord.size () != 0) ? hist_coord[historySize - 1] : Vector3D (0, 0, 0);
  Vector3D vi =
      VecMult ((forecastPosition () - pi), 1 / ((m_neighborReliabilityTimeout.GetSeconds () != 0)
                                                    ? m_neighborReliabilityTimeout.GetSeconds ()
                                                    : 1.0));

  double px = (pj - pi).x;
  double vx = (vj - vi).x;
  double py = (pj - pi).y;
  double vy = (vj - vi).y;
  double pz = (pj - pi).z;
  double vz = (vj - vi).z;

  double a = pow (vx, 2) + pow (vy, 2) + pow (vz, 2);
  double b = 2 * (px * vx + py * vy + pz * vz);
  double c = pow (px, 2) + pow (py, 2) + pow (pz, 2) - pow (rangeOffset + r_com, 2);

  if (a == 0)
    {
      return (c < 0) ? 1.0 : 0.0;
    }
  double t1 = (-b + sqrt (pow (b, 2) - 4 * a * c)) / (2 * a);
  double t2 = (-b - sqrt (pow (b, 2) - 4 * a * c)) / (2 * a);
  double t = (t2 >= 0.0 || (t2 < 0.0 && t1 < 0.0)) ? 0.0 : t1;

  if (std::isnan (t))
    {
      t = 0.0;
    }
  return t;
}

void
RoutingProtocol::updatePhi_Coh ()
{
  int exclusive = 0;
  int merged = 0;
  std::vector<Ipv4Address> currentSetOfNeighbors;
  for (std::map<Ipv4Address, PDC *>::iterator it = Vi.begin (); it != Vi.end (); it++)
    {
      if (Simulator::Now () - it->second->lastSeen () <= m_neighborReliabilityTimeout)
        {
          currentSetOfNeighbors.push_back (it->first);
        }
    }

  for (std::vector<Ipv4Address>::iterator o = lastSetOfNeighbors.begin ();
       o != lastSetOfNeighbors.end (); o++)
    {
      if (std::find (currentSetOfNeighbors.begin (), currentSetOfNeighbors.end (), (*o)) ==
          currentSetOfNeighbors.end ())
        {
          // If entry is not in other vector, it is exclusive in first vector
          exclusive++;
        }
      // Every entry in first vector is a member of the union
      merged++;
    }
  for (std::vector<Ipv4Address>::iterator n = currentSetOfNeighbors.begin ();
       n != currentSetOfNeighbors.end (); n++)
    {
      if (std::find (lastSetOfNeighbors.begin (), lastSetOfNeighbors.end (), (*n)) ==
          lastSetOfNeighbors.end ())
        {
          // Only exclusive entries of second vector are considered to the merged counter, as all common entries have already been concerned in first iteration
          exclusive++;
          merged++;
        }
    }

  lastSetOfNeighbors = currentSetOfNeighbors;
  m_Phi_Coh = (merged != 0)
                  ? sqrt (1 - static_cast<double> (exclusive) / static_cast<double> (merged))
                  : 0.0;
}

} // namespace parrot
} // namespace ns3
