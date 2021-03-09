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

#define steeringVectorAvailable false
#define WP_REACHED_RANGE 10.0

namespace ns3 {
namespace parrot {

void
RoutingProtocol::trackPosition (Vector3D pos)
{
  hist_coord_t.push_back (Simulator::Now ());
  hist_coord.push_back (pos);
  if ((int) hist_coord.size () > historySize)
    {
      hist_coord_t.pop_front ();
      hist_coord.pop_front ();
    }
}

Vector3D
RoutingProtocol::forecastPosition ()
{
  if (hist_coord.size () == 0)
    {
      // Not enough previous points for forecast available
      return Vector3D (0, 0, 0);
    }
  else
    {
      // Fit slope
      if (predictionMethod == "slope")
        {
          double mx = 0.0;
          double my = 0.0;
          double mz = 0.0;
          for (int i = 1; i < historySize; i++)
            {
              mx += (hist_coord[i].x - hist_coord[i - 1].x) / (mhChirpInterval.GetSeconds ());
              my += (hist_coord[i].y - hist_coord[i - 1].y) / (mhChirpInterval.GetSeconds ());
              mz += (hist_coord[i].z - hist_coord[i - 1].z) / (mhChirpInterval.GetSeconds ());
            }
          mx /= (historySize - 1);
          my /= (historySize - 1);
          mz /= (historySize - 1);

          // Forecast
          Vector3D p = hist_coord[historySize - 1];

          return Vector3D ((mx * m_neighborReliabilityTimeout.GetSeconds () + p.x),
                           (my * m_neighborReliabilityTimeout.GetSeconds () + p.y),
                           (mz * m_neighborReliabilityTimeout.GetSeconds () + p.z));
        }
      else if (predictionMethod == "waypoint")
        {
          // Fit
          if (m_neighborReliabilityTimeout.GetSeconds () <= 0)
            {
              return hist_coord[historySize - 1];
            }
          double m_updateInterval_ms = mhChirpInterval.GetSeconds () * 1000;
          double _currentTime_ms = hist_coord_t[historySize - 1].GetSeconds () * 1000;

          std::deque<Vector3D> predictedData;
          std::deque<Vector3D> historyData = hist_coord;

          std::deque<Vector3D> plannedWaypoints; // = mob->gcWP (5); // Try to get some Waypoints
          if (mobility->GetObject<ControlledRandomWaypointMobilityModel>()){
            plannedWaypoints = mobility->GetObject<ControlledRandomWaypointMobilityModel>()->gcWP(5);
          }

          if (hist_coord.size () > 0)
            {
              int time_ms =
                  (int) floor (_currentTime_ms / m_updateInterval_ms) * m_updateInterval_ms;

              Vector3D lastValidData = hist_coord[historySize - 1];
              Vector3D currentData = lastValidData;
              for (int i = 0; i < (int) floor (m_neighborReliabilityTimeout.GetSeconds () /
                                               mhChirpInterval.GetSeconds ());
                   i++)
                {
                  time_ms += m_updateInterval_ms;
                  if (steeringVectorAvailable)
                    {
                      throw;
                    }
                  else if (plannedWaypoints.size () > 0)
                    {
                      currentData =
                          predictWithTarget (currentData, m_updateInterval_ms, plannedWaypoints[0]);
                      if ((plannedWaypoints[0] - currentData).GetLength () <= WP_REACHED_RANGE)
                        {
                          plannedWaypoints.pop_front ();
                        }
                    }
                  else
                    {
                      currentData = predictWithHistory (historyData, hist_coord_t, time_ms);
                    }
                  predictedData.push_back (currentData);
                  historyData.push_back (currentData);
                  historyData.pop_front ();
                }

              return predictedData[predictedData.size () - 1];
            }
          else
            {
              return Vector3D (0, 0, 0);
            }
        }
      else
        {
          throw;
        }
    }
}

Vector3D
RoutingProtocol::predictWithTarget (Vector3D _currentData, int m_updateInterval_ms, Vector3D wp0)
{
  Vector3D nextData = _currentData;

  // compute the next position
  Vector3D position = _currentData;
  Vector3D target = wp0;

  double m_maxSpeed_mpMs = 50 / 3.6 / 1000;

  nextData = position + VecMult (target - position, 1 / ((target - position).GetLength () *
                                                         (m_updateInterval_ms) *m_maxSpeed_mpMs));

  return nextData;
}

Vector3D
RoutingProtocol::predictWithHistory (std::deque<Vector3D> _historyData, std::deque<Time> times,
                                     int _nextTime_ms)
{
  std::deque<Vector3D> historyData = _historyData;
  if (historyData.size () == 1)
    {
      Vector3D nextData = historyData.at (historyData.size () - 1);
      return nextData;
    }
  else
    {
      Vector3D lastValidData = _historyData.at (historyData.size () - 1);

      // compute the position increment
      Vector3D positionIncrement;
      float totalWeight = 0;
      for (int unsigned i = 1; i < _historyData.size (); i++)
        {
          Vector3D currentData = _historyData.at (i);
          Vector3D lastData = _historyData.at (i - 1);

          float weight = 1;
          Vector3D increment = VecMult (
              currentData - lastData,
              weight / (times.at (i) - times.at (i - 1)).GetSeconds ()); // going to seconds here!

          positionIncrement = increment + positionIncrement;
          totalWeight += weight;
        }
      positionIncrement = VecMult (positionIncrement, 1 / totalWeight);

      //
      Vector3D nextData =
          lastValidData +
          VecMult (positionIncrement,
                   (_nextTime_ms / 1000 - times.at (historyData.size () - 1).GetSeconds ()));
      return nextData;
    }
}

} // namespace parrot
} // namespace ns3
