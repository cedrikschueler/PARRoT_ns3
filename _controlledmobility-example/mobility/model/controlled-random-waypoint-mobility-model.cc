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
/*
 * Copyright (c) 2007 INRIA
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
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */
#include <cmath>
#include "ns3/integer.h"
#include "ns3/simulator.h"
#include "ns3/random-variable-stream.h"
#include "ns3/pointer.h"
#include "ns3/string.h"
#include "controlled-random-waypoint-mobility-model.h"
#include "position-allocator.h"

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (ControlledRandomWaypointMobilityModel);

TypeId
ControlledRandomWaypointMobilityModel::GetTypeId (void)
{
  static TypeId tid =
      TypeId ("ns3::ControlledRandomWaypointMobilityModel")
          .SetParent<MobilityModel> ()
          .SetGroupName ("Mobility")
          .AddConstructor<ControlledRandomWaypointMobilityModel> ()
          .AddAttribute ("Speed",
                         "A random variable used to pick the speed of a random waypoint model.",
                         StringValue ("ns3::UniformRandomVariable[Min=0.3|Max=0.7]"),
                         MakePointerAccessor (&ControlledRandomWaypointMobilityModel::m_speed),
                         MakePointerChecker<RandomVariableStream> ())
          .AddAttribute ("Pause",
                         "A random variable used to pick the pause of a random waypoint model.",
                         StringValue ("ns3::ConstantRandomVariable[Constant=2.0]"),
                         MakePointerAccessor (&ControlledRandomWaypointMobilityModel::m_pause),
                         MakePointerChecker<RandomVariableStream> ())
          .AddAttribute ("PositionAllocator",
                         "The position model used to pick a destination point.", PointerValue (),
                         MakePointerAccessor (&ControlledRandomWaypointMobilityModel::m_position),
                         MakePointerChecker<PositionAllocator> ())
          .AddAttribute ("WaypointBufferSize", "The size of the waypoint buffer", IntegerValue (5),
                         MakeIntegerAccessor (&ControlledRandomWaypointMobilityModel::m_bufferSize),
                         MakeIntegerChecker<int> ());

  return tid;
}

void
ControlledRandomWaypointMobilityModel::ControlBuffer ()
{
  NS_ASSERT_MSG (m_position, "No position allocator added before using this model");
  while ((int)m_buffer.size () < m_bufferSize)
    {
      m_buffer.push_back (m_position->GetNext ());
    }
}

std::deque<Vector>
ControlledRandomWaypointMobilityModel::gcWP (int n)
{
  std::deque<Vector> res;
  for (int i = 0; i < std::min (n, (int) m_buffer.size ()); i++)
    {
      res.push_back (m_buffer[i]);
    }
  return res;
}

void
ControlledRandomWaypointMobilityModel::BeginWalk (void)
{
  m_helper.Update ();
  Vector m_current = m_helper.GetCurrentPosition ();

  // Waypoint routine
  // 1. Get first element of buffer and remove it from the buffer
  Vector destination = m_buffer.front ();
  m_buffer.pop_front ();
  // 2. Re-fill buffer
  ControlBuffer ();

  double speed = m_speed->GetValue ();
  double dx = (destination.x - m_current.x);
  double dy = (destination.y - m_current.y);
  double dz = (destination.z - m_current.z);
  double k = speed / std::sqrt (dx * dx + dy * dy + dz * dz);

  m_helper.SetVelocity (Vector (k * dx, k * dy, k * dz));
  m_helper.Unpause ();
  Time travelDelay = Seconds (CalculateDistance (destination, m_current) / speed);
  m_event.Cancel ();
  m_event = Simulator::Schedule (travelDelay,
                                 &ControlledRandomWaypointMobilityModel::DoInitializePrivate, this);
  NotifyCourseChange ();
}

void
ControlledRandomWaypointMobilityModel::DoInitialize (void)
{
  ControlBuffer ();
  DoInitializePrivate ();
  MobilityModel::DoInitialize ();
}

void
ControlledRandomWaypointMobilityModel::DoInitializePrivate (void)
{
  m_helper.Update ();
  m_helper.Pause ();
  Time pause = Seconds (m_pause->GetValue ());
  m_event = Simulator::Schedule (pause, &ControlledRandomWaypointMobilityModel::BeginWalk, this);
  NotifyCourseChange ();
}

Vector
ControlledRandomWaypointMobilityModel::DoGetPosition (void) const
{
  m_helper.Update ();
  return m_helper.GetCurrentPosition ();
}
void
ControlledRandomWaypointMobilityModel::DoSetPosition (const Vector &position)
{
  m_helper.SetPosition (position);
  m_event.Cancel ();
  m_event =
      Simulator::ScheduleNow (&ControlledRandomWaypointMobilityModel::DoInitializePrivate, this);
}
Vector
ControlledRandomWaypointMobilityModel::DoGetVelocity (void) const
{
  return m_helper.GetVelocity ();
}
int64_t
ControlledRandomWaypointMobilityModel::DoAssignStreams (int64_t stream)
{
  int64_t positionStreamsAllocated;
  m_speed->SetStream (stream);
  m_pause->SetStream (stream + 1);
  NS_ASSERT_MSG (m_position, "No position allocator added before using this model");
  positionStreamsAllocated = m_position->AssignStreams (stream + 2);
  return (2 + positionStreamsAllocated);
}

} // namespace ns3
