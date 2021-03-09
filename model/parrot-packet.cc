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

#include "parrot-packet.h"
#include "ns3/address-utils.h"

/**
 * Helper functions to serialize floating point numbers
 * 
 * Float is stored within an union, and the uint32_t (4-byte) representation of the stored bytes can be gathered.
 * This can then be serialized with the appropriate existing method.
 * 
 */
static uint32_t
FtoU32 (float v)
{
  /**
   * Converts a float to an uint32_t binary representation.
   * \param v The float value
   * \returns The uint32_t binary representation
   */
  union Handler {
    float f;
    uint32_t b;
  };

  Handler h;
  h.f = v;
  return h.b;
}

static float
U32toF (uint32_t v)
{
  /**
   * Converts an uint32_t binary representation to a float.
   * \param v The uint32_t binary representation
   * \returns The float value.
   */
  union Handler {
    float f;
    uint32_t b;
  };

  Handler h;
  h.b = v;
  return h.f;
}

namespace ns3 {
namespace parrot {

MultiHopChirp::MultiHopChirp (Ipv4Address orig, Vector3D p, Vector3D p_hat, float V, float Phi_Coh,
                              uint16_t seq, uint16_t ttl)
    : m_orig (orig),
      m_p (p),
      m_p_hat (p_hat),
      m_V (V),
      m_Phi_Coh (Phi_Coh),
      m_seq (seq),
      m_ttl (ttl)
{
}

MultiHopChirp::~MultiHopChirp ()
{
}

TypeId
MultiHopChirp::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::parrot::MultiHopChirp")
                          .SetParent<Header> ()
                          .SetGroupName ("PARRoT")
                          .AddConstructor<MultiHopChirp> ();
  return tid;
}

TypeId
MultiHopChirp::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

uint32_t
MultiHopChirp::GetSerializedSize () const
{
  return 40;
}

void
MultiHopChirp::Serialize (Buffer::Iterator i) const
{
  // TODO
  WriteTo (i, m_orig);
  i.WriteHtonU32 (FtoU32 ((float) m_p.x));
  i.WriteHtonU32 (FtoU32 ((float) m_p.y));
  i.WriteHtonU32 (FtoU32 ((float) m_p.z));
  i.WriteHtonU32 (FtoU32 ((float) m_p_hat.x));
  i.WriteHtonU32 (FtoU32 ((float) m_p_hat.y));
  i.WriteHtonU32 (FtoU32 ((float) m_p_hat.z));
  i.WriteHtonU32 (FtoU32 ((float) m_V));
  i.WriteHtonU32 (FtoU32 ((float) m_Phi_Coh));
  i.WriteHtonU16 (m_seq);
  i.WriteHtonU16 (m_ttl);
}

uint32_t
MultiHopChirp::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;
  ReadFrom (i, m_orig);
  float px = U32toF (i.ReadNtohU32 ());
  float py = U32toF (i.ReadNtohU32 ());
  float pz = U32toF (i.ReadNtohU32 ());
  float vx = U32toF (i.ReadNtohU32 ());
  float vy = U32toF (i.ReadNtohU32 ());
  float vz = U32toF (i.ReadNtohU32 ());


  m_V = U32toF (i.ReadNtohU32 ());
  m_Phi_Coh = U32toF (i.ReadNtohU32 ());
  m_seq = i.ReadNtohU16 ();
  m_ttl = i.ReadNtohU16 ();
  m_p = Vector3D (px, py, pz);
  m_p_hat =
      Vector3D (vx, vy, vz);
  uint32_t dist = i.GetDistanceFrom (start);
  NS_ASSERT (dist == GetSerializedSize ());
  return dist;
}

void
MultiHopChirp::Print (std::ostream &os) const
{
  os << "Originator: " << m_orig << " Position: (" << m_p.x << ", " << m_p.y << ", " << m_p.z << ")"
     << " Predicted Position: (" << m_p_hat.x << ", " << m_p_hat.y << ", " << m_p_hat.z << ")"
     << " Reward: " << m_V << " Cohesion: " << m_Phi_Coh << " SEQ: " << m_seq << " TTL: " << m_ttl;
}

} // namespace parrot

} // namespace ns3
