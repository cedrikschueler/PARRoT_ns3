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

#ifndef PARROT_PACKET_H
#define PARROT_PACKET_H

#include <iostream>
#include "ns3/header.h"

#include "ns3/ipv4-address.h"
#include "ns3/vector.h"

namespace ns3 {
namespace parrot {
/**
 * \ingroup parrot
 * \brief PARRoT Multi-Hop Chirp Packet Format (40 Byte)
 * \verbatim
 |      0        |      1        |      2        |       3       |
  0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |                      Originator Address                       |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |   x                                                           |
 |   y                  Current Position p(t)                    |
 |   z                                                           |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |   x                                                           |
 |   y            Predicted Position ^p(t + tau)                 |
 |   z                                                           |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |                           Reward V                            |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |                       Cohesion Phi_{Coh}                      |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |               SEQ             |              TTL              |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * \endverbatim
 * 
 */

class MultiHopChirp : public Header
{

public:
  /**
     * Constructor
     * 
     * \param orig originator IP address
     * \param p current position
     * \param p_hat predicted position
     * \param V reward
     * \param Phi_Coh cohesion metric
     * \param seq sequence number
     * \param ttl time to live
     */
  MultiHopChirp (Ipv4Address orig = Ipv4Address (), Vector3D p = Vector3D (0, 0, 0),
                 Vector3D p_hat = Vector3D (0, 0, 0), float V = 1.0, float Phi_Coh = 1.0,
                 uint16_t seq = 0, uint16_t ttl = 0);
  virtual ~MultiHopChirp ();
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize () const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;

  /**
 * Set originator address
 * \param orig the originator address
 */
  void
  SetOrig (Ipv4Address orig)
  {
    m_orig = orig;
  }
  /**
 * Get originator address 
 * \returns the originator address
 */
  Ipv4Address
  GetOrig () const
  {
    return m_orig;
  }
  /**
   * Set current position
   * \param p the current position
   */
  void
  SetP (Vector3D p)
  {
    m_p = p;
  }
  /**
   * Set current position
   * \param x the x coordinate
   * \param y the y coordinate
   * \param z the z coordinate
   */
  void
  SetP (double x, double y, double z)
  {
    m_p = Vector3D (x, y, z);
  }
  /**
   * Get current position
   * \returns the current position
   */
  Vector3D
  GetP () const
  {
    return m_p;
  }
  /**
   * Set predicted position
   * \param p_hat the predicted position
   */
  void
  SetP_hat (Vector3D p_hat)
  {
    m_p_hat = p_hat;
  }
  /**
   * Set predicted position
   * \param x_hat the predicted x coordinate
   * \param y_hat the predicted y coordinate
   * \param z_hat the predicted z coordinate
   */
  void
  SetP_hat (double x_hat, double y_hat, double z_hat)
  {
    m_p_hat = Vector3D (x_hat, y_hat, z_hat);
  }
  /**
   * Get predicted position
   * \returns the predicted position
   */
  Vector3D
  GetP_hat () const
  {
    return m_p_hat;
  }
  /**
   * Set reward V
   * \param V the reward
   */
  void
  SetV (float V)
  {
    m_V = V;
  }
  /**
   * Get reward V
   * \returns the reward V
   */
  float
  GetV () const
  {
    return m_V;
  }
  /**
   * Set cohesion metric Phi_Coh
   * \param Phi_Coh
   */
  void
  SetPhi_Coh (float Phi_Coh)
  {
    m_Phi_Coh = Phi_Coh;
  }
  /**
   * Get cohesion metric Phi_Coh
   * \returns the cohesion metric Phi_Coh
   */
  float
  GetPhi_Coh () const
  {
    return m_Phi_Coh;
  }
  /**
   * Set sequence number
   * \param seq the sequence number
   */
  void
  SetSeq (uint16_t seq)
  {
    m_seq = seq;
  }
  /**
   * Get sequence number
   * \returns the sequence number
   */
  uint16_t
  GetSeq () const
  {
    return m_seq;
  }
  /**
   * Set time to live
   * \param ttl the time to live
   */
  void
  SetTtl (uint16_t ttl)
  {
    m_ttl = ttl;
  }
  /**
   * Get time to live
   * \returns the time to live
   */
  uint16_t
  GetTtl () const
  {
    return m_ttl;
  }

private:
  Ipv4Address m_orig;
  Vector3D m_p;
  Vector3D m_p_hat;
  float m_V;
  float m_Phi_Coh;
  uint16_t m_seq;
  uint16_t m_ttl;
};
static inline std::ostream &
operator<< (std::ostream &os, const MultiHopChirp &packet)
{
  packet.Print (os);
  return os;
}
} // namespace parrot
} // namespace ns3

#endif // PARROT_PACKET_H
