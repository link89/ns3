/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 IITP RAS
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
 * Based on
 *      NS-2 AODV model developed by the CMU/MONARCH group and optimized and
 *      tuned by Samir Das and Mahesh Marina, University of Cincinnati;
 *
 *      AODV-UU implementation by Erik Nordström of Uppsala University
 *      http://core.it.uu.se/core/index.php/AODV-UU
 *
 * Authors: Elena Buchatskaia <borovkovaes@iitp.ru>
 *          Pavel Boyko <boyko@iitp.ru>
 */

#include "aodv-neighbor.h"
#include "ns3/log.h"
#include <algorithm>
#include <limits>


namespace ns3
{
  
NS_LOG_COMPONENT_DEFINE ("AodvNeighbors");

namespace aodv
{
Neighbors::Neighbors (Time delay) : 
  m_ntimer (Timer::CANCEL_ON_DESTROY)
{
  m_ntimer.SetDelay (delay);
  m_ntimer.SetFunction (&Neighbors::Purge, this);
  m_txErrorCallback = MakeCallback (&Neighbors::ProcessTxError, this);
}

bool
Neighbors::IsNeighbor (Ipv4Address addr)
{
  Purge ();
  for (std::vector<Neighbor>::const_iterator i = m_nb.begin ();
       i != m_nb.end (); ++i)
    {
      if (i->m_neighborAddress == addr)
        return true;
    }
  return false;
}

Time
Neighbors::GetExpireTime (Ipv4Address addr)
{
  Purge ();
  for (std::vector<Neighbor>::const_iterator i = m_nb.begin (); i
       != m_nb.end (); ++i)
    {
      if (i->m_neighborAddress == addr)
        return (i->m_expireTime - Simulator::Now ());
    }
  return Seconds (0);
}

void
Neighbors::Update (Ipv4Address addr, Time expire)
{
  for (std::vector<Neighbor>::iterator i = m_nb.begin (); i != m_nb.end (); ++i)
    if (i->m_neighborAddress == addr)
      {
        i->m_expireTime
          = std::max (expire + Simulator::Now (), i->m_expireTime);
        if (i->m_hardwareAddress == Mac48Address ())
          i->m_hardwareAddress = LookupMacAddress (i->m_neighborAddress);
        return;
      }

  NS_LOG_LOGIC ("Open link to " << addr);
  Neighbor neighbor (addr, LookupMacAddress (addr), expire + Simulator::Now ());
  m_nb.push_back (neighbor);
  Purge ();
}

void
Neighbors::Update (Ipv4Address addr, Time expire,
                   Vector2D pos, Ptr<NetDevice> dev, Ipv4InterfaceAddress iface)
{
  for (std::vector<Neighbor>::iterator i = m_nb.begin (); i != m_nb.end (); ++i)
    if (i->m_neighborAddress == addr)
      {
        i->m_expireTime
          = std::max (expire + Simulator::Now (), i->m_expireTime);
        i->m_pos = pos;
        if (i->m_hardwareAddress == Mac48Address ())
          i->m_hardwareAddress = LookupMacAddress (i->m_neighborAddress);
        if (!i->m_ipv4Route)
          i->m_ipv4Route = Create<Ipv4Route> ();
        i->m_ipv4Route->SetDestination (addr);
        i->m_ipv4Route->SetGateway (addr);
        i->m_ipv4Route->SetSource (iface.GetLocal ());
        i->m_ipv4Route->SetOutputDevice (dev);
        return;
      }

  NS_LOG_LOGIC ("Open link to " << addr);
  Neighbor neighbor (addr, LookupMacAddress (addr), expire + Simulator::Now (),
                     pos, dev, iface);
  m_nb.push_back (neighbor);
  Purge ();
}

bool
Neighbors::BestNeighbor(Vector2D curPos, Vector2D dstPos, Ptr<Ipv4Route> & ipv4Route)
{
  if (m_nb.empty ())
    return false;

  double curDist = CalculateDistance(curPos, dstPos);
  double minDist = std::numeric_limits<double>::max();
  Neighbor* bestNeighbor;

  bool isBestNeighbor = false;
  for (std::vector<Neighbor>::iterator i = m_nb.begin (); i != m_nb.end (); ++i) {
    double toDstDist = CalculateDistance(i->m_pos, dstPos);
    if (i->m_ipv4Route && toDstDist < minDist && toDstDist < curDist ) {
      isBestNeighbor = true;
      minDist = toDstDist;
      bestNeighbor = &(*i);
    }
  }

  if (isBestNeighbor) {
    ipv4Route = bestNeighbor->m_ipv4Route;
    NS_LOG_DEBUG("best route = " << *ipv4Route << " curDist = " << curDist << " minDist = " << minDist);
  }

  return isBestNeighbor;
}

bool
Neighbors::RecoveryNeighbor(Vector2D curPos, Vector2D dstPos, Vector2D failPos,
                            Ptr<Ipv4Route> & ipv4Route)
{
  for (std::vector<Neighbor>::iterator i = m_nb.begin (); i != m_nb.end (); ++i) {
  }
  return true;
}

struct CloseNeighbor
{
  bool operator() (const Neighbors::Neighbor & nb) const
  {
    return ((nb.m_expireTime < Simulator::Now ()) || nb.close);
  }
};

void
Neighbors::Purge ()
{
  if (m_nb.empty ())
    return;

  CloseNeighbor pred;
  if (!m_handleLinkFailure.IsNull ())
    {
      for (std::vector<Neighbor>::iterator j = m_nb.begin (); j != m_nb.end (); ++j)
        {
          if (pred (*j))
            {
              NS_LOG_LOGIC ("Close link to " << j->m_neighborAddress);
              m_handleLinkFailure (j->m_neighborAddress);
            }
        }
    }
  m_nb.erase (std::remove_if (m_nb.begin (), m_nb.end (), pred), m_nb.end ());
  m_ntimer.Cancel ();
  m_ntimer.Schedule ();
}

void
Neighbors::ScheduleTimer ()
{
  m_ntimer.Cancel ();
  m_ntimer.Schedule ();
}

void
Neighbors::AddArpCache (Ptr<ArpCache> a)
{
  m_arp.push_back (a);
}

void
Neighbors::DelArpCache (Ptr<ArpCache> a)
{
  m_arp.erase (std::remove (m_arp.begin (), m_arp.end (), a), m_arp.end ());
}

Mac48Address
Neighbors::LookupMacAddress (Ipv4Address addr)
{
  Mac48Address hwaddr;
  for (std::vector<Ptr<ArpCache> >::const_iterator i = m_arp.begin ();
       i != m_arp.end (); ++i)
    {
      ArpCache::Entry * entry = (*i)->Lookup (addr);
      if (entry != 0 && entry->IsAlive () && !entry->IsExpired ())
        {
          hwaddr = Mac48Address::ConvertFrom (entry->GetMacAddress ());
          break;
        }
    }
  return hwaddr;
}

void
Neighbors::ProcessTxError (WifiMacHeader const & hdr)
{
  Mac48Address addr = hdr.GetAddr1 ();

  for (std::vector<Neighbor>::iterator i = m_nb.begin (); i != m_nb.end (); ++i)
    {
      if (i->m_hardwareAddress == addr)
        i->close = true;
    }
  Purge ();
}
}
}

