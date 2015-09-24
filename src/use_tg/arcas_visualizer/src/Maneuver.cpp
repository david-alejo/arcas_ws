/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2014  sinosuke <email>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/


#include "Maneuver.h"

Maneuver::Maneuver(const std::vector<functions::Point3D> &pos, 
	   const std::vector< functions::Point3D >& vel, 
	   const std::vector< functions::Point3D >& d_vel, 
	   unsigned int uav_id)
{
  this->neighbor_dist = neighbor_dist;
  involved_uavs.push_back(uav_id);
  for (unsigned int i = 0; i < pos.size(); i++) {
    if (uav_id != i && pos.at(i).distance(pos.at(uav_id)) < neighbor_dist && 
        vel.at(uav_id).distance(d_vel.at(uav_id)) > MIN_DIFFERENCE
    ) {
      involved_uavs.push_back(i);
    }
  }
}

bool Maneuver::isInManeuver(unsigned int uav_id)
{
  bool ret_val = false;
  for (unsigned int i = 0; i < involved_uavs.size() && !ret_val; i++) {
    ret_val = uav_id == involved_uavs.at(i);
  }
  return ret_val;
}

bool Maneuver::updateManeuver(const std::vector< functions::Point3D >& pos, 
			      const std::vector<functions::Point3D> &vel,
			      const std::vector<functions::Point3D> &d_vel )
{
  bool still_alive = false;
  for (unsigned int i = 0; !still_alive && i < involved_uavs.size(); i++) {
    unsigned int &uav_id = involved_uavs.at(i);
    if (vel.at(uav_id).distance(d_vel.at(uav_id)) > MIN_DIFFERENCE) {
      still_alive = true;
    }
  }
  
  for (unsigned int i = 0; still_alive && i < pos.size(); i++) {
    if ( isInManeuver(i) ) {
      continue;
    }
    
    bool is_neighbor = false;
    for (unsigned int j = 0; !is_neighbor && j < involved_uavs.size(); j++) {
      is_neighbor = pos.at(involved_uavs.at(j)).distance(pos.at(i));
    }
    
    if (is_neighbor && vel.at(i).distance(d_vel.at(i)) > MIN_DIFFERENCE) {
      involved_uavs.push_back(i);
    }
  }

  return still_alive;
}

