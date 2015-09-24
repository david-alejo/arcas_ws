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


#include "orca_module/Maneuver.h"
#include <functions/FormattedTime.h>
#include <functions/functions.h>

using namespace std;
using functions::FormattedTime;
using functions::minimum;

Maneuver::Maneuver(const std::vector<functions::Point3D> &pos, 
	   const functions::Point3D &vel, 
	   const functions::Point3D &d_vel, 
	   unsigned int uav_id)
{
  this->neighbor_dist = neighbor_dist;
  involved_uavs.push_back(uav_id);
  alive = true;
  FormattedTime t;
  t.getTime();
  start_time = t.getSec() + t.getUSec() / 1000000.0;
  minimum_miss_distance = 1e100;
  minimum_obstacle_distance = 1e100;
  cont_not_alive = 5;
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
			      const functions::Point3D &vel,
			      const functions::Point3D &d_vel , unsigned int uav_id, double obstacle_dist)
{
  if (!alive) 
    return false;
  
  minimum_obstacle_distance = minimum(obstacle_dist, minimum_obstacle_distance);
  
  if (vel.distance(d_vel) < MIN_DIFFERENCE / 2.0) {
    cont_not_alive--;
    alive = cont_not_alive > 0;
  } else {
    cont_not_alive = 5;
  }
  
  for (unsigned int i = 0; alive && i < pos.size(); i++) {
    if (i == uav_id) 
      continue;
    
    if ( isInManeuver(i) ) {
      minimum_miss_distance = minimum(minimum_miss_distance, pos.at(uav_id).distance(pos.at(i)));
      continue;
    }
    
    bool is_neighbor = false;
    for (unsigned int j = 0; !is_neighbor && j < involved_uavs.size(); j++) {
      is_neighbor = pos.at(involved_uavs.at(j)).distance(pos.at(i)) < MIN_DIST_MANEUVER;
    }
    if (is_neighbor) {
      involved_uavs.push_back(i);
    }
  }
  
  if (!alive) {
    FormattedTime t;
    t.getTime();
    end_time = t.getSec() + t.getUSec() / 1000000.0;
  }

  return alive;
}

string Maneuver::toString() const {
  ostringstream os;
  
  os.precision(15);
  
  os << start_time << " " << end_time << " " << involved_uavs.size() << " ";
  for (unsigned int i = 0; i < involved_uavs.size(); i++) {
    os << involved_uavs.at(i) + 1 << " ";
  }
  os << minimum_miss_distance << " " << minimum_obstacle_distance;
  
  
  return os.str();
}

