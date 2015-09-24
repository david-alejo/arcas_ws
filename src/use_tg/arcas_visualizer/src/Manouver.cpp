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


#include "Manouver.h"

Manouver::Manouver(const std::vector< functions::Point3D >& pos, unsigned int uav_id, double neighbor_dist)
{
  this->neighbor_dist = neighbor_dist;
  involved_uavs.push_back(uav_id);
  for (unsigned int i = 0; i < pos.size(); i++) {
    if (uav_id != i && pos.at(i).distance(pos.at(uav_id)) < neighbor_dist) {
      involved_uavs.push_back(i);
    }
  }
}

bool Manouver::isInManouver(unsigned int uav_id)
{
  bool ret_val = false;
  for (unsigned int i = 0; i < involved_uavs.size() && !ret_val; i++) {
    ret_val = uav_id == involved_uavs.at(i);
  }
  return ret_val;
}

bool Manouver::updateManouver(const std::vector< functions::Point3D >& pos, const std::vector<functions::Point3D> &vel)
{
  
}

