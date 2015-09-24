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


#ifndef MANOUVER_H
#define MANOUVER_H
#include <vector>
#include <functions/Point3D.h>

#define MIN_DIFFERENCE 0.1

class Maneuver
{
public:
  //! @brief Creates a manouver with a uav_id and searches for surrounding UAVs
  Maneuver(const std::vector<functions::Point3D> &pos, 
	   const std::vector< functions::Point3D >& vel, 
	   const std::vector< functions::Point3D >& d_vel, 
	   unsigned int uav_id);
  
  bool isInManeuver(unsigned int uav_id);
  
  //! @brief Updates an alive manouver
  //! @retval true The Maneuver is still alive
  //! @retval false The Maneuver has been completed
  bool updateManeuver(const std::vector< functions::Point3D >& pos, 
		      const std::vector< functions::Point3D >& vel, 
		      const std::vector< functions::Point3D >& d_vel);
  
  
  
private:
  double start_time, end_time;
  double neighbor_dist;
  std::vector<unsigned int> involved_uavs;
  double minimum_miss_distance;
  double minimum_obstacle_distance;
};

#endif // MANOUVER_H
