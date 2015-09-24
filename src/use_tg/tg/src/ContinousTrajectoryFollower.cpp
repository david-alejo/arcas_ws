#include "tg/ContinuousTrajectoryFollower.h"

using namespace arcas_msgs;

ContinuousTrajectoryFollower::ContinuousTrajectoryFollower(const PathWithCruise &path, double look_ahead_distance) {
  this->path = path;
  current_index = 0;
  this->lookahead_distance = look_ahead_distance;
  actualizeTargetIndex();
}

uint ContinuousTrajectoryFollower::actualizeIndex(const arcas_msgs::Position &pos) {
  unsigned int cont = current_index + 1;
  unsigned int new_index = current_index;
  functions::Point3D position(pos.x, pos.y, pos.z);
  functions::Point3D current(path.waypoints.at(current_index).x,
			     path.waypoints.at(current_index).y,
			     path.waypoints.at(current_index).z);
  double last_dist = position.distance(current);
  double curr_dist = last_dist;
  
  int min_ways_check = 2;
  int cont_2 = 0;
  
  do {
    
    if (cont < path.waypoints.size()) {
      current.x = path.waypoints.at(cont).x;
      current.y = path.waypoints.at(cont).y;
      current.z = path.waypoints.at(cont).z;
      curr_dist = position.distance(current);
      if (curr_dist <= last_dist) {
	new_index = cont;
	last_dist = curr_dist;
      }
    }
    cont++;
    cont_2++;
  } while ((cont < path.waypoints.size() && curr_dist <= last_dist) || cont_2 < min_ways_check);
  
 
  if (new_index != current_index) {
    current_index = new_index;
    new_index = actualizeTargetIndex(); // Actualize the target index value
  } else {
    current_index++; // little step further in order to avoid local minima
    new_index = actualizeTargetIndex(); // Actualize the target index value
  }
  
  return target_index; // Returns the waypoint index the uav has to point to.
}

uint ContinuousTrajectoryFollower::actualizeTargetIndex() {
  uint ret_val = current_index;
  double acc_dist = 0.0;
  
  functions::Point3D current(path.waypoints.at(current_index).x,
			     path.waypoints.at(current_index).y,
			     path.waypoints.at(current_index).z);
  
  functions::Point3D target(path.waypoints.at(current_index).x,
			     path.waypoints.at(current_index).y,
			     path.waypoints.at(current_index).z);
  
  for (;ret_val < path.waypoints.size() && acc_dist < lookahead_distance; ret_val++) {
    target.x = path.waypoints.at(ret_val).x;
    target.y = path.waypoints.at(ret_val).y;
    target.z = path.waypoints.at(ret_val).z;
    acc_dist += current.distance(target);
    current = target;
  }
  
  target_index = ret_val - 1;
  
  return ret_val - 1;
}
