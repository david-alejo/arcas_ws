#include "tg/DiscreteTrajectoryFollower.h"

using namespace arcas_msgs;

DiscreteTrajectoryFollower::DiscreteTrajectoryFollower(const PathWithCruise &path, double min_dist) {
  this->path = path;
  this->min_dist = min_dist;
  current_index = 0;
  done_ = false;
}

uint DiscreteTrajectoryFollower::actualizeIndex(const arcas_msgs::Position &pos) {
  functions::Point3D position(pos.x, pos.y, pos.z);
  functions::Point3D current(path.waypoints.at(current_index).x,
			     path.waypoints.at(current_index).y,
			     path.waypoints.at(current_index).z);
  
  if (path.waypoints.size() == 0) {
    done_ = true;
    current_index = 0;
  } else if (position.distance(current) < min_dist) {
    if (current_index == path.waypoints.size() - 1) {
      done_ = true;
    } else {
      current_index++;    
    }
  }
  
  return current_index;
}

