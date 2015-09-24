#ifndef __TG_DISCRETE_TRAJECTORY__
#define __TG_DISCRETE_TRAJECTORY__

#include <arcas_msgs/QuadStateEstimationStamped.h>
#include <arcas_msgs/Position.h>
#include <arcas_msgs/PathWithCruiseStamped.h>

#include <functions/Point3D.h>
#include <functions/functions.h>
#include "tg/TrajectoryFollower.h"

//! @class DiscreteTrajectoryFollower This class makes the UAV visit a list of points of interest (PoIs). Considers that a point has been 
//! visited when the distance from the poistion of the UAV to the PoI goes below the min_dist
class DiscreteTrajectoryFollower:public TrajectoryFollower {
public:
  DiscreteTrajectoryFollower(const arcas_msgs::PathWithCruise &path, double min_dist);
  
  inline bool done() const {
    return done_;
  };
  
  uint actualizeIndex(const arcas_msgs::Position &pos);
  
private:
  uint current_index;
  double min_dist;
  arcas_msgs::PathWithCruise path;
  bool done_;
  
  uint actualizeTargetIndex();
};

#endif
