#ifndef __TG_TRAJECTORY_FOLLOWER__
#define __TG_TRAJECTORY_FOLLOWER__

#include <arcas_msgs/QuadStateEstimationStamped.h>
#include <arcas_msgs/Position.h>
#include <arcas_msgs/PathWithCruiseStamped.h>
#include <arcas_actions_msgs/TakeOffAction.h>

#include <functions/Point3D.h>
#include <functions/functions.h>

//! @class TrajectoryFollower Abstract class to be implemented by all trajectories followers
class TrajectoryFollower {
public:
  virtual bool done() const = 0;  
  virtual uint actualizeIndex(const arcas_msgs::Position &pos) = 0;
  virtual inline uint getCurrentIndex() {return current_index;}
  
protected:
  uint current_index;
  arcas_msgs::PathWithCruise path;
};

#endif
