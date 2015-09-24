#ifndef _UTIL_ROS_BRIDGE_CPP_
#define _UTIL_ROS_BRIDGE_CPP_

#include <arcas_msgs/PathWithCruiseStamped.h>

#include <functions/Point3D.h>
#include <functions/functions.h>

#include <visualization_msgs/Marker.h>
#include <ros/publisher.h>

arcas_msgs::PathWithCruise loadTrajetory(const char *filename, bool velocity, bool debug);

class Visualization {
public:
  Visualization(double width);
  
  void representTrajectory(int uav, const arcas_msgs::PathWithCruise &path);
  
private:
  ros::Publisher pub_marker;
  double width;
};

#endif