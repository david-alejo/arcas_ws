#include <ros/ros.h>
#include <iostream>
#include <string>
#include <functions/FormattedTime.h>
#include <functions/ArgumentData.h>
#include <arcas_msgs/PathWithCruiseStamped.h>
#include <boost/lexical_cast.hpp>

using boost::lexical_cast;
using boost::bad_lexical_cast;

#include "util.h"

using namespace std;
using namespace ros;
using namespace arcas_msgs;

int main (int argc, char **argv) {
  if (argc < 4) {
    cerr << "Usage: " << argv[0] << " <uav_id_first> <uav_id_last> <base_file> [--except <uav_except_id1> ..] [--velocity]\n";
    return -1;
  }
  
  bool velocity = false;
  bool debug = false;
  int first = 0;
  int last = -1;
  int except = -1;
  
  functions::ArgumentData ar(argc, argv);
  
  try {
    first = lexical_cast<int>(argv[1]);
    last = lexical_cast<int>(argv[2]);
  } catch (bad_lexical_cast &) {
    cerr << "The second argument must be the UAV id.\n";
    return -2;
  }
  
  if (ar.isOption("velocity")) {
    velocity = true;
    argc--;
  }
  
  if (ar.isOption("except")) {
    ar.getOption("except",except);
  }
  
  //Init ROS.
  string node_name = "RosBridgeDummyIncremental";
  ros::init(argc, argv, node_name.c_str());
  ros::NodeHandle n;
  
  vector<Publisher> way_pubs;
  
  for (int i = first; i <= last; i++) {
    if (i == except) {
      continue;
    }
    // Create the publisher
    ostringstream topicname;
    topicname << "/quad" << i << "_trajectory/";
    way_pubs.push_back(n.advertise<arcas_msgs::PathWithCruiseStamped> (topicname.str().c_str(), 1));
  }
  
  bool quit = false;
  bool first_time = true;
  while (!quit) {
  
    for (int i = first; i <= last; i++) {
      if (i==except) continue;
      // Load each trajectory, connect to the subscriber and send the files
  //     ROS_INFO("Publishing trajectory %d" , i + 1);
      ostringstream os;
      os << argv[3] << i << ".m";
      PathWithCruise path = loadTrajetory(os.str().c_str(), velocity, debug);
      ROS_INFO("Loaded trajectory. Number of waypoints: %d.", (int) path.waypoints.size());
      PathWithCruiseStamped path_stamp;
      path_stamp.path_with_cruise = path;
      path_stamp.header.seq = 0;
      functions::FormattedTime t;
      t.getTime();
      path_stamp.header.stamp.sec = t.getSec();
      path_stamp.header.stamp.nsec = t.getUSec() * 1000;
      way_pubs.at(i -first).publish(path_stamp);
    }
    ROS_INFO("All trajectories published successfully.");
    if (first_time) {
      first_time = false;
      sleep(1);
    } else {
      sleep(1);
      quit = true;
    }
  }
  
  
  return 0;
}
