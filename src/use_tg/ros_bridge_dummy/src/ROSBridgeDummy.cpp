#include <ros/ros.h>
#include <iostream>
#include <string>
#include <functions/FormattedTime.h>
#include <functions/ArgumentData.h>
#include <arcas_msgs/PathWithCruiseStamped.h>

#include "util.h"

using namespace std;
using namespace ros;
using namespace arcas_msgs;

int main (int argc, char **argv) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " <filename 1> [<filename 2> ...] [--offset] [--velocity] [--arm] [--visualize [--width <width>]]\n";
    return -1;
  }
  
  bool velocity = false;
  bool debug = false;
  
  //Init ROS.
  string node_name = "RosBridge";
  ros::init(argc, argv, node_name.c_str());
  ros::NodeHandle n;
  
  //Publisher for send our next waypoint (will receive it the waypoint follower and the other uavs)
//   ifstream ifs("/home/sinosuke/remap");
  functions::ArgumentData arg(argc, argv);
  int offset = 3;
  if (arg.isOption("offset")) {
    arg.getOption("offset", offset);
    argc-=2;
  }
  
  if (arg.isOption("velocity")) {
    velocity = true;
    argc--;
  }
  
  if (arg.isOption("debug")) {
    debug = true;
    argc--;
  }
  
  bool visualize = false;
  if (arg.isOption("visualize")) {
    visualize = true;
    argc--;
  }
  
  double width = 0.15;
  if (arg.isOption("width")) {
    arg.getOption<double>("width", width);
    argc-=2;
  }
  
  vector<Publisher> way_pubs;
  
  for (int i = 1; i < argc; i++) {
    // Create the publisher
    ostringstream topicname;
    topicname << "/quad" << i + offset << "_trajectory/";
    way_pubs.push_back(n.advertise<arcas_msgs::PathWithCruiseStamped> (topicname.str().c_str(), 1));
  }
  bool quit = false;
  bool first = true;
  Visualization v(width);
  
  vector<PathWithCruise> paths;
  
  while (!quit) {
  
    for (int i = 1; i < argc; i++) {
      // Load each trajectory, connect to the subscriber and send the files
  //     ROS_INFO("Publishing trajectory %d" , i + 1);
      PathWithCruise path = loadTrajetory(argv[i], velocity, debug);
      if (first) {
	paths.push_back(path);
      }
      
      ROS_INFO("Loaded trajectory. Number of waypoints: %d.", (int) path.waypoints.size());
      
      if (visualize) {
	v.representTrajectory(i + offset, path);
      }
      
      PathWithCruiseStamped path_stamp;
      path_stamp.path_with_cruise = path;
      path_stamp.header.seq = 0;
      functions::FormattedTime t;
      t.getTime();
      path_stamp.header.stamp.sec = t.getSec();
      path_stamp.header.stamp.nsec = t.getUSec() * 1000;
      way_pubs.at(i - 1).publish(path_stamp);
    }
    ROS_INFO("All trajectories published successfully.");
    if (first) {
      first = false;
      sleep(1);
    } else {
//       cout << "Retry(y/N)?.\n";
//       string s;
//       cin >> s;
//       quit = s != "s" && s !="y" && s != "Y";
      sleep(2);
      quit = true;
    }
  }
  
  if (visualize) {
    
    for (unsigned int i = 0; i < paths.size(); i++) {
      v.representTrajectory(i + offset + 1, paths.at(i));
    }
  }
  sleep(2);
  
  return 0;
}
