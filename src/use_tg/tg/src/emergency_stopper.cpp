#include <std_msgs/Bool.h>
#include <arcas_msgs/QuadStateEstimationStamped.h>
#include <arcas_msgs/Position.h>
#include <ros/ros.h>
#include <functions/Point3D.h>
#include <iostream>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>

using namespace std;
using namespace ros;

using functions::Point3D;
using boost::lexical_cast;
using boost::bad_lexical_cast;


double min_dist_xy;
double min_dist_z;
vector <Point3D> state_v;
Point3D no_state(-10000, -10000, -100);


void UALStateCallback(uint uav, const arcas_msgs::QuadStateEstimationStampedConstPtr &msg);

int main (int argc, char **argv) {
  if (argc < 4) {
    cerr << "Usage: " << argv[0] << "<first_uav> <last_uav> <min_dist_xy> [<min_dist_z>] [<rate>]\n";
    return -1;
  }
  
  int first_uav;
  int last_uav;
  double rate = 30.0;
  
  try {
    first_uav = lexical_cast<int>(argv[1]);
    last_uav = lexical_cast<int>(argv[2]);
    min_dist_z = min_dist_xy = lexical_cast<double>(argv[3]);
  }  catch (bad_lexical_cast &e) {
    cerr << "Warning: could not cast the mandatory arguments, using default.\n";
    return -2;
  }
  try {
    if (argc > 4) {
      min_dist_z = lexical_cast<double>(argv[4]);
    }
    if (argc > 5) {
      rate = lexical_cast<double>(argv[5]);
    }
    
  } catch (bad_lexical_cast &e) {
    cerr << "Warning: could not cast the extra arguments, using default.\n";
  }
  
  if (first_uav < 0 || last_uav < 0 || min_dist_xy < 0.0 || min_dist_z < 0.0) {
    cerr << "The id of the UAVs and the minimum distances must be positive. \n";
    return -3;
  }
  
  if (first_uav >= last_uav) {
    cerr << "The id of the first UAV has to be lower or equal than id of the last UAV. \n";
    return -4;
  }
  
  ros::init(argc, argv, "emergency_stopper");
  ros::NodeHandle n;
  
  vector<ros::Publisher > publishers;
  vector<ros::Subscriber> subscribers;
  
  
  
  for (int i = first_uav; i <= last_uav; i++) {
    ostringstream os;
    os << "/ual_" << i << "/emergency_stop";
    ostringstream os2;
    os2 << "ual_" << i << "/quad_state_estimation";
    boost::function<void (const arcas_msgs::QuadStateEstimationStampedConstPtr &)> f = boost::bind(UALStateCallback, i - first_uav, _1);
    publishers.push_back(n.advertise<std_msgs::Bool>(os.str(), 0));
    subscribers.push_back(n.subscribe(os2.str().c_str(), 0, f));
    state_v.push_back(no_state);
  }
  
  
  ros::Rate r(rate);
  while (ros::ok()) {
    for (int i = 0; i < last_uav - first_uav; i++) {
      if (state_v.at(i).distance2d(no_state) < 0.01 ) {
	continue;
      }
      for (int j = i + 1; j <= last_uav - first_uav; j++) {
	if (state_v.at(j).distance2d(no_state) < 0.01 ) {
	  continue;
	}
	
	if (state_v.at(i).distance2d(state_v.at(j)) < min_dist_xy && fabs(state_v.at(i).z - state_v.at(j).z) < min_dist_z) {
	  // Error --> send emergency stop
	  std_msgs::Bool b;
	  b.data = 1;
	  publishers.at(i).publish(b);
	  publishers.at(j).publish(b);
	  ROS_ERROR("Stopping UAVs %d. Position = %s . And %d = %s.", i + first_uav, 
		    state_v[i].toString().c_str(), j + first_uav, state_v[j].toString().c_str());
	}
      }
    }
    ros::spinOnce();
    r.sleep();
  }
  
  subscribers.clear();
  publishers.clear();
  state_v.clear();
  
  return 0;
}

void UALStateCallback(uint uav, const arcas_msgs::QuadStateEstimationStampedConstPtr &m) {
  arcas_msgs::Position a = m.get()->quad_state_estimation.position;
  state_v[uav].x = a.x;
  state_v[uav].y = a.y;
  state_v[uav].z = a.z;
  
}

// void UALStateCallback(uint uav, const arcas_msgs::UALStateStamped &msg) {
//   arcas_msgs::PositionWithCheck &st = msg.ual_state.dynamic_state.position;
//   if (st.valid) {
//     state_v.at(uav - 1) = Point3D(st.x, st.y, st.z);
//   }
// }