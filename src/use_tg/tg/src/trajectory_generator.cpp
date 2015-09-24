#include <ros/ros.h>
#include <iostream>
#include <string>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

#include <arcas_msgs/QuadStateEstimationWithCovarianceStamped.h>
#include <arcas_msgs/PositionWithCheck.h>
#include <arcas_msgs/PathWithCruiseStamped.h>
#include <arcas_actions_msgs/TakeOffAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include <functions/ArgumentData.h>
#include <functions/Point3D.h>
#include <functions/functions.h>
#include <boost/lexical_cast.hpp>

#include "tg/TrajectoryFollower.h"
#include "tg/ContinuousTrajectoryFollower.h"
#include "tg/DiscreteTrajectoryFollower.h"

using namespace std;
using namespace ros;
using namespace arcas_msgs;
using namespace arcas_actions_msgs;
using functions::Point3D;

actionlib::SimpleActionClient<TakeOffAction>* cTakeOff;

//UAV id
int uav_id;

//Node name
string node_name;

// Parameter: the lookahead time of the waypoint
double T = 1.0;

// Flag: emergency stop. It is set to false but any publisher can switch it to true
bool emergency_stop = false;

//Last state of our uav
QuadStateEstimationStamped last_ual_state;

// The desired velocity
geometry_msgs::Twist my_vel;

//nextWayPoint
ros::Publisher my_vel_pub;

//My next waypoint
void waypointListCallback(const arcas_msgs::PathWithCruiseStamped::ConstPtr& vel);

//Actual state of our uav
void UALStateCallback(const QuadStateEstimationWithCovarianceStamped::ConstPtr& state);

//Emergency mode callback
void emergencyStopCallback(const std_msgs::Bool::ConstPtr& stop);

// Publishes the waypoint. This waypoint will be followed by the waypoint follower
void publishVelocity();

// Takes-off the UAV if necessary
void takeoff();

// Necessary things for navigating a flight plan.
unsigned int current_way = 0;
bool plan_received_and_being_executed = false;
double min_dist = 0.3;
bool loop = false;
QuadStateEstimationWithCovarianceStamped curr_state, first_state;
struct GoalID {
  unsigned int id;
  PathWithCruiseStamped goal;
};

struct GoalID curr_goal; 
unsigned int n_received = 0;
bool received_first_state = false;

TrajectoryFollower *follower = NULL;

int main(int argc, char** argv)
{

  if (argc < 2) {
    cout << "Use: " << argv[0] << " <uav_id> [--min_dist <min_dist>] [--loop]\n";
    return -1;
  }
  
  try
  {
    uav_id = boost::lexical_cast<int>(argv[1]);
  }
  catch(boost::bad_lexical_cast const&)
  {
    perror("The second argument is not a number.");
    return 1;
  }

  //Init ROS.
  node_name = "trajectory_generator_";
  node_name.append(argv[1]);
  ros::init(argc, argv, node_name.c_str());
  ros::NodeHandle n;

  // Path subscriber
  std::stringstream ss;
  ss << "/quad" << uav_id << "_trajectory";
  ros::Subscriber subWaypointList = n.subscribe(ss.str().c_str(), 0, waypointListCallback);

  //Subscribe to ours uav state
  stringstream ss2;
  ss2 << "ual_" << uav_id << "/quad_state_estimation";
  ros::Subscriber subState = n.subscribe(ss2.str().c_str(), 0, UALStateCallback);
  
  // Subscribe to emergency stop
  stringstream ss3;
  ss3 << "ual_" << uav_id << "/emergency_stop";
  ros::Subscriber subEmergency = n.subscribe(ss3.str().c_str(), 0, emergencyStopCallback);

  //Publisher for send the desire velocity
  string topicname= node_name;
  topicname.append("/out_velocity");
  
  functions::ArgumentData ar(argc, argv);
  
  if (ar.isOption("remap")) {
    ostringstream os;
    os << "/orca_" << uav_id << "/v_pref";
    
    topicname = os.str();
  }
  
  my_vel_pub = n.advertise<geometry_msgs::Twist> (topicname.c_str(), 0);
  
  // First publish a zero velocity
  my_vel.linear.x = 0.0;
  my_vel.linear.y = 0.0;
  my_vel.linear.z = 0.0;
  
  double rate = 30.0;
  
  loop = ar.isOption("loop");
  
  if (ar.isOption("min_dist")) {
    ar.getOption("min_dist", min_dist);
    ROS_INFO("Changing the minimum distance, or lookahead distance to %f", min_dist);
  }
  
  if (ar.isOption("rate")) {
    ar.getOption("rate", rate);
  }
  
  ros::Rate r(rate);
  while (ros::ok()) {
    publishVelocity();

  // Starts an asynchronous spinner in order to inmediately response the publications of state, etc
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}

void publishVelocity()
{
  functions::Point3D vel(0.0, 0.0, 0.0);

  if (!plan_received_and_being_executed) {
    if (curr_goal.goal.path_with_cruise.waypoints.size() > 0) {
      int index = curr_goal.goal.path_with_cruise.waypoints.size() - 1;
      functions::Point3D goal(curr_goal.goal.path_with_cruise.waypoints[index].x, 
			    curr_goal.goal.path_with_cruise.waypoints[index].y,
			    curr_goal.goal.path_with_cruise.waypoints[index].z);
      functions::Point3D current_pos(curr_state.quad_state_estimation_with_covariance.position.x, 
				     curr_state.quad_state_estimation_with_covariance.position.y, 
				     curr_state.quad_state_estimation_with_covariance.position.z);
      vel = (goal - current_pos) * 0.5;
    }
  } else if (!emergency_stop) {
    unsigned int last_way = current_way;
    current_way = follower->actualizeIndex(curr_state.quad_state_estimation_with_covariance.position);
    
    if (last_way != current_way) {
      ROS_INFO("Reached waypoint %u, next waypoint: %u", last_way, current_way);
    }
    
    if (follower->done()) {
      plan_received_and_being_executed = false;
      ROS_INFO("The plan has been executed successfully.");
    }
    functions::Point3D goal(curr_goal.goal.path_with_cruise.waypoints[current_way].x, 
			    curr_goal.goal.path_with_cruise.waypoints[current_way].y,
			    curr_goal.goal.path_with_cruise.waypoints[current_way].z);
    functions::Point3D current_pos(curr_state.quad_state_estimation_with_covariance.position.x, 
				     curr_state.quad_state_estimation_with_covariance.position.y, 
				     curr_state.quad_state_estimation_with_covariance.position.z);
    vel = (goal - current_pos);
    vel.normalize();
    vel = vel * curr_goal.goal.path_with_cruise.waypoints[current_way].cruise;
  }
  
  my_vel.linear.x = vel.x;
  my_vel.linear.y = vel.y;
  my_vel.linear.z = vel.z;
  my_vel.angular.x = 0.0;
  my_vel.angular.y = 0.0;
  my_vel.angular.z = 0.0;
  
  ROS_INFO("Publishing velocity %s.", vel.toString().c_str());
  
  my_vel_pub.publish(my_vel);
}

void UALStateCallback(const QuadStateEstimationWithCovarianceStamped::ConstPtr& state) {
  curr_state = *state.get();
  if (!received_first_state) {
    ROS_INFO("Received first state of the UAV.");
    received_first_state = true;
    first_state = *state.get();
  }
}

void waypointListCallback(const PathWithCruiseStamped_< std::allocator< void > >::ConstPtr& plan)
{
  if (!plan_received_and_being_executed && plan.get()->path_with_cruise.waypoints.size() > 0) {
    delete follower;
    plan_received_and_being_executed = true;
    current_way = 0;
    curr_goal.goal = *plan.get();
    curr_goal.id = n_received;
    
    if (curr_goal.goal.path_with_cruise.waypoints.size() > 100) {
      follower = new ContinuousTrajectoryFollower(curr_goal.goal.path_with_cruise, min_dist);
      ROS_INFO("Created a ContinuousTrajectoryFollower.");
    } else {
      follower = new DiscreteTrajectoryFollower(curr_goal.goal.path_with_cruise, min_dist);
      ROS_INFO("Created a DiscreteTrajectoryFollower.");
    }
    
    ROS_INFO("Plan received. Number of waypoints: %lu", curr_goal.goal.path_with_cruise.waypoints.size());
    takeoff();
  } else {
    if (plan_received_and_being_executed) {
      ROS_ERROR("Plan discarded, because of a task was being already processed.");
    } else {
      ROS_ERROR("Plan discarded: no waypoints inside.");
    }
  }
  n_received++;
}

void takeoff() {
  while (!received_first_state) {
    sleep(1);
  }
  
  std::stringstream idAction;
  idAction<< "/ual_" << uav_id << "/take_off";
  cTakeOff = new actionlib::SimpleActionClient<TakeOffAction>(idAction.str(), true);
  cTakeOff->waitForServer();
  TakeOffGoal tOff_goal;
  cTakeOff->sendGoal(tOff_goal);
  sleep(1);
  
  
}

void emergencyStopCallback(const std_msgs::Bool_< std::allocator< void > >::ConstPtr& stop)
{
  emergency_stop = stop.get()->data != 0;
  ROS_ERROR("Emergency stop received in ual %d!\n", uav_id);
  ROS_INFO("Emergency stop received in ual %d!\n", uav_id);
}
