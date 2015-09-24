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


#include "Comms.h"

#include <fstream>
#include "functions/functions.h"

using namespace functions;
using namespace std;

Comms::Comms(int argc, char** argv):spinner(NULL)
{
  ros::init(argc,argv,"arcas_visualizer");
}

Comms::~Comms()
{
  shutdownComms();
  clearLogs();
}



void Comms::startComms(vector<uint> uavs)
{
  uavs_ = uavs;
  clearLogs();
  
  spinner = new ros::AsyncSpinner(2);
  spinner->start();
  
  init_log_time.getTime();
  
  ros::NodeHandle n;
  
  arcas_msgs::QuadStateEstimationWithCovariance pos;
  
  boost::mutex::scoped_lock lock(pos_mutex);
  boost::mutex::scoped_lock lock2(vel_mutex);
  boost::mutex::scoped_lock lock3(desired_mutex);
  
  for (unsigned int i = 0; i < uavs.size(); i++) {
    // Initialize the vectors
    current_position.push_back(pos);
    current_pos.push_back(Point3D(0.0,0.0,0.0));
    current_vel.push_back(Point3D(0.0, 0.0, 0.0));
    desired_vel.push_back(Point3D(0.0, 0.0, 0.0));
  }
    
  // Turn on the subscribers
  cout << "Uavs size: " << uavs.size() << "\t Content: " << functions::printVector(uavs) << endl;
  for (unsigned int i = 0; i < uavs.size(); i++) {
      // Start state subscribers
      ostringstream os;
      os << topic_prefix << uavs.at(i) << "/"<< state_topic;
      boost::function<void (const arcas_msgs::QuadStateEstimationWithCovarianceStamped::ConstPtr &)> f = boost::bind(&Comms::UALStateCallback, this, _1, i);
      state_subscribers.push_back(n.subscribe<arcas_msgs::QuadStateEstimationWithCovarianceStamped::ConstPtr&>(os.str(), 1, f));
      
      // Then desired velocity subscribers
      os.str(string());
      os << "orca_" << uavs.at(i) << "/" << desired_velocity_topic;
      boost::function<void (const geometry_msgs::Twist::ConstPtr &)> vel_fun = 
			      boost::bind(&Comms::DesiredVelocityCallback, this, _1, i);
      desired_vel_subscribers.push_back(n.subscribe<geometry_msgs::Twist::ConstPtr&>(os.str(), 1, vel_fun));
      
      // And the ORCA velocity subscribers
      os.str(string());
      os << "orca_" << uavs.at(i) << "/" << orca_velocity_topic;
      boost::function<void (const geometry_msgs::Twist::ConstPtr &)> orca_fun = 
			      boost::bind(&Comms::ORCAVelocityCallback, this, _1, i);
      orca_vel_subscribers.push_back(n.subscribe<geometry_msgs::Twist::ConstPtr&>(os.str(), 1, orca_fun));
      
      // Publishers
      os.str(string());
      os << topic_prefix << uavs.at(i) << "/" << emergency_stop_topic;
      emergency_publishers.push_back(n.advertise<std_msgs::Bool>(os.str(), 1));
      
      os.str(string());
      os << topic_prefix << uavs.at(i) << "/" << reference_topic;
      reference_publishers.push_back(n.advertise<arcas_msgs::QuadControlReferencesStamped>(os.str(), 1));
      
      // Land
      os.str(string());
      os << topic_prefix << uavs.at(i) << "/land";
      LandClient* myLand = new LandClient(os.str(),true);
      std::cout << "Waiting for land server: " << os.str() << std::endl;
      myLand->waitForServer();
      land_actions.push_back(myLand);

      // Takeoff
      os.str(string());
       os << topic_prefix << uavs.at(i) << "/take_off";
      TakeoffClient* myTake = new TakeoffClient(os.str(),true);
      std::cout << "Waiting for takeoff server: " << os.str() << std::endl;
      myTake->waitForServer();
      take_actions.push_back(myTake);
    }
}

void Comms::shutdownComms()
{
  delete spinner;
  spinner = NULL;
  for (unsigned int i = 0; i < state_subscribers.size(); i++) {
    state_subscribers.at(i).shutdown();
  }
  state_subscribers.clear();
  for (unsigned int i = 0; i < desired_vel_subscribers.size(); i++) {
    desired_vel_subscribers.at(i).shutdown();
  }
  desired_vel_subscribers.clear();
  for (unsigned int i = 0; i < orca_vel_subscribers.size(); i++) {
    orca_vel_subscribers.at(i).shutdown();
  }
  orca_vel_subscribers.clear();
  for (unsigned int i = 0; i < emergency_publishers.size(); i++) {
    emergency_publishers.at(i).shutdown();
    reference_publishers.at(i).shutdown();
  }
  emergency_publishers.clear();
  reference_publishers.clear();
}

void Comms::UALStateCallback(const arcas_msgs::QuadStateEstimationWithCovarianceStamped_< std::allocator< void > >::ConstPtr& s, unsigned int uav_id)
{
  functions::FormattedTime t_;
  t_.setTime(s.get()->header.stamp.sec, (long int)(s.get()->header.stamp.nsec / 1000.0));
  
  // Log the data to file
  ostringstream file;
  file << state_file << "_" << uav_id;
  std::ofstream ofs;
  const arcas_msgs::QuadStateEstimationWithCovariance &st = s.get()->quad_state_estimation_with_covariance;
  ofs.open(file.str().c_str(), std::fstream::out | std::fstream::out | std::fstream::app);
  ofs << t_ - init_log_time << " ";
  ofs << st.position.x << " " << st.position.y << " " << st.position.z << " ";
  ofs << st.attitude.roll << " " << st.attitude.pitch << " " << st.attitude.yaw << " ";
  ofs << st.linear_velocity.x << " " << st.linear_velocity.y << " " << st.linear_velocity.z << " ";
  ofs << st.angular_velocity.roll << " " << st.angular_velocity.pitch << " " << st.angular_velocity.yaw << " ";
  ofs << st.linear_acceleration.x << " " << st.linear_acceleration.y << " " << st.linear_acceleration.z << std::endl;
  
  current_position.at(uav_id) = st;
  Point3D pos(st.position.x, st.position.y, st.position.z);
  current_pos.at(uav_id) = pos;
  Point3D vel(st.linear_velocity.x, st.linear_velocity.y, st.linear_velocity.z);
  current_vel.at(uav_id) = vel;
  
  // Save the position into the current UAV position
}

void Comms::DesiredVelocityCallback(const geometry_msgs::Twist_< std::allocator< void > >::ConstPtr& s, unsigned int uav_id)
{
  boost::mutex::scoped_lock sl(desired_mutex);
  Point3D p(s->linear.x, s->linear.y, s->linear.z); 
//   desired_vel_log.at(uav_id).push_back(p);
  desired_vel.at(uav_id) = p;
}

void Comms::ORCAVelocityCallback(const geometry_msgs::Twist_< std::allocator< void > >::ConstPtr& s, unsigned int uav_id)
{
  boost::mutex::scoped_lock lock(orca_mutex);
  Point3D p(s->linear.x, s->linear.y, s->linear.z); 
//   orca_vel_log.at(uav_id).push_back(p);
  orca_vel.at(uav_id) = p;
}

void Comms::clearLogs()
{
  current_position.clear();
  current_pos.clear();
  current_vel.clear();
  desired_vel.clear();
}

void Comms::setTopics(string topic_prefix, string state_topic, string desired_velocity_topic, string orca_velocity_topic, string state_file,
  string emergency_stop, string reference)
{
  this->topic_prefix = topic_prefix;
  this->state_topic = state_topic;
  this->desired_velocity_topic = desired_velocity_topic;
  this->orca_velocity_topic = orca_velocity_topic;
  this->state_file = state_file;
  this->emergency_stop_topic = emergency_stop;
  this->reference_topic = reference;
}

Point3D Comms::getCurrentPos(unsigned int uav)
{
  boost::mutex::scoped_lock lock(pos_mutex);
  return current_pos.at(uav);
}

vector< Point3D > Comms::getCurrentPos()
{
  boost::mutex::scoped_lock lock(pos_mutex);
  return current_pos;
}

Point3D Comms::getCurrentVel(unsigned int uav)
{
  boost::mutex::scoped_lock lock(vel_mutex);
  return current_vel.at(uav);
}

vector< Point3D > Comms::getCurrentVel()
{
  boost::mutex::scoped_lock lock(vel_mutex);
  return current_vel;
}

Point3D Comms::getDesiredVel(unsigned int uav)
{
  boost::mutex::scoped_lock l(desired_mutex);
  return desired_vel.at(uav);
}

vector< Point3D > Comms::getDesiredVel()
{
  boost::mutex::scoped_lock l(desired_mutex);
  return desired_vel;
}

void Comms::emergencyStop(unsigned int i)
{
  std_msgs::Bool b;
  b.data = 1;
  emergency_publishers.at(i).publish(b);
}

void Comms::land(unsigned int i)
{
  arcas_actions_msgs::LandGoal land_goal;
  land_actions.at(i)->sendGoal(land_goal);
  land_actions.at(i)->waitForResult();
}

void Comms::takeoff(unsigned int i)
{
  // First check if we have received one valid state and make the UAV go to the state with 0.35 altitude
  functions::Point3D p = current_pos.at(i);
  arcas_msgs::QuadControlReferencesStamped ref;
  ref.quad_control_references.heading = 0.0;
  ref.quad_control_references.position_ref.x = p.x;
  ref.quad_control_references.position_ref.y = p.y;
  ref.quad_control_references.position_ref.z = 0.35;
  ref.quad_control_references.velocity_ref = 0.1;
  ref.header.stamp = ros::Time::now();
  ref.header.seq = 0;
  ref.header.frame_id = "global";
  reference_publishers.at(i).publish(ref);
  
  // Then take off.
  arcas_actions_msgs::TakeOffGoal tOff_goal;
  take_actions.at(i)->sendGoal(tOff_goal);
  take_actions.at(i)->waitForResult();
}

