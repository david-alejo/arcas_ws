/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2013  sinosuke <email>

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


#ifndef ROSORCASIMULATOR_H
#define ROSORCASIMULATOR_H

#include "RVO2-3D/RVOSimulator.h"
#include "RVO2-3D_unstable/RVOSimulator.h"
#include <functions/RealVector.h>
#include <sparser/all.h>

#ifndef PI
#define PI 3.14159265359
#endif

// ROS stuff ---------------
#include <ros/ros.h>

// Dynamic configuration stuff
#include <orca_module/activeConfig.h>
#include <dynamic_reconfigure/server.h>

// Messages
#include <geometry_msgs/Twist.h>
#include <arcas_msgs/Position.h>
#include <arcas_msgs/QuadStateEstimationWithCovarianceStamped.h>
#include <std_msgs/Bool.h>

// End of ROS stuff

#include "Maneuver.h"

//! @class ORCAModule
//! @brief This class receives a preferred velocity and transmits a new velocity in the hector quadrrotor simulator format. 
//! In order to make calculation, it takes into account the position of surrounding robots

struct ORCAConfig {
  float timeStep;
  float neighborDist;
  float obstacleDist;
  size_t maxNeighbors;
  float timeHorizon;
  float radius;
  float radius_z; // The z radius can be different so different horizontal and vertical separations can be configured
  float radius_obstacle;
  float radius_obstacle_z;
  float radius_warning;
  float maxSpeed;
  float distObstacle;
  float timeObstacle;
  float collision_multiplier;
  bool make_convex;
  float a_max;
  float frozen_mult;
  float z_mult;
  float exponent, exponent_z;
  bool propagate_commands;
  
  Checker *getChecker() const;
  
  void fromBlock(ParseBlock& data);
};

class ORCAModule
{
public:
    ORCAModule(unsigned int id, std::string& filename);
    
    void mainLoop();
    
    ~ORCAModule();
    
    inline void stop() { end_ = true; }
    
    inline uint getID() const { return id; } 
    
    int startROSComms();
    
    //! @brief Gets a mesh and adds obstacles to the detector :)
    bool addObstacles(const std::string &filename);
    
protected:
    RVO::RVOSimulator *sim;
    RVO_UNSTABLE::RVOSimulator *sim_unst;
    uint n_uavs;
    uint id;
    uint filter_size;
    std::vector<std::list<functions::Point3D> > velocity_history;
    std::vector<std::list<functions::Point3D>::iterator> last_velocity;
    std::string st_id;
    std::string prefix;
    std::vector<ros::Subscriber> state_subscribers;
    std::vector<ros::Subscriber> pref_vel_subscribers;
    ros::Subscriber emergency_subscriber;
    
    ros::Publisher control_reference_pub;
    std::vector<RVO::Vector3> pos; // Position of the robots in the system
    std::vector<RVO::Vector3> vel; // Velocities of the robots in the system
    std::vector<RVO_UNSTABLE::Vector3> pos_unst; // Position of the robots in the system
    std::vector<RVO_UNSTABLE::Vector3> vel_unst; // Velocities of the robots in the system
    std::vector<functions::Point3D> pos_3d;
    std::string data_type;
    bool unstable; // If true --> uses then unstable RVO-3D library with static obstacles
    
    // Log files
    std::string scenario_file;
    std::string distance_file; // Saves the distance to the obstacles
    std::string preferred_file; // Saves the distance to the obstacles
    std::string orca_file; // Saves the distance to the obstacles
    std::vector<int> remap;
    std::vector<double> min_separation_z; // Saves the separation between the other UAVs in the system and with the static obstacles
    std::vector<double> min_separation_xy; // Saves the separation between the other UAVs in the system and with the static obstacles
    std::vector<Maneuver> maneuver_log;
    std::string file_sep, file_maneuvers;
    std::string time_file;
    // Status flag
    bool end_, started, loaded;

    // Logging
    std::vector<bool> state_received;
    
    //! Checker to use sparser with the Particle data
    virtual Checker *getChecker() const;

    // ROS communication related functions
    void publish_orca_vel(RVO::Vector3 velocity_);
    void sendControlReferences(const functions::Point3D new_vel);
    void receivePreferredVelocity(uint uav, const geometry_msgs::Twist& pref_vel);
    void receiveARCAS(uint uav, const arcas_msgs::QuadStateEstimationWithCovarianceStamped &msg);
    void receiveActive(orca_module::activeConfig &config, uint32_t level);
    void receiveEmergency(const std_msgs::Bool& b);
    void fromBlock(const string& filename);
    bool saveLog() const;
    void log(); // Makes the log of distance, etc.
    void filterVelocities();
    void filterVelocity(functions::Point3D &vel, unsigned int uav_id);
    
    
    string v_pref_topic;
    string publish_topic;
    string state_topic;
    string state_topic_others;
    ORCAConfig conf;
    orca_module::activeConfig dynamic_conf;
    std::vector<functions::Point3D> last_vel;
    bool transmit_preferred;
    bool weighted_filter;
    double heading;
    bool emergency_stop; 
    double pure_delay;
};

#endif //ROSORCASIMULATOR_H
