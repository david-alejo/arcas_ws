//=================================================================================================
// Copyright (c) 2012, Jonathan Ruiz Páez, FADA-CATEC
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef BONEBRAKER_CONTROLLER_H
#define BONEBRAKER_CONTROLLER_H

#include <gazebo/common/Plugin.hh>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <arcas_msgs/QuadControlReferencesStamped.h>
#include <arcas_msgs/ArmControlReferencesStamped.h>
#include <arcas_msgs/JointControl.h>
#include <arcas_msgs/Joystick.h>
#include <sensor_msgs/JointState.h>

#include <hector_gazebo_plugins/update_timer.h>

//MATLAB INCLUDE

extern "C"
{
 #include "quadrotor_controller.h"      /* Model's header file */
 #include "rtwtypes.h"                  /* MathWorks types */
 #include "ext_work.h"                  /* External mode header file */
}

namespace gazebo
{

class BonebrakerController : public ModelPlugin
{
public:
  BonebrakerController();
  virtual ~BonebrakerController();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Update();
  virtual void Reset();



private:

  void MatlabUpdate();
  /// \brief The parent World
  physics::WorldPtr world;

  /// \brief The link referred to by this plugin
  physics::LinkPtr link;

  ros::NodeHandle* node_handle_;
  ros::CallbackQueue callback_queue_;
  ros::Subscriber control_ref_rw_subscriber_;
  ros::Subscriber arm_control_ref_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber state_subscriber_;
  ros::Subscriber joystick_subscriber_;
  ros::Subscriber joint_state_subscriber_;
  ros::Publisher  joint_position_publisher_;

  // void CallbackQueueThread();
  // boost::mutex lock_;
  // boost::thread callback_queue_thread_;

  arcas_msgs::QuadControlReferencesStamped control_ref_rw_;
  arcas_msgs::ArmControlReferencesStamped arm_control_ref_;
  arcas_msgs::JointControl joint_position_to_send_;
  arcas_msgs::Joystick joystick_;
  sensor_msgs::JointState joint_state_;
  void QuadControlRefRW_Callback(const arcas_msgs::QuadControlReferencesStampedConstPtr &cr);
  void ArmControlRef_Callback(const arcas_msgs::ArmControlReferencesStampedConstPtr &acr);
  void ImuCallback(const sensor_msgs::ImuConstPtr&);
  void StateCallback(const nav_msgs::OdometryConstPtr&);
  void Joystick_Callback(const arcas_msgs::JoystickConstPtr&);
  void JointsState_Callback(const sensor_msgs::JointStateConstPtr &);

  ros::Time state_stamp;
  math::Pose pose;
  math::Vector3 euler, velocity, acceleration, angular_velocity;

  std::string link_name_;
  std::string namespace_;
  std::string control_ref_rw_topic_;
  std::string arm_control_ref_topic_;
  std::string imu_topic_;
  std::string state_topic_;
  std::string joystick_topic_;
  std::string joint_position_publisher_topic_;
  std::string joint_state_subscriber_topic_;
  double max_force_;


  math::Vector3 inertia;
  double mass;

  math::Vector3 force, torque;

  UpdateTimer controlTimer;
  event::ConnectionPtr updateConnection;

  //Arm contorl Only publish data if output is different
  std::string joint_names_[9];
  double joint_positions_[8]; //joints,claw,battery position
  double joint_positions_state_[8]; //joints,claw,battery position
  bool arm_is_extended_;
};

}

#endif // BONEBRAKER_CONTROLLER_H
