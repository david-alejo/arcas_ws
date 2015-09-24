/*************************************************************************
 *
 * FADA-CATEC
 * __________________
 *
 *  [2013] FADA-CATEC
 *  All Rights Reserved.
 *
 * NOTICE:  All information contained herein is, and remains
 * the property of FADA-CATEC and its suppliers,
 * if any.  The intellectual and technical concepts contained
 * herein are proprietary to FADA-CATEC
 * and its suppliers and may be covered by Europe and Foreign Patents,
 * patents in process, and are protected by trade secret or copyright law.
 * Dissemination of this information or reproduction of this material
 * is strictly forbidden unless prior written permission is obtained
 * from FADA-CATEC.
 *
 * Created on: 20-Mar-2013
 * Engineer: Jonathan Ruiz Páez
 * Email: jruiz@catec.aero
 */

#ifndef __BONE_BRAKER_ARM_CONTROLLER_H__
#define __BONE_BRAKER_ARM_CONTROLLER_H__

#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "arcas_msgs/JointControl.h"
#include "arcas_msgs/JointParameters.h"

#include "bonebraker_arm_controller/joint_control.h"

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

#define SEND_RATE 20

using namespace BoneBraker;

namespace gazebo
{
  class BoneBrakerArmController : public ModelPlugin
  {
public: 
	  BoneBrakerArmController();
  virtual ~BoneBrakerArmController();

protected:  
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Update();
  virtual void Reset();  

private: 

  /// \brief The model refered to by this plugin
  physics::ModelPtr model_;
  

  ros::NodeHandle* node_handle_;
  ros::CallbackQueue callback_queue_commands_;
  ros::CallbackQueue callback_queue_parameters_;
  ros::Subscriber joints_commands_subscriber_;
  ros::Subscriber joints_parameters_subscriber_;
  ros::Publisher joints_publisher_;
  common::Time last_send_time_, min_time_between_send_;

  
  std::string namespace_;
  std::string joints_topic_;
  
  std::vector<JointControl*> joints;

  void JointsCommandsCallback(const arcas_msgs::JointControlConstPtr&);
  void JointsParametersCallback(const arcas_msgs::JointParametersConstPtr&);
  

  /// \brief Pointer to the update event connection
  event::ConnectionPtr updateConnection_;
  
  };
}
#endif
