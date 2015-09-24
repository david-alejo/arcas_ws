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
#include <stdint.h>
#include "bonebraker_arm_controller/bonebraker_arm_controller.h"
#include "gazebo/common/Events.hh"
#include "gazebo/common/PID.hh"
#include "gazebo/physics/physics.hh"
namespace gazebo {

///////////////////////////////////////////////////////////////////////////////
BoneBrakerArmController::BoneBrakerArmController()
{
   node_handle_ = NULL;
   last_send_time_ = 0;
   printf("Arm controller loaded!\n");
}
BoneBrakerArmController::~BoneBrakerArmController()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);

  node_handle_->shutdown();
  delete node_handle_;
  for (unsigned int i = 1; i < joints.size(); i++)
  {
      delete joints[i];
      joints[i]=NULL;
  }
}
///////////////////////////////////////////////////////////////////////////////
// Load the controller
void BoneBrakerArmController::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model_ = _model;

  last_send_time_ = this->model_->GetWorld()->GetSimTime();
  min_time_between_send_.Set(0,1e9/ 10); //Change for send_rate

  JointControl *joint_temp;
  gazebo::physics::Joint_V jointV = _model->GetJoints();
  for(unsigned int jnt=0;jnt< _model->GetJointCount(); jnt++)
  {
     joint_temp = new JointControl(jointV[jnt], this->model_->GetWorld()->GetSimTime());
     joints.push_back(joint_temp);
  }

    node_handle_ = new ros::NodeHandle("bonebraker");

    ros::SubscribeOptions commandOptions = ros::SubscribeOptions::create<arcas_msgs::JointControl>("set_joint_position", 20, boost::bind(&BoneBrakerArmController::JointsCommandsCallback, this, _1), ros::VoidPtr(), &callback_queue_commands_);
    ros::SubscribeOptions parameterOptions = ros::SubscribeOptions::create<arcas_msgs::JointParameters>("set_joint_parameters", 1, boost::bind(&BoneBrakerArmController::JointsParametersCallback, this, _1), ros::VoidPtr(), &callback_queue_parameters_);

    this->joints_commands_subscriber_ = node_handle_->subscribe(commandOptions);
    this->joints_parameters_subscriber_ = node_handle_->subscribe(parameterOptions);

    this->joints_publisher_ = node_handle_->advertise<sensor_msgs::JointState> ("joints_state", 0);

  Reset();

  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&BoneBrakerArmController::Update, this));
}

///////////////////////////////////////////////////////////////////////////////
// Callback
void BoneBrakerArmController::JointsCommandsCallback(const arcas_msgs::JointControlConstPtr& _msg)
{
  for (unsigned int i = 0; i < joints.size(); i++)
   {
     if(_msg->joint_name == joints[i]->GetJoint()->GetName())
     {
        joints[i]->SetFinalPosition(_msg->position);
        break;
     }

   }
}

void BoneBrakerArmController::JointsParametersCallback(const arcas_msgs::JointParametersConstPtr& _msg)
{
  for (unsigned int i = 0; i < joints.size(); i++)
   {
     if(_msg->joint_name == joints[i]->GetJoint()->GetName())
     {
        joints[i]->SetPIDGains(_msg->Kp,_msg->Ki,_msg->Kd,_msg->i_max,_msg->i_min);
        joints[i]->SetMaxEffort(_msg->effort);
        joints[i]->SetVelocity(_msg->velocity);
        break;
     }
   }
}

///////////////////////////////////////////////////////////////////////////////
// Update
void BoneBrakerArmController::Update()
{

  // Get new commands/parameters
   callback_queue_commands_.callAvailable();
   callback_queue_parameters_.callAvailable();

  for (unsigned int i = 0; i < joints.size(); i++)
  {
     joints[i]->Update(this->model_->GetWorld()->GetSimTime());
  }



  if(this->model_->GetWorld()->GetSimTime() - last_send_time_ > min_time_between_send_)
  {

   //Publish joint state;
     for(unsigned int i=0;i< joints.size(); i++)
     {
        sensor_msgs::JointState joint;
        joint.name.push_back(this->joints[i]->GetJoint().get()->GetName());
        joint.effort.push_back(this->joints[i]->GetJoint().get()->GetForce(0u));
        joint.velocity.push_back(this->joints[i]->GetJoint().get()->GetVelocity(0));
        joint.position.push_back(this->joints[i]->GetJoint().get()->GetAngle(0).Radian());
        this->joints_publisher_.publish(joint);
     }
     last_send_time_ = this->model_->GetWorld()->GetSimTime();
  }


}

///////////////////////////////////////////////////////////////////////////////
// Reset the controller
void BoneBrakerArmController::Reset()
{
  for (unsigned int i = 1; i < joints.size(); i++)
   {
     joints[i]->Reset();
   }
}

GZ_REGISTER_MODEL_PLUGIN(BoneBrakerArmController)

}
