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

#include <bonebraker_controller/bonebraker_controller.h>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <cmath>

namespace gazebo {

BonebrakerController::BonebrakerController()
{
   joint_positions_[0] = 0;
   joint_positions_[1] = 0;
   joint_positions_[2] = 0;
   joint_positions_[3] = 0;
   joint_positions_[4] = 0;
   joint_positions_[5] = 0;
   joint_positions_[6] = 0;
   joint_positions_[7] = 0;
   arm_is_extended_ = false;

   joint_names_[0]="shoulder_y_arm_joint";
   joint_names_[1]="shoulder_p_arm_joint";
   joint_names_[2]="elbow_0_p_arm_joint";
   joint_names_[3]="elbow_0_r_arm_joint";
   joint_names_[4]="wirst_0_p_arm_joint";
   joint_names_[5]="wirst_1_r_arm_joint";
   joint_names_[6]="claw_0_y_arm_joint";
   joint_names_[7]="claw_1_y_arm_joint";
   joint_names_[8]="battery_joint";
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
BonebrakerController::~BonebrakerController()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection);

  //Close matlab model
  rtExtModeShutdown(1);
  quadrotor_controller_terminate();

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void BonebrakerController::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

   std::cerr << std::endl << std::endl << "Cargando plugin" << std::endl << std::endl;
  world = _model->GetWorld();

  // load parameters
  if (!_sdf->HasElement("robotNamespace") || !_sdf->GetElement("robotNamespace")->GetValue())
   namespace_.clear();
  else
   namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("controlReferencesRW") || !_sdf->GetElement("controlReferencesRW")->GetValue())
   control_ref_rw_topic_ = "/bonebraker/control_ref_gazebo";
  else
   control_ref_rw_topic_ = _sdf->GetElement("control_ref_gazebo")->Get<std::string>();


  if (!_sdf->HasElement("joystickTopic") || !_sdf->GetElement("joystickTopic")->GetValue())
   joystick_topic_ = "/bonebraker/joystick_control";
  else
   joystick_topic_ = _sdf->GetElement("joystickTopic")->Get<std::string>();

  if (!_sdf->HasElement("jointsState") || !_sdf->GetElement("jointsState")->GetValue())
   joint_state_subscriber_topic_ = "/bonebraker/joints_state";
  else
   joint_state_subscriber_topic_ = _sdf->GetElement("jointsState")->Get<std::string>();

  if (!_sdf->HasElement("armControlReference") || !_sdf->GetElement("armControlReference")->GetValue())
   arm_control_ref_topic_ = "/bonebraker/arm_control_ref_gazebo";
  else
   arm_control_ref_topic_ = _sdf->GetElement("armControlReference")->Get<std::string>();


  if (!_sdf->HasElement("jointPosition_gazebo") || !_sdf->GetElement("jointPosition_gazebo")->GetValue())
   joint_position_publisher_topic_ = "/bonebraker/set_joint_position";
  else
   joint_position_publisher_topic_ = _sdf->GetElement("jointPosition_gazebo")->Get<std::string>();

  if (!_sdf->HasElement("imuTopic") || !_sdf->GetElement("imuTopic")->GetValue())
   imu_topic_.clear();
  else
   imu_topic_ = _sdf->GetElement("imuTopic")->Get<std::string>();

  if (!_sdf->HasElement("stateTopic") || !_sdf->GetElement("stateTopic")->GetValue())
   state_topic_.clear();
  else
   state_topic_ = _sdf->GetElement("stateTopic")->Get<std::string>();

  if (!_sdf->HasElement("bodyName") || !_sdf->GetElement("bodyName")->GetValue())
  {
   link = _model->GetLink();
   link_name_ = link->GetName();
  }
  else {
   link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
   link = _model->GetLink(link_name_);
  }

  if (!link)
  {
   ROS_FATAL("gazebo_ros_baro plugin error: bodyName: %s does not exist\n", link_name_.c_str());
   return;
  }

  if (!_sdf->HasElement("maxForce") || !_sdf->GetElement("maxForce")->GetValue())
   max_force_ = -1;
  else
   max_force_ = _sdf->GetElement("maxForce")->Get<double>();


  // Get inertia and mass of quadrotor body
  inertia = link->GetInertial()->GetPrincipalMoments();
  mass = link->GetInertial()->GetMass();

  node_handle_ = new ros::NodeHandle(namespace_);

  // joystick command
  if (!joystick_topic_.empty())
  {
   ros::SubscribeOptions ops = ros::SubscribeOptions::create<arcas_msgs::Joystick>(
     joystick_topic_, 1,
     boost::bind(&BonebrakerController::Joystick_Callback, this, _1),
     ros::VoidPtr(), &callback_queue_);
   joystick_subscriber_ = node_handle_->subscribe(ops);
  }

  // joints State
  if (!joint_state_subscriber_topic_.empty())
  {
   ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::JointState>(
     joint_state_subscriber_topic_, 15,
     boost::bind(&BonebrakerController::JointsState_Callback, this, _1),
     ros::VoidPtr(), &callback_queue_);
   joint_state_subscriber_ = node_handle_->subscribe(ops);
  }

  // subscribe command
  if (!control_ref_rw_topic_.empty())
  {
   ros::SubscribeOptions ops = ros::SubscribeOptions::create<arcas_msgs::QuadControlReferencesStamped>(
     control_ref_rw_topic_, 1,
     boost::bind(&BonebrakerController::QuadControlRefRW_Callback, this, _1),
     ros::VoidPtr(), &callback_queue_);
   control_ref_rw_subscriber_ = node_handle_->subscribe(ops);
  }
  // subscribe command
  if (!arm_control_ref_topic_.empty())
  {
   ros::SubscribeOptions ops = ros::SubscribeOptions::create<arcas_msgs::ArmControlReferencesStamped>(
     arm_control_ref_topic_, 1,
     boost::bind(&BonebrakerController::ArmControlRef_Callback, this, _1),
     ros::VoidPtr(), &callback_queue_);
   arm_control_ref_subscriber_ = node_handle_->subscribe(ops);
  }


  // subscribe imu
  if (!imu_topic_.empty())
  {
   ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::Imu>(
     imu_topic_, 1,
     boost::bind(&BonebrakerController::ImuCallback, this, _1),
     ros::VoidPtr(), &callback_queue_);
   imu_subscriber_ = node_handle_->subscribe(ops);

   ROS_INFO_NAMED("quadrotor_simple_controller", "Using imu information on topic %s as source of orientation and angular velocity.", imu_topic_.c_str());
  }

  // subscribe state
  if (!state_topic_.empty())
  {
   ros::SubscribeOptions ops = ros::SubscribeOptions::create<nav_msgs::Odometry>(
     state_topic_, 1,
     boost::bind(&BonebrakerController::StateCallback, this, _1),
     ros::VoidPtr(), &callback_queue_);
   state_subscriber_ = node_handle_->subscribe(ops);

   ROS_INFO_NAMED("quadrotor_simple_controller", "Using state information on topic %s as source of state information.", state_topic_.c_str());
  }



  if(!joint_position_publisher_topic_.empty())
  {
     joint_position_publisher_ = node_handle_->advertise<arcas_msgs::JointControl>(joint_position_publisher_topic_,0);
  }

  // callback_queue_thread_ = boost::thread( boost::bind( &GazeboQuadrotorSimpleController::CallbackQueueThread,this ) );


  Reset();

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  controlTimer.Load(world, _sdf);
  updateConnection = event::Events::ConnectWorldUpdateBegin(
     boost::bind(&BonebrakerController::Update, this));


  //Initialize matlab model
   /* External mode */
   char *argv[3];
   char arg0[32],arg1[32],arg2[32];

   argv[0] = arg0;
   argv[1] = arg1;
   argv[2] = arg2;

   sprintf(argv[0],"exec");

   std::cerr << std::endl << std::endl << "Initialize Ext Mode" << std::endl << std::endl;
   rtERTExtModeParseArgs(1, (const char **)argv);
   /* Initialize model */
   std::cerr << std::endl << std::endl << "Initialize Matlab Model" << std::endl << std::endl;
   quadrotor_controller_initialize();

      std::cerr << std::endl << std::endl << "Matlab Initicializado" << std::endl << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
// Callbacks
void BonebrakerController::QuadControlRefRW_Callback(const arcas_msgs::QuadControlReferencesStampedConstPtr &cr)
{
   control_ref_rw_= *cr;
}

void BonebrakerController::ArmControlRef_Callback(const arcas_msgs::ArmControlReferencesStampedConstPtr &acr)
{
   arm_control_ref_ = *acr;
}

void BonebrakerController::Joystick_Callback(const arcas_msgs::JoystickConstPtr &jsk)
{
   joystick_ = *jsk;
}
void BonebrakerController::JointsState_Callback(const sensor_msgs::JointStateConstPtr &jointState)
{
   for(int i=0;i < 8; i++)
   {
      if(joint_names_[i].compare(jointState->name[0])==0)
      {

         joint_positions_state_[i] = jointState->position[0];
         // std::cerr << joint_names_[i]<< ": " << joint_positions_state_[i] << std::endl;
         break;
      }else
      {
         // std::cerr << joint_names_[i]<< ": " << jointState->name[0] << std::endl;

      }
   }
}

void BonebrakerController::ImuCallback(const sensor_msgs::ImuConstPtr& imu)
{
  pose.rot.Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
  euler = pose.rot.GetAsEuler();
  angular_velocity = pose.rot.RotateVector(math::Vector3(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z));
}

void BonebrakerController::StateCallback(const nav_msgs::OdometryConstPtr& state)
{
  math::Vector3 velocity1(velocity);

  if (imu_topic_.empty()) {
   pose.pos.Set(state->pose.pose.position.x, state->pose.pose.position.y, state->pose.pose.position.z);
   pose.rot.Set(state->pose.pose.orientation.w, state->pose.pose.orientation.x, state->pose.pose.orientation.y, state->pose.pose.orientation.z);
   euler = pose.rot.GetAsEuler();
   angular_velocity.Set(state->twist.twist.angular.x, state->twist.twist.angular.y, state->twist.twist.angular.z);
  }

  velocity.Set(state->twist.twist.linear.x, state->twist.twist.linear.y, state->twist.twist.linear.z);

  // calculate acceleration
  double dt = !state_stamp.isZero() ? (state->header.stamp - state_stamp).toSec() : 0.0;
  state_stamp = state->header.stamp;
  if (dt > 0.0) {
   acceleration = (velocity - velocity1) / dt;
  } else {
   acceleration.Set();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void BonebrakerController::Update()
{
  // Get new commands/state
  callback_queue_.callAvailable();

  double dt;
  if (controlTimer.update(dt) && dt > 0.0) {
   // Get Pose/Orientation from Gazebo (if no state subscriber is active)
   if (imu_topic_.empty()) {
     pose = link->GetWorldPose();

     angular_velocity = link->GetWorldAngularVel();
     euler = pose.rot.GetAsEuler();
   }
   if (state_topic_.empty()) {
     acceleration = (link->GetWorldLinearVel() - velocity) / dt;
     velocity = link->GetWorldLinearVel();
   }

   // Get gravity
//	math::Vector3 gravity_body = pose.rot.RotateVector(world->GetPhysicsEngine()->GetGravity());
//	double gravity = gravity_body.GetLength();
   //double load_factor = gravity * gravity / world->GetPhysicsEngine()->GetGravity().GetDotProd(gravity_body);  // Get gravity

   // Rotate vectors to coordinate frames relevant for control
   math::Quaternion heading_quaternion(cos(euler.z/2),0,0,sin(euler.z/2));
   math::Vector3 velocity_xy = heading_quaternion.RotateVectorReverse(velocity);
   //math::Vector3 acceleration_xy = heading_quaternion.RotateVectorReverse(acceleration);
   math::Vector3 angular_velocity_body = pose.rot.RotateVectorReverse(angular_velocity);


   //Init matlab inputs

   //Commands

   quadrotor_controller_U.PositionCommand[0] = control_ref_rw_.quad_control_references.position_ref.x;
   quadrotor_controller_U.PositionCommand[1] = control_ref_rw_.quad_control_references.position_ref.y;
   quadrotor_controller_U.PositionCommand[2] = control_ref_rw_.quad_control_references.position_ref.z;
   quadrotor_controller_U.YawCommand = control_ref_rw_.quad_control_references.heading;
   quadrotor_controller_U.VelocityCommand = control_ref_rw_.quad_control_references.velocity_ref;

   if(arm_control_ref_.arm_control_references.position_ref.size()>6)
   {
      quadrotor_controller_U.ARMAnglesIN[0] = arm_control_ref_.arm_control_references.position_ref[0];
      quadrotor_controller_U.ARMAnglesIN[1] = arm_control_ref_.arm_control_references.position_ref[1];
      quadrotor_controller_U.ARMAnglesIN[2] = arm_control_ref_.arm_control_references.position_ref[2];
      quadrotor_controller_U.ARMAnglesIN[3] = arm_control_ref_.arm_control_references.position_ref[3];
      quadrotor_controller_U.ARMAnglesIN[4] = arm_control_ref_.arm_control_references.position_ref[4];
      quadrotor_controller_U.ARMAnglesIN[5] = arm_control_ref_.arm_control_references.position_ref[5];
      quadrotor_controller_U.ARMAnglesIN[6] = arm_control_ref_.arm_control_references.position_ref[6];
      quadrotor_controller_U.ARMAnglesIN[7] = arm_control_ref_.arm_control_references.battery;
   }



   quadrotor_controller_U.Joystick[0] = joystick_.thrust;
   quadrotor_controller_U.Joystick[1] = joystick_.yaw;
   quadrotor_controller_U.Joystick[2] = joystick_.pitch;
   quadrotor_controller_U.Joystick[3] = joystick_.roll;



   //State
   quadrotor_controller_U.AngularVelocity[0] = angular_velocity_body.x;
   quadrotor_controller_U.AngularVelocity[1] = angular_velocity_body.y;
   quadrotor_controller_U.AngularVelocity[2] = angular_velocity_body.z;

   quadrotor_controller_U.LinealVelocity[0] = velocity_xy.x;
   quadrotor_controller_U.LinealVelocity[1] = velocity_xy.y;
   quadrotor_controller_U.LinealVelocity[2] = velocity_xy.z;

   quadrotor_controller_U.Position[0] = link->GetWorldPose().pos.x;
   quadrotor_controller_U.Position[1] = link->GetWorldPose().pos.y;
   quadrotor_controller_U.Position[2] = link->GetWorldPose().pos.z;


   //Euler 321 for control
   quadrotor_controller_U.Attitude[0] = euler.x;//pose.rot.x;
   quadrotor_controller_U.Attitude[1] = euler.y;//pose.rot.y;
   quadrotor_controller_U.Attitude[2] = euler.z;//pose.rot.z;



      for(int i=0;i<7;i++)
      {
        //  std::cerr << "PositionJoint Despues: " << joint_positions_state_[i] << std::endl;
         quadrotor_controller_U.ARMAnglesStateIN[i] = joint_positions_state_[i];
      }


   //update model

   MatlabUpdate();

   //get matlab outputs

   force.x = quadrotor_controller_Y.Force[0];
   force.y = quadrotor_controller_Y.Force[1];
   force.z = quadrotor_controller_Y.Force[2];

   torque.x = quadrotor_controller_Y.Torque[0];
   torque.y = quadrotor_controller_Y.Torque[1];
   torque.z = quadrotor_controller_Y.Torque[2];


   //Get arm matlab outputs.
   for(int i=0;i<7;i++)
   {
      if(quadrotor_controller_Y.ARMAnglesOUT[i]!=joint_positions_[i])
      {
         joint_positions_[i] = quadrotor_controller_Y.ARMAnglesOUT[i];

         joint_position_to_send_.position = joint_positions_[i];
         joint_position_to_send_.joint_name = joint_names_[i];

         joint_position_publisher_.publish(joint_position_to_send_);
      }
   }

   if(joint_positions_[6]!=joint_positions_[7])
   {
      joint_positions_[7] = joint_positions_[6];
      joint_position_to_send_.position = joint_positions_[7];
      joint_position_to_send_.joint_name = joint_names_[7];

      joint_position_publisher_.publish(joint_position_to_send_);
   }

   if(quadrotor_controller_Y.ARMAnglesOUT[7]!=joint_positions_[8])
   {
      joint_positions_[8] = quadrotor_controller_Y.ARMAnglesOUT[7];

      joint_position_to_send_.position = joint_positions_[8];
      joint_position_to_send_.joint_name = joint_names_[8];

      joint_position_publisher_.publish(joint_position_to_send_);
   }


  }


  /*math::Pose actual_pose = link->GetWorldPose();

  actual_pose.pos.x = 0.0;
  actual_pose.pos.y = 0.0;
  actual_pose.pos.z = 1.5;
*/

  link->AddRelativeForce(force);
  link->AddRelativeTorque(torque - link->GetInertial()->GetCoG().Cross(force));

 // link->SetWorldPose(actual_pose);



}

void BonebrakerController::MatlabUpdate()
{//Matlab Autogenerated code
   static boolean_T OverrunFlag = 0;
      if (OverrunFlag) {
      rtmSetErrorStatus(quadrotor_controller_M, "Overrun");
      return;
     }
     OverrunFlag = TRUE;
     quadrotor_controller_step();
     OverrunFlag = FALSE;

     if((rtmGetErrorStatus(quadrotor_controller_M) == (NULL)) &&
           !rtmGetStopRequested(quadrotor_controller_M))
     {
         rtExtModeCheckEndTrigger();
     }else
     {
         rtExtModeShutdown(1);
     }
}

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void BonebrakerController::Reset()
{
  force.Set();
  torque.Set();

  // reset state
  pose.Reset();
  velocity.Set();
  angular_velocity.Set();
  acceleration.Set();
  euler.Set();
  state_stamp = ros::Time();
}

////////////////////////////////////////////////////////////////////////////////


// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(BonebrakerController)

} // namespace gazebo
