/*!
 * \file
 * \brief The Arm abstraction layer base class.
 */
#ifndef ARM_ABSTRACTION_LAYER_H_
#define ARM_ABSTRACTION_LAYER_H_

#include <ros/ros.h>
#include<arcas_msgs/ArmControlReferencesStamped.h>
#include<arcas_msgs/ArmStateEstimationStamped.h>
#include<arcas_msgs/JointControl.h>
#include<arcas_msgs/AALStateStamped.h>
#include<arcas_msgs/AALControlReferenceStamped.h>
#include<sensor_msgs/JointState.h>


/*!
 * \brief Value for position calculation of the arm
 */
#define L2 0.0070

/*!
 * \brief Value for position calculation of the arm
 */
#define L3 0.2468

/*!
 * \brief Value for position calculation of the arm
 */
#define L4 0.1105

/*!
 * \brief Value for position calculation of the arm
 */
#define L6 0.1113

//using namespace arcas_msgs;
using namespace sensor_msgs;

/*!
 * \brief The base class for arm abstraction layer
 */
class ArmAbstractionLayer
{
public:
	ArmAbstractionLayer(ros::NodeHandle n)
	{
		//joints names
		joint_names_[0]="shoulder_y_arm_joint";
		joint_names_[1]="shoulder_p_arm_joint";
		joint_names_[2]="elbow_0_p_arm_joint";
		joint_names_[3]="elbow_0_r_arm_joint";
		joint_names_[4]="wirst_0_p_arm_joint";
		joint_names_[5]="wirst_1_r_arm_joint";
		joint_names_[6]="claw_0_y_arm_joint";
		joint_names_[7]="claw_1_y_arm_joint";


		//Receive commands
		joint_command_sub = n.subscribe("arm_control_references", 0,										&ArmAbstractionLayer::jointCommandCallBack,
										this);

		aal_state_pub =
				n.advertise<arcas_msgs::ArmStateEstimationStamped>("arm_state_estimation",0);

		//send periodically data to arm controller
		send_loop = n.createTimer(ros::Duration(1/50.0),
								  &ArmAbstractionLayer::main_loop,
								  this);

		send_loop.start();
	}

   ~ArmAbstractionLayer()
	{
		joint_command_sub.shutdown();
		send_loop.stop();
	}

private:

	/*!
	 * \brief Receive commands to control the arm from ROS
	 * param states of the joints
	 */
	void jointCommandCallBack(const arcas_msgs::ArmControlReferencesStampedConstPtr &control)
	{
		this->last_control_command_ = *control;
	}

	/*!
	 * \brief Main loop to run at fixed step.
	 * param ros timerevent
	 */
	void main_loop(const ros::TimerEvent& te)
	{
		receiveJointState();
		update();
		sendJointControl();
		aal_state_pub.publish(arm_state_estimation_to_send_);

	}

protected:

	/*!
	 * \brief The child class have to implement the protocol to receive joints state
	 */
	virtual void receiveJointState()=0;


	/*!
	 * \brief The child class have to implement the update method
	 */
	virtual void update()=0;


	/*!
	 * \brief The child class have to implement the protocol to send joints control commands
	 */
	virtual void sendJointControl()=0;


	/*!
	 * \brief Hold last joints state
	 */
	std::map<std::string,JointState> last_joint_states_;


	/*!
	 * \brief have to know the name of each joint
	 */
	std::string joint_names_[8];


	/*!
	 * \brief Hold last control command receive from ros.
	 */
	arcas_msgs::ArmControlReferencesStamped last_control_command_;


	/*!
	 * \brief Message to send the state of the arm ro ros
	 */
	arcas_msgs::ArmStateEstimationStamped  arm_state_estimation_to_send_;


	/*!
	 * \brief Subscribe to control command topic
	 */
	ros::Subscriber joint_command_sub;


	/*!
	 * \brief To publish the state of the arm
	 */
	ros::Publisher  aal_state_pub;


	/*!
	 * \brief Have to send the state of the arm at fixed step
	 */
	ros::Timer send_loop;



};


#endif /* ARM_ABSTRACTION_LAYER_H_ */
