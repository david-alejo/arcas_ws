/*!
 * \file
 * \brief Arm Abtraction layer gazebo class.
 *  Control the arm simulated in gazebo.
 */
#ifndef ARM_ABSTRACTION_LAYER_GAZEBO_H
#define ARM_ABSTRACTION_LAYER_GAZEBO_H
#include<aal/arm_abstraction_layer.h>
#include<aal/aal_extension_action.h>
#include "brazo_ros.h"
#include "rtwtypes.h"


/*!
 * \brief Control the arm in gazebo
 */
class ArmAbstractionLayerGazebo : public ArmAbstractionLayer
{
public:
	ArmAbstractionLayerGazebo(ros::NodeHandle n);
	~ArmAbstractionLayerGazebo();

private:

	/*!
	 * \brief Implement the protocol to receive joints state
	 */
	void receiveJointState();

	/*!
	 * \brief Implement the update method
	 */
	void update();

	/*!
	 * \brief Implement the protocol to send joints control commands
	 */
	void sendJointControl();

	/*!
	 * \brief Receive joint states from gazebo
	 * \param joint states sended from gazebo
	 */
	void jointStatesCallback(const sensor_msgs::JointStateConstPtr &js);

	/*!
	 * \brief to subscribe to joint state topic
	 */
	ros::Subscriber joint_states_sub_;

	/*!
	 * \brief To publish control commands to gazebo
	 */
	ros::Publisher  joint_gazebo_pub_;

	/*!
	 * \brief Hold wanted angles of each joint.
	 */
	arcas_msgs::JointControl command_to_send_[8];

	/*!
	 * \brief Action class to control contracting/extending tasks.
	 */
	AALExtensionAction action_object_;

	/*!
	 * \brief To calculate the position of the final effector.
	 */
	void matlab_step();

	void updateAALState();
};

#endif // ARM_ABSTRACTION_LAYER_GAZEBO_H
