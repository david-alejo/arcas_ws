/*!
 * \file
 * \brief Arm Abtraction layer simulink class.
 *  Control the arm simulated in gazebo with simulink model.
 */
#ifndef ARM_ABSTRACTION_LAYER_SIMULINK_H
#define ARM_ABSTRACTION_LAYER_SIMULINK_H
#include<aal/arm_abstraction_layer.h>
#include<aal/aal_extension_action.h>
#include <arcas_msgs/ArmControlReferencesStamped.h>
#include "brazo_ros.h"
#include "rtwtypes.h"

/*!
 * \brief Control the arm in gazebo
 */
class ArmAbstractionLayerSimulink : public ArmAbstractionLayer
{
public:
	ArmAbstractionLayerSimulink(ros::NodeHandle n);
	~ArmAbstractionLayerSimulink();

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
	 * joint states sended from gazebo
	 */
	void jointStatesCallback(const sensor_msgs::JointStateConstPtr &js);

	/*!
	 * \brief to subscribe to joint state topic
	 */
	ros::Subscriber joint_states_sub_;

	/*!
	 * \brief To publish control commands to gazebo
	 */
	ros::Publisher  arm_control_references_gazebo_pub_;

	/*!
	 * \brief Message to send to simulink model.
	 */
	arcas_msgs::ArmControlReferencesStamped arm_control_references_to_send_;

	/*!
	 * \brief Action class to control contracting/extending tasks.
	 */
	AALExtensionAction action_object_;

	/*!
	 * \brief To calculate the position of the final effector.
	 */
	void matlab_step();
};

#endif // ARM_ABSTRACTION_LAYER_SIMULINK_H
