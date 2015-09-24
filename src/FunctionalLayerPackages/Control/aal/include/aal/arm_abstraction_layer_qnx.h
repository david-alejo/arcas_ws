/*!
 * \file
 * \brief Arm Abtraction layer gazebo class.
 *  Control the arm simulated in gazebo.
 */
#ifndef ARM_ABSTRACTION_LAYER_QNX_H
#define ARM_ABSTRACTION_LAYER_QNX_H
#include<aal/arm_abstraction_layer.h>
#include<aal/aal_extension_action_qnx.h>
#include<aal/TransmisionDataType.h>
#include<aal/CUDPClient.h>


/*!
 * \brief Control the arm in gazebo
 */
class ArmAbstractionLayerQNX : public ArmAbstractionLayer
{
public:
	ArmAbstractionLayerQNX(ros::NodeHandle n,string host, int port);
	~ArmAbstractionLayerQNX();

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
	 * \brief Receive joint states from qnx
	 * \param joint states sended from gazebo
	 */
	void jointStatesCallback(const arcas_msgs::ArmStateEstimationStamped &js);

	/*!
	 * \brief to subscribe to joint state topic
	 */
	ros::Subscriber joint_states_sub_;

	/*!
	 * \brief UDP Client to send control references to
	 */
	CUDPClient client;

	/*!
	 * \brief Action class to control contracting/extending tasks.
	 */
	AALExtensionAction action_object_;

	/*!
	* \brief Data to send
	*/
	ArcasUDPControlReferences data_to_send_to_qnx;

};

#endif // ARM_ABSTRACTION_LAYER_QNX_H
