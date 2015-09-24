/*!
 * \file
 * \brief The main class for the aal extension action.
 */
#ifndef AAL_EXTENSION_ACTION_H
#define AAL_EXTENSION_ACTION_H

#include<ros/ros.h>
#include<actionlib/server/action_server.h>
#include<arcas_actions_msgs/AALExtensionActionAction.h>
#include <arcas_msgs/ArmStateEstimation.h>
#include<sensor_msgs/JointState.h>

/*!
 * \brief ANGLE CONTRACTING 1 JOINT
 */
#define CONTRACTING_JOINT_1 0

/*!
 * \brief ANGLE CONTRACTING 2 JOINT
 */
#define CONTRACTING_JOINT_2 0

/*!
 * \brief ANGLE CONTRACTING 3 JOINT
 */
#define CONTRACTING_JOINT_3 0

/*!
 * \brief ANGLE CONTRACTING 4 JOINT
 */
#define CONTRACTING_JOINT_4 0

/*!
 * \brief ANGLE CONTRACTING 5 JOINT
 */
#define CONTRACTING_JOINT_5 0

/*!
 * \brief ANGLE CONTRACTING 6 JOINT
 */
#define CONTRACTING_JOINT_6 0

/*!
 * \brief ANGLE CONTRACTING CLAW JOINT
 */
#define CONTRACTING_JOINT_CLAW 0

/*!
 * \brief ANGLE EXTENDING 1 JOINT
 */
#define EXTENDING_JOINT_1 0

/*!
 * \brief ANGLE EXTENDING 2 JOINT
 */
#define EXTENDING_JOINT_2 -3.1415

/*!
 * \brief ANGLE EXTENDING 3 JOINT
 */
#define EXTENDING_JOINT_3 3.1415

/*!
 * \brief ANGLE EXTENDING 4 JOINT
 */
#define EXTENDING_JOINT_4 0

/*!
 * \brief ANGLE EXTENDING 5 JOINT
 */
#define EXTENDING_JOINT_5 0

/*!
 * \brief ANGLE EXTENDING 6 JOINT
 */
#define EXTENDING_JOINT_6 0

/*!
 * \brief ANGLE EXTENDING CLAW JOINT
 */
#define EXTENDING_JOINT_CLAW 0

/*!
 * \brief MIN DISTANCE ANGLE.
 */
#define EQUAL_ANGLE 0.02

using namespace std;
using namespace sensor_msgs;

/*!
 * \brief The main class for the aal extension action.
 */
class AALExtensionAction
{
public:
	AALExtensionAction(ros::NodeHandle n);
	~AALExtensionAction();


	/*!
	 * \brief Get the state of the arm
	 * \return true if arm is extended or false if it is contracted.
	 */
	bool isExtended();

	/*!
	 * \brief Get angles to send to arm
	 * \return vector including angles to extend or contract the arm in right order.
	 */
	vector<double> getExtensionAngles();


	/*!
	 * \brief Action need to know the states of the joints
	 * param states of the joints
	 */
	void updateJointState(std::map<std::string,JointState> *last_joint_states,
						  string *joint_names);

	/// For contorl wich joint is moving now
	enum Phase
	{
		JOINT_1 = 1,
		JOINT_2 = 2,
		JOINT_3 = 3,
		JOINT_4 = 4,
		JOINT_5 = 5,
		JOINT_6 = 6,
		CLAW    = 7,
		FINISHED
	};

	/// To control the state of the arm.
	enum State
	{
		EXTENDED= arcas_msgs::ArmStateEstimation::EXTENDED,
		CONTRACTED= arcas_msgs::ArmStateEstimation::CONTRACTED,
		EXTENDING = arcas_msgs::ArmStateEstimation::EXTENDING,
		CONTRACTING = arcas_msgs::ArmStateEstimation::CONTRACTING
	};

	/// To send information about the aborted state
	enum AbortedState
	{
		CORRECTLY_DONE,
		ALREADY_CONTRACTED,
		ALREADY_EXTENDED,
		IMPOSIBLE
	};

	/*!
	 * \brief Update state to send to the arm
	 *
	 */
	void  updateState(unsigned char extended,unsigned char contracted);

	/*!
	 * \brief Update state to send to the arm
	 *
	 */
	void  updateCommands(unsigned char &extending,unsigned char &contracting);

private:

	/*!
	 * \brief Callback to receive new goals from action client.
	 * \param goal handle of the new goal.
	 */
	void goalCB(
			actionlib::ServerGoalHandle<arcas_actions_msgs::AALExtensionActionAction> goal_handle);

	/*!
	 * \brief Callback to receive cancel calls from action client
	 * \param goal handle of the goal.
	 */
	void preemptCB(
			actionlib::ServerGoalHandle<arcas_actions_msgs::AALExtensionActionAction> goal_handle);

	/*!
	 * \brief Calculate the next phase.
	 * \return the next phase value.
	 */
	Phase nextPhase();

	/*!
	 * \brief Update the action, actual joints angles and next state
	 * \return the next phase value.
	 */
	void  mainLoop(const ros::TimerEvent& te);

	/*!
	 * \brief Update angles to send to the arm
	 *
	 */
	void  updateAngles();


	/*!
	 * \brief Send done message to action client
	 * \param state in wich the action has done.
	 */
	void  sendDoneMessage(AbortedState st);

	/*!
	 * \brief Control if have an active goal, we have to know if is posible to accept new goals.
	 */
	bool hasGoal;

	/*!
	 * \brief Hold current phase of the extension/contraction task
	 */
	Phase current_phase;

	/*!
	 * \brief Hold current phase of the extension/contraction task
	 */
	ros::Time received_goal;

	/*!
	 * \brief Hold current state of the arm.
	 */
	State current_state;

	/*!
	 * \brief Ros timer for run main loop at fixed step.
	 */
	ros::Timer main_loop_timer;

	/*!
	 * \brief Ros action server
	 */
	actionlib::ActionServer<arcas_actions_msgs::AALExtensionActionAction> as_;

	/*!
	 * \brief Goal handle of the current active goal.
	 */
	actionlib::ServerGoalHandle<arcas_actions_msgs::AALExtensionActionAction> goal_handle_;

	/*!
	 * \brief Hold last received joints state.
	 */
	std::map<std::string,JointState> *last_joint_states_;

	/*!
	 * \brief Hold the name of each joint.
	 */
	std::string *joint_names_;

	/*!
	 * \brief Has angles of each joint
	 */
	vector<double> next_contracting_angles,next_extending_angles, next_angles;

	/*!
	 * \brief Result msg to send to action client.
	 */
	arcas_actions_msgs::AALExtensionActionResult result_msg;

	/*!
	 * \brief Feedback msg to send to action client.
	 */
	arcas_actions_msgs::AALExtensionActionFeedback feedback_msg;

};

#endif // AAL_EXTENSION_ACTION_H
