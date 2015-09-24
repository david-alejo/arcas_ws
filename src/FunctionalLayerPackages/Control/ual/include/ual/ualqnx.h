/*!
 * \file
 * \brief Main class for ual gazebo with simulink model version.
 *  This class contains all functionality to communicate ros with Gazebo with simulink simulator.
 */
#ifndef UALSIMULINKGAZEBO_H
#define UALSIMULINKGAZEBO_H
#include<ual/common.h>
#include<ual/roscommunication.h>
#include<ual/qnxcommunication.h>
#include<ual/actions/land_action.h>
#include<ual/actions/take_off_action.h>

/*!
 * \brief Update rate of ual node
 */
#define UPDATE_RATE 100.0


/*!
 * \brief Ual Simulink Class to communicate simulator with ros.
 */
class UALQnx
{
public:
	UALQnx(ros::NodeHandle *n, int uavId, std::string qnx_host, unsigned int statePort, unsigned int commandPort);

private:

	/*!
	 * \brief Update loop at fixed step
	 * \param Timer event message from ros call.
	 */
	void updateLoop(const ros::TimerEvent& te);

	/*!
	 * \brief timer for main loop
	 */
	ros::Timer update_timer_;

	/*!
	 * \brief Ros communication object; Implements standart interface from/to ROS.
	 */
	RosCommunication generalCommunications;

	/*!
	 * \brief Simulink Gazebo Communication object; Implements standart interface from/to gazebo/simulink quad rotor simulator.
	 */
	QnxCommunication qnxCommunications;

	/*!
	 * \brief Contains actual state of the uav
	 */
	StateUAV actual_state;

	/*!
	 * \brief Land action server
	 */
	LandActionClass landActionServer;

	/*!
	 * \brief TakeOff Action Server
	 */
	TakeOffActionClass takeoffActionServer;
};

#endif // UALSIMULINKGAZEBO_H
