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
 * Created on: 23-Oct-2012
 * Engineer: Jonathan Ruiz Páez
 * Email: jruiz@catec.aero
 */
#include <actionlib/server/action_server.h>
#include <arcas_actions_msgs/TakeOffAction.h>
class TakeOffActionClass
{

public:
	TakeOffActionClass(std::string name, std::string uavID) :
		as_(nh_,std::string("ual_") + uavID + std::string("/") + name,false),
		action_name_(std::string("ual_") + uavID + std::string("/") + name)
	{
		hasGoal =false;
		last_state = LANDED;
		phase = arcas_actions_msgs::TakeOffFeedback::STOPPED;

		as_.registerGoalCallback(boost::bind(&TakeOffActionClass::goalCB, this, _1));
		as_.registerCancelCallback(boost::bind(&TakeOffActionClass::preemptCB, this, _1));
		uavID_ = uavID;
		as_.start();
	}

	~TakeOffActionClass()
	{

	}
	bool hasReceivedTakeoffAction()
	{
		return hasGoal;
	}
	void uavUpdateState(StateUAV actual_state)
	{
		//Receive state of uav
		last_state = actual_state;

		if(hasGoal)
		{
			if(actual_state==FLYING)
			{
				hasGoal = false;

				arcas_actions_msgs::TakeOffResult res;
				res.result = arcas_actions_msgs::TakeOffResult::FLYING;

				goal_handle_.setSucceeded(res);

			}
		}
	}

private:
	void goalCB(actionlib::ServerGoalHandle<arcas_actions_msgs::TakeOffAction> goal_handle)
	{
		arcas_actions_msgs::TakeOffResult res;

		if(last_state==FLYING)
		{
			res.result = arcas_actions_msgs::TakeOffResult::ALREADY_FLYING;
			goal_handle.setRejected();
		}else if(last_state==TAKING_OFF)
		{
			//Ignore...
		}else if(last_state==LANDING)
		{
			res.result = arcas_actions_msgs::TakeOffResult::LANDING;
			goal_handle.setRejected();
		}else
		{
			goal_handle_ = goal_handle;
			hasGoal = true;
			goal_handle.setAccepted();
		}

	}
	void preemptCB(actionlib::ServerGoalHandle<arcas_actions_msgs::TakeOffAction> goal_handle)
	{
		//Try to cancel callback
		//Ignore, actually cant cancel take off action.
	}

protected:

	ros::NodeHandle nh_;
	actionlib::ActionServer<arcas_actions_msgs::TakeOffAction> as_;
	actionlib::ServerGoalHandle<arcas_actions_msgs::TakeOffAction> goal_handle_;
	bool hasGoal;
	std::string action_name_;
	arcas_actions_msgs::TakeOffFeedback feedback_;
	arcas_actions_msgs::TakeOffResult result_;
	StateUAV last_state;
	unsigned char phase;
	std::string uavID_;
};
