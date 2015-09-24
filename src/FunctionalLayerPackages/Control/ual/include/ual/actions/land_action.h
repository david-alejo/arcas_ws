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
#ifndef _LAND_ACTION_CLASS
#define _LAND_ACTION_CLASS


#include<ros/ros.h>
#include<actionlib/server/action_server.h>
#include<arcas_actions_msgs/LandAction.h>
#include<ual/common.h>


class LandActionClass
{

public:
	LandActionClass(std::string name, std::string uavID) :
		as_(nh_,std::string("ual_") + uavID + std::string("/") + name,false),
		action_name_(std::string("ual_") + uavID + std::string("/") + name)
	{
		hasGoal =false;
		last_state = FLYING;
		phase = arcas_actions_msgs::LandFeedback::STOPPED;

		as_.registerGoalCallback(boost::bind(&LandActionClass::goalCB, this, _1));
		as_.registerCancelCallback(boost::bind(&LandActionClass::preemptCB, this, _1));
		uavID_ = uavID;
		as_.start();

	}

	~LandActionClass()
	{

	}

	bool hasReceivedLandAction()
	{
		return hasGoal;
	}
	void uavUpdateState(StateUAV actual_state)
	{
		//Receive state of uav
		last_state = actual_state;

		if(hasGoal)
		{
			if(actual_state==LANDED)
			{
				hasGoal = false;
				arcas_actions_msgs::LandResult res;
				res.result = arcas_actions_msgs::LandResult::LANDED;
				goal_handle_.setSucceeded(res);
			}
		}
	}

private:
	void goalCB(actionlib::ServerGoalHandle<arcas_actions_msgs::LandAction> goal_handle)
	{
		arcas_actions_msgs::LandResult res;
		//TakeOff advertise;
		if(last_state==LANDED)
		{
			res.result = arcas_actions_msgs::LandResult::ALREADY_LANDED;
			goal_handle.setRejected(res);
		}else if(last_state==LANDING)
		{
			//Ignore...
		}else if(last_state==TAKING_OFF)
		{
			res.result = arcas_actions_msgs::LandResult::TAKINGOFF;
			goal_handle.setRejected(res);
		}else
		{
			goal_handle_ = goal_handle;
			hasGoal = true;
			goal_handle.setAccepted();
		}

	}
	void preemptCB(actionlib::ServerGoalHandle<arcas_actions_msgs::LandAction> goal_handle)
	{
		//Try to cancel callback
		//Ignore actually cant cancel take off action.
	}

protected:

	ros::NodeHandle nh_;
	actionlib::ActionServer<arcas_actions_msgs::LandAction> as_;
	actionlib::ServerGoalHandle<arcas_actions_msgs::LandAction> goal_handle_;
	bool hasGoal;
	std::string action_name_;
	arcas_actions_msgs::LandFeedback feedback_;
	arcas_actions_msgs::LandResult result_;
	StateUAV last_state;
	unsigned char phase;
	std::string uavID_;
};
#endif
