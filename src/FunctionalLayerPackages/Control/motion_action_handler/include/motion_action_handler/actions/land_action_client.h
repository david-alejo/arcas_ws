#ifndef _LAND_ACTION_WRAPPER_
#define _LAND_ACTION_WRAPPER_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <arcas_actions_msgs/LandAction.h>

using namespace std;
using namespace arcas_actions_msgs;

typedef actionlib::SimpleActionClient<LandAction> LandClient;

class LandActionWrapper
{
public:
	LandActionWrapper(std::string prefix):
		cLand(NULL),
		goalRunning(false)
	{
		cLand = new LandClient(prefix + std::string("/land"),true);
	}
	~LandActionWrapper()
	{

	}
	bool waitForServer()
	{
		return cLand->waitForServer();
	}
	bool land()
	{
		if(goalRunning)
		{
			return false;
		}else
		{
			LandGoal land_goal;
			cLand->sendGoal(land_goal,
				boost::bind(&LandActionWrapper::land_Done_CB,this,_1,_2),
				boost::bind(&LandActionWrapper::land_Active_CB,this),
				boost::bind(&LandActionWrapper::land_Feedback_CB,this,_1)
							   );
			goalRunning = true;
			return true;
		}
	}
	bool hasGoalRunning()
	{
		return goalRunning;
	}

private:
	LandClient *cLand;
	bool goalRunning;

	void land_Active_CB()
	{

	}

	void land_Feedback_CB(const LandFeedbackConstPtr& feedback)
	{

	}

	void land_Done_CB(const actionlib::SimpleClientGoalState& state, const LandResultConstPtr& result)
	{
		goalRunning = false;
	}

};

#endif //_LAND_ACTION_WRAPPER_
