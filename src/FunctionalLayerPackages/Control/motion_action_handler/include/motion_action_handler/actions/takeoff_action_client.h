#ifndef _TAKEOFF_ACTION_WRAPPER_
#define _TAKEOFF_ACTION_WRAPPER_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <arcas_actions_msgs/TakeOffAction.h>

using namespace std;
using namespace arcas_actions_msgs;

typedef actionlib::SimpleActionClient<TakeOffAction> TakeOffClient;

class TakeOffActionWrapper
{
public:
	TakeOffActionWrapper(std::string prefix):
		cTakeOff(NULL),
		goalRunning(false)
	{
		cTakeOff = new TakeOffClient(prefix + std::string("/take_off"),true);
	}
	~TakeOffActionWrapper()
	{

	}
	bool waitForServer()
	{
		return cTakeOff->waitForServer();
	}
	bool takeOff()
	{
		if(goalRunning)
		{
			return false;
		}else
		{
			TakeOffGoal tOff_goal;
			cTakeOff->sendGoal(tOff_goal,
				boost::bind(&TakeOffActionWrapper::tOff_Done_CB,this,_1,_2),
				boost::bind(&TakeOffActionWrapper::tOff_Active_CB,this),
				boost::bind(&TakeOffActionWrapper::tOff_Feedback_CB,this,_1)
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
	TakeOffClient *cTakeOff;
	bool goalRunning;

	void tOff_Active_CB()
	{

	}

	void tOff_Feedback_CB(const TakeOffFeedbackConstPtr& feedback)
	{

	}

	void tOff_Done_CB(const actionlib::SimpleClientGoalState& state, const TakeOffResultConstPtr& result)
	{
		goalRunning = false;
	}

};

#endif //_TAKEOFF_ACTION_WRAPPER_
