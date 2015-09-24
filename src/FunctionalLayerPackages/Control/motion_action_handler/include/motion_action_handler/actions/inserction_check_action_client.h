#ifndef _INSERCTION_CHECK_ACTION_WRAPPER_
#define _INSERCTION_CHECK_ACTION_WRAPPER_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <arcas_actions_msgs/InserctionCheckAction.h>

using namespace std;
using namespace arcas_actions_msgs;

typedef actionlib::SimpleActionClient<InserctionCheckAction> InserctionCheckClient;

class InserctionCheckActionWrapper
{
public:
    InserctionCheckActionWrapper():
		cInserctionCheck(NULL),
		goalRunning(false)
	{
		cInserctionCheck = new InserctionCheckClient("inserction_check",true);
	}
    ~InserctionCheckActionWrapper()
	{

	}
	bool waitForServer()
	{
		return cInserctionCheck->waitForServer();
	}
	bool hasGoalRunning()
	{
		return goalRunning;
	}

private:
	InserctionCheckClient *cInserctionCheck;
	bool goalRunning;

	void inserction_Check_Active_CB()
	{

	}

	void inserction_Check_Feedback_CB(const InserctionCheckFeedbackConstPtr& feedback)
	{

	}

	void inserction_Check_Done_CB(const actionlib::SimpleClientGoalState& state, const InserctionCheckResultConstPtr& result)
	{
		goalRunning = false;
	}

};
#endif //_INSERCTION_CHECK_ACTION_WRAPPER_
