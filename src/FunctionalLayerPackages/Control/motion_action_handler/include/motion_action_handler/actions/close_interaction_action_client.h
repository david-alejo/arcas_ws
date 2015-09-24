#ifndef _CLOSE_INTERACTION_ACTION_WRAPPER_
#define _CLOSE_INTERACTION_ACTION_WRAPPER_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <arcas_actions_msgs/CloseInteractionAction.h>

using namespace std;
using namespace arcas_actions_msgs;

typedef actionlib::SimpleActionClient<CloseInteractionAction> CloseInteractionClient;

class CloseInteractionActionWrapper
{
public:
    CloseInteractionActionWrapper():
		cCloseInteraction(NULL),
		goalRunning(false)
	{
		cCloseInteraction = new CloseInteractionClient("close_interaction",true);
	}
    ~CloseInteractionActionWrapper()
	{

	}
	bool waitForServer()
	{
		return cCloseInteraction->waitForServer();
	}
	bool hasGoalRunning()
	{
		return goalRunning;
	}

private:
	CloseInteractionClient *cCloseInteraction;
	bool goalRunning;

	void close_Interaction_Active_CB()
	{

	}

	void close_Interaction_Feedback_CB(const CloseInteractionFeedbackConstPtr& feedback)
	{

	}

	void close_Interaction_Done_CB(const actionlib::SimpleClientGoalState& state, const CloseInteractionResultConstPtr& result)
	{
		goalRunning = false;
	}

};

#endif //_CLOSE_INTERACTION_ACTION_WRAPPER_
