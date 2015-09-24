#ifndef _AAL_EXTENSION_ACTION_WRAPPER_
#define _AAL_EXTENSION_ACTION_WRAPPER_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <arcas_actions_msgs/AALExtensionActionAction.h>

using namespace std;
using namespace arcas_actions_msgs;

typedef actionlib::SimpleActionClient<AALExtensionActionAction> AALExtensionClient;

class AALExtensionActionWrapper
{
public:
    AALExtensionActionWrapper():
		cAALExtension(NULL),
		goalRunning(false)
	{
        cAALExtension = new AALExtensionClient("aal_extension",true);
	}
    ~AALExtensionActionWrapper()
	{

	}
	bool waitForServer()
	{
		return cAALExtension->waitForServer();
	}
	bool extendArm()
	{
		if(goalRunning)
		{
			return false;
		}else
		{
			AALExtensionActionGoal extend_goal;
			extend_goal.action = extend_goal.EXTENDED;
			cAALExtension->sendGoal(extend_goal,
                boost::bind(&AALExtensionActionWrapper::extensionAction_Done_CB,this,_1,_2),
                boost::bind(&AALExtensionActionWrapper::extensionAction_Active_CB,this),
                boost::bind(&AALExtensionActionWrapper::extensionAction_Feedback_CB,this,_1)
							   );
			goalRunning = true;
			return true;
		}
	}
	bool contractArm()
	{
		if(goalRunning)
		{
			return false;
		}else
		{
			AALExtensionActionGoal extend_goal;
			extend_goal.action = extend_goal.CONTRACTED;
			cAALExtension->sendGoal(extend_goal,
                boost::bind(&AALExtensionActionWrapper::extensionAction_Done_CB,this,_1,_2),
                boost::bind(&AALExtensionActionWrapper::extensionAction_Active_CB,this),
                boost::bind(&AALExtensionActionWrapper::extensionAction_Feedback_CB,this,_1)
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
	AALExtensionClient *cAALExtension;
	bool goalRunning;

	void extensionAction_Active_CB()
	{

	}

	void extensionAction_Feedback_CB(const AALExtensionActionFeedbackConstPtr& feedback)
	{

	}

	void extensionAction_Done_CB(const actionlib::SimpleClientGoalState& state, const AALExtensionActionResultConstPtr& result)
	{
		goalRunning = false;
	}

};

#endif //_AAL_EXTENSION_ACTION_WRAPPER_
