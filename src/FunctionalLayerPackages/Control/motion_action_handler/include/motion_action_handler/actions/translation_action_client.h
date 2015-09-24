#ifndef _TRANSLATION_ACTION_WRAPPER_
#define _TRANSLATION_ACTION_WRAPPER_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <arcas_actions_msgs/TranslationAction.h>

using namespace std;
using namespace arcas_actions_msgs;

typedef actionlib::SimpleActionClient<TranslationAction> TranslationClient;

class TranslationActionWrapper
{
public:
	TranslationActionWrapper(std::string prefix):
		cTranslation(NULL),
		goalRunning(false)
	{
		std::string translation_name = prefix;
		translation_name.append("/translation");
		cTranslation = new TranslationClient(translation_name,true);
	}
	~TranslationActionWrapper()
	{

	}
	bool waitForServer()
	{
		return cTranslation->waitForServer();
	}
	bool initTranslation(TranslationGoal translation_goal)
	{
		if(goalRunning)
		{
			return false;
		}else
		{
			cTranslation->sendGoal(translation_goal,
				boost::bind(&TranslationActionWrapper::translation_Done_CB,this,_1,_2),
				boost::bind(&TranslationActionWrapper::translation_Active_CB,this),
				boost::bind(&TranslationActionWrapper::translation_Feedback_CB,this,_1)
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
	TranslationClient *cTranslation;
	bool goalRunning;

	void translation_Active_CB()
	{

	}

	void translation_Feedback_CB(const TranslationFeedbackConstPtr& feedback)
	{

	}

	void translation_Done_CB(const actionlib::SimpleClientGoalState& state, const TranslationResultConstPtr& result)
	{
		goalRunning = false;
	}

};

#endif //_TRANSLATION_ACTION_WRAPPER_
