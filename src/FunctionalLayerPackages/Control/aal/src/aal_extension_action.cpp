#include "aal/aal_extension_action.h"

AALExtensionAction::AALExtensionAction(ros::NodeHandle n):
	hasGoal(false),
	current_phase(FINISHED),
	current_state(CONTRACTED),
	as_(n,"/aal_extension",false),
	last_joint_states_(NULL),
	joint_names_(NULL)
{
	//Initialize pointers
	 joint_names_ =  new std::string[7];
	 last_joint_states_ = new map<std::string,sensor_msgs::JointState>();

	 //Initialize action server
	 as_.registerGoalCallback(boost::bind(&AALExtensionAction::goalCB, this, _1));
	 as_.registerCancelCallback(boost::bind(&AALExtensionAction::preemptCB, this, _1));
	 as_.start();

	 next_contracting_angles.push_back(CONTRACTING_JOINT_1);
	 next_contracting_angles.push_back(CONTRACTING_JOINT_2);
	 next_contracting_angles.push_back(CONTRACTING_JOINT_3);
	 next_contracting_angles.push_back(CONTRACTING_JOINT_4);
	 next_contracting_angles.push_back(CONTRACTING_JOINT_5);
	 next_contracting_angles.push_back(CONTRACTING_JOINT_6);
	 next_contracting_angles.push_back(CONTRACTING_JOINT_CLAW);
	 next_extending_angles.push_back(EXTENDING_JOINT_1);
	 next_extending_angles.push_back(EXTENDING_JOINT_2);
	 next_extending_angles.push_back(EXTENDING_JOINT_3);
	 next_extending_angles.push_back(EXTENDING_JOINT_4);
	 next_extending_angles.push_back(EXTENDING_JOINT_5);
	 next_extending_angles.push_back(EXTENDING_JOINT_6);
	 next_extending_angles.push_back(EXTENDING_JOINT_CLAW);

	 next_angles.resize(7);

	 next_angles[0] = next_contracting_angles[0];
	 next_angles[1] = next_contracting_angles[1];
	 next_angles[2] = next_contracting_angles[2];
	 next_angles[3] = next_contracting_angles[3];
	 next_angles[4] = next_contracting_angles[4];
	 next_angles[5] = next_contracting_angles[5];
	 next_angles[6] = next_contracting_angles[6];

	 //Initialize timer:
	 main_loop_timer = n.createTimer(ros::Duration(1/30.0),&AALExtensionAction::mainLoop,this);
	 main_loop_timer.start();
}

AALExtensionAction::~AALExtensionAction()
{

}
bool AALExtensionAction::isExtended()
{
	return !hasGoal && current_state == EXTENDED;
}

vector<double> AALExtensionAction::getExtensionAngles()
{
	return next_angles;
}

AALExtensionAction::Phase AALExtensionAction::nextPhase()
{
	if(current_phase!=FINISHED && last_joint_states_!=NULL)
	{
		JointState jstate = (*last_joint_states_)[joint_names_[current_phase-1]];


		/*cerr << "ALLExtensionAction: Current_Phase: " <<
				current_phase << " current_state: " <<
				current_state <<
				" Diff: " << abs(jstate.position[0] - next_angles[current_phase-1]) << endl;*/
		if(abs(jstate.position[0] - next_angles[current_phase-1]) > EQUAL_ANGLE)
		{//do not change phase
			return current_phase;
		}else
		{//have to change to the next phase.
			if(current_state==CONTRACTING)
			{
				switch(current_phase)
				{
					case JOINT_1:
						return JOINT_4;
						break;
					case JOINT_2:
						return JOINT_3;
						break;
					case JOINT_3:
						return FINISHED;
						break;
					case JOINT_4:
						return JOINT_5;
						break;
					case JOINT_5:
						return JOINT_6;
						break;
					case JOINT_6:
						return JOINT_2;
						break;
					default:
						return current_phase;
						break;
				}

			}
			else if(current_state==EXTENDEDING)
			{
				switch(current_phase)
				{
					case JOINT_1:
						return JOINT_3;
						break;
					case JOINT_2:
						return JOINT_4;
						break;
					case JOINT_3:
						return JOINT_2;
						break;
					case JOINT_4:
						return JOINT_5;
						break;
					case JOINT_5:
						return JOINT_6;
						break;
					case JOINT_6:
						return FINISHED;
						break;
					default:
						return current_phase;
						break;
				}
			}
			else
			{
				return current_phase;
			}
		}
	}
	return current_phase;
}

void  AALExtensionAction::mainLoop(const ros::TimerEvent& te)
{
	if(hasGoal)
	{
		if(current_phase==FINISHED)
		{
			if(current_state==CONTRACTING)
			{
				this->current_state = CONTRACTED;
				this->hasGoal = false;
				//send done message
				this->sendDoneMessage(CORRECTLY_DONE);
			}
			else if(current_state == EXTENDEDING)
			{
				this->current_state = EXTENDED;
				this->hasGoal = false;
				//send done message
				this->sendDoneMessage(CORRECTLY_DONE);
			}
			else
			{
				//Error
				this->sendDoneMessage(IMPOSIBLE);
			}
		}
		else
		{

			current_phase = this->nextPhase();
			this->updateAngles();

		}


	}

}

void AALExtensionAction::updateAngles()
{
	switch(current_phase)
	{
		case JOINT_1:
			if(current_state==CONTRACTING)
				next_angles[JOINT_1-1] = next_contracting_angles[JOINT_1-1];
			else if(current_state==EXTENDEDING)
				next_angles[JOINT_1-1] = next_extending_angles[JOINT_1-1];
			break;
		case JOINT_2:
			if(current_state==CONTRACTING)
				next_angles[JOINT_2-1] = next_contracting_angles[JOINT_2-1];
			else if(current_state==EXTENDEDING)
				next_angles[JOINT_2-1] = next_extending_angles[JOINT_2-1];
			break;
		case JOINT_3:
			if(current_state==CONTRACTING)
				next_angles[JOINT_3-1] = next_contracting_angles[JOINT_3-1];
			else if(current_state==EXTENDEDING)
				next_angles[JOINT_3-1] = next_extending_angles[JOINT_3-1];
			break;
		case JOINT_4:
			if(current_state==CONTRACTING)
				next_angles[JOINT_4-1] = next_contracting_angles[JOINT_4-1];
			else if(current_state==EXTENDEDING)
				next_angles[JOINT_4-1] = next_extending_angles[JOINT_4-1];
			break;
		case JOINT_5:
			if(current_state==CONTRACTING)
				next_angles[JOINT_5-1] = next_contracting_angles[JOINT_5-1];
			else if(current_state==EXTENDEDING)
				next_angles[JOINT_5-1] = next_extending_angles[JOINT_5-1];
			break;
		case JOINT_6:
			if(current_state==CONTRACTING)
				next_angles[JOINT_6-1] = next_contracting_angles[JOINT_6-1];
			else if(current_state==EXTENDEDING)
				next_angles[JOINT_6-1] = next_extending_angles[JOINT_6-1];
			break;
		case CLAW:
			if(current_state==CONTRACTING)
				next_angles[CLAW-1] = next_contracting_angles[CLAW-1];
			else if(current_state==EXTENDEDING)
				next_angles[CLAW-1] = next_extending_angles[CLAW-1];
			break;
		default:
			break;
	}

//	std::cerr << "Current phase of extension" << current_phase << std::endl;
}

void AALExtensionAction::sendDoneMessage(AbortedState st)
{

	if(st==CORRECTLY_DONE)
	{
		switch(current_state)
		{
			case EXTENDED:
				result_msg.result = result_msg.EXTENDED;
				break;
			case CONTRACTED:
				result_msg.result = result_msg.CONTRACTED;
				break;
			default:
				break;
		}
	}else
	{
		switch(st)
		{
			case ALREADY_CONTRACTED:
				result_msg.result = result_msg.ALREADY_CONTRACTED;
				break;
			case ALREADY_EXTENDED:
				result_msg.result = result_msg.ALREADY_EXTENDED;
				break;
			case IMPOSIBLE:
				result_msg.result = result_msg.IMPOSIBLE;
			default:
				break;
		}
	}

	goal_handle_.setSucceeded(result_msg);

}

void AALExtensionAction::updateJointState(std::map<string, JointState> *last_joint_states,
										  string *joint_names)
{
	this->joint_names_ = joint_names;
	this->last_joint_states_ = last_joint_states;
}

void AALExtensionAction::goalCB(
	actionlib::ServerGoalHandle<arcas_actions_msgs::AALExtensionActionAction> goal_handle)
{

	arcas_actions_msgs::AALExtensionActionGoal new_goal = *goal_handle.getGoal();

	if(this->hasGoal || current_state == EXTENDEDING || current_state == CONTRACTING)
	{
		result_msg.result = result_msg.IMPOSIBLE;
		goal_handle.setRejected(result_msg);
	}
	else if(current_state == EXTENDED && new_goal.action == new_goal.EXTENDED)
	{
		result_msg.result = result_msg.ALREADY_EXTENDED;
		goal_handle.setRejected(result_msg);
	}
	else if(current_state == CONTRACTED && new_goal.action == new_goal.CONTRACTED)
	{
		result_msg.result = result_msg.ALREADY_CONTRACTED;
		goal_handle.setRejected(result_msg);
	}else
	{//Acept goal.
		goal_handle_ = goal_handle;
		hasGoal = true;
		switch(new_goal.action)
		{
			case arcas_actions_msgs::AALExtensionActionGoal::CONTRACTED:
				current_state = CONTRACTING;
				current_phase = JOINT_1;
				break;
			case arcas_actions_msgs::AALExtensionActionGoal::EXTENDED:
				current_state = EXTENDEDING;
				current_phase = JOINT_1;
				break;
			default:
				break;

		}
		goal_handle.setAccepted();

	}

}
void AALExtensionAction::preemptCB(
		actionlib::ServerGoalHandle<arcas_actions_msgs::AALExtensionActionAction> goal_handle)
{
	//Try to cancel callback
	//Ignore actually cant cancel take off action.
}
