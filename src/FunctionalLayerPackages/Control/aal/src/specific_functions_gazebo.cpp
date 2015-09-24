/*
 * specific_functions_gazebo.cpp
 *
 *  Created on: 22/03/2013
 *      Author: catec
 */

#include <aal/specific_functions_gazebo.h>
#include <map>



	SpecificFunctionsGazebo::SpecificFunctionsGazebo(ros::NodeHandle n)
	{
		//Inicializar el map con los nombres de los joints.
		std::string joint_temp;
		int num_of_joints;
		if(n.getParam("/num_of_joints",num_of_joints))
		{

		}
		else
		{
			ROS_ERROR("Can't find joint parameters.");
			ros::shutdown();
		}

		this->joint_control_pub_ = n.advertise<arcas_msgs::JointControl> ("set_joint_position", 0);
		this->joint_parameters_pub_ = n.advertise<arcas_msgs::JointParameters> ("set_joint_parameters", 0);
		this->joint_states_sub_ = n.subscribe("joints_state",0,&SpecificFunctionsGazebo::jointStatesCallback, this);
	}
	SpecificFunctionsGazebo::~SpecificFunctionsGazebo()
	{

	}
	void SpecificFunctionsGazebo::jointStatesCallback(const sensor_msgs::JointStateConstPtr &js)
	{
		this->last_states_[js->name[0]] = *js;
	}
	std::vector<double> SpecificFunctionsGazebo::getJointsState()
	{
		this->angles.clear();
		for( std::map<std::string,sensor_msgs::JointState>::iterator it = last_states_.begin(); it != last_states_.end(); ++it ) {
					angles.push_back( it->second.position[0] );
		}
		return angles;
	}


	void SpecificFunctionsGazebo::sendJointPosition(std::vector<double> angles)
	{
		for( std::vector<double>::iterator it = angles.begin(); it != angles.end(); ++it ) {
			arcas_msgs::JointParameters p_joint;
			//p_joint.
		}
	}
