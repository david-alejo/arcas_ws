/*
 * aar_sim.cpp
 *
 *  Created on: 22/03/2013
 *      Author: catec
 */
#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <aal/arm_abstraction_layer_qnx.h>


using namespace std;

ArmAbstractionLayer *aal_pointer=NULL;

void quit(int sig) {
	if(aal_pointer!=NULL)
		delete aal_pointer;

	ros::shutdown();
	exit(0);
}

bool is_number(const std::string& s)
{
	std::string::const_iterator it = s.begin();
	while (it != s.end() && std::isdigit(*it)) ++it;
	return !s.empty() && it == s.end();
}

int main(int argc, char** argv)
{

	if(argc < 4)
	{
		std::cerr << "UAL Qnx: too few parameters, this node needs 3 parameters"
				  << std::endl
				  << "Usage: aal_qnx UAV_ID QNX_HOST_IP COMMAND_PORT"
				  << std::endl;
		return -1;
	}

	std::string uav_id(argv[1]);
	std::string qnx_host(argv[2]);

	if(!is_number(uav_id))
	{
		std::cerr << "UAL Qnx: parameter number 1 must be a natural number"
				  << std::endl
				  << "Usage: aal_qnx UAV_ID QNX_HOST_IP COMMAND_PORT"
				  << std::endl;
		return -1;
	}

	if(!is_number(argv[3]))
	{
		std::cerr << "UAL Qnx: parameter number 3 must be a natural number"
				  << std::endl
				  << "Usage: aal_qnx UAV_ID QNX_HOST_IP COMMAND_PORT"
				  << std::endl;
		return -1;
	}

	//Register SIGINT callback
	signal(SIGINT, quit);

	//Initialize ros.
	string node_name = "aal_";
	node_name.append(argv[1]);
	ros::init(argc,argv,node_name);
	ros::NodeHandle n(node_name);

	//Construct object AAL
	aal_pointer = new ArmAbstractionLayerQNX(n,qnx_host,atoi(argv[3]));

	//ros spin
	ros::spin();

	//If ros exit
	if(aal_pointer!=NULL)
		delete aal_pointer;

	return 0;
}

