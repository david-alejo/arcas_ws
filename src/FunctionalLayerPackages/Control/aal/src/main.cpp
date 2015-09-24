/*
 * aar_sim.cpp
 *
 *  Created on: 22/03/2013
 *      Author: catec
 */
#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <aal/arm_abstraction_layer_gazebo.h>


using namespace std;

ArmAbstractionLayer *aal_pointer=NULL;

void quit(int sig) {
	if(aal_pointer!=NULL)
		delete aal_pointer;

	ros::shutdown();
	exit(0);
}
int main(int argc, char** argv)
{
	if(argc < 2)
	{
		cout << "This aplication need one input parameter" << endl <<
				"The first input parameter is the aal node number" << endl;
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
	aal_pointer = new ArmAbstractionLayerGazebo(n);

	//ros spin
	ros::spin();

	//If ros exit
	if(aal_pointer!=NULL)
		delete aal_pointer;

	return 0;
}

