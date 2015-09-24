#include <iostream>
#include <ros/ros.h>
#include <ual/ualsimulinkgazebo.h>

int main(int argc, char **argv)
{
	ros::init(argc,argv,"ual_1");

	ros::NodeHandle n("ual_1");

    UALSimulinkGazebo ualSimulinkGazebo(&n,1);

	ros::spin();

	return 0;
}
