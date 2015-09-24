#include <iostream>
#include <ros/ros.h>
#include <ual/ualhectorgazebo.h>

int main(int argc, char **argv)
{

	if (argc < 2)
	{
		std::cout << "This program need one input parameter.\n"<<
				"The first input parameter is the number of the UAV." << std::endl;
		return -1;
	}



	// The UAV ID is stored in a global variable
	std::string uavID="ual_";
	uavID.append(std::string(argv[1]));

	ros::init(argc,argv,uavID);

	ros::NodeHandle n(uavID);

	// TODO: Correct the atoi with a function call that checks if the argument is really a number 
	UALHectorGazebo ualHectorGazebo(&n, atoi(argv[1]));

	ros::spin();

	return 0;
}
