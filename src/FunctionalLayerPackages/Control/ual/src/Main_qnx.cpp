#include <iostream>
#include <ros/ros.h>
#include <ual/ualqnx.h>


bool is_number(const std::string& s)
{
	std::string::const_iterator it = s.begin();
	while (it != s.end() && std::isdigit(*it)) ++it;
	return !s.empty() && it == s.end();
}
int main(int argc, char **argv)
{


	if(argc < 5)
	{
		std::cerr << "UAL Qnx: too few parameters, this node needs 4 parameters"
				  << std::endl
				  << "Usage: ual_qnx UAV_ID QNX_HOST_IP STATE_PORT COMMAND_PORT"
				  << std::endl;
		return -1;
	}

	std::string uav_id(argv[1]);
	std::string qnx_host(argv[2]);

	if(!is_number(uav_id))
	{
		std::cerr << "UAL Qnx: parameter number 1 must be a natural number"
				  << std::endl
				  << "Usage: ual_qnx UAV_ID QNX_HOST_IP STATE_PORT COMMAND_PORT"
				  << std::endl;
		return -1;
	}
	if(!is_number(argv[3]))
	{
		std::cerr << "UAL Qnx: parameter number 3 must be a natural number"
				  << std::endl
				  << "Usage: ual_qnx UAV_ID QNX_HOST_IP STATE_PORT COMMAND_PORT"
				  << std::endl;
		return -1;
	}
	if(!is_number(argv[4]))
	{
		std::cerr << "UAL Qnx: parameter number 4 must be a natural number"
				  << std::endl
				  << "Usage: ual_qnx UAV_ID QNX_HOST_IP STATE_PORT COMMAND_PORT"
				  << std::endl;
		return -1;
	}


	std::string nodename("ual_");
	nodename.append(uav_id);

	ros::init(argc,argv,nodename);

	ros::NodeHandle n(nodename);

	UALQnx ualQnx(&n,atoi(uav_id.c_str()),qnx_host,atoi(argv[3]),atoi(argv[4]));

	ros::spin();

	return 0;
}
