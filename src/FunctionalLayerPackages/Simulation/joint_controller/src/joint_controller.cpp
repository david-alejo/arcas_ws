/*************************************************************************
 *
 * FADA-CATEC
 * __________________
 *
 *  [2013] FADA-CATEC
 *  All Rights Reserved.
 *
 * NOTICE:  All information contained herein is, and remains
 * the property of FADA-CATEC and its suppliers,
 * if any.  The intellectual and technical concepts contained
 * herein are proprietary to FADA-CATEC
 * and its suppliers and may be covered by Europe and Foreign Patents,
 * patents in process, and are protected by trade secret or copyright law.
 * Dissemination of this information or reproduction of this material
 * is strictly forbidden unless prior written permission is obtained
 * from FADA-CATEC.
 *
 * Created on: 20-Mar-2013
 * Engineer: Jonathan Ruiz Páez
 * Email: jruiz@catec.aero
 */


#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <arcas_msgs/JointControl.h>



using namespace std;
using namespace sensor_msgs;

void quit(int sig) {
	ros::shutdown();
	exit(0);
}

int main(int argc, char **argv)
{
	if (argc < 2)
	{
		cout << "This program has one input parameters.\n"<< endl <<
				"The first input parameter is the name of joint." << endl;
		return -1;
	}

	signal(SIGINT, quit);

	string nodeName = "simple_joint_state_publisher";
	nodeName.append(argv[1]);
	ros::init(argc, argv, nodeName);

	ros::NodeHandle n;

	string topicname = "bonebraker";
			topicname.append("/set_joint_position");
			ros::Publisher joint_state_publisher = n.advertise<arcas_msgs::JointControl> (topicname.c_str(), 0);

	ros::AsyncSpinner spinner_(0);
	spinner_.start();

	double position;
	arcas_msgs::JointControl state;
	state.joint_name = argv[1];
	while(ros::ok())
	{
		cout << "Insert position of joint:" << endl;
		cin >> position;



		state.position = position;
		joint_state_publisher.publish(state);

		cout << "Enviando state a gazebo..." << endl << endl;
	}

}
