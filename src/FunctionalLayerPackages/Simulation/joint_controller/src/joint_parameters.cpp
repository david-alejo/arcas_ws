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
#include <arcas_msgs/JointParameters.h>



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

	string nodeName = "simple_joint_parameters_publisher";
	nodeName.append(argv[1]);
	ros::init(argc, argv, nodeName);

	ros::NodeHandle n;

	string topicname = "bonebraker";
			topicname.append("/set_joint_parameters");
			ros::Publisher joint_state_publisher = n.advertise<arcas_msgs::JointParameters> (topicname.c_str(), 0);

	ros::AsyncSpinner spinner_(0);
	spinner_.start();

	arcas_msgs::JointParameters state;
	state.joint_name = argv[1];
	while(ros::ok())
	{
		cout << "Insert Kp of joint:" << endl;
		cin >> state.Kp;
		cout << "Insert Kd of joint:" << endl;
		cin >> state.Kd;
		cout << "Insert Ki of joint:" << endl;
		cin >> state.Ki;
		cout << "Insert iMax of joint:" << endl;
		cin >> state.i_max;
		cout << "Insert iMin of joint:" << endl;
		cin >> state.i_min;
		cout << "Insert Effort of joint:" << endl;
		cin >> state.effort;
		cout << "Insert velocity of joint:" << endl;
		cin >> state.velocity;

		joint_state_publisher.publish(state);

		cout << "Enviando state a gazebo..." << endl << endl;
	}

}
