/*
 * joystick_control.cpp
 *
 *  Created on: 17/01/2013
 *      Author: catec
 */

#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include<arcas_msgs/Joystick.h>

#include "joystick/joystick.h"

using namespace arcas_msgs;

void quit(int sig) {
	ros::shutdown();
	exit(0);
}

int main(int argc, char **argv)
{

		signal(SIGINT, quit);

		string node_name="arcas_joystick_1";
		string uavID="ual_1";

		ros::init(argc, argv, node_name.c_str());

		ros::NodeHandle n;

        string topicname = "/bonebraker/joystick_control";
		ros::Publisher joystick_publisher = n.advertise<Joystick> (topicname.c_str(), 0);
		Joystick message_to_send;

		int fd;
		struct wwvi_js_event wjse;

		wjse.stick2_x = 0;
		wjse.stick2_y = 0;
		wjse.stick_x = 0;
		wjse.stick_y = 0;
		wjse.button[0]=0;

		CJoystick joystick;
		fd = joystick.open_joystick();

		if(!fd)
		{
			cerr << "Error opening joystick, is still connected?\r\n" << endl;
			return(-1);
		}

		ros::AsyncSpinner spinner_(0);
		spinner_.start();

		while(ros::ok())
		{

            if(joystick.get_joystick_status(&wjse)!=-1)
            {
                message_to_send.thrust = wjse.stick_y*(-1) +2048;
                message_to_send.yaw = wjse.stick2_y*(-1) +2048;
                message_to_send.pitch = wjse.stick_x*(-1) +2048;
                message_to_send.roll = wjse.stick2_x*(-1) +2048;
            }else
            {
                message_to_send.thrust =0;
                message_to_send.yaw = 0;
                message_to_send.pitch = 0;
                message_to_send.roll = 0;
            }

			joystick_publisher.publish(message_to_send);

			//~20Hz
			usleep(50000);
		}
		return 0;
}
