/*
 * joystick_control.cpp
 *
 *  Created on: 03/10/2013
 *      Author: catec
*/

// ROS includes
#include <ros/ros.h>
#include <iostream>
#include <signal.h>

//Twist message to send a message to the cmd_vel topic
#include <geometry_msgs/Twist.h>

//Include the Joystick
#include "joystick/joystick.h"

// Include the motion_action_handler land and take-off client
#include <motion_action_handler/actions/land_action_client.h>
#include <motion_action_handler/actions/takeoff_action_client.h>

//Include the message to get the position and publish it later on
#include <arcas_msgs/QuadStateEstimationWithCovarianceStamped.h>
#include <arcas_msgs/QuadControlReferencesStamped.h>

// Creating global variables
geometry_msgs::Twist command_to_send;
arcas_msgs::QuadStateEstimationWithCovarianceStamped quad_state_estimation;
arcas_msgs::QuadControlReferencesStamped quad_control_references;

bool firstPosition = false;


void quit(int sig)
{
   ros::shutdown();
   exit(0);
}


// Callback to get the current state of the quadrotor
void quadStateEstimationCallback(const arcas_msgs::QuadStateEstimationWithCovarianceStampedConstPtr &st)
{
   quad_state_estimation = *st;


   if(firstPosition==false)
   {
      quad_control_references.quad_control_references.position_ref.x = quad_state_estimation.quad_state_estimation_with_covariance.position.x;

   quad_control_references.quad_control_references.position_ref.y = quad_state_estimation.quad_state_estimation_with_covariance.position.y;

   quad_control_references.quad_control_references.position_ref.z = quad_state_estimation.quad_state_estimation_with_covariance.position.z;



      firstPosition = true;
   }
   //ROS_INFO_STREAM ("The x position is: " << quad_state_estimation.quad_state_estimation_with_covariance.position.x);
   //ROS_INFO_STREAM ("The y position is: " << quad_state_estimation.quad_state_estimation_with_covariance.position.y);
   //ROS_INFO_STREAM ("The z position is: " << quad_state_estimation.quad_state_estimation_with_covariance.position.z);
   //ROS_INFO_STREAM ("The heading is: " << quad_state_estimation.quad_state_estimation_with_covariance.attitude.yaw);
}


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


   signal(SIGINT, quit);
   string node_name="hector_joystick";
   node_name.append(std::string(argv[1]));





   ros::init(argc, argv, node_name.c_str());
   ros::NodeHandle n(uavID);
   string topicname = "/cmd_vel";





   //Publisher and Subscribers
   ros::Publisher joystick_publisher = n.advertise<geometry_msgs::Twist> (topicname.c_str(), 0);

   topicname = "/";
   topicname.append(uavID);
   topicname.append("/quad_control_references");
   ros::Publisher quad_control_ref_pub = n.advertise<arcas_msgs::QuadControlReferencesStamped>(topicname,0);


   topicname = "/";
   topicname.append(uavID);
   topicname.append("/quad_state_estimation");
   ros::Subscriber state_estimation_subscriber = n.subscribe(topicname,
               1000, &quadStateEstimationCallback);


   int fd;
   struct wwvi_js_event wjse;

   wjse.stick2_x = 0; // Roll
   wjse.stick2_y = 0; // Yaw
   wjse.stick_x = 0;  // Pitch
   wjse.stick_y = 0;  // Thrust
   wjse.button[0]=0;  // Take-Off, Button 1 on Joystick
   wjse.button[1]=0;  // Land, Button 2 on Joystick

   CJoystick joystick;
   fd = joystick.open_joystick("/dev/input/js0");

   // Initialize ActionClient
   LandActionWrapper landActionClient(uavID);
   TakeOffActionWrapper takeoffActionclient(uavID);


   if(!fd)
   {
      cerr << "Can not open joystick, is it still connected?\r\n" << endl;
      return(-1);
   }

   ros::AsyncSpinner spinner_(0);
   spinner_.start();

   quad_control_references.quad_control_references.velocity_ref = 1.0;
   while(ros::ok())
   {

      // Calculate the quad_control_references
      if(wjse.stick_x!=0)
      {
         quad_control_references.quad_control_references.position_ref.x = quad_state_estimation.quad_state_estimation_with_covariance.position.x + wjse.stick_x/4000.0;
      }

      if(wjse.stick2_x!=0)
      {
      quad_control_references.quad_control_references.position_ref.y = quad_state_estimation.quad_state_estimation_with_covariance.position.y + wjse.stick2_x/4000.0;
      }

      if(wjse.stick_y!=0)
      {
      quad_control_references.quad_control_references.position_ref.z = quad_state_estimation.quad_state_estimation_with_covariance.position.z + wjse.stick_y/4000.0;
      }


      if( wjse.stick2_y!=0)
      {
      quad_control_references.quad_control_references.heading = quad_state_estimation.quad_state_estimation_with_covariance.attitude.yaw + wjse.stick2_y/4000.0;
      }


/*
      if(joystick.get_joystick_status(&wjse)!=-1)
      {
         command_to_send.linear.x = wjse.stick_x/2500.0;
         //std::cout << "Pitch control: " << command_to_send.linear.x << std::endl;

         command_to_send.linear.y = wjse.stick2_x/2500.0;
         //std::cout << "Roll control: " << command_to_send.linear.y << std::endl;

         command_to_send.linear.z = wjse.stick_y/4000.0;
         //std::cout << "Thrust control: " << command_to_send.linear.z << std::endl;

         command_to_send.angular.z = wjse.stick2_y/4000.0;
         //std::cout << "Yaw control: " <<  command_to_send.angular.z << std::endl;

      }
      else
      {
         ROS_ERROR_STREAM("Problems with joystick!");
      }
*/

      // Implement take-off
      if((joystick.get_joystick_status(&wjse)!=-1) && (wjse.button[0] == 1))
      {
         ROS_INFO_STREAM("Take-Off Action Sent");
         takeoffActionclient.takeOff();
         usleep(500000); // In order to not send it twice accidentally
      }

      // Implement landing
      if((joystick.get_joystick_status(&wjse)!=-1) && (wjse.button[1] == 1))
      {
         ROS_INFO_STREAM("Land Action Sent");
         landActionClient.land();
         usleep(500000); // In order to not send it twice accidentally
      }


    //joystick_publisher.publish(command_to_send);
    quad_control_ref_pub.publish(quad_control_references);

    //~20Hz
    usleep(50000);
   }

   return 0;
}
