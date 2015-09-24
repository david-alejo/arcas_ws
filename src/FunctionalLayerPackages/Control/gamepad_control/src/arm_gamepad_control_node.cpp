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
#include <motion_action_handler/actions/extend_arm_action_client.h>

//Include the message to get the position and publish it later on
#include <arcas_msgs/ArmStateEstimationStamped.h>
#include <arcas_msgs/ArmControlReferencesStamped.h>

// Creating global variables
geometry_msgs::Twist command_to_send;
arcas_msgs::ArmStateEstimationStamped arm_state_estimation;
arcas_msgs::ArmControlReferencesStamped arm_control_references;

bool firstPosition = false;


void quit(int sig)
{
   ros::shutdown();
   exit(0);
}


// Callback to get the current state of the quadrotor
void armStateEstimationCallback(const arcas_msgs::ArmStateEstimationStampedConstPtr &st)
{
   arm_state_estimation = *st;
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
   std::string uavID="aal_";
   uavID.append(std::string(argv[1]));


   signal(SIGINT, quit);
   string node_name="arm_joystick";
   node_name.append(std::string(argv[1]));





   ros::init(argc, argv, node_name.c_str());
   ros::NodeHandle n(uavID);
   string topicname = "/cmd_vel";





   //Publisher and Subscribers
   ros::Publisher arm_control_ref_pub = n.advertise<arcas_msgs::ArmControlReferencesStamped>("/aal_1/arm_control_references",0);

   ros::Subscriber state_estimation_subscriber = n.subscribe("/aal_1/arm_state_estimation",
               1000, &armStateEstimationCallback);


   int fd;
   struct wwvi_js_event wjse;

   wjse.stick2_x = 0; // Roll
   wjse.stick2_y = 0; // Yaw
   wjse.stick_x = 0;  // Pitch
   wjse.stick_y = 0;  // Thrust
   wjse.button[0]=0;  // Take-Off, Button 1 on Joystick
   wjse.button[1]=0;  // Land, Button 2 on Joystick

   CJoystick joystick;
   fd = joystick.open_joystick("/dev/input/js1");

   // Initialize ActionClient
   AALExtensionActionWrapper aalExtensionActionWrapper;


   if(!fd)
   {
      cerr << "Can not open joystick, is it still connected?\r\n" << endl;
      return(-1);
   }

   ros::AsyncSpinner spinner_(0);
   spinner_.start();

   while(ros::ok())
   {

      // Calculate the quad_control_references
      if(wjse.stick_x!=0)
      {

         arm_control_references.arm_control_references.position_ref[2] = arm_state_estimation.arm_state_estimation.position[2] + wjse.stick_x/6000.0;
      }

      if(wjse.stick2_x!=0)
      {
      arm_control_references.arm_control_references.position_ref[3] = arm_state_estimation.arm_state_estimation.position[3] + wjse.stick2_x/6000.0;
      }

      if(wjse.stick_y!=0)
      {
      arm_control_references.arm_control_references.position_ref[1] = arm_state_estimation.arm_state_estimation.position[1] + wjse.stick_y/6000.0;
      }


      if( wjse.stick2_y!=0)
      {
         arm_control_references.arm_control_references.position_ref[0] = arm_state_estimation.arm_state_estimation.position[0] + wjse.stick2_y/6000.0;
      }


      if(wjse.stick3_x!=0)
      {
      arm_control_references.arm_control_references.position_ref[4] = arm_state_estimation.arm_state_estimation.position[4] + wjse.stick3_x/12000.0;
      }

      if(wjse.stick3_y!=0)
      {
      arm_control_references.arm_control_references.position_ref[5] = arm_state_estimation.arm_state_estimation.position[5] + wjse.stick3_y/12000.0;
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
         ROS_INFO_STREAM("Extracting Sent");
         aalExtensionActionWrapper.extendArm();
         usleep(100000); // In order to not send it twice accidentally
      }

      // Implement landing
      if((joystick.get_joystick_status(&wjse)!=-1) && (wjse.button[1] == 1))
      {
         ROS_INFO_STREAM("Contracting Sent");
         aalExtensionActionWrapper.contractArm();
         usleep(100000); // In order to not send it twice accidentally
      }

      if((joystick.get_joystick_status(&wjse)!=-1) && (wjse.button[2] == 1))
      {

         arm_control_references.arm_control_references.position_ref[6] = 3.14159/5;

         usleep(100000); // In order to not send it twice accidentally
      }
      if((joystick.get_joystick_status(&wjse)!=-1) && (wjse.button[3] == 1))
      {

         arm_control_references.arm_control_references.position_ref[6] =0.0;

         usleep(100000); // In order to not send it twice accidentally
      }


    //joystick_publisher.publish(command_to_send);
    arm_control_ref_pub.publish(arm_control_references);

    //~20Hz
    usleep(50000);
   }

   return 0;
}
