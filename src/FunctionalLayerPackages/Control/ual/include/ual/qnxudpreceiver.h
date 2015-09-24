#ifndef QNXUDPRECEIVER_H
#define QNXUDPRECEIVER_H
#include<iostream>
#include <ros/ros.h>
#include<ual/CUDPServer.h>
#include <ual/TransmisionDataType.h>
#include <arcas_msgs/QuadStateEstimationWithCovarianceStamped.h>
#include <arcas_msgs/ArmStateEstimationStamped.h>

class QnxUDPReceiver
{
public:
   QnxUDPReceiver(ros::NodeHandle &n, int port):
      port(port)
   {

      std::cerr << "Qnx Receiver: starting server at port " << port << std::endl;
      qnx_arm_state =n.advertise<arcas_msgs::ArmStateEstimationStamped>("qnx_udp_arm_state",0);
        _last_quad_state_stimation = new arcas_msgs::QuadStateEstimationWithCovarianceStamped();
   }

   arcas_msgs::QuadStateEstimationWithCovarianceStamped getLastQuadState()
   {
      return *_last_quad_state_stimation;
   }


   void operator()()
   {
      struct sockaddr_in si_other;
      unsigned int slen = sizeof(si_other);
      ArcasUDPState received_state;
      _udp_server.connect(port);

      while(true)
      {
         int readed;
         if(sizeof(ArcasUDPState) != (readed = _udp_server.receive(&received_state,
                                                     sizeof(ArcasUDPState),
                                                     (sockaddr*)&si_other,
                                                     (socklen_t*)&slen)))
         {
            std::cerr << "Qnx Receiver: Error reading udp datagram... size: " << readed
                    << " Espected: " << sizeof(ArcasUDPState) << std::endl;
         }
         else
         {
            if(received_state.type==ArmState)
            {
               //Publish to armState
               _last_arm_state_estimation.arm_state_estimation.battery =
                     received_state.arm_state.armPosition.sJoint8;
               _last_arm_state_estimation.arm_state_estimation.position[0] =
                     received_state.arm_state.armPosition.sJoint1;
               _last_arm_state_estimation.arm_state_estimation.position[1] =
                     received_state.arm_state.armPosition.sJoint2;
               _last_arm_state_estimation.arm_state_estimation.position[2] =
                     received_state.arm_state.armPosition.sJoint3;
               _last_arm_state_estimation.arm_state_estimation.position[3] =
                     received_state.arm_state.armPosition.sJoint4;
               _last_arm_state_estimation.arm_state_estimation.position[4] =
                     received_state.arm_state.armPosition.sJoint5;
               _last_arm_state_estimation.arm_state_estimation.position[5] =
                     received_state.arm_state.armPosition.sJoint6;
               _last_arm_state_estimation.arm_state_estimation.position[6] =
                     received_state.arm_state.armPosition.sJoint7;

               _last_arm_state_estimation.arm_state_estimation.velocity[0] =
                     received_state.arm_state.armVelocity.sJoint1;
               _last_arm_state_estimation.arm_state_estimation.velocity[1] =
                     received_state.arm_state.armVelocity.sJoint2;
               _last_arm_state_estimation.arm_state_estimation.velocity[2] =
                     received_state.arm_state.armVelocity.sJoint3;
               _last_arm_state_estimation.arm_state_estimation.velocity[3] =
                     received_state.arm_state.armVelocity.sJoint4;
               _last_arm_state_estimation.arm_state_estimation.velocity[4] =
                     received_state.arm_state.armVelocity.sJoint5;
               _last_arm_state_estimation.arm_state_estimation.velocity[5] =
                     received_state.arm_state.armVelocity.sJoint6;
               _last_arm_state_estimation.arm_state_estimation.velocity[6] =
                     received_state.arm_state.armVelocity.sJoint7;


               //std::cerr << "Flags: " << (unsigned short)received_state.arm_state.uiArmRetractionFlag << " - "
                  //		 << (unsigned short)received_state.arm_state.uiArmExtensionFlag << std::endl;
               if(received_state.arm_state.uiArmRetractionFlag==1)
               {
                  _last_arm_state_estimation.arm_state_estimation.arm_state =
                        _last_arm_state_estimation.arm_state_estimation.CONTRACTED;
               }else if(received_state.arm_state.uiArmExtensionFlag==1)
               {
                  _last_arm_state_estimation.arm_state_estimation.arm_state =
                        _last_arm_state_estimation.arm_state_estimation.EXTENDED;
               }

               _last_arm_state_estimation.header.stamp = ros::Time::now();
               _last_arm_state_estimation.header.seq++;

               //Publish data
               qnx_arm_state.publish(_last_arm_state_estimation);
            }
            else if(received_state.type==QuadState)
            {
               _last_quad_state_stimation->quad_state_estimation_with_covariance.altitude =
                     received_state.uav_state.hAgl.dHAgl;

               _last_quad_state_stimation->quad_state_estimation_with_covariance.angular_velocity.roll =
                     received_state.uav_state.rates.dRateP;
               _last_quad_state_stimation->quad_state_estimation_with_covariance.angular_velocity.pitch =
                     received_state.uav_state.rates.dRateQ;
               _last_quad_state_stimation->quad_state_estimation_with_covariance.angular_velocity.yaw =
                     received_state.uav_state.rates.dRateR;


               _last_quad_state_stimation->quad_state_estimation_with_covariance.attitude.roll =
                     received_state.uav_state.attitude.dPhiEuler;
               _last_quad_state_stimation->quad_state_estimation_with_covariance.attitude.pitch =
                     received_state.uav_state.attitude.dThetaEuler;
               _last_quad_state_stimation->quad_state_estimation_with_covariance.attitude.yaw =
                     received_state.uav_state.attitude.dPsiEuler;


               _last_quad_state_stimation->quad_state_estimation_with_covariance.attitude_commands.roll =
                     received_state.uav_state.contQuad.dRollQuad;
               _last_quad_state_stimation->quad_state_estimation_with_covariance.attitude_commands.pitch =
                     received_state.uav_state.contQuad.dPitchQuad;
               _last_quad_state_stimation->quad_state_estimation_with_covariance.attitude_commands.yaw =
                     received_state.uav_state.contQuad.dYawQuad;


               _last_quad_state_stimation->quad_state_estimation_with_covariance.linear_acceleration.x =
                     received_state.uav_state.accel.dAx;
               _last_quad_state_stimation->quad_state_estimation_with_covariance.linear_acceleration.y =
                     received_state.uav_state.accel.dAy;
               _last_quad_state_stimation->quad_state_estimation_with_covariance.linear_acceleration.z =
                     received_state.uav_state.accel.dAz;


               _last_quad_state_stimation->quad_state_estimation_with_covariance.linear_velocity.x =
                     received_state.uav_state.velEarth.dVelNorth;
               _last_quad_state_stimation->quad_state_estimation_with_covariance.linear_velocity.y =
                     received_state.uav_state.velEarth.dVelWest;
               _last_quad_state_stimation->quad_state_estimation_with_covariance.linear_velocity.z =
                     received_state.uav_state.velEarth.dVelUp;

               _last_quad_state_stimation->quad_state_estimation_with_covariance.position.x =
                     received_state.uav_state.posRel.dXRel;
               _last_quad_state_stimation->quad_state_estimation_with_covariance.position.y =
                     received_state.uav_state.posRel.dYRel;
               _last_quad_state_stimation->quad_state_estimation_with_covariance.position.z =
                     received_state.uav_state.posRel.dZRel;
               _last_quad_state_stimation->quad_state_estimation_with_covariance.position.status = 1;
               _last_quad_state_stimation->quad_state_estimation_with_covariance.flying_state =
                     received_state.uav_state.flyingState.uiFlyingState;

                    //std::cerr << "UDP Receiver: FlyingState: " <<  (unsigned short)_last_quad_state_stimation->quad_state_estimation_with_covariance.flying_state << std::endl;

               _last_quad_state_stimation->header.stamp = ros::Time::now();
               _last_quad_state_stimation->header.seq = received_state.uav_state.heartBeat.uiHeartBeat;
            }
            else
            {
               std::cerr << "Unknoknw state received.." << std::endl;
            }

         }
      }
   }

private:
   int port;
   CUDPServer _udp_server;
   ros::Publisher qnx_arm_state;
   arcas_msgs::QuadStateEstimationWithCovarianceStamped *_last_quad_state_stimation;
   arcas_msgs::ArmStateEstimationStamped _last_arm_state_estimation;

};

#endif // QNXUDPRECEIVER_H
