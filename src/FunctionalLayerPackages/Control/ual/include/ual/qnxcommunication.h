/*!
 * \file
 * \brief Communication class for gazebo with simulink simulator
 *  This class contains the protocol to communicate with simulated simulink controller
 */
#ifndef QNXCOMMUNICATION_H
#define QNXCOMMUNICATION_H
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <arcas_msgs/QuadControlReferencesStamped.h>
#include <arcas_msgs/QuadStateEstimationWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <boost/thread.hpp>
//Added 10/07/14 size IControlRefArcas 62, withput pragma is 72
#pragma pack(1)
#include<ual/qnxudpreceiver.h>
#include<ual/CUDPClient.h>

/*!
 * \brief Update rate of ual node
 */
#define SEND_RATE 100.0

/*!
 * \brief Communication class for gazebo simulink simulator
 */
class QnxCommunication
{
public:
   QnxCommunication(ros::NodeHandle *n, std::string host, int commandPort, int statePort);

   /*!
    * \brief Get last quad state estimation message
    * \param message contains information about the state of the uav
    */
   arcas_msgs::QuadStateEstimationWithCovarianceStamped getQuadStateEstimation();

   /*!
    * \brief Set Quad Control References message to send to simulink model
    * \param Quad control references message
    */
   void setQuadControlReferences(arcas_msgs::QuadControlReferences ctrl_ref,unsigned char takeoff, unsigned char land);

private:

   /*!
    * \brief To send data to qnx at fixed step
    * \param Timer event message from ros call.
    */
   void sendLoop(const ros::TimerEvent& te);


   /*!
    * \brief timer for main loop
    */
   ros::Timer _send_loop_timer_;


   /*!
    * \brief Thread for receive from udp
    * \param Timer event message from ros call.
    */
   QnxUDPReceiver _receive_task;


   /*!
    * \brief Thread for receive from udp
    * \param Timer event message from ros call.
    */
   boost::thread *_udp_receiver_thread;



   /*!
    * \brief UDP socket to send to qnx;
    */
   CUDPClient _qnx_sender;

   /*!
    * \brief UDP data to send to qnx;
    */
   ArcasUDPControlReferences _control_ref_to_qnx;
};

#endif // QNXCOMMUNICATION_H
