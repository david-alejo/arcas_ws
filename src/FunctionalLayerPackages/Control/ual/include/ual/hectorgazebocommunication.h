/*!
 * \file
 * \brief Communication class for gazebo simulator
 *  This class contains the protocol to communicate with simulated uav
 */
#ifndef HECTORGAZEBOCOMMUNICATION_H
#define HECTORGAZEBOCOMMUNICATION_H
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <arcas_msgs/QuadStateEstimationWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>

/*!
 * \brief Update rate of ual node
 */
#define SEND_RATE 100.0

/*!
 * \brief Communication class for gazebo simulator
 */
class HectorGazeboCommunication
{
public:
   HectorGazeboCommunication(ros::NodeHandle *n);

    /*!
     * \brief Get last quad state estimation message
     * \param message contains information about the state of the uav
     */
   arcas_msgs::QuadStateEstimationWithCovarianceStamped getQuadStateEstimation();

    /*!
     * \brief Set the actual command that have to be sent to gazebo.
     * \param Twist message, contains lineal and angular velocity
     */
   void setTwist(geometry_msgs::Twist t);

private:

    /*!
     * \brief Callback to receive position and attitude of the uav form Gazebo
     * \param Odometry standar message
     */
   void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom);

    /*!
     * \brief To send data to gazebo at fixed step
     * \param Timer event message from ros call.
     */
   void sendLoop(const ros::TimerEvent& te);

    /*!
     * \brief Velocity publisher, tos end commands to gazebo
     */
   ros::Publisher velocity_control_pub_;

    /*!
     * \brief Subscribe to odometry information from gazebo
     */
   ros::Subscriber odometry_sub_;

    /*!
     * \brief timer for main loop
     */
   ros::Timer send_loop_timer_;

    /*!
     * \brief Velocity command to send to gazebo
     */
   geometry_msgs::Twist twist_command_to_send_;

    /*!
     * \brief Message to hold the currect state of the uav.
     */
   arcas_msgs::QuadStateEstimationWithCovarianceStamped last_quad_state_estimation;
};

#endif // HECTORGAZEBOCOMMUNICATION_H
