/*!
 * \file
 * \brief Communication class for gazebo with simulink simulator
 *  This class contains the protocol to communicate with simulated simulink controller
 */
#ifndef SIMULINGAZEBOCOMMUNICATION_H
#define SIMULINGAZEBOCOMMUNICATION_H
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <arcas_msgs/QuadControlReferencesStamped.h>
#include <arcas_msgs/QuadStateEstimationWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>

/*!
 * \brief Update rate of ual node
 */
#define SEND_RATE 100.0

/*!
 * \brief Communication class for gazebo simulink simulator
 */
class SimulinkGazeboCommunication
{
public:
    SimulinkGazeboCommunication(ros::NodeHandle *n);

    /*!
     * \brief Get last quad state estimation message
     * \param message contains information about the state of the uav
     */
   arcas_msgs::QuadStateEstimationWithCovarianceStamped getQuadStateEstimation();

    /*!
     * \brief Set Quad Control References message to send to simulink model
     * \param Quad control references message
     */
    void setQuadControlReferences(arcas_msgs::QuadControlReferences ctrl_ref);

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
     * \brief Quad control references publisher to send command to simulink model.
     */
    ros::Publisher quad_control_references_pub_;

    /*!
     * \brief Subscribe to odometry information from gazebo
     */
   ros::Subscriber odometry_sub_;

    /*!
     * \brief timer for main loop
     */
   ros::Timer send_loop_timer_;

    /*!
     * \brief Message to send quad control references to send
     */
    arcas_msgs::QuadControlReferencesStamped quad_control_references_to_send;

    /*!
     * \brief Message to hold the currect state of the uav.
     */
   arcas_msgs::QuadStateEstimationWithCovarianceStamped last_quad_state_estimation;
};

#endif // SIMULINGAZEBOCOMMUNICATION_H
