/*!
 * \file
 * \brief Main class for ual hector gazebo version.
 *  This class contains all functionality to communicate ros with Hectro Gazebo simulator.
 */
#ifndef UALHECTORGAZEBO_H
#define UALHECTORGAZEBO_H
#include<ual/common.h>
#include<ual/roscommunication.h>
#include<ual/hectorgazebocommunication.h>
#include<ual/actions/land_action.h>
#include<ual/actions/take_off_action.h>
#include<gazebo/common/PID.hh>

/*!
 * \brief Update rate of ual node
 */
#define UPDATE_RATE 100.0

/*!
 * \brief Altitude for land
 */
#define LAND_Z 0.15

/*!
 * \brief Altitude for take off
 */
#define TAKE_OFF_Z 0.3

#define PURE_DELAY 0.5

using namespace gazebo;

/*!
 * \brief Ual Hector Class to communicate simulator with ros.
 */
class UALHectorGazebo
{
public:
   UALHectorGazebo(ros::NodeHandle *n, int uavId);

private:

    /*!
     * \brief Update loop at fixed step
     * \param Timer event message from ros call.
     */
   void updateLoop(const ros::TimerEvent& te);

    /*!
     * \brief timer for main loop
     */
   ros::Timer update_timer_;

    /*!
     * \brief Ros communication object; Implements standart interface from/to ROS.
     */
   RosCommunication generalCommunications;

    /*!
     * \brief Hector Gazebo Communication object; Implements standart interface from/to Modified hector quad rotor simulator.
     */
   HectorGazeboCommunication gazeboCommunications;

    /*!
     * \brief Contains actual state of the uav
     */
   StateUAV actual_state;
   std::list<arcas_msgs::QuadStateEstimationWithCovarianceStamped> buffer_;

    /*!
     * \brief When we receive the las callback for land takeoff
     */
   ros::Time takeoff_land_time_callback_;

    /*!
     * \brief Simulate the time that uav needs to takeoff/land
     */
   ros::Duration takeoff_land_minimmum_time_;

    /*!
     * \brief Pids for control position of uav
     */
   common::PID pid_x,pid_y,pid_z,pid_yaw;

    /*!
     * \brief To calculate the exact time between calls
     */
   ros::Time lastUpdatePID;

    /*!
     * \brief Land action server
     */
   LandActionClass landActionServer;

    /*!
     * \brief TakeOff Action Server
     */
   TakeOffActionClass takeoffActionServer;
};

#endif // UALHECTORGAZEBO_H
