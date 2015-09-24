#include <ual/hectorgazebocommunication.h>
HectorGazeboCommunication::HectorGazeboCommunication(ros::NodeHandle *n)
{
   velocity_control_pub_ = n->advertise<geometry_msgs::Twist> ("cmd_vel", 0);
   odometry_sub_ = n->subscribe("ground_truth/state",
               1, &HectorGazeboCommunication::odometryCallback,
                        this);

   send_loop_timer_ = n->createTimer(ros::Duration(1/SEND_RATE),
               boost::bind(&HectorGazeboCommunication::sendLoop, this, _1));
}

arcas_msgs::QuadStateEstimationWithCovarianceStamped HectorGazeboCommunication::getQuadStateEstimation()
{
   return last_quad_state_estimation;
}

void HectorGazeboCommunication::setTwist(geometry_msgs::Twist t)
{
   twist_command_to_send_ = t;
}

void HectorGazeboCommunication::odometryCallback(const nav_msgs::Odometry::ConstPtr& odom)
{

   //Calculate lineal acceleration
   tf::Vector3 lastVelocity(
            last_quad_state_estimation.quad_state_estimation_with_covariance.linear_velocity.x,
            last_quad_state_estimation.quad_state_estimation_with_covariance.linear_velocity.y,
            last_quad_state_estimation.quad_state_estimation_with_covariance.linear_velocity.z);

   tf::Vector3 actualVelocity(odom->twist.twist.linear.x,
                        odom->twist.twist.linear.y,
                        odom->twist.twist.linear.z);

   tf::Vector3 acceleration = (lastVelocity -actualVelocity) * 100;



   last_quad_state_estimation.quad_state_estimation_with_covariance.altitude =
         odom->pose.pose.position.z;

   last_quad_state_estimation.quad_state_estimation_with_covariance.angular_velocity.roll = odom->twist.twist.angular.x;
   last_quad_state_estimation.quad_state_estimation_with_covariance.angular_velocity.pitch = odom->twist.twist.angular.y;
   last_quad_state_estimation.quad_state_estimation_with_covariance.angular_velocity.yaw = odom->twist.twist.angular.z;

   last_quad_state_estimation.quad_state_estimation_with_covariance.attitude_commands.roll = 0;
   last_quad_state_estimation.quad_state_estimation_with_covariance.attitude_commands.pitch = 0;
   last_quad_state_estimation.quad_state_estimation_with_covariance.attitude_commands.yaw = 0;

   last_quad_state_estimation.quad_state_estimation_with_covariance.linear_acceleration.x = acceleration.getX();
   last_quad_state_estimation.quad_state_estimation_with_covariance.linear_acceleration.y = acceleration.getY();
   last_quad_state_estimation.quad_state_estimation_with_covariance.linear_acceleration.z = acceleration.getZ();

   last_quad_state_estimation.quad_state_estimation_with_covariance.linear_velocity.x = actualVelocity.getX();
   last_quad_state_estimation.quad_state_estimation_with_covariance.linear_velocity.y = actualVelocity.getY();
   last_quad_state_estimation.quad_state_estimation_with_covariance.linear_velocity.z = actualVelocity.getZ();


   last_quad_state_estimation.quad_state_estimation_with_covariance.position.x = odom->pose.pose.position.x;
   last_quad_state_estimation.quad_state_estimation_with_covariance.position.y = odom->pose.pose.position.y;
   last_quad_state_estimation.quad_state_estimation_with_covariance.position.z = odom->pose.pose.position.z;


   tf::Quaternion orientation(odom->pose.pose.orientation.x,
               odom->pose.pose.orientation.y,
               odom->pose.pose.orientation.z,
               odom->pose.pose.orientation.w);

   /*
    *If do not work propertly, have to change solution numer parameter.
    */
   tf::Matrix3x3(orientation).getRPY(last_quad_state_estimation.quad_state_estimation_with_covariance.attitude.roll,
                     last_quad_state_estimation.quad_state_estimation_with_covariance.attitude.pitch,
                     last_quad_state_estimation.quad_state_estimation_with_covariance.attitude.yaw);


   //Update Header
   last_quad_state_estimation.header.stamp = ros::Time::now();
   last_quad_state_estimation.header.seq++;
   last_quad_state_estimation.header.frame_id = "/world";
}
void HectorGazeboCommunication::sendLoop(const ros::TimerEvent& te)
{
   velocity_control_pub_.publish(twist_command_to_send_);
}
