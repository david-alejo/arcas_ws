/*
 * scan_to_octomap.cpp
 *
 * Created on: 05/12/2013
 *      Author: Andreas Pfrunder
*/

// ROS Standard Includes
#include <ros/ros.h>
#include <iostream>

// Messagetypes
#include <arcas_msgs/QuadStateEstimationStamped.h>
#include <sensor_msgs/LaserScan.h>

// Conversion includes
#include<geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

// TF publication includes
#include <tf/transform_broadcaster.h>

// Quaternion includes
#include <eth_slam/rotateOP/Quaternion.h>

class scanToOctomap
{
public:
    scanToOctomap()
    {
        //Initialize subscribers
        quad_state_sub_ = n_.subscribe("/ual_1/quad_state_estimation",1,&scanToOctomap::quadStateCallback, this);
        scan_sub_ = n_.subscribe("/scan",1,&scanToOctomap::scanCallback, this);

        //Initialize publishers
        scan_octomap_pub_ = n_.advertise<sensor_msgs::LaserScan>("/scan_octomap", 1);
    }

    void quadStateCallback(const arcas_msgs::QuadStateEstimationStamped quad_state)
    {
        position_z_ = quad_state.quad_state_estimation.position.z;

        q_s2.fromEuler(quad_state.quad_state_estimation.attitude.roll,
                       quad_state.quad_state_estimation.attitude.pitch,
                       quad_state.quad_state_estimation.attitude.yaw,rotateOp::TransformationTypes::EULER123);

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(quad_state.quad_state_estimation.position.x,
                                         quad_state.quad_state_estimation.position.y,
                                         quad_state.quad_state_estimation.position.z) );


        tf::Quaternion orientation;
        orientation.setX(q_s2.getX());
        orientation.setY(q_s2.getY());
        orientation.setZ(q_s2.getZ());
        orientation.setW(q_s2.getW());

        transform.setRotation(orientation);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/scan_octomap"));
    }

    void scanCallback(const sensor_msgs::LaserScan scan)
    {
        scan_octo_ = scan;
        scan_octo_.header.frame_id = "/scan_octomap";

        //Filter non valid VICON data which are sent by VICON when it looses the object
        if ((position_z_ != ultimate_position_z_) &&
                (position_z_ != penultimate_position_z))
        {
            scan_octomap_pub_.publish(scan_octo_);
        }
        else
        {
            ROS_WARN_STREAM("I'm not publishing scan_to_octomap!");
        }

        penultimate_position_z = ultimate_position_z_;
        ultimate_position_z_ = position_z_;

    }

private:
    ros::NodeHandle n_;
    ros::Subscriber quad_state_sub_;
    ros::Subscriber scan_sub_;
    ros::Publisher scan_octomap_pub_;

    rotateOp::Quaternion q_s2;
    sensor_msgs::LaserScan scan_octo_;

    double position_z_;
    double ultimate_position_z_;
    double penultimate_position_z;
};


int main(int argc, char **argv)
{
    //Initialize ROS and specify the node name;
    ros::init(argc, argv, "scan_to_octomap");
    scanToOctomap scanToOctomapObject;

    ros::spin();
    ros::shutdown();

    return 0;
}
