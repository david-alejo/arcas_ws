/*
 * scan_to_octomap.cpp
 *
 * Created on: 05/12/2013
 *      Author: Andreas Pfrunder
*/

// ROS Standard Includes
#include <ros/ros.h>
#include <iostream>


#include <visualization_msgs/Marker.h>


int main(int argc, char **argv)
{
    //Initialize ROS and specify the node name;
    ros::init(argc, argv, "rvizScenario");

    ros::NodeHandle node_handle;
    ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_scenario", 0 );
    
    std::string mesh_resource = "package://ual/Media/models/ARCAS_PipesScenario.STL";
    
    if (argc > 1) {
      std::string s(argv[1]);
      std::ostringstream os;
      os << "file://" << s;
      mesh_resource = os.str();
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = mesh_resource;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1.0;
    marker.color.r = 0.7;
    marker.color.g = 0.7;
    marker.color.b = 0.7;

    while(ros::ok()){
        vis_pub.publish( marker );
        sleep(5);
        ros::spinOnce();
    }

    ros::shutdown();

    return 0;
}
