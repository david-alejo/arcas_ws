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
#include <arcas_msgs/QuadStateEstimationWithCovarianceStamped.h>
#include <arcas_msgs/QuadControlReferencesStamped.h>
#include <sensor_msgs/LaserScan.h>

// Conversion includes
#include<geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

// TF publication includes
#include <tf/transform_broadcaster.h>

// Quaternion includes
#include <transform_from_vicon/rotateOP/Quaternion.h>

#include <visualization_msgs/Marker.h>

#include <RVO2-3D_unstable/RVOSimulator.h>
#include <RVO2-3D_unstable/Agent.h>

#include <functions/ArgumentData.h>

using namespace RVO_UNSTABLE;
using std::cout;
using std::cerr;
using std::endl;

class scanToOctomap
{
public:
    scanToOctomap(std::string uav, uint id, RVOSimulator *sim, double xy_dist, double z_dist): 
    uav(uav), id(id), xy_dist(xy_dist), z_dist(z_dist), added_to_sim(false)
    {
        std::string temp_uav = uav;
        temp_uav.append("/quad_state_estimation");
	std::string temp_ref = uav;
	temp_ref.append("/quad_control_references");
	
	simulator_ = sim;
	
        //Initialize subscribers
        quad_state_sub_ = n_.subscribe(temp_uav.c_str(),1,&scanToOctomap::quadStateCallback, this);
	ref_sub_ = n_.subscribe(temp_ref.c_str(),1,&scanToOctomap::refCallback, this);
	std::ostringstream os, os_elli, os_robot, os_line;
	os << "arrow_marker" << id;
	mark_pub_ = n_.advertise<visualization_msgs::Marker>(os.str().c_str(), 0);
	os_elli << "ellipse_marker" << id;
	ellipse_pub_ = n_.advertise<visualization_msgs::Marker>(os_elli.str().c_str(), 0);
	os_robot << "robot_marker" << id;
	robot_pub_ = n_.advertise<visualization_msgs::Marker>(os_robot.str().c_str(), 0);
	os_line << "line_marker" << id;
	line_pub_ = n_.advertise<visualization_msgs::Marker>(os_line.str().c_str(), 0);
    }

    void quadStateCallback(const arcas_msgs::QuadStateEstimationWithCovarianceStamped quad_state)
    {

        q_s2.fromEuler(quad_state.quad_state_estimation_with_covariance.attitude.roll,
                       quad_state.quad_state_estimation_with_covariance.attitude.pitch,
                       quad_state.quad_state_estimation_with_covariance.attitude.yaw,
                       rotateOp::TransformationTypes::EULER123);

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(quad_state.quad_state_estimation_with_covariance.position.x,
                                         quad_state.quad_state_estimation_with_covariance.position.y,
                                         quad_state.quad_state_estimation_with_covariance.position.z) );


        orientation.setX(q_s2.getX());
        orientation.setY(q_s2.getY());
        orientation.setZ(q_s2.getZ());
        orientation.setW(q_s2.getW());

        transform.setRotation(orientation);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", uav));
   	curr_pos_x = quad_state.quad_state_estimation_with_covariance.position.x;
	curr_pos_y = quad_state.quad_state_estimation_with_covariance.position.y;
	curr_pos_z = quad_state.quad_state_estimation_with_covariance.position.z;

	publishEllipse();
	publishRobot();
	publishDistanceToMesh();
    }

    void refCallback(const arcas_msgs::QuadControlReferencesStamped quad_ref)
    {
      visualization_msgs::Marker marker;
      marker.id = id;
      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      marker.header.frame_id = "/map";
      marker.header.stamp = ros::Time::now();

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "";
      marker.id = 0;

      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      marker.type = visualization_msgs::Marker::ARROW;

      // Set the marker action.  Options are ADD and DELETE
      marker.action = visualization_msgs::Marker::ADD;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      geometry_msgs::Point p;
      p.x = curr_pos_x;
      p.y = curr_pos_y;
      p.z = curr_pos_z;
      marker.points.push_back(p);
      geometry_msgs::Point p2;
      p2.x = quad_ref.quad_control_references.position_ref.x;
      p2.y = quad_ref.quad_control_references.position_ref.y;
      p2.z = quad_ref.quad_control_references.position_ref.z;
      marker.points.push_back(p2);

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 0.1;
      marker.scale.y = 0.2;
      marker.scale.z = 0.0;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;

      marker.lifetime = ros::Duration();

      // Publish the marker
      mark_pub_.publish(marker);

    }

    void publishEllipse() {
      visualization_msgs::Marker marker;
      marker.id = id;
      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      marker.header.frame_id = "/map";
      marker.header.stamp = ros::Time::now();

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "";
      marker.id = 0;

      // Set the marker type. 
      marker.type = visualization_msgs::Marker::SPHERE;

      // Set the marker action.  Options are ADD and DELETE
      marker.action = visualization_msgs::Marker::ADD;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.x = curr_pos_x;
      marker.pose.position.y = curr_pos_y;
      marker.pose.position.z = curr_pos_z;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = xy_dist;
      marker.scale.y = xy_dist;
      marker.scale.z = z_dist;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 0.5;

      marker.lifetime = ros::Duration();

      // Publish the marker
      ellipse_pub_.publish(marker);
    }
    
    void publishRobot() {
      visualization_msgs::Marker marker;
      marker.id = id;
      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      marker.header.frame_id = "/map";
      marker.header.stamp = ros::Time::now();

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "";
      marker.id = 0;

      // Set the marker type.
      marker.type = visualization_msgs::Marker::MESH_RESOURCE;
      marker.mesh_resource = "package://ual/models/quadrotor/quadrotor_base.dae";

      // Set the marker action.  Options are ADD and DELETE
      marker.action = visualization_msgs::Marker::ADD;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.x = curr_pos_x;
      marker.pose.position.y = curr_pos_y;
      marker.pose.position.z = curr_pos_z;
      marker.pose.orientation.x = orientation.getX();
      marker.pose.orientation.y = orientation.getY();
      marker.pose.orientation.z = orientation.getZ();
      marker.pose.orientation.w = orientation.getW();

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
//       marker.scale.x = xy_dist;
//       marker.scale.y = xy_dist;
//       marker.scale.z = z_dist;
       marker.scale.x = 1.0;
       marker.scale.y = 1.0;
       marker.scale.z = 1.0; 
//       marker.color.r = 1.0f;
//       marker.color.g = 0.0f;
//       marker.color.b = 1.0f;
//       marker.color.a = 0.5;

      marker.lifetime = ros::Duration();

      // Publish the marker
      robot_pub_.publish(marker);
    }
    
    void publishDistanceToMesh() {
      // set the marker
      visualization_msgs::Marker marker;
      marker.id = id;
      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      marker.header.frame_id = "/map";
      marker.header.stamp = ros::Time::now();

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "";
      marker.id = 0;

      // Set the marker type.
      marker.type = visualization_msgs::Marker::LINE_LIST;

      // Set the marker action.  Options are ADD and DELETE
      marker.action = visualization_msgs::Marker::ADD;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.5;
      
      // Before calling the simulator, make sure that the agent is already registered
      if (!added_to_sim) {
	id_sim = simulator_->getNumAgents();
	Vector3 v(curr_pos_x, curr_pos_y, curr_pos_z);
	simulator_->addAgent(v);
      }
      
      // Calculate distances
      std::vector<PQP_DistanceResult> dists = simulator_->getClosestPoints(id_sim);
//       cerr << "Dists.size() = " << dists.size() << "\t";
//       cerr << "ObstacleDist = " << simulator_->getAgent(id_sim)->obstacleDist_ <<  endl;
      for (unsigned int i = 0; i < dists.size(); i++) {
	geometry_msgs::Point p, q;
	q.x = dists[i].p1[0];
	q.y = dists[i].p1[1];
	q.z = dists[i].p1[2];
	p.x = dists[i].p2[0];
	p.y = dists[i].p2[1];
	p.z = dists[i].p2[2];
	marker.points.push_back(q);
	marker.points.push_back(p);
      }
      line_pub_.publish(marker);
    }
    
private:
    ros::NodeHandle n_;
    ros::Subscriber quad_state_sub_;
    ros::Subscriber ref_sub_;
    ros::Publisher mark_pub_;
    ros::Publisher robot_pub_;
    ros::Publisher ellipse_pub_;
    ros::Publisher line_pub_;
    std::string uav;
    rotateOp::Quaternion q_s2;
    uint id;
    double curr_pos_x;
    double curr_pos_y;
    double curr_pos_z;
    double xy_dist, z_dist;
    tf::Quaternion orientation;
    RVOSimulator *simulator_;
    uint id_sim; // ID in the simulator
    bool added_to_sim; // Starts to false and then adds the agent to the simulator
};

using std::string;

int main(int argc, char **argv)
{
    // COnf stuff
  
    
    double xy_dist = 0.65; 
    double z_dist = 0.4; 
    string s("/home/sinosuke/catkin_ws/src/use_tg/experiments/Media/models/mockup_separate_convex.3ds");
    
    //Initialize ROS and specify the node name;
    ros::init(argc, argv, "scan_to_octomap");
    
    functions::ArgumentData arg(argc, argv);
    
    if (arg.isOption("r_xy")) {
      arg.getOption<double>("r_xy", xy_dist);
      argc -= 2;
    }
    if (arg.isOption("r_z")) {
      arg.getOption<double>("r_z", z_dist);
      argc -= 2;
    }
    if (arg.isOption("file")) {
      arg.getOption<string>("file", s);
      argc -= 2;
    }
    

    //RVOSimulator(float timeStep, float neighborDist, size_t maxNeighbors, float timeHorizon, float radius, float radius_obstacle, float maxSpeed, 
		//			      const float timeObstacle_ = -1.0, const Vector3 &velocity = Vector3(), float pure_delay = 0.0, float exponent = 1.0);
    RVOSimulator sim(1.0, 4.0, 4, 4.0, xy_dist, xy_dist, 1.0, 1.0);
    sim.setAgentDefaultObstacleDist(2.0);
    
    if (!sim.loadScenario(s)) {
      ROS_ERROR("Could not load scenario file.");
    }
    
    std::string uav;
    scanToOctomap *publishers[argc-1];
    for(int i=1; i < argc; i++)
    {
        uav = "/ual_";
        uav.append(argv[i]);
        publishers[i-1] = new scanToOctomap(uav, atoi(argv[i]), &sim, xy_dist, z_dist);
    }

    ros::spin();
    ros::shutdown();

    return 0;
}

