#include "util.h"
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <functions/ArgumentData.h>
#include <functions/FormattedTime.h>
#include <boost/lexical_cast.hpp>

#include <arcas_actions_msgs/AALExtensionActionAction.h>
#include <arcas_msgs/ArmControlReferencesStamped.h>
#include <std_msgs/Float64.h>

#define BUFFER_SIZE_ 32768

using namespace std;
using boost::lexical_cast;
using boost::bad_lexical_cast;
using namespace arcas_actions_msgs;
using namespace arcas_msgs;

bool quit = false;
int arm_messages_counter = 0;
ros::Publisher arm_references_publisher;
ros::Publisher pub;
ros::Publisher yaw_pub;

arcas_msgs::ArmControlReferencesStamped ref;

void process_command(char *command, int argc, int id, actionlib::SimpleActionClient<AALExtensionActionAction> &extension_client, int line = -1);

int main(int argc, char **argv) {
  // This is a mono-UAV version form arm handling
  char *line = new char[BUFFER_SIZE_];
  int id = -1;

  if (argc != 2) {
    cerr << "Usage: " << argv[0] << " <uav_id>\n";
    return -1;
  }
  
  try {
    id = lexical_cast<short>(argv[1]);
  } catch (bad_lexical_cast &) {
    cerr << "The second argument must be the UAV id.\n";
    return -2;
  }
  
  // Establishing the ROS communications
  
  ROS_INFO("Initializing ROS.");
  //Init ROS.
  string node_name = "arm_commander_";
  node_name.append(argv[1]);
  ros::init(argc, argv, node_name.c_str());
  ros::NodeHandle n;
  
  // 1. The waypoint list command
  ostringstream topicname;
  topicname << "/quad" << id << "_trajectory/";
  ROS_INFO("Advertising %s topic.", topicname.str().c_str());
  pub = n.advertise<arcas_msgs::PathWithCruiseStamped> (topicname.str().c_str(), 1);
  
  // 2. The extend arm client
  ostringstream extension_server_name;
  
  if (id > 1) {
    extension_server_name << "/aal_" << id << "/extension";
  } else {
    extension_server_name << "/aal_extension";
  }
  ROS_INFO("Connecting to the server %s", extension_server_name.str().c_str());
  actionlib::SimpleActionClient<AALExtensionActionAction> extension_client(extension_server_name.str().c_str(), true);
  
  // 3. Arm references publiser
  ostringstream arm_references_topic;
  arm_references_topic << "/aal_" << id << "/arm_control_references";
  ROS_INFO("Advertising %s topic.", arm_references_topic.str().c_str());
  arm_references_publisher = n.advertise<arcas_msgs::ArmControlReferencesStamped>(arm_references_topic.str().c_str(), 1);
  
  // 3. Arm references publiser
  ostringstream yaw_topic;
  yaw_topic << "/ual_" << id << "/yaw";
  ROS_INFO("Advertising %s topic.", arm_references_topic.str().c_str());
  yaw_pub= n.advertise<std_msgs::Float64>(yaw_topic.str().c_str(), 1);
  
  while (!quit && ros::ok()) {
    cout << " > ";
    cin.getline(line, BUFFER_SIZE_);
    process_command(line,argc , id, extension_client);
  }
  delete[] line;
  
  return 0;
}

void process_command(char *command, int argc, int id, actionlib::SimpleActionClient<AALExtensionActionAction> &extension_client, int line) {
  char *tok = strtok(command, " ");
  if (tok == NULL) {
    return;
  }
  string s(tok);
    
    if ( s == "quit" ) {
      quit = true;
    } else if (id > 0) {
      if ( s == "extend" ) {
	ROS_INFO("Commanding arm extension. Waiting for the task to be completed");
	AALExtensionActionGoal extend_goal;
	extend_goal.action = extend_goal.EXTENDED;
	extension_client.sendGoalAndWait(extend_goal);
	ROS_INFO("Arm extension executed successfully.");
      } else if ( s == "contract" ) {
      ROS_INFO("Commanding arm contraction. Waiting for the task to be completed");
	AALExtensionActionGoal extend_goal;
	extend_goal.action = extend_goal.CONTRACTED;
	extension_client.sendGoalAndWait(extend_goal);
	ROS_INFO("Arm contraction executed successfully.");
      } else if ( s == "go") {
	// Load trajectories
	char *name = strtok(NULL, " ");
	ROS_INFO("Commanding the trajectory in file: %s", name);
	
	if (name != NULL) {
	  PathWithCruise path = loadTrajetory(name, false, false);
	  ROS_INFO("Loaded trajectory. Number of waypoints: %d.", (int) path.waypoints.size());
	  PathWithCruiseStamped path_stamp;
	  path_stamp.path_with_cruise = path;
	  path_stamp.header.seq = 0;
	  path_stamp.header.stamp = ros::Time::now();
	  pub.publish(path_stamp);
	}
	sleep(1);
      } else if ( s == "arm" ) {
	char *token = strtok(NULL, " "); 
	
	ref.header.frame_id = "arm_frame";
	ref.header.seq = arm_messages_counter;
	ref.header.stamp = ros::Time::now();
	int n_tok = 0;
	
	while (token != NULL) {
	  if (n_tok < 7) {
	    try {
	      ref.arm_control_references.position_ref.at(n_tok) = lexical_cast<double>(token);
	      n_tok++;
	      token = strtok(NULL, " ");
	    } catch (bad_lexical_cast &) {
	      cerr << "Error while processing the command.";
	      if (line >= 0) {
		cerr << " In line: " << line;
	      }
	      cerr << endl;
	      n_tok = -1;
	      token = NULL;
	    }
	  }
	}
	if (n_tok >= 7) {
	  ROS_INFO("Sending arm references. Command: %s", command);
	  arm_references_publisher.publish(ref);
	  arm_messages_counter++;
	}
      } else if (s == "load") {
	// Load a sequence of commands
	char *name = strtok(NULL, " ");
	if (name != NULL) {
	  ROS_INFO("Loading an arm command file. Filename: %s", name);
	  int line = 0;
	  ifstream ifs(name);
	  if (!ifs.is_open()) {
	    cerr << " Could not opend the file " << name << endl;
	  }
	  char * buffer = new char[BUFFER_SIZE_];
	  while (ifs.good() && ros::ok()) {
	    ifs.getline(buffer, BUFFER_SIZE_);
	    process_command(buffer, argc, id, extension_client, line);
	    line++;
	  }
	  delete[] buffer;
	}
      } else if (s == "open") {
	ref.arm_control_references.position_ref[6] = 1.0;
	ROS_INFO("Open command. Sending arm references. Command: %s", command);
	arm_references_publisher.publish(ref);
	arm_messages_counter++;
      } else if (s == "close") {
	tok = strtok(NULL, " ");
	double actuator_value = 0.2;
	
	if (tok != NULL) {
	  try {
	    actuator_value = lexical_cast<double>(tok);
	  } catch (bad_lexical_cast &) {
	    cerr << "Error while processing the close command.";
	    if (line >= 0) {
	      cerr << " In line: " << line;
	    }
	    cerr << endl;
	  }
	}
	ref.arm_control_references.position_ref[6] = actuator_value;
	ROS_INFO("Close command. Actuator value: %f", actuator_value);
	arm_references_publisher.publish(ref);
	arm_messages_counter++;
      } else if (s == "sleep") {
	tok = strtok(NULL, " ");
	int sleep_time;
	if (tok != NULL) {
	  try {
	    sleep_time = lexical_cast<int>(tok);
	    sleep(sleep_time);
	  } catch (bad_lexical_cast &) {
	    cerr << "Error while processing the sleep command.";
	    if (line >= 0) {
	      cerr << " In line: " << line;
	    }
	    cerr << endl;
	  }
	} 
      } else if (s == "yaw") {
	tok = strtok(NULL, " ");
	double yaw;
	if (tok != NULL) {
	  try {
	    yaw = lexical_cast<double>(tok);
	    std_msgs::Float64 y;
	    y.data = yaw;
	    yaw_pub.publish(y);
	    ROS_INFO("Yaw command. Actuator value: %f", yaw);
	  } catch (bad_lexical_cast &) {
	    cerr << "Error while processing the yaw command.";
	    if (line >= 0) {
	      cerr << " In line: " << line;
	    }
	    cerr << endl;
	  }
	} 
      } else {
	 cerr << "Error: could not understand the command.";
	if (line >= 0) {
	  cerr << " In line: " << line;
	}
	cerr << endl;
      }
    
    } else {
      cerr << "Error: Command not found.";
      if (line >= 0) {
	  cerr << " In line: " << line;
	}
	cerr << endl;
    }
    sleep(1);
}
