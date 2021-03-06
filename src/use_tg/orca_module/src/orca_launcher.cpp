/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%

#include "orca_module/ORCAModule.h"
#include <sstream>
#include "functions/ArgumentData.h"

using namespace std;

/**
 * This file launchs one ORCA module for one UAV
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
// %Tag(INIT)%
  
  functions::ArgumentData arg(argc, argv);
  if (arg.size() < 3) {
    ROS_ERROR("Use: %s <id> <data_filename> [<mesh_file_1> ... <mesh_file_n>]", arg.at(0).c_str());
    
    return -1;
  }
  
  uint id;
  istringstream is(arg.at(1));
  is >> id;
  
  ORCAModule orca(id, arg.at(2));
  for (unsigned int i = 3; i < arg.size(); i++) {
    if (arg.at(i).size() > 2 && arg.at(i).at(0) == '_' && arg.at(i).at(1) == '_' ) {
      // ROS arguments --> ignore them
      continue;
    }
    ROS_INFO("Argument %d = %s" , i , arg.at(i).c_str());
    if (orca.addObstacles(arg.at(i))) {
      ROS_INFO("%s scenario file loaded successfully.", arg.at(i).c_str());
      
    } else {
      ROS_ERROR("%s scenario file could not be loaded .", arg.at(i).c_str()); 
    }
  }
  
  ostringstream name;
  name << "orca" << orca.getID();
  ros::init(argc, argv, name.str());
  
  ROS_INFO("Orca launcher: starting ROS communications...");
  
  orca.startROSComms();
  
  // %End tag(INIT)%
  orca.mainLoop();
//   ros::spin();

  return 0;
}
