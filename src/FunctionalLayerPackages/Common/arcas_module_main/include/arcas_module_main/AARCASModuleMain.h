#ifndef AARCASMODULEMAIN_H
#define AARCASMODULEMAIN_H
#include <ros/ros.h>

#define RATE 100.0

class AARCASModuleMain
{
public:
  AARCASModuleMain(ros::NodeHandle *n, float rate=RATE)
  {
	/*
	 ROS Timer
	 */
	main_loop_timer_ = n->createTimer(ros::Duration(1/rate),
									  boost::bind(&AARCASModuleMain::mainLoop, this, _1));
  }
  ~AARCASModuleMain()
  {

  }

private:
  /*
   * Mainloop callback
   */
  virtual void mainLoop(const ros::TimerEvent& te)=0;

  /*
   Ros timer for main loop
   */
  ros::Timer main_loop_timer_;

};

#endif // AARCASMODULEMAIN_H
